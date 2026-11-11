# Running the Flutter app against the local FastAPI backend

The upload workflow (create → captures → finalise → status) is the same code
path in both modes; the mode is declared, never guessed. `mock` is the
in-app simulation, `backend` is the real service:

```
VIRTUAL_TOUR_MODE=mock      LocalVirtualTourRemoteDataSource  (default when unset)
VIRTUAL_TOUR_MODE=backend   HttpVirtualTourRemoteDataSource   (needs a base URL)
```

`backend` mode without `VIRTUAL_TOUR_API_BASE_URL` is a boot failure with a
message that says so. There is no silent downgrade to simulation, because a
tour that was never uploaded must not look uploaded.

## 1. Backend

From `backend/`:

```
uv sync                                             # Python 3.12
cp .env.example .env                                # then edit
uv run alembic upgrade head                         # applies the migrations
uv run uvicorn sitesense.main:app --host 127.0.0.1 --port 8000
```

Two settings matter here.

* `SITESENSE_DATABASE_URL` — the default
  `sqlite+aiosqlite:///./var/sitesense.db` is fine for development. Point it at
  a **fresh** file; never run the app's uploads against a database that holds
  real data, and back any such file up first.
* `SITESENSE_ALLOW_DEVELOPMENT_IDENTITY=true` — without it every request is
  answered `401`. The client sends the developer's id in `X-User-Id`, which
  this flag accepts; the value still has to parse as a UUID. This is the
  development identity mechanism, not authentication: leave the flag `false`
  anywhere real users are served, and do not add production JWT to the client
  in its place.

Catches like "capture count above `SITESENSE_MAX_CAPTURE_COUNT` (default 30),
non-JPEG file, image below 640×480, same target index re-posted with different
bytes" are answered `409` by the service, and the client shows the server's own
message rather than a generic failure.

## 2. Flutter

`VIRTUAL_TOUR_PROPERTY_ID` must be a UUID belonging to a row the backend
accepts; the `property_id` column has no foreign key, so nothing needs seeding
to exercise the flow, but a garbage value is stored as given.

```
# Android emulator — 10.0.2.2 is the host machine as seen from the AVD
flutter run --debug \
  --dart-define=VIRTUAL_TOUR_MODE=backend \
  --dart-define=VIRTUAL_TOUR_API_BASE_URL=http://10.0.2.2:8000 \
  --dart-define=VIRTUAL_TOUR_PROPERTY_ID=22222222-2222-2222-2222-222222222222 \
  --dart-define=VIRTUAL_TOUR_USER_ID=11111111-1111-1111-1111-111111111111

# Any device, no network changes on the host
adb reverse tcp:8000 tcp:8000
flutter run --debug --dart-define=VIRTUAL_TOUR_MODE=backend \
  --dart-define=VIRTUAL_TOUR_API_BASE_URL=http://127.0.0.1:8000 …

# Physical device on the same Wi-Fi — uvicorn must listen on the LAN
uv run uvicorn sitesense.main:app --host 0.0.0.0 --port 8000
flutter run --debug --dart-define=VIRTUAL_TOUR_MODE=backend \
  --dart-define=VIRTUAL_TOUR_API_BASE_URL=http://<your-lan-ip>:8000 …
```

The LAN address is a launch argument, never a literal in `lib/`: it changes
with every network. `--host 0.0.0.0` binds uvicorn to all interfaces, so use it
only on a trusted network and only for the run.

Plain `http://` is permitted in debug builds by
`android/app/src/debug/AndroidManifest.xml`; release builds keep the platform
default and require TLS.

## 3. What the app will not show

Finalisation writes a stitch **manifest** and leaves the tour at
`stitch_pending`. The screen then reads "Awaiting stitching" with that status
and the manifest key. No panorama URL exists at this point and none is
invented: stitching is a worker that has not been built, so a progress bar that
reached "stitched" would be a lie.

## 4. Checking it worked

The live suite talks to a running server over real sockets; it is skipped, not
passed, when no endpoint is declared:

```
# terminal 1, from backend/
uv run alembic upgrade head
uv run uvicorn sitesense.main:app --host 127.0.0.1 --port 8000

# terminal 2, from the repository root
VIRTUAL_TOUR_TEST_API_URL=http://127.0.0.1:8000 \
  flutter test test/virtual_tour_backend_integration_test.dart

# optional: a different identity or property than the shared development ones
VIRTUAL_TOUR_TEST_USER_ID=… VIRTUAL_TOUR_TEST_PROPERTY_ID=… flutter test …

# from the repository root
flutter analyze lib/features/virtual_tour
uv run pytest -q                                               # from backend/
```

`VIRTUAL_TOUR_TEST_API_URL` is read at runtime from the process environment —
this is a host-side test, so it needs none of the `--dart-define` plumbing the
app itself uses. `virtual_tour_upload_test.dart` covers the same contract
against recorded answers and runs without a server.

The server's side of the same run: rows in `virtual_tours` /
`virtual_tour_captures`, JPEGs under `var/captures/tours/<tour-id>/captures/`,
and a `stitch-manifest-<sha16>.json` beside them once the tour is finalised.

Two defects found only by exercising the real thing, both fixed:

* `alembic upgrade head` could not complete on a fresh database — the first
  migration asked `create_table` for `index=True` and then created the same
  `ix_*` indexes again (`index ix_virtual_tours_property_id already exists`).
* `POST /virtual-tours` and `POST …/finalise` answered 500 with
  `MissingGreenlet`: `session.refresh()` leaves `tour.captures` expired, and
  counting it afterwards is lazy IO an `AsyncSession` refuses. The repository
  now reads a tour with `selectinload` in the same SELECT.

