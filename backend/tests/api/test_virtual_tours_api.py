"""The four FastAPI routes, tested over HTTP against a real SQLite database.

These are the contract the Flutter client is written against, so every test
asserts what the app actually reads back: the status word it maps to a Bloc
state, the `storage_key` and `sha256` it stores per capture, and the manifest
file finalisation leaves on disk. Nothing here mocks the service —
`httpx.ASGITransport` drives the same ASGI app `uvicorn` serves, through the
same dependency graph. The only things swapped are where the session and the
object storage point, both under `tmp_path`, so a test run never touches `./var`
and never needs a running server.
"""

import hashlib
import io
import json
import uuid
from collections.abc import AsyncIterator
from datetime import datetime
from pathlib import Path

import httpx
import pytest
from httpx import ASGITransport
from PIL import Image
from sqlalchemy import text
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)

import sitesense.db.models  # noqa: F401 - registers the tables create_all() builds
from sitesense.core.config import Settings
from sitesense.db.base import Base
from sitesense.db.session import get_db_session
from sitesense.dependencies import get_settings
from sitesense.main import app

API = "/api/v1/virtual-tours"

OWNER = str(uuid.uuid4())
INTRUDER = str(uuid.uuid4())
PROPERTY = str(uuid.uuid4())
CAPTURED_AT = "2026-10-25T09:30:00.000Z"


def jpeg(width: int = 640, height: int = 480, *, colour: str = "white") -> bytes:
    """A real encoded JPEG, because Pillow validates the pixels the app sent."""
    out = io.BytesIO()
    Image.new("RGB", (width, height), colour).save(out, format="JPEG")
    return out.getvalue()


def settings_for(tmp_path: Path, *, development_identity: bool = True) -> Settings:
    """Settings built from these arguments and nothing else.

    `_env_file=None` matters: without it a developer's `backend/.env` would leak
    in and the object storage would land in `./var`.
    """
    return Settings(
        _env_file=None,
        database_url=f"sqlite+aiosqlite:///{(tmp_path / 'api.db').as_posix()}",
        capture_storage_root=tmp_path / "objects",
        max_capture_bytes=1_000_000,
        max_capture_count=30,
        cors_origins=[],
        allow_development_identity=development_identity,
    )


@pytest.fixture
async def client(tmp_path) -> AsyncIterator[httpx.AsyncClient]:
    settings = settings_for(tmp_path)
    engine: AsyncEngine = create_async_engine(settings.database_url)
    async with engine.begin() as connection:
        await connection.run_sync(Base.metadata.create_all)
    factory = async_sessionmaker(engine, expire_on_commit=False)

    async def session() -> AsyncIterator[AsyncSession]:
        async with factory() as active:
            yield active

    app.dependency_overrides[get_db_session] = session
    app.dependency_overrides[get_settings] = lambda: settings
    try:
        async with httpx.AsyncClient(
            transport=ASGITransport(app=app),
            base_url="http://testserver",
            headers={"X-User-Id": OWNER},
        ) as http:
            yield http
    finally:
        app.dependency_overrides.clear()
        await engine.dispose()


@pytest.fixture
async def db(tmp_path) -> AsyncIterator[async_sessionmaker[AsyncSession]]:
    """Direct database access, for the assertions an HTTP response cannot make.

    Reads the same file the `client` fixture created its schema in, so a row the
    route committed is visible here without going through the API.
    """
    engine = create_async_engine(settings_for(tmp_path).database_url)
    try:
        yield async_sessionmaker(engine, expire_on_commit=False)
    finally:
        await engine.dispose()


@pytest.fixture
def storage(tmp_path) -> Path:
    return tmp_path / "objects"


async def create_tour(client: httpx.AsyncClient, *, expected: int = 2) -> dict:
    response = await client.post(
        API,
        json={"property_id": PROPERTY, "expected_capture_count": expected},
    )
    assert response.status_code == 201, response.text
    return response.json()


async def upload(
    client: httpx.AsyncClient,
    tour_id: str,
    target_index: int,
    *,
    image: bytes | None = None,
    content_type: str = "image/jpeg",
    pose: dict | None = None,
    headers: dict | None = None,
) -> httpx.Response:
    return await client.post(
        f"{API}/{tour_id}/captures",
        data={
            "target_index": str(target_index),
            "captured_at": CAPTURED_AT,
            "pose_json": json.dumps(
                {"position_x": 1.5, "quaternion_w": 1.0} if pose is None else pose
            ),
        },
        files={
            "image": (
                f"t{target_index:03d}.jpg",
                jpeg() if image is None else image,
                content_type,
            )
        },
        headers={"Idempotency-Key": f"{tour_id}/{target_index}", **(headers or {})},
    )


# ------------------------------------------------------------------ create


async def test_create_returns_the_ids_the_server_generated(client):
    tour = await create_tour(client, expected=3)

    assert uuid.UUID(str(tour["id"]))  # generated by the server, not echoed
    assert tour["property_id"] == PROPERTY
    assert tour["status"] == "uploading"
    assert tour["expected_capture_count"] == 3
    assert tour["uploaded_capture_count"] == 0
    assert tour["stitch_manifest_key"] is None


async def test_create_files_the_tour_under_the_declared_owner(client, db):
    await create_tour(client)

    async with db() as session:
        rows = (
            await session.execute(
                text("select owner_id, property_id, status from virtual_tours")
            )
        ).fetchall()

    assert rows == [(OWNER.replace("-", ""), PROPERTY.replace("-", ""), "uploading")]


async def test_create_rejects_a_property_id_the_server_cannot_parse(client):
    response = await client.post(
        API, json={"property_id": "property_local", "expected_capture_count": 3}
    )
    assert response.status_code == 422


async def test_create_rejects_counts_the_plan_cannot_hold(client):
    below = await client.post(
        API, json={"property_id": PROPERTY, "expected_capture_count": 0}
    )
    assert below.status_code == 422  # the schema's own `ge=1`

    above = await client.post(
        API, json={"property_id": PROPERTY, "expected_capture_count": 31}
    )
    assert above.status_code == 409
    assert "expected_capture_count" in above.json()["error"]["message"]


async def test_an_identity_is_refused_when_the_development_flag_is_off(
    client, tmp_path
):
    """The check the deployment leaves closed: a self-declared header buys nothing."""
    app.dependency_overrides[get_settings] = lambda: settings_for(
        tmp_path, development_identity=False
    )
    response = await client.post(
        API, json={"property_id": PROPERTY}, headers={"X-User-Id": OWNER}
    )
    assert response.status_code == 401
    assert "authentication" in response.json()["detail"].lower()


async def test_the_development_identity_must_be_a_uuid(client):
    empty = await client.post(
        API, json={"property_id": PROPERTY}, headers={"X-User-Id": ""}
    )
    assert empty.status_code == 401

    named = await client.post(
        API, json={"property_id": PROPERTY}, headers={"X-User-Id": "developer"}
    )
    assert named.status_code == 401
    assert "UUID" in named.json()["detail"]


# ----------------------------------------------------------------- captures


async def test_upload_stores_the_bytes_and_answers_with_their_digest(client, storage):
    tour = await create_tour(client)
    image = jpeg(1024, 768)

    response = await upload(client, str(tour["id"]), 1, image=image)
    assert response.status_code == 200, response.text
    ack = response.json()

    assert ack["target_index"] == 1
    assert ack["sha256"] == hashlib.sha256(image).hexdigest()
    assert (ack["width"], ack["height"]) == (1024, 768)
    assert ack["duplicate"] is False
    assert ack["storage_key"] == (
        f"tours/{tour['id']}/captures/001-{ack['sha256'][:16]}.jpg"
    )

    stored = storage / ack["storage_key"]
    assert stored.read_bytes() == image
    # Nothing half-written is left beside the committed object.
    assert list(stored.parent.glob("*.uploading")) == []


async def test_repeating_a_capture_is_answered_as_a_duplicate(client, storage, db):
    tour = await create_tour(client)
    image = jpeg()

    first = (await upload(client, str(tour["id"]), 1, image=image)).json()
    retry = await upload(client, str(tour["id"]), 1, image=image)

    assert retry.status_code == 200
    assert retry.json()["duplicate"] is True
    assert retry.json()["storage_key"] == first["storage_key"]
    assert len(list((storage / f"tours/{tour['id']}/captures").iterdir())) == 1

    async with db() as session:
        rows = (
            await session.execute(
                text(
                    "select target_index, byte_size, captured_at, pose "
                    "from virtual_tour_captures"
                )
            )
        ).fetchall()
    assert len(rows) == 1
    assert rows[0][0] == 1 and rows[0][1] == len(image)
    assert json.loads(rows[0][3]) == {"position_x": 1.5, "quaternion_w": 1.0}

    status = await client.get(f"{API}/{tour['id']}")
    assert status.json()["uploaded_capture_count"] == 1


async def test_different_bytes_for_a_filled_position_are_refused(client):
    tour = await create_tour(client)
    await upload(client, str(tour["id"]), 1, image=jpeg(colour="black"))

    conflict = await upload(client, str(tour["id"]), 1, image=jpeg(colour="white"))
    assert conflict.status_code == 409
    assert "target_index" in conflict.json()["error"]["message"]


async def test_a_target_outside_the_plan_is_refused(client):
    tour = await create_tour(client, expected=2)
    response = await upload(client, str(tour["id"]), 3)
    assert response.status_code == 409
    assert "capture plan" in response.json()["error"]["message"]


async def test_a_target_below_one_is_refused_by_the_form(client):
    tour = await create_tour(client, expected=2)
    assert (await upload(client, str(tour["id"]), 0)).status_code == 422


@pytest.mark.parametrize(
    ("image", "content_type", "reason"),
    [
        (jpeg(320, 240), "image/jpeg", "resolution is too small"),
        (b"not a jpeg at all", "image/jpeg", "not a valid image"),
        (b"", "image/jpeg", "empty or exceeds"),
        (jpeg(), "image/heic", "only image/jpeg"),
    ],
    ids=["320x240", "not-an-image", "zero-bytes", "image/heic"],
)
async def test_a_frame_that_can_never_be_stitched_says_why(
    client, image, content_type, reason
):
    """409, not 422: the request was well formed and only the frame is wrong.

    The app maps a 409 to a non-retryable failure, which is right here —
    re-sending these bytes returns the same answer, so recapturing is the only
    way forward.
    """
    tour = await create_tour(client)
    response = await upload(
        client, str(tour["id"]), 1, image=image, content_type=content_type
    )
    assert response.status_code == 409
    assert reason in response.json()["error"]["message"]


async def test_malformed_pose_json_is_a_validation_error(client):
    tour = await create_tour(client)
    response = await client.post(
        f"{API}/{tour['id']}/captures",
        data={
            "target_index": "1",
            "captured_at": CAPTURED_AT,
            "pose_json": "{not json",
        },
        files={"image": ("t001.jpg", jpeg(), "image/jpeg")},
    )
    assert response.status_code == 422
    assert "pose_json" in response.text


async def test_the_pose_reaches_the_manifest_unchanged(client, storage):
    tour = await create_tour(client, expected=1)
    pose = {"position_x": 1.5, "quaternion_w": 1.0, "alignment_error_deg": 3.5}
    await upload(client, str(tour["id"]), 1, pose=pose)
    finalised = await client.post(f"{API}/{tour['id']}/finalise")
    assert finalised.status_code == 200, finalised.text

    manifest = json.loads(
        (storage / finalised.json()["stitch_manifest_key"]).read_text()
    )
    assert manifest["captures"][0]["pose"] == pose


# ---------------------------------------------------------------- ownership


async def test_a_tour_of_another_owner_does_not_exist(client):
    tour = await create_tour(client)
    await upload(client, str(tour["id"]), 1)
    as_intruder = {"X-User-Id": INTRUDER}

    read = await client.get(f"{API}/{tour['id']}", headers=as_intruder)
    assert read.status_code == 404
    assert read.json()["error"]["code"] == "not_found"

    finalise = await client.post(f"{API}/{tour['id']}/finalise", headers=as_intruder)
    assert finalise.status_code == 404

    late = await upload(client, str(tour["id"]), 2, headers=as_intruder)
    assert late.status_code == 404


async def test_an_unknown_tour_is_404_rather_than_a_new_session(client):
    response = await client.get(f"{API}/{uuid.uuid4()}")
    assert response.status_code == 404


# ----------------------------------------------------------------- finalise


async def test_finalise_writes_the_manifest_the_stitcher_will_read(client, storage):
    tour = await create_tour(client, expected=2)
    # Deliberately out of order: the manifest is the stitcher's recipe and has
    # to read 1, 2 whatever order the frames arrived in.
    await upload(client, str(tour["id"]), 2, image=jpeg(colour="#000002"))
    await upload(client, str(tour["id"]), 1, image=jpeg(colour="#000001"))

    response = await client.post(f"{API}/{tour['id']}/finalise")
    assert response.status_code == 200, response.text
    ticket = response.json()

    # Stops at stitch_pending: there is no panorama until a worker runs, and
    # the API does not pretend there is one.
    assert ticket["status"] == "stitch_pending"
    assert ticket["uploaded_capture_count"] == 2

    key = ticket["stitch_manifest_key"]
    assert key is not None
    body = (storage / key).read_bytes()
    assert key == (
        f"tours/{tour['id']}/stitch-manifest-"
        f"{hashlib.sha256(body).hexdigest()[:16]}.json"
    )

    manifest = json.loads(body)
    assert manifest["schema_version"] == 1
    assert manifest["tour_id"] == tour["id"]
    assert manifest["property_id"] == PROPERTY
    assert datetime.fromisoformat(manifest["created_at"]).tzinfo is not None
    assert [c["target_index"] for c in manifest["captures"]] == [1, 2]
    for capture in manifest["captures"]:
        stored = storage / capture["storage_key"]
        assert hashlib.sha256(stored.read_bytes()).hexdigest() == capture["sha256"]


async def test_finalise_refuses_a_station_that_is_not_complete(client):
    tour = await create_tour(client, expected=3)
    await upload(client, str(tour["id"]), 1)

    response = await client.post(f"{API}/{tour['id']}/finalise")
    assert response.status_code == 409
    assert "all 3 captures are required" in response.json()["error"]["message"]


async def test_a_finalised_tour_stops_accepting_images_and_rejects_replay(client):
    tour = await create_tour(client, expected=1)
    await upload(client, str(tour["id"]), 1)
    await client.post(f"{API}/{tour['id']}/finalise")

    late = await upload(client, str(tour["id"]), 1)
    assert late.status_code == 409
    assert "stitch_pending" in late.json()["error"]["message"]

    again = await client.post(f"{API}/{tour['id']}/finalise")
    assert again.status_code == 409
    assert "already stitch_pending" in again.json()["error"]["message"]


async def test_status_reports_the_manifest_after_finalise(client):
    tour = await create_tour(client, expected=1)
    await upload(client, str(tour["id"]), 1)
    finalised = (await client.post(f"{API}/{tour['id']}/finalise")).json()

    status = await client.get(f"{API}/{tour['id']}")
    assert status.status_code == 200
    body = status.json()
    assert body["status"] == "stitch_pending"
    assert body["stitch_manifest_key"] == finalised["stitch_manifest_key"]
    # The server's id is the app's resume key, so reading must not rename it.
    assert body["id"] == tour["id"]


async def test_the_lifecycle_leaves_one_tour_and_the_frames_it_was_told_to_expect(
    client, storage, db
):
    tour = await create_tour(client, expected=3)
    for index in (1, 2, 3):
        response = await upload(
            client, str(tour["id"]), index, image=jpeg(colour=f"#{index:06x}")
        )
        assert response.status_code == 200, response.text
    assert (await client.post(f"{API}/{tour['id']}/finalise")).status_code == 200

    async with db() as session:
        tours = (
            await session.execute(text("select count(*) from virtual_tours"))
        ).scalar_one()
        captures = (
            await session.execute(text("select count(*) from virtual_tour_captures"))
        ).scalar_one()
    assert (tours, captures) == (1, 3)
    assert len(list((storage / f"tours/{tour['id']}/captures").iterdir())) == 3
