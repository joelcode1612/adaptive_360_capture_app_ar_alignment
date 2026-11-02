# Phase 10 — ARCore Capture Alignment Deliverables

Branch: `feature/arcore-capture-alignment`

This document reports what was actually found, changed, and verified against the
consolidated 10-phase AR-alignment task. Every claim below is backed by a
file/line citation or an explicit test result. Nothing that was not run on a
real device is claimed to work on a real device.

---

## 1. Root-cause analysis of the "markers appear far away" problem

### 1.1 No transform error exists at source level

The task's premise was that target positions are computed with a transform
error (the spec's own fix proposed `z = −radius·cos(yaw)·cos(pitch)`). Walking
the full pipeline shows the math is internally consistent and no sign/scale
defect exists in this repository:

| Step | Location | What it does |
| --- | --- | --- |
| Station reference from camera pose | `lib/features/capture/domain/services/station_frame.dart:241–302` (`captureReferenceFromPose`) | Builds the station frame from ARCore's pose + gravity. Refuses within 10° of vertical (`kMaxCalibrationVerticalityDeg = 10`, line 191). |
| Direction vector | `lib/features/capture/domain/services/station_frame.dart:123–124` | Station-local +Z forward; `localToWorld` applies the documented `−x` mirror (left-handed camera convention, line 12). |
| Resolve target | `lib/features/capture/domain/services/target_sphere.dart:442–456` (`resolveTarget`) | `worldPosition = station.origin + radius × localToWorld(direction)`. |
| Radius | `lib/core/math/vio_sphere_math.dart:54` (`targetRadiusM = 1.0`) | Single source of truth; used at every call site (`target_sphere.dart:398`, `plane_anchor_service.dart:215`, `capture_controller.dart:209`). No per-callsite scaling. |
| Model matrix (Dart) | `lib/core/math/vio_sphere_math.dart:901–952` (`buildModelMatrix`) | Column-major `[X|Y|Z|worldPosition]`, +Z back at the station. Identity fallback for non-finite input. |
| Model matrix (Kotlin render) | `android/.../ar/ArGlRenderer.kt:259–281` | Uses only the **rotation** of `modelMatrix` and zeroes its translation when the anchor carries the world translation — anchor and basis can never double-translate. |
| Placement under station anchor | `lib/features/capture/data/services/plane_anchor_service.dart:241–247` | `modelMatrix[12..14] = worldPosition − station.origin`, i.e. **station-local** coordinates composed with the station anchor. |

The spec's Phase 6 formula differs only in **sign convention** from the repo's
(`z = +cos(yaw)·cos(pitch)` with station-local +Z forward plus the documented
`−x` mirror). Under the repo's convention the two are equivalent. This is not a
defect, and "fixing" the sign would introduce one.

### 1.2 The screenshot that started this task is not from this codebase

The far-away-marker screen attached to the original task was compared against
**every file and every commit** in this repository. The UI it shows does not
exist here, on any branch or in any historical commit. The phone in question is
running a **diverging build** (different app or different version). This is
restated in the Phase 1 assessment (`docs/PHASE1_ARCORE_CAPTURE_ALIGNMENT_ASSESSMENT.md`).

### 1.3 The repo has no marker-image detection at all

The spec's calibration and "marker disappearance" sections assume ARKit and
Augmented-Image (photo-marker) detection. This repository:

- contains **no ARKit / iOS code** (Android-only, ARCore),
- contains **no ARCore Augmented Image** detection.

Calibration here is **camera-pose + gravity based**, not image based. The spec's
ARKit and marker-image steps map to nothing present and were not implemented
(there is nothing to implement them against). See §3.1.

---

## 2. What was already implemented (spec Phases 2–5) — verified, not rewritten

Investigation established that most of the spec's core behaviour was **already
present and correct**. No code was rewritten for these; they were verified with
file/line citations and are reported as such.

### 2.1 One-time calibration / one station anchor (spec Phase 2)

- Exactly **one** station anchor: `PlaneAnchorService._stationAnchor`
  (`plane_anchor_service.dart:111`), created once in `placeAll()`
  (`:172–257`).
- All 26 targets are derived as **frozen local transforms below that one
  anchor** (`:241–247`) — the anchor is never re-created per target.
- Dart-side one-time guard added in this branch: `calibrateStation()` returns
  `true` with "Centre already calibrated." when `state.station != null`
  (`capture_controller.dart:1341`), committed in `1bd6b68`. Explicit reset paths
  remain `recalibrateHere()` (`:1661`) and `recalibrateAndContinue()`.

### 2.2 Persistent anchor + target count (spec Phase 3 / "26 targets")

- **26 nodes.** Resolved: the user confirmed the 26-target layout is the
  intended one, and this branch now implements it. The 30-node layout that
  briefly existed on this branch (copy corrected in `7ec68ef`) was a prior,
  deliberate repository improvement; it was reverted to 26 on user instruction
  in the 26-target refactor commit. Layout: 8 horizontal (0°, every 45°), 8
  upper (+45°), 8 lower (−45°), zenith (+90°), nadir (−90°) —
  `target_sphere.dart` / `VioSphereConstants`.
- **Pole tradeoff (documented, accepted):** the zenith and nadir nodes sit at
  exact ±90° elevation where yaw is undefined; the spec assigns them yaw 0
  with pitch ±90 and the capture UI handles them with vertical-only guidance.
- **Device divergence note:** the phone's installed build does not match this
  branch (it shows a far-away-marker layout from another build); its visuals
  cannot be used to judge this branch and are NOT TESTED here.

### 2.3 Tracking state machine (spec Phase 4)

Native side, `ArCoreSessionController.kt`:
- Reads `camera.trackingState` (`:841`, `:886`, `:972`, `:1080`).
- `emitPose()` (`:1079–1163`) emits `tracking`, `failureReason`,
  `trackingEpoch`, `trackingTransition`, `trackingContinuityBroken`,
  `trackingContinuityReason`, active-anchor pose/tracking, and plane counts.
- Latched continuity-break epoch machine (`ARCORE_RECOVERY` log, `:1092–1117`)
  treats several bad frames in one PAUSED period as **one** broken sequence.

Dart side, `capture_state.dart`:
- `tracking` + `trackingRecovering` (`:181–192`).
- Captures are gated on both (`:514–565`).

### 2.4 Automatic recovery, single-anchor reuse (spec Phase 5)

- `PlaneAnchorService.refresh()` (`plane_anchor_service.dart:273–294`)
  distinguishes `gone` vs `stale` vs paused — a marker disappearing is **not**
  a calibration reset.
- `CaptureController._refreshAnchorIfDue()` (`capture_controller.dart:3930`)
  auto-recovers `gone` anchors, bounded by `kAnchorReplacementsMax`, and
  invalidates the station **only** if the shared anchor is gone (`:3969`).
  A tracking interruption therefore reuses the existing anchor when it is still
  valid and never silently spawns another station anchor.

---

## 3. What was actually changed in this branch

Seven commits, one phase each, working tree clean.

| Commit | Phase | Change |
| --- | --- | --- |
| `0d55f1f` | 1 | Assessment doc (`docs/PHASE1_..._ASSESSMENT.md`) incl. test-classification addendum. |
| `7ec68ef` | 2 | Corrected stale **26-target** copy to the **30-node** reality. |
| `748732c` | 2b | `test/capture_test_fakes.dart`: fake `takeStill` timestamp + record-shape fixes so fakes match the real contract (classification, not behaviour, change). |
| `17efbc8` | 3 | Restored the station-drift gate: flipped both bypass flags to their safe values (`capture_state.dart:30` `kStationDriftGateBypassed = false`, `:20` `kStationDriftWarningVisualsEnabled = true`). |
| `a803de9` | 4 | Threshold **recommendation** comments beside the active constants (0.10 m / 0.05 m / 5° / 500 ms). Active values unchanged. |
| `1bd6b68` | 7 | One-time calibration guard (`capture_controller.dart:1341`) + **removed "Scan Again"** from the post-calibration panel (`ar_capture_screen.dart:758–793`); kept "Recalibrate here" (only when `stationDriftWarningVisible`) and "Review targets". Updated `_trackingAdvice` (`:985–1002`). |
| `7ccdcb7` | 7b | Dead-message/doc-comment cleanup (9 files, +33/−23): every user-facing string naming the removed button re-pointed to a real action. |
| `a8f24e7` | 10 | This report. |
| `b1aea45` | 11 | Restored the **user-confirmed 26-target** sphere layout (20 files, +262/−254): lib constants/nodes, controller copy, UI labels, and the six layout-coupled test files. 30→26 only; no behaviour change (full suite delta-free vs the 30-node baseline). |

### 3.1 Deliberately NOT changed

- `arcore_ar.dart:549/:552` still say "scan again" — this is the **virtual-tour
  wizard**, out of scope for the capture module.
- Comments that *document* the removal (`capture_state.dart:699`,
  `capture_controller.dart:2025`, `ar_capture_screen.dart:770/:786`) were left
  intact — they are accurate historical notes.
- `rescanActive()` (`capture_controller.dart:2038`) is **UI-dead** (only tests
  call it) but retained; `_recoverTarget()` (`:1968`) is **dead code** and was
  left in place (removal deferred — see §6).

---

## 4. Scan Again — removal confirmed

- **UI:** the "Scan Again" button no longer exists in the post-calibration
  expanded panel (`ar_capture_screen.dart:758–793`). The only re-entry points
  are the explicit `Recalibrate here` / `Recalibrate and continue` paths.
- **Logic:** `calibrateStation()` is one-shot (`capture_controller.dart:1341`).
- **Messages:** no live user-facing string names the removed button
  (`7ccdcb7`). The remaining "scan all directions" text on the target sheet
  (`ar_capture_screen.dart:585–591`) refers to the **live look-around probe**,
  which is a real, functioning control — not the removed button.
- **Residual:** `rescanActive()` and its doc comment remain but have **no UI
  caller**; they are exercised only by tests.

---

## 5. Test results (actual, this branch)

### 5.1 `flutter analyze`

**PASS (with 4 pre-existing warnings, unchanged from baseline):**

- `capture_controller.dart:986` unused `_anchorReplacements`
- `capture_controller.dart:1968` unused `_recoverTarget`
- `capture_controller.dart:2924` unnecessary `!`
- `capture_controller.dart:3011` prefer_null_aware

No new issues introduced by any commit in this branch.

### 5.2 Targeted three-file run (`capture_flow_test.dart`, `capture_progress_test.dart`, `target_guidance_test.dart`)

**49 passed / 28 failed** — identical to the pre-change baseline; delta zero.

### 5.3 Full suite (`flutter test`)

**424 passed / 64 failed** — the failure **count and identity** are identical to
the baseline. **Zero new failures** were introduced by any commit in this branch.

All 64 failures fall inside previously classified stale-or-unimplemented
clusters (see `docs/PHASE1_..._ASSESSMENT.md` §10). Per cluster:

| Cluster (files) | Count | Classification |
| --- | --- | --- |
| `alignment_test.dart` — "aimed phone told to hold still" | 1 | Stale stability-window expectation |
| `capture_flow_test.dart` — manual-shutter / drift / placement | 13 | Stale (manual-shutter cluster) |
| `capture_progress_test.dart` — resume / recalibrate / one-station | 9 | Stale (drift-bypass flipped green in Phase 3) |
| `target_guidance_test.dart` — saved-target / retake | 7 | Stale |
| `target_scale_test.dart` — readiness message | 4 | Stale (message-wording) |
| `target_transition_test.dart` — 26-anchor / placement / persistence | 15 | Stale vs the **one-anchor** contract |
| `vio_sphere_math_test.dart` — 600 ms window / shutter block | 3 | Stale (Vio E-window) |
| `world_target_test.dart` — paused-anchor aim, drift, auto-shutter | 12 | **Mixed:** paused-anchor aim (`:456/:501`) is a **genuine unimplemented frozen-placement contract** and is RED **by design**; the rest are stale drift-gate expectations |

> The frozen-placement aim assertions in `world_target_test.dart` were kept
> failing on purpose: an attempted implementation broke tracked-aim behaviour
> and was fully reverted (net diff zero). They turn green only when a later
> phase implements the frozen-placement contract properly.

### 5.4 NOT TESTED (honest)

- **Real-device ARCore behaviour** — no device available in this environment.
  Everything about actual tracking, the far-away-marker symptom, and recovery
  on hardware is **NOT TESTED**.
- **Android APK build** — does not complete within the environment timeout;
  **NOT TESTED** end-to-end.
- The spec's **ARKit / Augmented-Image** steps — no code exists to test
  (§1.3).

---

## 6. Remaining limitations / open items

1. **26 vs 30 targets — RESOLVED.** The user confirmed 26; this branch now
   implements the 26-target layout (8 horizontal, 8 upper, 8 lower, zenith,
   nadir) throughout lib + tests. The 30-node layout was a prior deliberate
   improvement, reverted on user instruction. Pole tradeoff (±90° yaw
   undefined → vertical-only guidance) documented in §2.2.
2. **Far-away marker screenshot.** Not present in this codebase on any
   commit — the phone runs a diverging build. Needs the user to identify the
   app/version to proceed.
3. **Frozen-placement contract.** `world_target_test.dart` paused-anchor aim
   tests remain RED **by design** until a dedicated phase implements the
   contract without breaking tracked-aim behaviour.
4. **Dead code.** `_recoverTarget()` (`capture_controller.dart:1968`) and the
   UI-dead `rescanActive()` (`:2038`) are candidates for removal in a later
   phase.
5. **Thresholds.** Active drift/pose values are unchanged (0.10 m / 0.05 m /
   5° / 500 ms recommendations are comments only, `a803de9`). Tuning them
   requires on-device measurement.
6. **Manual-shutter cluster.** ~30 stale tests still reference the old
   shutter semantics; re-pointing them is deferred to its own phase so it can
   be reviewed independently.

---

## 7. Acceptance-criteria walkthrough (spec → actual)

| Spec criterion | Status | Evidence |
| --- | --- | --- |
| Marker-based calibration (ARKit / Augmented Image) | **N/A** — no such code exists | §1.3 |
| Calibration runs once per session | **Implemented** | `capture_controller.dart:1341` |
| Exactly ONE station anchor | **Implemented** | `plane_anchor_service.dart:111`, `:172–257` |
| Marker is init-only; disappearance ≠ reset | **Implemented** | `plane_anchor_service.dart:273–294` |
| Tracking interruption ≠ new anchor | **Implemented** | `capture_controller.dart:3930`, `:3969` |
| Recovery reuses valid anchor | **Implemented** | `plane_anchor_service.dart:259–294` |
| Anchor does not follow the camera | **Implemented** | `plane_anchor_service.dart:241–247` (station-local, frozen) |
| Camera not frozen | **Implemented** | `ArGlRenderer.kt:259–281` (rotation-only use of basis) |
| No distance defect via scaling / random offsets | **Confirmed absent** | §1.1 |
| 26 capture targets | **Implemented (26 confirmed by user)** | `b1aea45`; `target_sphere.dart` (`VioSphereConstants`) |
| "Scan Again" removed | **Implemented** | `ar_capture_screen.dart:758–793` |
| Thresholds configurable, active values unchanged | **Implemented** | `a803de9` (comments only) |
| Duplicate captures prevented | **Implemented** | `capture_state.dart:514–565` |
| Data survives tracking interruption | **Implemented** | §2.4 |
| Honest limits; no device-success claims | **This document** | §5.4 |

---

## 8. Root-cause analysis: transform discontinuity, target displacement, false drift, marker suppression

This section is the post-implementation root-cause analysis required by the
brief. It covers the four symptoms observed together on device (Xiaomi
2201117TG, Android 13): an anchor transform discontinuity of 1.298 m /
44.2°, a frozen-vs-guidance target delta of 1.797 m, an orange "Too far from
station" warning that appeared without physical movement, and markers
disappearing from the scene. All four trace back to the same design flaw:
**the code measures continuity and drift in the ARCore world frame, but a
shared world correction moves the camera and every anchor together, so every
world-frame measurement reports a change that did not physically happen.**

### 8.1 Native camera and anchor poses are same-frame — timestamp mismatch is ruled out

`ArCoreSessionController.emitPose` (`ArCoreSessionController.kt:1079-1214`)
samples the camera pose and the anchor pose inside one `frame.update` pass and
emits them in one event. The Dart side (`capture_controller.dart:_onPose`,
`:2638`) therefore already receives a coherent snapshot: camera pose and
anchor pose describe the same ARCore frame. There is no cross-frame mixing,
no epoch skew inside a sample, and no separate timestamp channels to
reconcile. This rules out the simplest explanation for the discontinuities
(mixing an old anchor pose with a new camera pose) and forces the analysis
onto *what frame the measurements are expressed in*.

### 8.2 The 1.797 m "target move": world-frame `_hasAnchorRefined`

`capture_controller.dart:2947-2951` recomputes the live target position every
pose as:

```
targetPosition = matchingLive.position + matchingLive.orientation.rotate(placement.stationLocalPosition)
```

Both `matchingLive.position` and `matchingLive.orientation` are **world-frame**
values from the live station anchor. `_hasAnchorRefined` (`:3679-3702`) then
compares this world-frame `targetPosition` against the previous reference with
a `maxPositionDriftM = 0.03` m allowance (`vio_sphere_math.dart:92`). When
ARCore applies a shared world correction (the camera pose and the anchor pose
are both re-anchored in a new world frame), `targetPosition` changes by the
correction magnitude even though the *physical* relationship between camera,
station and target is untouched. The device log recorded exactly this:
frozen node `(1.546, 0.660, -1.900)` vs guidance `(-0.028, -0.189, -1.729)`,
delta **1.7967 m** — a number that is 60× the 0.030 m allowance and therefore
always trips `_hasAnchorRefined`, which logs "station anchor refinement moved
the target" and restarts the stability hold. The hold restart is correct as a
*conservative* reaction to a rendered-target jump, but the *diagnosis* is
wrong: the target did not move relative to the station; the world frame moved
under both.

### 8.3 The 1.298 m / 44.2° discontinuity: world-frame `_stationAnchorTransformDiscontinuity`

`_stationAnchorTransformDiscontinuity` (`capture_controller.dart:3638-3658`)
stores `_lastTrustedStationAnchor` as a **world-frame** pose at the moment it
was last trusted, then compares each new live anchor pose against it:

```
positionM = (anchor.position - previous.position).length
rotationDeg = quaternionAngleDeg(anchor.orientation, previous.orientation)
```

A shared world correction moves the anchor's world pose by the correction
magnitude. The device log recorded 1.298 m / 44.2° — above the
`kRecoveryAnchorPositionDiscontinuityM` / `kRecoveryAnchorOrientationDiscontinuityDeg`
recovery thresholds — so recovery verification is begun (`:2920-2941`) for a
correction that is entirely benign for the station-relative geometry. The
thresholds themselves are not the defect (they were tuned for genuine anchor
loss); the *frame of reference* is. The anchor's station-local meaning
(`Quat.identity` at `station.origin`, `plane_anchor_service.dart:183-187`)
does not change across a shared correction, so a station-local comparison
would correctly report ~0 change.

### 8.4 A design constraint: with one shared anchor, a naive station-local refinement check is a no-op

All 26 targets share the single station anchor (`plane_anchor_service.dart:174-257`),
and each target's `stationLocalPosition` is frozen at placement
(`:241-247`). Consequently, for the live anchor pose `A`:

```
stationLocalTarget = A.orientation⁻¹ × (targetPosition − A.origin) ≡ placement.stationLocalPosition
```

exactly, for every pose. A `_hasAnchorRefined` rewritten naively to compare
station-local target positions would therefore never fire — it would be
algebraically identical to comparing the stored placement to itself. The
correction-invariant discriminator must instead compare **target motion
against camera motion** (i.e. the camera-to-target vector in station-local
coordinates, or an explicit classification of world-correction events), so
that a shared correction (camera and target move together → camera-to-target
unchanged) is distinguished from an anchor-only jump (target moves, camera
does not → camera-to-target changes).

### 8.5 Marker suppression: `_sceneTracked` hides markers whenever `trackingRecovering`

`_sceneTracked` (`capture_controller.dart:4405-4408`) is:

```
s.activeAnchorTracking && s.tracking == TrackingState.tracking && !s.trackingRecovering
```

Because §8.3 enters `trackingRecovering` on a false discontinuity, a benign
shared correction hides **all** markers even though the anchor is still valid
and tracked, and the station-local geometry is intact. The brief's Priority
Three requirement — keep rendering when the anchor is valid/usable and only
mark the reference as "temporarily unverified" — is violated by the single
`!trackingRecovering` term doing double duty for both rendering and capture
gating.

### 8.6 False orange "Too far from station": stale world-frame drift and relocalization painting

The orange warning is drawn from `CaptureState.stationDriftWarningVisible`
(`capture_state.dart:406`) fed by `driftM`, which is normally the
correction-invariant gate measurement (`gate.stationDriftM`, computed from
`frame.origin + stationLocalCamera` with rotation
`anchorInverse × rawVioSample.rotation` — see `gatePose`,
`capture_controller.dart:3051-3079`). However two paths bypass that
correction-invariant measurement:

1. **The stale fallback.** Where no live station pose is available, the code
   falls back to the previous `state.driftM`
   (`capture_controller.dart:3179`, `:3203` —
   `liveStationPose == null ? state.driftM : gate.stationDriftM`). A value
   measured in the *old* world frame, compared against the *new* station
   position, is not a physical displacement.
2. **The relocalization UI path.** During recovery/relocalization the screen
   still evaluates the orange drift visuals from the state that was published
   before the correction was understood, painting orange for what is in fact
   world refinement in progress.

Device evidence: drift read 0.201 → 0.208 m (crossing the 0.20 m limit,
`vio_sphere_math.dart:61`) without the operator moving, then settled to
0.118 m after relocalization — the classic signature of a world-frame
measurement, not physical displacement. Priority Two/Three require the orange
"Too far from station" to fire only on genuine physical camera-to-station
displacement, with a neutral "Verifying spatial reference" state during world
refinement.

### 8.7 Summary of root causes and the Priority One fix

| # | Root cause | Symptom | Fix direction (per brief) |
| --- | --- | --- | --- |
| 1 | `_hasAnchorRefined` compares world-frame `targetPosition` across a shared correction with 0.030 m allowance | 1.797 m "target move", false stability-hold restart | Compare camera-to-target (station-local) motion; classify world corrections separately into `ARCORE_WORLD_REFINEMENT` |
| 2 | `_stationAnchorTransformDiscontinuity` compares raw world anchor pose to stored world reference | 1.298 m / 44.2° false recovery entry | Measure continuity station-locally (or via camera-to-target), not in world frame |
| 3 | `_sceneTracked` includes `!trackingRecovering` | Markers hidden during benign recovery | Render when anchor valid/usable; separate "unverified" from "unusable" |
| 4 | Stale `state.driftM` fallback + orange painting during relocalization | False orange "Too far from station" | Neutral "Verifying spatial reference" during world refinement; orange only for genuine physical displacement |
| 5 | One shared anchor at `Quat.identity` | Station-local target ≡ stored placement (no-op check) | Camera-to-target discriminator as in §8.4 |

Priority One (this phase's implementation) addresses items 1 and 2 at their
source: a single coherent frame snapshot in `_onPose`, with
`T_camera_to_station = inverse(T_world_camera) × T_world_station` computed
once per frame and used as the authoritative transform for drift and
guidance, so that every continuity and drift measurement is expressed in the
station frame and is invariant to shared world corrections. Items 3 and 4
(Priorities Three and Four) are addressed in their own follow-up commits, as
are the calibration-instruction and UI-state-machine items (Priorities Five
and Six) and the anchor-lifecycle audit (Priority Seven).

---

## 9. Priority One implementation — world-frame consistency (§8.7 items 1, 2, 5)

### 9.1 Modified files

| File | Change |
| --- | --- |
| `lib/features/capture/application/capture_controller.dart` | Only file changed. `_onPose` snapshot reorder; station-local-camera witness in `_stationAnchorTransformDiscontinuity` and `_hasAnchorRefined`; new field `_anchorStabilityStationLocalCamera` (+ cleared in `_clearStation`); `_tracePoseSample` delta corrections. |

No thresholds were changed. No other module was touched. `flutter analyze`
on the file reports exactly the same 4 pre-existing warnings as the baseline
(`unused_field _anchorReplacements`, `unused_element _recoverTarget`,
`prefer_null_aware_operators`, `unnecessary_non_null_assertion`) — no new
issues.

### 9.2 The coherent frame snapshot

`liveStationPose`, `stationLocalCamera` and `poseFresh` are now computed
once, **before** the anchor classifiers, instead of after them:

```dart
final liveStationPose = matchingLive != null &&
        matchingLive.tracking == TrackingState.tracking
    ? matchingLive
    : null;
final stationLocalCamera = liveStationPose == null
    ? null
    : liveStationPose.orientation.conjugate.rotate(
        pose.position - liveStationPose.position,
      );
```

i.e. `T_camera_to_station = inverse(T_world_station) × T_world_camera` from a
single AR-frame snapshot. The discontinuity check (`:2966`), the hold
refinement check (`:3007`), the station jump check and the gate all read the
same values; the duplicate definitions that previously ran *after* those
checks were deleted (grep confirms single definitions at `:2943`, `:2947`,
`:2952`).

### 9.3 Before/after math — the 1.797 m "target move" (§8.2)

`stationLocalCamera` is invariant under any shared rigid correction `C`:

```
C(x) = live.position + live.orientation.rotate(x − frame.origin)
```

Camera and anchor both map through `C`, so
`C(cam) − C(anchor)` rotated by `C(anchor)⁻¹` equals the original
station-local vector. Therefore:

| Measurement | Before (world frame) | After (station-local witness) |
| --- | --- | --- |
| `_hasAnchorRefined` on a shared correction | `movedM = 1.797 m > 0.030 m` → **hold restarted** | world `movedM = 1.797 m` but `cameraMovedM ≈ 0 m ≤ 0.030 m` → re-base, `ARCORE_WORLD_REFINEMENT`, **hold continues** |
| Device target-1 evidence | frozen `(1.546, 0.660, −1.900)` vs guidance `(−0.028, −0.189, −1.729)` = **1.7967 m** printed as a target move | frozen node mapped through the live anchor first (`frozenToWorld=`) → delta ≈ 0 (correction-invariant) |

The witness stored at baseline (`_anchorStabilityStationLocalCamera`) is
never refreshed on ordinary frames, so a *slow genuine* drift still exceeds
the allowance and restarts the hold exactly as before. Null witness (no live
station pose) → today's conservative behavior unchanged.

### 9.4 Before/after math — the 1.298 m / 44.2° false recovery (§8.3)

| | Before | After |
| --- | --- | --- |
| World anchor delta vs `_lastTrustedStationAnchor` | `1.298 m / 44.2° > 0.08 m / 5.0°` → **recovery started** (guidance suppressed) | same world delta, but station-local camera jump `≤ 0.08 m` → `ARCORE_WORLD_REFINEMENT` logged, returns `null` → **no recovery** |
| Station-local camera jump > threshold (anchor-only / camera-only) | n/a (not measured) | still returns the discontinuity → `ARCORE_RECOVERY` (now with the camera-jump value in the message) |

The existing thresholds (`kRecoveryAnchorPositionDiscontinuityM` 0.08 m,
`kRecoveryAnchorOrientationDiscontinuityDeg` 5.0°) are reused unchanged; the
classification only decides *which side of the threshold the station-local
camera* falls on. `_recordTrustedStationReference` refreshes the
station-local witness every healthy frame, so it is a true per-frame delta.

### 9.5 `_tracePoseSample` log corrections

- `anchorDeltaM` now measures the anchor against `_frame.origin` (its own
  creation pose) instead of `placement.worldPosition`, which printed the
  sphere radius (a constant ~1.7 m) as an "anchor displacement".
- `frozenToGuidanceDeltaM` now maps the frozen calibration-frame node through
  the live station anchor into the current world before subtracting —
  correction-invariant. The old subtraction printed the world correction
  itself as a target move (the 1.797 m device artefact).
- New `frozenToWorld=` field shows the mapped frozen node.

### 9.6 Test results (this phase)

| Test | Result |
| --- | --- |
| `flutter analyze capture_controller.dart` | Same 4 pre-existing warnings as baseline — **PASS (no new issues)** |
| `test/capture_flow_test.dart` | 31 passed / 13 failed — failure set **identical to baseline** — **PASS (no regression)** |
| `test/world_target_test.dart` | 9 passed / 12 failed — failure set **identical to baseline** — **PASS (no regression)** |
| Full suite (`flutter test`) | **424 passed / 64 failed** — failure set byte-identical to the 26-target baseline (`full_suite_26.log`) — **PASS (no regression)** |
| Unit tests for the new witness logic | **NOT TESTED** (to be added in the dedicated test phase) |
| Real-device ARCore verification (TEST A–J) | **NOT TESTED** — no device access in this environment; APK build does not complete within environment timeout |

Known baseline failure clusters unchanged: alignment 1, flow 13, progress 9,
target_guidance 7, target_scale 4, target_transition 15, vio_sphere_math 3,
world_target 12 (frozen-placement aim tests RED by design until their
dedicated phase).

---

*Generated on branch `feature/arcore-capture-alignment` after commit `a8f24e7`,
then updated for the confirmed 26-target layout (commit `b1aea45`), the
root-cause analysis (§8), and the Priority One implementation (§9).*
