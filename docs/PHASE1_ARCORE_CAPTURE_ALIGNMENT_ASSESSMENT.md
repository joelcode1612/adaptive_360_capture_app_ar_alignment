# Phase 1 — ARCore Capture Alignment: Architecture Assessment

**Branch:** `feature/arcore-capture-alignment` · **Date:** 2026-10-31 · **Status:** assessment complete, no code changed yet.

Assessment precedes any code change, as required. Every line reference below was
re-verified against the worktree immediately before this date; nothing in this
document is inferred from memory of an earlier reading.

---

## 1. Scope

The enhancement targets the **AR capture module** — `CaptureModuleScreen`
(`lib/features/capture/presentation/screens/ar_capture_screen.dart`,
entered from the launcher card 1 "AR capture" in `main.dart`) and the
`lib/features/capture/` feature tree backed by the hand-written ARCore bridge.

Out of scope (documented for contrast only): the guided
`CaptureFlowScreen` wizard (`lib/features/virtual_tour/`), its `plan26`
target map, and its provisional `CaptureFlowSettings.driftToleranceM = 0.30`.

Two user decisions anchor this work (4-question clarify, earlier in the
collaboration):

1. **Layouts stay; copy is corrected.** The AR capture stays a 30-node capture;
   the guided flow keeps `plan26`. Stale "26" copy that misdescribes the
   launcher path is fixed (see §3.5, §7 Phase 2).
2. **Both drift-gate bypass flags are re-enabled** (de-bypassed):
   `kStationDriftGateBypassed = false`, `kStationDriftWarningVisualsEnabled = true`.
   Thresholds are *documented* as recommendations; active values stay unchanged
   (see §5, §7 Phase 4).

## 2. Method

- Full read of all spec-relevant source: `capture_controller.dart` (4 411 lines),
  `capture_flow_engine.dart` (1 410), all four Kotlin bridge files
  (`ArCoreCapturePlugin.kt`, `ArCoreSessionController.kt`, `ArGlRenderer.kt`,
  `YuvJpegEncoder.kt`), `plane_anchor_service.dart`, `photo_capture_service.dart`,
  `arcore_ar.dart`, `capture_repository_impl.dart`, `capture_state.dart`,
  `target_sphere.dart` (`CaptureGeometryConfig`), `vio_sphere_math.dart`
  (`VioSphereConstants`), all domain models, `main.dart`.
- Line-level verification of every copy site, flag read and constant cited here.
- `git status` clean on the worktree before writing; this document is the first
  commit of the branch.

## 3. Architecture map

### 3.1 Screen topology

```
main.dart (launcher grid)
 ├─ card 1 "AR capture" ─────────► CaptureModuleScreen      ← the enhanced module
 ├─ CaptureFlowScreen (wizard) ► ArCaptureScreen (virtual-tour variant)
 └─ virtual tour camera screen ► VirtualTourCameraScreen
```

Despite the shared-ish naming, `CaptureModuleScreen` (file
`ar_capture_screen.dart`) and `ArCaptureScreen` (`virtual_tour/`)
`presentation/screens/ar_capture_screen.dart`) are distinct screens with distinct
engines. Card 1's launch path captures **30** photographs per station
(`VioSphereConstants.nodeCount = 30`, `vio_sphere_math.dart:132`); the launcher
subtitle currently claims 26 — fixed in Phase 2.

### 3.2 Platform bridge (no Flutter AR plugin — by decision)

Hand-written bridge in `lib/features/virtual_tour/platform/arcore_ar.dart`:

- `ArcoreAr` method channel `adaptive360/arcore` (`arcore_ar.dart:128`), used for
  request/response lifecycle and capture calls.
- Event channel `adaptive360/arcore/events` (`arcore_ar.dart:129`): coalesced
  latest-wins pose delivery, with `trackingTransition` /
  `trackingContinuityBroken` events delivered **in order ahead of** coalesced
  poses so the controller can stamp an epoch before acting on stale data.

Native side (read in full):

- `ArCoreCapturePlugin.kt` — method/event dispatch, pose coalescing, JPEG pipeline.
- `ArCoreSessionController.kt` — session lifecycle, snapshot request handling.
- `ArGlRenderer.kt` — GL background rendering + model-matrix composition
  (`useModelTranslation` flag draws targets at their frozen local translation)
  and vertex snapping for pose deltas.
- `YuvJpegEncoder.kt` — YUV→NV21→JPEG, snapshot/encode split, quarter-turn
  rotation for portrait orientation.

### 3.3 Session lifecycle semantics

- `_lastSessionReused` (controller): a **reused** ARCore session keeps the same
  ARCore world origin, so a previously calibrated station stays valid. A **new**
  session invalidates it — `_clearStation()` discards the in-memory station with
  a UI notice while **saved photographs on disk are preserved**.
- Boot dedup: a second `boot()` cancels the prior `_poseSub`/`_diagSub`;
  without this, stability-window bookkeeping would double-advance and the
  shutter could double-fire during a station.
- Pause/resume forwards Android lifecycle to the bridge; on resume the pose age
  gate (`maxPoseAge = 250 ms`) immediately blocks shuttering until fresh arrives
  — no separate resume-pooling logic exists, which is fine because stale-pose
  rejection is structural.

### 3.4 Capture-origin confirmation (fixed-station mode)

`calibrateStation` (`capture_controller.dart` ≈1333–1442):

1. **Explicit user confirmation** triggers calibration (no automatic origin).
2. Gated on `calibrationReadiness.isReady` — fresh pose **and**
   `referenceOrientation` present.
3. Freezes a `StationFrame`: gravity-aligned reference orientation, yaw taken
   from the user's facing direction at confirmation.
4. Persists station + target sheet to disk, runs `probeTargets()` (informational
   only — surface presence never gates placement), then `_anchors.placeAll`.
5. Candidates that can become new capture-origin entry points (Phase 5+):
   `recalibrateHere()`, `continueSavedStation()`, `rescanActive()`, `retake()`,
   `probeTargets()`.

One anchor hosts everything: `placeAll`
(`plane_anchor_service.dart`, comment at :240) creates **one** `_stationAnchor`
via `createAnchor(position: station.origin, orientation: identity,
trackableKey: null)`, and each `AnchoredTarget` freezes a **station-local**
model matrix (world translation replaced by `worldPosition − station.origin`).
The 30 targets deliberately do **not** get 30 independently refinable anchors —
one capture origin for all directions, exactly as specified. Because
`placeAll` is idempotent by target id, retries/recovery cannot duplicate a node.

Related entry points already exist and need no work: `continueSavedStation()`
restores the sheet and re-anchors; `rescanActive()` rebuilds the current
session; `retake(targetId)` repeats one capture; `probeTargets()` marks pairs.

### 3.5 30-node target model & the 26/30 divergence (resolved)

Authority: `target_sphere.dart` (`CaptureTargetSpec` 30 nodes) +
`VioSphereConstants`:

- ids 1–12 Horizontal (every 30°, y=0°), 13–20 Upper (+35°, every 45°),
  21–28 Lower (−35°), 29 Zenith (+85°), 30 Nadir (−85°);
- radius 1.0 m (`targetRadiusM`); station-local `+Z` forward;
- comment at `target_sphere.dart:41` already says 30 **supersedes** the earlier
  26-target layout — verified.

Copy that still describes the old world (Phase 2 edits, text-only):

| File | Line | Problem |
|---|---|---|
| `main.dart` | :11 | launcher doc comment says "the 26-point grid" (30-node reality) |
| `main.dart` | :91 | card subtitle "26 photographs per station" (launch path = 30) |
| `virtual_tour_camera_screen.dart` | :31 | says "26-target AR capture screen" |
| `virtual_tour_capture_engine.dart` | :8–11 | claims old sphere is "26 directions at 1.5 m" — wrong on count *and* radius (older sphere is 30 at 1.0 m) |
| `capture_controller.dart` | :1434 | "All 30 anchors are created here" — stale semantics: `placeAll` creates **one** station anchor with 30 placements (comment :240 in `plane_anchor_service.dart` is the true story) |

Guided-flow `plan26` copy is internally consistent and stays.

### 3.6 Quaternion alignment

- ARCore delivers raw `(x,y,z,w)` quaternions with a **−Z optical axis**;
  the capture module converts to the app's convention where the camera's aim is
  the `+Z` station-local axis (see `station_frame.dart` rationale).
- Alignment error is the angle between the camera's current optical axis and the
  axis pointing at the target's frozen world position — computed from
  quaternion rotations (`startOrientation.rotate(directionFromYawPitch(…))`
  yields a world direction; the same math the engine asserts across modes).
- Final acceptance is a strict instantaneous test:
  `VioSphereConstants.maxAlignmentErrorDeg = 3.0°`, inclusive. Hysteresis lives
  in the controller's capture-hold, never widening the final test.

### 3.7 Auto-capture verdict (`_autoCaptureVerdict`)

A 12-condition gate decides whether the shutter may fire unattended. The
important properties verified from source:

- It is a blocking minimax: **any** failing condition blocks, and the verdict
  names the blocking reason; `ShotPhase.derive` in
  `shot_phase.dart:86–94` orders `stationDrifted` **above** `stable` — a steady
  phone 5 m from the station is reported as "align", not "stable".
- Manual shutter path re-checks the same state predicates independently
  (`gate.isWithinStation`, alignment, stability), so the verdict is a UI truth
  source rather than the only authority.
- With `kStationDriftGateBypassed = true` (current), the two `station-drift`
  verdict blocks are skipped and the manual path ignores `CaptureState.stationDrifted`.
  This changes in Phase 3.

### 3.8 Tracking recovery

- `trackingEpoch` increments on `trackingTransition` /
  `trackingContinuityBroken` events; commands keyed to the epoch are dropped
  when they arrive from before the break, so stale poses cannot be mistaken for
  continuity.
- `_beginRecoveryVerification` re-checks anchors after a gross tracking event
  before resuming capture; `_refreshAnchorIfDue` re-anchors on a ~2 Hz cadence
  so refinement keeps pace with pose updates without hammering `createAnchor`.
- `_stationReferenceIntact` guards against silently continuing to capture after
  the station anchor itself expired. When a continuity break makes the frozen
  frame invalid, the flapping decision is reported and the sequence is paused
  with an explicit message; photographs already saved are retained.

### 3.9 Camera-displacement monitoring (translation only)

- The gate watches **camera translation** (station-local camera position), not
  body rotation: `stationLocalCamera.length` is compared against
  `geometry.stationDriftWarningM` (`capture_controller.dart:526–527`), whose
  default is `VioSphereConstants.maxStationDriftM = 0.20`
  (`target_sphere.dart:410`). Exactly 0.20 m is inside; 0.2000001 m is not.
- Contrast (out of scope, documented): the virtual-tour engine's
  `evaluateStationDrift` uses **horizontal** distance with a `resumeFraction`
  hysteresis; the capture module path's `CaptureState.stationDrifted`
  (`capture_state.dart:394–395`) is a full-3D inclusive hard gate. **Only the
  module path is in scope for this work.**
- Guidance/flow contrast: `CaptureFlowSettings.driftToleranceM = 0.30` is
  provisional and lives in the guided flow, which keeps its own AR session
  spanning (`beginStationSetup`/`finishStation` + pose-only fallback
  `validateStationHit` 0.4–4.0 m). Left untouched.

### 3.10 Photo persistence & pose metadata

- `photo_capture_service.dart`: save→verify→record pipeline; a shutter-pose lag
  check ensures the recorded pose corresponds to the actual exposure, not a
  newer pose that arrived during encode.
- Duplicate prevention is intrinsic: records replace by target id in
  `capture_repository_impl.dart`; `CaptureTargetStatus.capturing` is live-only
  (disk re-reads it as pending); portrait lock in `main()` keeps capture
  semantics stable.
- `CaptureRecord` already carries pose metadata (orientation + translation for
  the exposure). Phase 5 evaluates whether additional fields are worth adding
  to the on-disk schema; changes must remain backward-compatible with existing
  records.

### 3.11 UI-state integration

- `_PhaseChip` / status-text colours derive from `ShotPhase`; drift-warning
  text and chip colour are current except the two large visuals — native 3D
  marker tint (`kBezelStationDriftTint`) and amber aim reticle — which the
  now-disabled `kStationDriftWarningVisualsEnabled` suppresses.
- After Phase 3 flips it, the warning visuals return. `ar_capture_screen.dart:298`
  is the read site; the tint is drawn through the bridge's model-matrix path and
  needs no native change.

## 4. Requirements → status matrix

| Requirement | Status |
|---|---|
| Reliable ARCore world tracking (bridge without Flutter AR plugin) | ✅ exists; risk register §8 |
| User-confirmed capture origin (fixed station) | ✅ exists (`calibrateStation`) |
| One capture-origin anchor for all directions | ✅ exists (one `_stationAnchor` + frozen local transforms) |
| Quaternion-based target alignment | ✅ exists (−Z optical axis → +Z aim; 3.0° inclusive) |
| Camera-displacement monitoring (translation of **camera**) | ✅ exists in module path (full-3D 0.20 m) |
| Tracking-state recovery | ✅ exists (`trackingEpoch`, `_beginRecoveryVerification`, `_refreshAnchorIfDue`, `_stationReferenceIntact`); device behaviour unverified |
| Centralized Capture Coordinator | ✅ effectively single controller; explicit façade only if justified (Phase 5 decision) |
| UI-state integration for warnings/phases | ✅ exists; visuals currently suppressed by bypass |
| Pose metadata preserved in records | ✅ exists (`CaptureRecord`); additive extensions optional |
| No duplicate captures | ✅ intrinsic (record replace-by-id, live-only `capturing` status) |
| Preserve captured data across tracking interruptions | ✅ station clear preserves disk records; epoch does not clear saves |
| Thresholds configurable, not hardcoded | ✅ all in `VioSphereConstants`/`CaptureGeometryConfig`/`CaptureFlowSettings` |
| Honest documentation of plugin/API limits | ✅ this document; final report to repeat |
| Keep MVVM + visual identity; keep existing layouts | ✅ plan: no structural rewrites |
| Copy truthfulness 26 vs 30 | ❌ stale → Phase 2 |
| Active drift gate on (de-bypass) | ❌ currently bypassed → Phase 3 |

## 5. Threshold table (documented recommendation — values unchanged)

The brief's guidance values vs the active code values. Per user decision, the
code value stays; the recommendation is recorded here and in Phase 4 as a
documented constant adjacent to the active definition.

| Quantity | Prompt recommendation | Active code value | Where | Diff |
|---|---|---|---|---|
| Station displacement (camera distance from origin) | 0.10 m | `maxStationDriftM = 0.20 m` | `vio_sphere_math.dart:58` via `CaptureGeometryConfig.stationDriftWarningM` (`target_sphere.dart:410`) | code 2× looser |
| Within-window drift (position) | 0.05 m | `maxWindowPositionDriftM = 0.03 m` | `vio_sphere_math.dart:81` | code tighter |
| Alignment error | 5° | `maxAlignmentErrorDeg = 3.0°` | `vio_sphere_math.dart:65` | code tighter |
| Stability window | 500 ms | `stabilityWindow = 600 ms` | `vio_sphere_math.dart:70` | code stricter |

Also active and unchanged (context, not part of the 4-value mapping):
`maxPoseAge 250 ms`, `maxWindowOrientationDriftDeg 1.5°`,
`maxAngularVelocityDegPerSec 5°/s` (over `rateBaseline 100 ms`),
`maxSampleGap 150 ms`, `minCalibrationHorizontalComponent sin(10°)`.

The guided flow's `CaptureFlowSettings.driftToleranceM = 0.30` is provisional,
belongs to the out-of-scope wizard path, and is listed as a note only.

## 6. Phase 3 flip plan (bypass flags)

Definitions to change (and only these, plus copy that says "temporary"):

- `capture_state.dart:38` — `kStationDriftGateBypassed = true → false`
- `capture_state.dart:20` — `kStationDriftWarningVisualsEnabled = false → true`

What each flip re-engages (verified read sites):

**Gate flag (`kStationDriftGateBypassed`):**

- `capture_state.dart:394–395` — `stationDrifted` becomes live again:
  `driftM > geometry.stationDriftWarningM` blocks.
- `capture_state.dart:415` and :388–395 — warning-active/visible predicates
  inside `CaptureState` start producing warning state for real.
- `capture_controller.dart:526` — station-drift phase inputs in the verdict.
- `capture_controller.dart:836–843` — `_withStationDriftWarning`: under bypass
  every crossing is *reported*; after flip, crossing raises the warning state
  again.
- `capture_controller.dart:1062` — comment only; DIAG remains transition-based.
- `capture_controller.dart:3101` — `enforceStationDrift: !flag` becomes `true`.
- `capture_controller.dart:3490–3506` — manual shutter's within-station block
  and warning-active line re-arm.
- `capture_controller.dart:4327/4372` — screen warning visibility no longer
  ANDed to zero by visuals flag (paired with visuals flip below).

**Visuals flag (`kStationDriftWarningVisualsEnabled`):**

- `ar_capture_screen.dart:298` — returns the two large orange graphics:
  native marker tint + amber reticle tint. Station-drift *text*, chip colour,
  and the status line are independent of this flag and are currently active.

Semantics kept exactly as documented in the definitions: the bypass affected
the distance gate only; after flip, alignment (3°), stability window (600 ms),
pose age (250 ms), anchor verification, and station-reference recovery all
bound the shutter as before, and the raw drift keeps being measured and logged.

**Risk (must ship with the flip and in the final report):** the bypass was
put in place during an unresolved drift investigation ("revert after the drift
investigation"). Re-enabling the gate restores real-device behaviour that
**may resurface the underlying drift problem** on real devices; this work
cannot verify on-device behaviour (§9). Test coverage for the re-armed gate
paths is assessed in Phase 3 and completed in Phase 5 if thin.

## 7. Phased plan (each phase = one committable diff)

- **Phase 1 — this document.** Committed first, before any code.
- **Phase 2 — copy truth fix (text-only).** The five "26"/anchor-semantics
  sites in §3.5. Reviewer-visible diff is comments/string only; then
  `flutter analyze` + `flutter test`.
- **Phase 3 — flag flip.** The two declarations in §6, and the two "TEMPORARY
  BYPASS" doc comments trimmed to reflect that the gate is restored. Verify
  the flagged-path tests still pass; add gate-path tests if coverage is thin.
- **Phase 4 — threshold documentation.** Adjacent documented recommendation
  constants near `VioSphereConstants.maxStationDriftM`,
  `maxWindowPositionDriftM`, `maxAlignmentErrorDeg`, `stabilityWindow`
  (0.10 m / 0.05 m / 5° / 500 ms) with pointer to this table; active values
  unchanged.
- **Phase 5 + — capability gaps from this assessment.** Decisions to make:
  additive `CaptureRecord` metadata (any), explicit coordinator façade (only
  if it earns its place), and any gate-path tests Phase 3 exposed. Each its
  own commit.
- **Final phase — verification + final report.** `flutter analyze`, `flutter
  test`, Android build; report repeats the honest limitation of §9. No device
  claims are made without a real device run.

## 8. Test plan

- 32 test files exist under `test/`. Baseline: `flutter analyze` and
  `flutter test` must be clean before and after each phase.
- Phase-level checks verified against the tree: node/threshold assertions live
  in `test/virtual_tour_nodes_test.dart` and the capture-gate tests; the
  Phase 3 flip should be covered by the existing drift-gate tests — any gap is
  created as a Phase 3 finding and closed in Phase 5.
- Android build (`flutter build apk --debug` or the gradle wrapper) is the
  compile-level gate for the four Kotlin bridge files; not a behavioural check.

## 9. Honest limitations

- **No real-device ARCore verification is possible in this environment.**
  Everything above is established from source-level analysis. Claims about
  on-device tracking recovery, drift-gate feel, reticle imagery, shutter
  timing, or JPEG pipeline throughput are *not* verified here — the final
  report will say exactly this, and nothing needs a device-level statement.
- The platform bridge is stable at the reviewed ref, but ARCore API limits
  (anchor expiry on long interruptions, pose jitter under sudden warps,
  renderer composition on lower-end devices) remain runtime unknowns that only
  a device run can settle.
- The guided flow's provisional `driftToleranceM = 0.30` is a note; the
  mismatch with the module's 0.20 m is a product decision that was left out of
  scope on purpose.

— Phase 1, end of assessment.
