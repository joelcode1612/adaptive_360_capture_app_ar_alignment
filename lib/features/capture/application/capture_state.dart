import 'dart:typed_data';

import 'package:adaptive_360_capture_app/features/capture/data/services/ar_session_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/shot_phase.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// Whether the station-drift warning is **drawn**.
///
/// This gates only the two large orange graphics — the native 3D marker tint
/// (`kBezelStationDriftTint`) and the amber aim reticle — while debugging
/// tracking on a physical device. Everything else is untouched: the drift
/// measurement, its `ARCORE_STATION` logging, the capture gate, the warning
/// text in the status line, and the `_PhaseChip`/status-text colours all keep
/// working. Flip to `true` to restore the presentation.
const bool kStationDriftWarningVisualsEnabled = true;

/// Whether the 0.20 m station-distance measurement blocks capture.
///
/// When `false` (default), the station-drift gate is live: the automatic
/// verdict raises `station-drift` / `station-drift-recovering`, the manual
/// shutter honours [CaptureState.stationDrifted], and the "Too far from
/// station — return within 20 cm" warning is shown. The raw drift keeps being
/// measured and logged regardless. Flip to `true` only to temporarily bypass
/// the distance check during a drift investigation.
const bool kStationDriftGateBypassed = false;

/// Who fires the shutter.
///
  /// [manual] is the default: ARCore can refine a target's world pose after
  /// calibration, so the operator explicitly opts into unattended capture only
  /// after the scene has settled.
enum CaptureMode {
  automatic,
  manual;

  bool get isAutomatic => this == CaptureMode.automatic;

  String get label => this == CaptureMode.automatic ? 'Automatic' : 'Manual';
}

/// What caused a shutter request.
///
/// Carried into the `ARCORE_CAPTURE` log so a successful capture can be
/// attributed: an `automatic` line can only be produced by the eligibility
/// machine, never by a button press that happened to coincide with it.
enum CaptureTriggerSource {
  manual,
  automatic;

  String get label => name;
}

/// The automatic-capture state machine's current position.
///
/// Published by [CaptureController] and read by the status line, so the
/// sentence the user sees is the machine's own state rather than a second,
/// independently derived guess. `waiting` covers every blocked frame; [reason]
/// on [CaptureState] says which condition refused it.
enum AutoCaptureStage {
  /// No station, no target, or automatic capture switched off.
  idle,

  /// Eligible target, but at least one of the twelve conditions is false.
  waiting,

  /// Every condition holds and the shutter is about to be requested.
  armed,

  /// A photograph is in flight.
  capturing,

  /// The photograph returned and was stored.
  complete,

  /// The attempt failed; the target is retryable.
  failed,
}

/// Where the capture module is in its own lifecycle.
enum CapturePhase {
  /// Nothing has been asked of the device yet.
  idle,

  /// Probing ARCore availability.
  checking,

  /// AR cannot run. [CaptureState.message] says why, in words for the user.
  unavailable,

  /// The session is starting.
  starting,

  /// The session is up; the user is being asked to look around so ARCore can
  /// find surfaces.
  scanning,

  /// A station is calibrated and targets can be captured.
  capturing,

  /// Every target has a photograph.
  complete,

  /// Something unrecoverable happened.
  failed;

  bool get isLive =>
      this == CapturePhase.scanning ||
      this == CapturePhase.capturing ||
      this == CapturePhase.complete;
}

/// Everything the capture screen paints, in one immutable snapshot.
///
/// Immutable and rebuilt on every change, so a widget can never read a
/// half-updated pose and alignment pair.
class CaptureState {
  const CaptureState({
    required this.phase,
    required this.geometry,
    this.message,
    this.capability,
    this.textureId,
    this.tracking = TrackingState.initialising,
    this.trackingFailureReason,
    this.trackingRecovering = false,
    this.planesTracked = 0,
    this.planeCount = 0,
    this.walls = 0,
    this.floors = 0,
    this.ceilings = 0,
    this.intrinsics,
    this.imageAxis,
    this.displayProjectionMatrix,
    this.cameraQuality,
    this.station,
    this.targets = const <CaptureTarget>[],
    this.activeTargetId = 0,
    this.probes = const <int, SurfaceProbe>{},
    this.alignment,
    this.previousGuidance,
    this.alignedHeldMs = 0,
    this.mode = CaptureMode.manual,
    this.autoStage = AutoCaptureStage.idle,
    this.autoBlockReason,
    this.measurementFresh = false,
    this.angularRateDegPerSec,
    this.stabilityRejection,
    this.stable = false,
    this.stabilityProgress = 0,
    this.capturing = false,
    this.retryPending = false,
    this.driftM = 0,
    this.stationDriftWarningActive = false,
    this.activeSurfaceType,
    this.activeAnchored = false,
    this.activeAnchorTracking = false,
    this.imageRotationDeg = 0,
    this.lastSavedPath,
    this.lastError,
    this.arFault,
    this.arFaultFrame,
    this.assetsMatchSpec = false,
    this.meshTriangles = 0,
  });

  final CapturePhase phase;

  /// The user-facing sentence for the current phase, or `null`.
  final String? message;

  final ArCapabilityReport? capability;

  /// The Flutter texture id the AR camera renders into.
  final int? textureId;

  final TrackingState tracking;
  final String? trackingFailureReason;

  /// Set from a detected tracking/pose continuity break until the controller
  /// proves that a fresh camera pose and the shared station anchor still
  /// describe the frozen station reference.
  ///
  /// The recovery state keeps guidance silent and automatic capture blocked;
  /// it does not move targets or silently replace the station. It clears only
  /// after the bounded station-reference verification has completed, then the
  /// normal 600 ms capture-stability window begins again.
  final bool trackingRecovering;

  final int planesTracked;
  final int planeCount;

  /// Tracked planes by type, so "0 planes" can say *what* is missing.
  final int walls;
  final int floors;
  final int ceilings;

  final ArCameraIntrinsics? intrinsics;

  /// The measured sensor-image +x axis in display coordinates.  When present,
  /// it is more reliable than display rotation for deciding whether sensor
  /// horizontal and vertical FOV must be swapped.
  final ({double x, double y})? imageAxis;

  /// ARCore's projection matrix for the current display geometry, when the
  /// native renderer supplied one. The guidance overlay uses it in preference
  /// to a reconstructed FOV so its arrow and the rendered target share the
  /// same rotation, crop, and principal point.
  final Float64List? displayProjectionMatrix;

  /// The camera config the session reported: still stream size and focus mode.
  /// `null` until the host reports it; the status detail only shows the line
  /// once the numbers are real. This is what tells a blurry result apart as
  /// "small stream" versus "fixed focus" — two problems, two fixes.
  final ArCameraQuality? cameraQuality;

  final CaptureStation? station;

  /// All 30 targets, in id order.
  final List<CaptureTarget> targets;

  /// 1-based id of the target being aimed at; 0 before one is chosen.
  final int activeTargetId;

  /// The result of probing each direction for a surface, keyed by target id.
  ///
  /// Populated once after calibration and refreshed on demand. Empty means "not
  /// probed", which is different from "probed and found nothing" — the UI says
  /// so rather than pretending every target has been checked.
  final Map<int, SurfaceProbe> probes;

  /// How many of the 30 directions have a surface, when probed.
  int get probesWithSurface => probes.values.where((p) => p.found).length;

  /// The **unsmoothed** alignment. This is the one the gate, the instruction,
  /// the arrow and the numbers all use: a smoothed value must never be allowed
  /// to declare a shot ready, and must never be allowed to point, either — a
  /// pointer built from a filtered copy of the pose lags the arithmetic it is a
  /// picture of.
  final AlignmentResult? alignment;

  /// The guidance state the **previous** frame settled on, for the indicator's
  /// hysteresis.
  ///
  /// `guidance` used to be computed with `previous: null` on every build, which
  /// silently disabled the entry/exit band it was documented to have: the
  /// indicator flipped at the raw 3.0° boundary, and a hand tremor crossing it
  /// produced `aligned` ↔ `left` several times a second. The controller now
  /// carries the last answer forward in this field so the band actually
  /// applies. It is display state only — the capture gate reads
  /// [AlignmentResult.withinTolerance] and never this.
  final GuidanceState? previousGuidance;

  /// How long the alignment error has been continuously inside tolerance, in
  /// milliseconds. It drives both the `aligned` indicator's 200 ms latch and
  /// the strict target-alignment half of the capture hold.
  final double alignedHeldMs;

  /// Whether the target has remained inside the strict alignment cone for the
  /// entire configured capture-hold duration. This is deliberately separate
  /// from [stable]: a phone can be physically still while aimed somewhere else.
  bool get alignmentHoldComplete =>
      alignment != null &&
      alignment!.usable &&
      alignment!.withinTolerance &&
      alignedHeldMs >= geometry.stabilityWindow.inMilliseconds;

  /// Who fires the shutter. See [CaptureMode].
  final CaptureMode mode;

  /// The automatic-capture state machine's current position.
  final AutoCaptureStage autoStage;

  /// The machine-readable reason the automatic shutter is blocked, or `null`
  /// when it is not.
  ///
  /// One of `mode-manual`, `tracking`, `pose-age`, `no-station`,
  /// `station-drift`, `no-target`, `already-captured`, `alignment`,
  /// `alignment-window`, `window`,
  /// `orientation-drift`, `position-drift`, `angular-velocity`,
  /// `in-progress`, `failure-latched`, `no-anchor`, `anchor-moving`.
  ///
  /// Kept separate from [captureBlockReason] because they answer different
  /// questions: that one is a sentence for the user, this one is a token for
  /// `ARCORE_CAPTURE` and for a device log that has to be greppable.
  final String? autoBlockReason;

  /// Whether the newest reading is younger than the accepted age.
  final bool measurementFresh;

  /// Angular rate of the last reading, degrees/second, when it could be measured.
  final double? angularRateDegPerSec;

  /// Why the stability gate last refused, in the engine's own words:
  /// `angular-rate`, `orientation-drift`, `position-drift`, `measurement-age`,
  /// `sample-gap`, `timestamp`, `alignment`, `window`, `invalid-pose`,
  /// `no-station`, or
  /// `null` when the whole 600 ms window holds.
  ///
  /// The list is the vocabulary of `VioSphereEngine._updateStability`, carried
  /// through verbatim so the sentence the user reads is derived from the fact
  /// that refused the frame rather than from a guess about it.
  final String? stabilityRejection;

  /// Whether the pose has been steady for the whole stability window.
  final bool stable;

  /// 0…1 progress of the combined strict-alignment and motion-stability hold.
  /// A ring is allowed to fill only as far as both have advanced.
  final double stabilityProgress;

  /// Whether a photograph is being written right now.
  final bool capturing;

  /// Whether the green alignment ring is **full**.
  ///
  /// The one definition of the state the UI shows as a completed ring, and the
  /// only thing target visibility is allowed to key on: the raw alignment has
  /// held inside tolerance *and* the stability window has elapsed. The motion
  /// history survives the configured hysteresis band, but the strict
  /// alignment-hold timer does not, so the shutter can never fire from an
  /// earlier off-target hold.
  ///
  /// Deliberately **not** `shotPhase == ShotPhase.capture`: that phase also
  /// folds in tracking, measurement freshness and drift, and those flip on
  /// their own (a pose gap over 250 ms, a momentary pause). Keying visibility
  /// on them is what made the marker flash on and off while the phone was held
  /// perfectly still.
  ///
  /// [captureBlockReasonBeforeSettle] is included so that "the ring is full"
  /// and "the target is hidden" are the same event: a full ring means every
  /// other capture condition holds, and a blocked shutter means the marker
  /// stays put for the user to re-aim with.
  bool get ringComplete =>
      alignment != null &&
      alignment!.usable &&
      alignment!.withinTolerance &&
      alignmentHoldComplete &&
      stable &&
      captureBlockReasonBeforeSettle == null;

  /// Whether the last attempt on the active target failed and has not been
  /// re-armed by re-aiming.
  ///
  /// **The latch that stops a capture loop.** `captureBlockReason` used to say
  /// nothing about a failed attempt, so a target whose photograph kept failing
  /// stayed "ready" — and with the auto shutter that meant fire, fail, fire,
  /// fail, forever, with the interface flickering between "Ready to capture"
  /// and "Saving the photograph…" and the count never leaving 0 of 30.
  ///
  /// It is cleared only by a deliberate move off the target — an error past
  /// `tolerance + hysteresis`, the same threshold that breaks the stability
  /// window — so one attempt is allowed per alignment episode, and the user
  /// re-aims to try again rather than the app spinning the shutter.
  final bool retryPending;

  /// How far the phone is from the station, in metres.
  ///
  /// Displayed always, and enforced at [geometry]'s
  /// [CaptureGeometryConfig.stationDriftWarningM]: the 30 targets are placed
  /// around the station, so a photograph taken metres away from it is composed
  /// from the wrong place. The comparison is one-sided by design — a **NaN**
  /// drift (the parser's non-finite guard exists so this cannot happen) would
  /// make `driftM > limit` false and pass the gate silently, so the guard and
  /// the gate are fixed together.
  final double driftM;

  /// Whether the phone is too far from the station to capture.
  ///
  /// The raw predicate used by the phase, block reason and shutter. There is no
  /// hysteresis anywhere on this gate: one sample at or below 0.20 m reopens it
  /// immediately, and the engine then requires a fresh full stability window
  /// before the shutter can fire.
  /// Whether the raw drift has crossed the station limit.
  ///
  /// **Bypassed while [kStationDriftGateBypassed] is `true`** — this predicate
  /// feeds every capture-blocking decision (manual block reason, instruction
  /// sentence, display reasons), so during the temporary testing bypass it
  /// reads `false` regardless of the measurement. The raw [driftM] value is
  /// still computed, published and logged; `_withStationDriftWarning` reports
  /// each crossing of the limit directly from the measurement.
  bool get stationDrifted =>
      !kStationDriftGateBypassed && driftM > geometry.stationDriftWarningM;

  /// Whether the station-distance warning is being presented.
  ///
  /// The controller enters it at the raw 0.20 m limit and clears it only after
  /// a short, below-0.18 m confirmation. This prevents visual flicker around
  /// the limit. The raw [stationDrifted] predicate remains the safety authority
  /// and this presentation latch can only make capture recovery stricter.
  final bool stationDriftWarningActive;

  /// Whether the station-distance warning may be shown to the operator.
  ///
  /// The controller normally clears [stationDriftWarningActive] as soon as the
  /// temporary distance-gate bypass is enabled.  Keeping this derived guard at
  /// the presentation boundary closes the one-event race where a state emitted
  /// just before that clear could briefly paint a "Return to station" message
  /// even though distance is intentionally not a capture restriction for this
  /// diagnostic build.  The raw flag remains available to the controller for
  /// its hysteresis bookkeeping and diagnostics.
  bool get stationDriftWarningVisible =>
      !kStationDriftGateBypassed && stationDriftWarningActive;

  /// The surface the active target is anchored to, e.g. `PLANE_VERTICAL`.
  final String? activeSurfaceType;
  final bool activeAnchored;

  /// Whether ARCore is **tracking** the active target's world anchor.
  ///
  /// Whether ARCore is currently refining the anchor, as opposed to merely
  /// having it.
  ///
  /// A capture target is a world anchor at a position this app computed, so a
  /// **paused** anchor is still drawn — the renderer reads `anchor.pose` every
  /// frame and the mesh stays at its world position, which is what keeps a
  /// target fixed while the phone turns. What a pause does cost is confidence:
  /// ARCore's estimate of the world is degraded while it lasts, so the shutter
  /// stays blocked on this flag and the guidance says
  /// "Waiting for AR to stabilise the target…".
  ///
  /// The two flags are kept separate because they answer different questions:
  /// [activeAnchored] is "the node exists", this is "the node is being
  /// tracked right now".
  final bool activeAnchorTracking;

  /// How far the saved photograph is rotated relative to the display.
  final int imageRotationDeg;

  /// Path of the last photograph written, for the confirmation line.
  final String? lastSavedPath;

  final String? lastError;

  /// A native failure that stops the AR view from rendering, with its origin.
  ///
  /// Kept separate from [lastError] because it is not a capture failure: it means
  /// the camera preview cannot work, and the screen shows it as a red banner with
  /// a Retry action rather than as a line of diagnostics.
  final String? arFault;
  final String? arFaultFrame;

  /// Whether the shipped GLB assets still match the recorded dimensions.
  final bool assetsMatchSpec;
  final int meshTriangles;

  final CaptureGeometryConfig geometry;

  /// The display field of view the on-screen pointer projects with, in degrees:
  /// screen-x and screen-y.
  ///
  /// The platform's own intrinsics when it reported them, otherwise the
  /// documented provisional default — the same single source `sensorFov()`
  /// falls back to, so a session that has not reported intrinsics yet still
  /// projects rather than drawing nothing. It is the pair `project()` needs to
  /// put a camera-frame direction on a `w × h` viewport, and nothing else in
  /// the capture state uses it: the readiness decision is untouched by it.
  ({double hfovDeg, double vfovDeg}) get displayFov {
    final intrinsics0 = intrinsics;
    final axis = imageAxis;
    if (intrinsics0 == null) return ArCameraIntrinsics.fallbackDisplayFov;
    if (axis != null) {
      return intrinsics0.displayFovForImageAxis(
        axisX: axis.x,
        axisY: axis.y,
      );
    }
    return intrinsics0.portraitDisplayFov();
  }

  CaptureTarget? get activeTarget => activeTargetId == 0
      ? null
      : targets.cast<CaptureTarget?>().firstWhere(
          (CaptureTarget? t) => t?.targetId == activeTargetId,
          orElse: () => null,
        );

  /// The target the arrow, ring and alignment overlay belong to, or `null`
  /// when the guidance must not be drawn at all.
  ///
  /// [activeTarget] is the authoritative selection; this adds the two states
  /// in which a stale or completed selection must not paint guidance: the
  /// target already has a saved photograph, and every target is captured.
  /// Retake puts a target back to `pending` before selecting it, so an
  /// explicit retake still gets guidance through the ordinary path.
  CaptureTarget? get guidanceTarget {
    if (phase == CapturePhase.complete) return null;
    final target = activeTarget;
    if (target == null || target.status.isDone) return null;
    return target;
  }

  int get capturedCount =>
      targets.where((CaptureTarget t) => t.status.isDone).length;

  int get totalTargets =>
      targets.isEmpty ? kCaptureTargetCount : targets.length;

  /// The named guidance state for this frame, from the same alignment the
  /// pointer and the shutter use.
  ///
  /// One mapping, shared by the instruction line and the diagnostics log, so a
  /// turn never gets described as three different directions in one frame.
  /// [previousGuidance] supplies the hysteresis band; without it the indicator
  /// would flip at the raw tolerance boundary.
  GuidanceState get guidance => guidanceStateOf(
    // A recovered ARCore pose is not yet an authority for the arrow.  The
    // controller keeps this false while it verifies that the station anchor
    // and the recovered camera pose still describe the calibrated world.
    trackingValid:
        tracking == TrackingState.tracking && !trackingRecovering,
    alignment: alignment,
    previous: previousGuidance,
    alignedHeldMs: alignedHeldMs,
  );

  /// Whether the guidance may be painted at all: a live pose, a placed target,
  /// and an alignment with an answer.
  ///
  /// The arrow, the ring and the instruction all read this one predicate, so
  /// "no arrow" and "no turn instruction" are the same event.
  bool get guidanceValid =>
      tracking == TrackingState.tracking &&
      !trackingRecovering &&
      activeAnchored &&
      activeAnchorTracking &&
      alignment != null &&
      alignment!.usable;

  /// The one status sentence for the capture strip.
  ///
  /// Derived from fields the controller published — never recomputed from the
  /// pose. It follows [captureBlockReasonBeforeSettle] except for the
  /// station-warning flag, which is suppressed while no live station anchor
  /// supplies a distance. The shutter still follows the raw block reason.
  String get captureStatus {
    if (capturing) return 'Capturing';
    if (phase == CapturePhase.complete) return 'Capture complete';
    if (retryPending) return 'Capture failed — retrying';
    if (tracking != TrackingState.tracking) return 'Waiting for AR tracking';
    if (trackingRecovering) return 'Verifying station reference';
    if (station == null) return 'Calibrate the station';
    if (phase == CapturePhase.unavailable || phase == CapturePhase.failed) {
      return message ?? 'AR is unavailable';
    }
    if (phase != CapturePhase.capturing) return 'Waiting for AR tracking';
    if (activeTarget == null) return 'Select a target';
    if (stationDriftWarningVisible) return 'Return within 20 cm of station';
    if (!activeAnchored || !activeAnchorTracking) return 'Waiting for AR tracking';
    if (guidance != GuidanceState.aligned) return 'Align target';
    if (!alignmentHoldComplete || !stable) return 'Hold steady';
    return mode.isAutomatic ? 'Ready to capture' : 'Ready — press Capture';
  }

  /// The shot's own phase, derived from the conditions below.
  ///
  /// A getter, not a stored field: it cannot drift out of sync with
  /// [captureBlockReason], because both read the same state.
  ShotPhase get shotPhase => deriveShotPhase(
    trackingValid:
        tracking == TrackingState.tracking && !trackingRecovering,
    measurementFresh: measurementFresh,
    targetSelected: activeTargetId != 0,
    targetAnchored: activeAnchored,
    withinTolerance: alignment?.withinTolerance ?? false,
    alignmentHoldComplete: alignmentHoldComplete,
    stable: stable,
    saving: capturing,
    lastAttemptFailed: activeTarget?.status == CaptureTargetStatus.failed,
    targetAccepted: activeTarget?.status.isDone ?? false,
    stationDrifted: stationDrifted,
  );

  /// Whether every condition Section 1 §11 lists is satisfied.
  ///
  /// Six conditions, in the order the UI explains them:
  ///
  /// 1. the AR session initialised and a frame is reaching the texture
  ///    ([arFault] is `null`, [textureId] is set);
  /// 2. tracking is valid;
  /// 3. the station is calibrated;
  /// 4. a target is selected and not already captured;
  /// 5. the target is placed — its world-space anchor exists;
  /// 6. the alignment error is inside tolerance, the pose has been steady for
  ///    the whole stability window, and no photograph is being saved.
  bool get readyToCapture => captureBlockReason == null;

  /// Why the shutter is disabled, in one sentence, or `null` when it is not.
  ///
  /// [readyToCapture] is defined as `captureBlockReason == null`, so the button
  /// state and the explanation can never disagree — which is the failure mode
  /// this exists to prevent: a greyed-out button with no stated reason.
  ///
  /// **No plane count appears here, on any path.** Targets are placed at
  /// calculated world positions, so "no surfaces detected" cannot block a
  /// capture, and telling the user to point at a wall would be advice for a
  /// problem this app no longer has. The prereqs are tracking, station,
  /// placement, alignment, stability, and not-already-saving — exactly the
  /// brief's list — plus **not having walked away from the station**: the
  /// geometry the targets are placed in is the station's, so the drift check
  /// is a real gate and not a warning. Walking 5 m away used to leave the
  /// shutter live because nothing read [driftM].
  String? get captureBlockReason {
    final early = captureBlockReasonBeforeSettle;
    if (early != null) return early;
    if (!alignmentHoldComplete) return 'Keep the target centred…';
    if (!stable) return 'Hold still…';
    return null;
  }

  /// The user-facing version of [captureBlockReason].
  ///
  /// The warning sentence follows [stationDriftWarningActive], including its
  /// short recovery confirmation. The raw drift gate remains active throughout;
  /// a retained value whose live station anchor has not been reacquired is
  /// reported as an anchor wait rather than as a fresh walk-away event.
  String? get displayCaptureBlockReason {
    if (stationDriftWarningVisible) return _stationDriftMessage;
    // `driftM` is intentionally retained until a matching live station anchor
    // confirms a new sample. While that anchor is absent, a raw old value must
    // still block the shutter but must not be shown as a new walk-away event.
    if (stationDrifted && !activeAnchorTracking) {
      return 'Waiting for AR to stabilise the target…';
    }
    return captureBlockReason;
  }

  /// The display counterpart of [captureBlockReasonBeforeSettle].
  ///
  /// The progress ring reads this one so it cannot begin filling while the
  /// visible station-distance warning is still shown.
  String? get displayCaptureBlockReasonBeforeSettle {
    if (stationDriftWarningVisible) return _stationDriftMessage;
    if (stationDrifted && !activeAnchorTracking) {
      return 'Waiting for AR to stabilise the target…';
    }
    return captureBlockReasonBeforeSettle;
  }

  /// Every capture condition **except** the stability window, in one sentence,
  /// or `null` when they are all met.
  ///
  /// Split out from [captureBlockReason] for one reason: the green ring fills
  /// with the stability window's progress, and it must not fill at all when the
  /// shutter is blocked for any *other* reason. Before the split the ring could
  /// sit at 100% while the controller was stuck on drift or an untracked
  /// anchor — a full ring, a dead shutter, and no stated reason, which is
  /// exactly the mismatch this closes. The ring may fill ⟹ every other
  /// condition holds ⟹ a full ring means [readyToCapture].
  String? get captureBlockReasonBeforeSettle {
    if (arFault != null) return 'AR is not rendering. See the error above.';
    if (textureId == null) return 'Waiting for the AR camera.';
    if (phase == CapturePhase.complete) {
      return 'Every target has a photograph.';
    }
    if (phase == CapturePhase.unavailable || phase == CapturePhase.failed) {
      return message ?? 'AR is unavailable.';
    }
    if (phase != CapturePhase.capturing) {
      return message ?? 'AR is still starting.';
    }
    if (tracking != TrackingState.tracking) {
      return 'AR tracking is not ready — move the phone slowly.';
    }
    if (trackingRecovering) {
      return 'Verifying the station reference — keep the phone at the station.';
    }
    if (station == null) return 'Calibrate the station first.';
    if (stationDriftWarningVisible) return _stationDriftMessage;
    final target = activeTarget;
    if (target == null) return 'Select a target.';
    if (target.status.isDone) {
      return 'This target is already captured. Use Retake to replace it.';
    }
    if (capturing) {
      // The photograph is being written. Saying "Ready to capture" here — which
      // the device recording showed while the chip read "Validating image" and
      // the button read "Saving…" — is the one contradiction the brief forbids.
      return 'Saving the photograph…';
    }
    if (retryPending) {
      // The last attempt failed. Neither the button nor the auto shutter may
      // fire again until the user re-aims — see [retryPending].
      return 'The last attempt failed — re-aim at the target to try again.';
    }
    if (stationDrifted) {
      // Before placement and alignment on purpose: they are all measured from
      // the station's frame, so "you are not where the capture assumes" is the
      // more fundamental fact. The number is in the sentence because "walk
      // back" without "from 5.0 m" leaves the user guessing.
      return _stationDriftMessage;
    }
    if (!activeAnchored) {
      // Placement fails only when AR refuses the world anchor, and the retry
      // action for that is the Scan again button.
      return 'The target is not placed — press Scan again.';
    }
    if (!activeAnchorTracking) {
      // Placed, but ARCore has not locked onto the anchor yet. The marker is
      // drawn at its world position either way; what is missing is ARCore's
      // confidence in the world, and a photograph taken without it is composed
      // from an estimate that is still moving.
      return 'Waiting for AR to stabilise the target…';
    }
    final alignment0 = alignment;
    if (alignment0 == null || !alignment0.usable) {
      return 'Waiting for a target position.';
    }
    if (!alignment0.withinTolerance) return alignment0.instruction;
    return null;
  }

  /// `3 planes · 2 walls · 1 floor · 0 ceilings`, or `no planes yet`.
  String get planeSummary {
    if (planesTracked == 0) {
      return planeCount == 0
          ? 'no planes yet'
          : '$planeCount seen, none tracked yet';
    }
    return '$planesTracked planes · $walls walls · $floors floors · '
        '$ceilings ceilings';
  }

  /// The phases in which AR is still coming up, and ARCore's own state is the
  /// most useful thing the user can be told.
  bool get _awaitingTracking =>
      phase == CapturePhase.idle ||
      phase == CapturePhase.checking ||
      phase == CapturePhase.starting ||
      phase == CapturePhase.scanning;

  /// What to do about the current AR tracking state, in one sentence, or
  /// `null` when tracking is fine and nothing needs saying.
  ///
  /// Lives here, with the state, because the **instruction** line needs it.
  /// `starting` and `scanning` are decided by the last pose, and while ARCore
  /// is paused the phase never leaves `starting` — so the instruction used to
  /// sit on "Starting the AR session…" indefinitely while the status bar read
  /// "AR tracking paused · no planes yet" and this sentence was one expand-tap
  /// away. ARCore's own failure reason is carried through, because every one of
  /// its reasons ("too dark", "moving too fast", "not enough detail") is a
  /// different action for the user.
  String? get trackingGuidance {
    switch (tracking) {
      case TrackingState.unavailable:
        return 'AR tracking is unavailable on this device.';
      case TrackingState.paused:
        final reason = trackingFailureReason;
        if (reason == null || reason.isEmpty) {
          return 'Tracking paused — move slowly toward a detailed surface.';
        }
        // ARCore's per-reason advice is actionable ("too dark", "too fast");
        // the raw code stays in parentheses so the sentence can be correlated
        // with the ARCORE_* log line that reported the same reason.
        return '${_trackingFailureAdvice(reason)} ($reason)';
      case TrackingState.initialising:
        return 'Tracking recovering — hold the phone steady.';
      case TrackingState.stopped:
        return 'AR tracking stopped. Point the phone back where you started.';
      case TrackingState.tracking:
        return null;
    }
  }

  /// ARCore reports *why* it paused, and the user can act on all of it.
  String _trackingFailureAdvice(String? reason) {
    switch (reason) {
      case 'INSUFFICIENT_LIGHT':
        return 'It is too dark. Move somewhere brighter.';
      case 'EXCESSIVE_MOTION':
        return 'You are moving too fast. Hold the phone still.';
      case 'INSUFFICIENT_FEATURES':
        return 'There is not enough detail to track. Point at a textured '
            'surface, not a blank wall.';
      case 'BAD_STATE':
        return 'AR lost its bearings. Point the phone back where you started.';
      case 'CAMERA_UNAVAILABLE':
        return 'The camera is in use by another app.';
      default:
        return 'AR lost tracking. Move to a brighter area with more visible '
            'detail.';
    }
  }

  /// The instruction line, e.g. "Turn right 20°".
  String get instruction {
    final alignment0 = alignment;
    if (phase != CapturePhase.capturing) {
      // ARCore's own state outranks the phase while it is still coming up —
      // see [trackingGuidance] for why the generic sentence was a dead end.
      final guidance = _awaitingTracking ? trackingGuidance : null;
      if (guidance != null) return guidance;
      return switch (phase) {
        CapturePhase.idle => 'Starting AR…',
        CapturePhase.checking => 'Checking AR support…',
        CapturePhase.unavailable => message ?? 'AR is unavailable.',
        CapturePhase.starting => 'Starting the AR session…',
        CapturePhase.scanning =>
          message ?? 'Move the phone slowly to initialize AR tracking.',
        CapturePhase.failed => message ?? 'Capture stopped.',
        CapturePhase.complete => 'All $totalTargets photographs captured.',
        CapturePhase.capturing => 'Calibrate the station to begin.',
      };
    }
    // No trustworthy pose: say so rather than naming a turn measured from a
    // stale one. This is the same condition that hides the arrow, so the
    // pointer and the sentence can never disagree about whether the phone is
    // being aimed with a live pose.
    if (tracking != TrackingState.tracking) {
      return trackingGuidance ?? 'AR is not tracking — move the phone slowly.';
    }
    // No target is selected after a capture when auto-advance is off (and for
    // the frame between completion and the next selection). Without this the
    // line fell through to "Placing the target…" for a target that does not
    // exist, which reads as a placement in progress rather than a decision the
    // user has to make.
    if (activeTarget == null) return 'Select a target.';
    if (trackingRecovering) {
      // Do not issue a turn or a walk-back instruction until the controller
      // has proved that the recovered pose still belongs to this station.
      return 'Verifying the station reference — keep the phone at the station.';
    }
    if (stationDriftWarningVisible) {
      // Ahead of placement and alignment for the same reason as in
      // [captureBlockReason]: everything below is measured from the station.
      // This line is always visible (it never collapses), so it is the one
      // that has to name the action. It names exactly one: return to the spot
      // the 30 targets were built around. Recalibration stays available in the
      // UI, but suggesting it here would invite moving the station to wherever
      // the phone happens to be — which silently redefines the capture
      // geometry instead of correcting the drift.
      return 'Too far from station — return within 20 cm.';
    }
    if (!activeAnchored) {
      return 'Placing the target…';
    }
    if (!activeAnchorTracking) {
      // The marker is intentionally hidden while ARCore has only a stale
      // anchor pose. Guidance and capture resume only after tracking returns.
      return 'Waiting for AR to stabilise the target…';
    }
    if (alignment0 == null || !alignment0.usable) {
      return 'Waiting for a target position…';
    }
    // Name the target in the instruction: "Aim at the front target. Turn
    // right 45°" answers both "which one" and "which way". **Aim, never
    // walk**: the targets are 1.0 m around the station and the shutter is
    // blocked beyond 0.20 m of drift, so "move to" was an instruction the app
    // then punished — a device screenshot showed exactly that, drift 2.19 m
    // after following it.
    final targetName = _activeTargetName;
    // The latched indicator, not the raw comparison: the two differ only while
    // a fresh alignment is settling (≤200 ms), and the shutter cannot fire in
    // that interval because the 600 ms stability window is still filling. Using
    // the latched state here means the sentence, the ring and the log name the
    // same thing at the same instant.
    if (guidance == GuidanceState.aligned) {
      if (!alignmentHoldComplete) return 'Keep the target centred…';
      return stable ? 'Target aligned — hold steady.' : _settlingInstruction;
    }
    return 'Aim at $targetName. ${alignment0.instructionForGuidance(guidance)}';
  }

  String get _stationDriftMessage =>
      'Too far from station — return within 20 cm.';

  /// Why the shutter is not armed yet, in the user's terms.
  ///
  /// A generic "Hold still…" hides the difference between a phone that is
  /// genuinely moving, one whose readings are stale, and one that is drifting
  /// off the target — three problems with three different fixes. The reason
  /// comes from [stabilityRejection], which the monitor sets per update.
  String get _settlingInstruction {
    final rate = angularRateDegPerSec;
    final rateText = rate == null ? null : '${rate.toStringAsFixed(1)}°/s';
    return switch (stabilityRejection) {
      'angular-rate' =>
        rateText == null
            ? 'Hold still — the phone is moving.'
            : 'Hold still — moving at $rateText.',
      'orientation-drift' => 'Hold still — the phone is still turning.',
      'position-drift' =>
        'Hold still — the phone is moving. Stand steady for a moment.',
      'measurement-age' => 'Waiting for a fresh pose…',
      'sample-gap' => 'Waiting for a continuous pose stream…',
      'timestamp' => 'Waiting for the pose stream to settle…',
      'invalid-pose' => 'Waiting for a valid camera pose…',
      'no-station' => 'Calibrate the station first.',
      _ => 'Hold still…',
    };
  }

  /// The active target as the guidance says it: "the front target",
  /// "the zenith target", "target 7".
  String get _activeTargetName {
    final target = activeTarget;
    if (target == null) return 'the target';
    if (target.pitchDegrees >= 89.9) return 'the zenith target';
    if (target.pitchDegrees <= -89.9) return 'the nadir target';
    if (target.yawDegrees.abs() < 1e-9 && target.pitchDegrees.abs() < 1e-9) {
      return 'the front target';
    }
    return 'target ${target.targetId}';
  }

  CaptureState copyWith({
    CapturePhase? phase,
    CaptureGeometryConfig? geometry,
    String? message,
    bool clearMessage = false,
    ArCapabilityReport? capability,
    int? textureId,
    TrackingState? tracking,
    String? trackingFailureReason,
    bool? trackingRecovering,
    int? planesTracked,
    int? planeCount,
    int? walls,
    int? floors,
    int? ceilings,
    ArCameraIntrinsics? intrinsics,
    ({double x, double y})? imageAxis,
    Float64List? displayProjectionMatrix,
    ArCameraQuality? cameraQuality,
    bool clearCameraQuality = false,
    CaptureStation? station,
    bool clearStation = false,
    List<CaptureTarget>? targets,
    int? activeTargetId,
    Map<int, SurfaceProbe>? probes,
    AlignmentResult? alignment,
    GuidanceState? previousGuidance,
    bool clearGuidanceHistory = false,
    double? alignedHeldMs,
    CaptureMode? mode,
    AutoCaptureStage? autoStage,
    String? autoBlockReason,
    bool clearAutoBlockReason = false,
    bool? measurementFresh,
    double? angularRateDegPerSec,
    String? stabilityRejection,
    bool clearAlignment = false,
    bool? stable,
    double? stabilityProgress,
    bool? capturing,
    bool? retryPending,
    double? driftM,
    bool? stationDriftWarningActive,
    String? activeSurfaceType,
    bool? activeAnchored,
    bool? activeAnchorTracking,
    int? imageRotationDeg,
    String? lastSavedPath,
    String? lastError,
    bool clearError = false,
    String? arFault,
    String? arFaultFrame,
    bool clearArFault = false,
    bool? assetsMatchSpec,
    int? meshTriangles,
  }) => CaptureState(
    phase: phase ?? this.phase,
    geometry: geometry ?? this.geometry,
    message: clearMessage ? null : (message ?? this.message),
    capability: capability ?? this.capability,
    textureId: textureId ?? this.textureId,
    tracking: tracking ?? this.tracking,
    trackingFailureReason: trackingFailureReason ?? this.trackingFailureReason,
    trackingRecovering: trackingRecovering ?? this.trackingRecovering,
    planesTracked: planesTracked ?? this.planesTracked,
    planeCount: planeCount ?? this.planeCount,
    walls: walls ?? this.walls,
    floors: floors ?? this.floors,
    ceilings: ceilings ?? this.ceilings,
    intrinsics: intrinsics ?? this.intrinsics,
    imageAxis: imageAxis ?? this.imageAxis,
    displayProjectionMatrix:
        displayProjectionMatrix ?? this.displayProjectionMatrix,
    cameraQuality: clearCameraQuality
        ? null
        : (cameraQuality ?? this.cameraQuality),
    // `station: null` cannot clear it — `??` keeps the old value — so the
    // discard paths (session restart, `retrySession`) need an explicit flag.
    station: clearStation ? null : (station ?? this.station),
    targets: targets ?? this.targets,
    activeTargetId: activeTargetId ?? this.activeTargetId,
    probes: probes ?? this.probes,
    alignment: clearAlignment ? null : (alignment ?? this.alignment),
    previousGuidance: clearGuidanceHistory || clearAlignment
        ? null
        : (previousGuidance ?? this.previousGuidance),
    alignedHeldMs: clearGuidanceHistory || clearAlignment
        ? 0
        : (alignedHeldMs ?? this.alignedHeldMs),
    mode: mode ?? this.mode,
    autoStage: autoStage ?? this.autoStage,
    autoBlockReason: clearAutoBlockReason
        ? null
        : (autoBlockReason ?? this.autoBlockReason),
    measurementFresh: measurementFresh ?? this.measurementFresh,
    angularRateDegPerSec: angularRateDegPerSec ?? this.angularRateDegPerSec,
    stabilityRejection: clearAlignment
        ? null
        : (stabilityRejection ?? this.stabilityRejection),
    stable: stable ?? this.stable,
    stabilityProgress: stabilityProgress ?? this.stabilityProgress,
    capturing: capturing ?? this.capturing,
    retryPending: retryPending ?? this.retryPending,
    driftM: driftM ?? this.driftM,
    stationDriftWarningActive:
        stationDriftWarningActive ?? this.stationDriftWarningActive,
    activeSurfaceType: activeSurfaceType ?? this.activeSurfaceType,
    activeAnchored: activeAnchored ?? this.activeAnchored,
    activeAnchorTracking: activeAnchorTracking ?? this.activeAnchorTracking,
    imageRotationDeg: imageRotationDeg ?? this.imageRotationDeg,
    lastSavedPath: lastSavedPath ?? this.lastSavedPath,
    lastError: clearError ? null : (lastError ?? this.lastError),
    arFault: clearArFault ? null : (arFault ?? this.arFault),
    arFaultFrame: clearArFault ? null : (arFaultFrame ?? this.arFaultFrame),
    assetsMatchSpec: assetsMatchSpec ?? this.assetsMatchSpec,
    meshTriangles: meshTriangles ?? this.meshTriangles,
  );
}

