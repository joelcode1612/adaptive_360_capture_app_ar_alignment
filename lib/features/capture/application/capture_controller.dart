// The constructor parameters are deliberately named without the leading
// underscore so the public API reads `session:` rather than `_session:`, and
// a private named parameter cannot be used outside this library. That is the
// one case `prefer_initializing_formals` cannot express, so it is suppressed
// for this file rather than worked around with a worse API.
// ignore_for_file: prefer_initializing_formals
import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart' as vio;
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/ar_session_service.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/glb_mesh_source.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/photo_capture_service.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/plane_anchor_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/adaptive_quaternion_smoother.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/motion_diagnostics.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/motion_sample.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/guidance/projection.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/glb/glb_model.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The one Flutter-aware piece of the Section 1 capture module.
///
/// Everything below it — the 26 directions, the station frame, the alignment
/// maths, the stability gate — is pure Dart and tested without a binding. This
/// class does the four things that genuinely need the platform:
///
/// * it probes ARCore, starts the session and subscribes to the pose stream;
/// * it turns a pose into a station, a target into an anchor, and an anchor into
///   a scene the native renderer draws;
/// * it decides when the shutter may fire, from tracking, anchoring, alignment
///   **and** steadiness together;
/// * it owns the photograph's save → verify → record ordering.
///
/// A [ChangeNotifier] rather than a state-management package, for the same
/// reason the existing capture flow uses one: the screen is the only consumer
/// and a second copy of the state is how the two drift apart.
///
/// ## Anchor recovery, in numbers
///
/// `_refreshAnchorIfDue` runs twice a second. A **paused** anchor is not
/// recovered by re-placing it at all: ARCore pauses a world anchor whenever
/// its own tracking degrades and resumes it by itself, and the marker keeps
/// its world position and stays in the scene throughout. Re-placement is
/// reserved for an anchor the host no longer has, and it is bounded — at most
/// [kAnchorReplacementsMax] re-placements of one target before the stage is
/// declared failed — because a loop that re-places forever is the hang this
/// class exists to prevent.
const int kAnchorReplacementsMax = 2;

/// A displayed station-drift warning enters at the unchanged 0.20 m safety
/// limit. It can clear only after a raw measurement is at or below 0.18 m for
/// this long. At the native 20 Hz pose delivery rate that is about ten samples:
/// enough to remove threshold flicker without masking the raw capture gate.
const double kStationDriftWarningRecoveryMarginM = 0.02;
const Duration kStationDriftWarningRecoveryHold = Duration(milliseconds: 500);

/// A recovered ARCore frame must prove it still belongs to the frozen station
/// before guidance resumes. The confirmation duration deliberately matches the
/// existing 600 ms capture window; capture itself starts a *new* window after
/// this verification, so it is never relaxed by the recovery check.
const Duration kRecoveryVerificationHold = Duration(milliseconds: 600);

/// Waiting indefinitely for a station anchor after a VIO/tracking break makes
/// a bad reference look like ordinary aiming. The Xiaomi device trace showed
/// ARCore leave the camera in TRACKING while its existing station anchor was
/// paused for longer than the former three-second grace. Keep guidance and the
/// shutter blocked, but allow a realistic relocalisation interval before
/// requiring an explicit recalibration.
const Duration kRecoveryVerificationTimeout = Duration(seconds: 8);

/// A station-anchor transform correction this large is worth reporting during
/// recovery. It is not, by itself, proof that the physical station moved:
/// ARCore can rebase both camera and anchor coherently. The authoritative
/// continuity test is the camera pose expressed *relative* to that anchor.
const double kRecoveryAnchorPositionDiscontinuityM = 0.08;
const double kRecoveryAnchorOrientationDiscontinuityDeg = 5.0;

/// Whether the target-transition trace is compiled in. Off unless the run asks
/// for it: `--dart-define=capture_transition_log=true`.
///
/// The trace answers the questions the "next GLB never appears" report could
/// not: which target is active, which anchor ids exist before and after a
/// transition, whether a placement was duplicated, and why a transition was
/// refused. Gated rather than always-on because the answer is only needed
/// while diagnosing a device, and an always-on debugPrint in the pose path
/// would cost a line per frame.
const bool kCaptureTransitionLog = bool.fromEnvironment(
  'capture_transition_log',
);

void _traceTransition(String message) {
  if (kCaptureTransitionLog) {
    debugPrint('capture: $message');
  }
}

/// Whether the camera/anchor/station trace is compiled in. Off unless the run
/// asks for it: `--dart-define=capture_pose_log=true`.
///
/// One line per pose would be 30–60 lines a second; the trace samples at 2 Hz
/// and immediately on a tracking transition. It exists to answer, from a
/// device log, whether the camera moved, whether the anchor moved, whether
/// either tracking state changed, and which station frame the target was
/// placed from.
const bool kCapturePoseLog = bool.fromEnvironment('capture_pose_log');

void _tracePose(String message) {
  if (kCapturePoseLog) {
    debugPrint('ARCORE_POSE: $message');
  }
}

String _formatVec(Vec3 v) =>
    '(${v.x.toStringAsFixed(3)},'
    '${v.y.toStringAsFixed(3)},${v.z.toStringAsFixed(3)})';

String _formatQuat(Quat q) =>
    '(${q.x.toStringAsFixed(3)},'
    '${q.y.toStringAsFixed(3)},${q.z.toStringAsFixed(3)},'
    '${q.w.toStringAsFixed(3)})';

/// A full live station-anchor transform, retained only as continuity evidence.
/// It never writes into [StationFrame] or the frozen target positions.
class _StationAnchorTransform {
  const _StationAnchorTransform({
    required this.id,
    required this.position,
    required this.orientation,
  });

  final String id;
  final Vec3 position;
  final Quat orientation;
}

/// The one authority for whether the current AR pose may establish a station.
///
/// The controller uses the carried pose and reference orientation to freeze a
/// station, while the UI uses [isReady] and [reason] to avoid offering a button
/// that the controller would reject on the same frame.
class CalibrationReadiness {
  const CalibrationReadiness._({
    required this.pose,
    required this.referenceOrientation,
    required this.reason,
  });

  const CalibrationReadiness.ready({
    required ArPoseSample pose,
    required Quat referenceOrientation,
  }) : this._(
         pose: pose,
         referenceOrientation: referenceOrientation,
         reason: null,
       );

  const CalibrationReadiness.blocked(String reason)
    : this._(pose: null, referenceOrientation: null, reason: reason);

  final ArPoseSample? pose;
  final Quat? referenceOrientation;
  final String? reason;

  bool get isReady => pose != null && referenceOrientation != null;
}

class CaptureController extends ChangeNotifier {
  CaptureController({
    required ArSessionService session,
    required PlaneAnchorService anchors,
    required PhotoCaptureService photos,
    required CaptureRepository repository,
    CaptureGeometryConfig geometry = const CaptureGeometryConfig(),
    DateTime Function()? clock,
    // Automatic capture is the only supported capture mode, so leaving a
    // completed target with no successor would strand the controller in
    // `awaitingTarget`: the arrow disappears and no later target can arm.
    // Keep the optional parameter for callers that deliberately build a
    // review-only controller, but make the production workflow continuous.
    this.autoAdvance = true,
    CaptureMode mode = CaptureMode.automatic,
    GlbMeshSource? meshes,
  }) : _session = session,
       _anchors = anchors,
       _photos = photos,
       _repository = repository,
       _clock = clock ?? DateTime.now,
       _mode = mode,
       // The live gate. One engine per session: it holds the frozen station
       // reference and the 26 world-locked targets, and it owns the 600 ms
       // stability window, the 0.20 m station gate, the 3-D alignment error and
       // the specification's shutter expression. Its thresholds are the
       // geometry's, so a non-default `CaptureGeometryConfig` still governs.
       _engine = vio.VioSphereEngine(
         radiusM: geometry.targetRadiusM,
         window: geometry.stabilityWindow,
         maxPoseAge: geometry.maxMeasurementAge,
         maxStationDriftM: geometry.stationDriftWarningM,
         maxAlignmentErrorDeg: geometry.alignmentToleranceDeg,
         maxWindowOrientationDriftDeg: geometry.maxOrientationDriftDeg,
         maxWindowPositionDriftM: geometry.maxPositionDriftM,
         maxAngularVelocityDegPerSec: geometry.maxAngularVelocityDegPerSec,
         maxSampleGap: geometry.maxSampleGap,
         rateBaseline: geometry.rateBaseline,
       ),
       _smoother = AdaptiveQuaternionSmoother(
         tauSeconds: geometry.smootherTauSeconds,
         snapAngleDeg: geometry.smootherSnapAngleDeg,
       ),
       _calculator = const AlignmentCalculator(),
       _specs = buildCaptureSphere(),
       _meshes = meshes,
       state = CaptureState(
         phase: CapturePhase.idle,
         geometry: geometry,
         mode: mode,
       );

  final ArSessionService _session;
  final PlaneAnchorService _anchors;
  final PhotoCaptureService _photos;
  final CaptureRepository _repository;
  final DateTime Function() _clock;

  /// The frozen station reference and the 26 world-locked targets, plus the
  /// live gate.
  ///
  /// One instance per session. `calibrateWithReference` is the only writer of
  /// the station; [`evaluateFrame`] is read-only with respect to it, so a live
  /// pose cannot move the station or a target.
  final vio.VioSphereEngine _engine;

  /// Display-only smoothing. The gate never sees its output.
  final AdaptiveQuaternionSmoother _smoother;

  /// Bounded motion record, for the comparison harness and for a defect report.
  final MotionDiagnostics diagnostics = MotionDiagnostics();

  /// The parsed bezel model, for the reticle.
  ///
  /// `null` until the asset has loaded, which is why the reticle falls back to a
  /// plain circle rather than blinking out.
  GlbModel? get bezelGeometry => _meshes?.bezelModel;

  /// The marker's physical size is configured in metres; the draw scale is
  /// that size divided by the plate's measured width. A blind constant is what
  /// made the 0.54 m-authored plate fill a third of the camera view.
  void _applyMarkerScale() {
    final meshes = _meshes;
    if (meshes == null) return;
    _anchors.markerScale = state.geometry.markerScaleFor(meshes.targetWidthM);
  }

  /// A monotonic clock for the filter and the stability window.
  ///
  /// `DateTime` is used rather than a raw `Stopwatch` because the existing
  /// smoother takes `DateTime`; the injected clock keeps tests able to walk a
  /// window without waiting for one.
  DateTime _lastPoseAt = DateTime.fromMillisecondsSinceEpoch(0);
  DateTime? _lastPoseReceivedAt;
  final AlignmentCalculator _calculator;

  /// The newest continuity generation observed from the native ARCore bridge.
  /// A changed generation means there was a tracking break, timestamp reset or
  /// long native frame gap even if Flutter never received an intermediate
  /// `PAUSED` event.
  int? _lastTrackingEpoch;

  /// Recovery is separate from ordinary capture stability. While it is pending
  /// the target geometry stays frozen, the arrow is neutral, and automatic
  /// capture is blocked until the station reference has been verified again.
  bool _recoveryVerificationPending = false;
  DateTime? _recoveryStartedAt;
  DateTime? _recoveryStableSince;
  Vec3? _recoveryStableStationLocalCamera;

  /// A map correction can be present on every pose while recovery verifies it.
  /// Keep one diagnostic for that episode rather than flooding Logcat at the
  /// pose-stream rate and obscuring the actual tracking transitions.
  bool _recoveryWorldCorrectionLogged = false;

  /// Last trusted relation between the shared station anchor and camera. These
  /// are evidence only; neither is ever used to reposition a target or station.
  _StationAnchorTransform? _lastTrustedStationAnchor;
  Vec3? _lastTrustedStationLocalCamera;

  /// A broken reference is cleared from state immediately. The asynchronous
  /// native teardown is guarded so a user cannot create a new station while an
  /// old scene-clear or detach request could still overtake it.
  bool _stationReferenceInvalidated = false;
  bool _stationInvalidationInFlight = false;

  /// ARCore's frame timestamps and this app's clock, anchored once.
  ///
  /// The engine's stability window runs on ARCore's own monotonic nanosecond
  /// timestamps — the only clock that makes a 600 ms hold testable without a
  /// device — while the age gate needs "how long ago was this frame, *now*".
  /// Those two live in different bases, so they are tied together at the first
  /// pose and advanced from the wall clock afterwards: the difference between
  /// them is then the frame's lag rather than a constant offset.
  int _poseAnchorNs = 0;
  DateTime? _poseAnchorAt;

  /// This pose's own timestamp and "now", both in ARCore's base.
  ///
  /// Two values, because the engine's age gate compares `now` against the
  /// sample's **own** timestamp and its gap gate compares two samples: feeding
  /// one value for both would make the age gate vacuous. A pose the platform
  /// did not timestamp is placed at the anchored "now", which is the honest
  /// reading of a frame with no timestamp of its own.
  ///
  /// A timestamp below the anchor means a new session with a new base, so the
  /// pair is re-anchored rather than measured against a stale origin.
  ({int sampleNanoseconds, int nowNanoseconds}) _poseTiming({
    required int? timestampNs,
    required DateTime now,
  }) {
    final anchorAt = _poseAnchorAt;
    if (timestampNs == null) {
      final nowNs = anchorAt == null
          ? now.microsecondsSinceEpoch * 1000
          : _poseAnchorNs + now.difference(anchorAt).inMicroseconds * 1000;
      return (sampleNanoseconds: nowNs, nowNanoseconds: nowNs);
    }
    if (anchorAt == null || timestampNs < _poseAnchorNs) {
      _poseAnchorNs = timestampNs;
      _poseAnchorAt = now;
      return (sampleNanoseconds: timestampNs, nowNanoseconds: timestampNs);
    }
    return (
      sampleNanoseconds: timestampNs,
      nowNanoseconds:
          _poseAnchorNs + now.difference(anchorAt).inMicroseconds * 1000,
    );
  }

  /// One ARCore sample as the engine consumes it.
  ///
  /// The position and the **raw** ARCore quaternion are copied straight across.
  /// The components are `(x, y, z, w)` on both sides — the order
  /// `Pose.getRotationQuaternion` fills and this app's Kotlin host forwards —
  /// so no re-ordering, transposition or extra rotation is applied. In
  /// particular the `captureWorld` mirror is **not** used here: the engine's
  /// convention already puts the optical axis on `−Z`, which is what ARCore
  /// reports.
  vio.VioPoseSample _sampleFrom(ArPoseSample pose, int timestampNanoseconds) {
    final raw = pose.raw;
    return vio.VioPoseSample(
      position: vio.Vec3(raw.position.x, raw.position.y, raw.position.z),
      rotation: vio.Quat(
        raw.orientation.x,
        raw.orientation.y,
        raw.orientation.z,
        raw.orientation.w,
      ),
      timestampNanoseconds: timestampNanoseconds,
    );
  }

  /// Freezes the station in **both** authorities at once: the [StationFrame] the
  /// UI, the placement and the alignment solve against, and the
  /// [vio.VioSphereEngine] whose live gate reads it.
  ///
  /// They are derived from the same frozen pose. The engine's station yaw is the
  /// horizontal yaw of the frame's forward axis — the *same* quantity
  /// `captureReferenceFromPose` kept when it discarded the phone's pitch and
  /// roll — so this cannot re-introduce tilt. And because
  /// `StationFrame.localToWorld(directionFromYawPitch(a, e))` produces exactly
  /// the world vector `buildNodeSphere` computes for that azimuth/elevation,
  /// the two describe the **same 26 world positions**.
  void _freezeStation(StationFrame frame) {
    // Only an explicit calibration may replace a reference that was invalidated
    // after a VIO discontinuity. Neither tracking recovery nor anchor refresh
    // calls this method.
    _stationReferenceInvalidated = false;
    _resetRecoveryVerification(clearTrustedReference: true);
    _resetStationDriftWarningLatch();
    _frame = frame;
    final f = frame.forwardAxis;
    final yawRad = math.atan2(f.x, -f.z);
    _engine.calibrateWithReference(
      stationOrigin: vio.Vec3(frame.origin.x, frame.origin.y, frame.origin.z),
      stationYawRad: yawRad,
    );
    // The snapshot the automatic machine's "targets unchanged" condition is
    // checked against. Taken from the engine, immediately after it built the
    // sphere, so it is the same numbers the gate will use.
    _frozenNodePositions = <Vec3>[
      for (final node in _engine.nodes)
        Vec3(node.worldPosition.x, node.worldPosition.y, node.worldPosition.z),
    ];
    // A new station is a new geometry: nothing about the previous guidance
    // survives it.
    _resetGuidanceHistory();
    final o = frame.origin;
    debugPrint(
      'ARCORE_STATION: station frozen; origin '
      '(${o.x.toStringAsFixed(3)}, ${o.y.toStringAsFixed(3)}, '
      '${o.z.toStringAsFixed(3)}) m, yaw '
      '${(yawRad * 180 / math.pi).toStringAsFixed(1)} deg, '
      '${_frozenNodePositions.length} targets.',
    );
  }

  /// Forgets the frozen station in both authorities.
  void _clearStation() {
    _frame = null;
    _engine.clearStation();
    _frozenNodePositions = const <Vec3>[];
    _resetRecoveryVerification(clearTrustedReference: true);
    _resetStationDriftWarningLatch();
    _lastStationTrustDiag = null;
    _anchorStabilityId = null;
    _anchorStabilityReference = null;
    _resetGuidanceHistory();
    debugPrint('ARCORE_STATION: station cleared.');
  }

  /// Drops recovery-only evidence. This never alters the station or the 26
  /// frozen nodes; only [_clearStation] or explicit calibration may do that.
  void _resetRecoveryVerification({bool clearTrustedReference = false}) {
    _recoveryVerificationPending = false;
    _recoveryStartedAt = null;
    _recoveryStableSince = null;
    _recoveryStableStationLocalCamera = null;
    _recoveryWorldCorrectionLogged = false;
    if (clearTrustedReference) {
      _lastTrustedStationAnchor = null;
      _lastTrustedStationLocalCamera = null;
    }
  }

  void _beginRecoveryVerification({
    required String reason,
    required int? epoch,
  }) {
    if (_recoveryVerificationPending || _stationReferenceInvalidated) return;
    _recoveryVerificationPending = true;
    _recoveryStartedAt = _clock();
    _recoveryStableSince = null;
    _recoveryStableStationLocalCamera = null;
    _recoveryWorldCorrectionLogged = false;
    _smoother.reset();
    _engine.resetStability();
    _resetGuidanceHistory();
    _resetStationDriftWarningLatch();
    debugPrint(
      'ARCORE_RECOVERY: continuity boundary reason=$reason '
      'epoch=${epoch?.toString() ?? 'unreported'} '
      'trustedAnchor=${_lastTrustedStationAnchor?.id ?? 'none'} '
      'trustedCameraLocal='
      '${_lastTrustedStationLocalCamera == null ? 'none' : _formatVec(_lastTrustedStationLocalCamera!)}. '
      'Guidance and automatic capture are blocked pending verification.',
    );
    _traceStationDiag(
      'recovering:boundary',
      'continuity boundary: $reason. Capture and guidance are blocked.',
    );
  }

  bool _didTrackingEpochChange(ArPoseSample pose) {
    final epoch = pose.trackingEpoch;
    if (epoch == null) return false;
    final previous = _lastTrackingEpoch;
    _lastTrackingEpoch = epoch;
    return previous != null && epoch != previous;
  }

  /// One `ARCORE_STATION_DIAG` line per **change** of the station-reference
  /// trust state — `trusted`, `recovering:*`, `invalidated`. Transition-only:
  /// a steady session logs nothing, and each line carries the epoch, the
  /// expected anchor id and the placement state so a device log can answer
  /// "which reference did the app trust, and when did it stop?".
  ///
  /// The wall-clock milliseconds and the last ARCore frame timestamp ride on
  /// every line: correlating a log line with a screen recording requires both
  /// clocks, because the recording has neither.
  String? _lastStationTrustDiag;

  /// The most recent ARCore frame timestamp seen, in milliseconds, and the
  /// tracking failure reason that came with it. Set in `_onPose`; used only
  /// for the diagnostics below.
  int? _lastPoseFrameTsMs;
  String? _lastPoseFailureReason;

  void _traceStationDiag(String trustState, String detail) {
    if (trustState == _lastStationTrustDiag) return;
    _lastStationTrustDiag = trustState;
    debugPrint(
      'ARCORE_STATION_DIAG: $trustState '
      'wallMs=${_clock().millisecondsSinceEpoch} '
      'frameTsMs=${_lastPoseFrameTsMs?.toString() ?? 'n/a'} '
      'failureReason=${_lastPoseFailureReason ?? 'none'} '
      'epoch=${_lastTrackingEpoch?.toString() ?? 'unreported'} '
      'anchor=${_placement?.anchor.id ?? 'none'} '
      'anchored=${state.activeAnchored} '
      'invalidated=$_stationReferenceInvalidated. $detail',
    );
  }

  /// Creates continuity evidence while the already-calibrated station is live
  /// and tracked. During the temporary distance-gate bypass, distance is still
  /// recorded but must not discard otherwise valid recovery evidence.
  void _recordTrustedStationReference({
    required String anchorId,
    required ({Vec3 position, Quat orientation, TrackingState tracking}) anchor,
    required Vec3 stationLocalCamera,
  }) {
    if (_recoveryVerificationPending ||
        _stationReferenceInvalidated ||
        anchor.tracking != TrackingState.tracking ||
        (!kStationDriftGateBypassed &&
            stationLocalCamera.length > state.geometry.stationDriftWarningM)) {
      return;
    }
    _lastTrustedStationAnchor = _StationAnchorTransform(
      id: anchorId,
      position: anchor.position,
      orientation: anchor.orientation,
    );
    _lastTrustedStationLocalCamera = stationLocalCamera;
  }

  bool _verifyRecoveryReference({
    required DateTime now,
    required StationFrame frame,
    required String expectedAnchorId,
    required ({Vec3 position, Quat orientation, TrackingState tracking})? anchor,
    required Vec3? stationLocalCamera,
    required bool poseFresh,
  }) {
    if (!_recoveryVerificationPending) return true;
    final startedAt = _recoveryStartedAt ?? now;
    _recoveryStartedAt ??= now;

    void failIfTimedOut(String reason) {
      if (now.difference(startedAt) >= kRecoveryVerificationTimeout) {
        _invalidateStationReference(
          'AR could not verify the station reference after tracking recovered '
          '($reason). Recalibrate at the station to continue.',
        );
      }
    }

    if (!poseFresh ||
        anchor == null ||
        anchor.tracking != TrackingState.tracking ||
        stationLocalCamera == null) {
      _recoveryStableSince = null;
      _recoveryStableStationLocalCamera = null;
      failIfTimedOut(
        poseFresh
            ? 'the station anchor did not become tracked'
            : 'the recovered camera pose was stale',
      );
      return false;
    }

    final trustedAnchor = _lastTrustedStationAnchor;
    if (trustedAnchor != null) {
      final sameAnchor = trustedAnchor.id == expectedAnchorId;
      if (!sameAnchor) {
        _invalidateStationReference(
          'The recovered station anchor is not the calibrated anchor. '
          'Recalibrate at the station to continue.',
        );
        return false;
      }
      final anchorShiftM = (anchor.position - trustedAnchor.position).length;
      final anchorRotationDeg = _quaternionAngleDeg(
        anchor.orientation,
        trustedAnchor.orientation,
      );
      if (anchorShiftM > kRecoveryAnchorPositionDiscontinuityM ||
          anchorRotationDeg > kRecoveryAnchorOrientationDiscontinuityDeg) {
        // A global ARCore map correction can move this world-space anchor and
        // the camera together. That preserves the station-local camera pose
        // and therefore preserves the frozen local target geometry. Log the
        // correction, reset the eventual capture window below, and let the
        // station-local position check decide whether the reference is still
        // continuous rather than treating every relocalisation as a reset.
        if (!_recoveryWorldCorrectionLogged) {
          _recoveryWorldCorrectionLogged = true;
          debugPrint(
            'ARCORE_RECOVERY: station anchor world transform corrected '
            '(${anchorShiftM.toStringAsFixed(3)} m, '
            '${anchorRotationDeg.toStringAsFixed(1)}°); validating the '
            'camera-to-station relation before guidance resumes.',
          );
        }
      }
      final trustedCamera = _lastTrustedStationLocalCamera;
      if (trustedCamera != null) {
        final maximumPlausibleJump = _maximumPlausibleRecoveryCameraJump;
        final cameraJumpM = (stationLocalCamera - trustedCamera).length;
        if (cameraJumpM > maximumPlausibleJump) {
          if (sameAnchor) {
            // The same anchor object is still held and tracked, so ARCore
            // itself vouches for its world identity. A large relative move is
            // then user translation across the pause — the tracked log shows
            // drift rising 0.137 → 0.205 → 0.304 m while walking to the next
            // target — not a coordinate-frame change. Reseed the comparison
            // point and restart the hold; the 0.20 m drift gate still blocks
            // capture until the phone is genuinely back, so nothing unsafe is
            // smuggled through.
            _recoveryStableSince = null;
            _recoveryStableStationLocalCamera = stationLocalCamera;
            debugPrint(
              'ARCORE_STATION_DIAG: ${cameraJumpM.toStringAsFixed(3)} m '
              'camera move while recovering with the same station anchor '
              'held; treating it as translation and restarting the '
              'stability hold.',
            );
            return false;
          }
          _invalidateStationReference(
            'AR reported a ${cameraJumpM.toStringAsFixed(3)} m camera-pose '
            'discontinuity while validating the station reference. '
            'Recalibrate at the station to continue.',
          );
          return false;
        }
      }
    }

    final stableCamera = _recoveryStableStationLocalCamera;
    if (stableCamera == null ||
        (stationLocalCamera - stableCamera).length >
            state.geometry.maxPositionDriftM) {
      _recoveryStableStationLocalCamera = stationLocalCamera;
      _recoveryStableSince = now;
      return false;
    }
    final stableSince = _recoveryStableSince ?? now;
    _recoveryStableSince ??= now;
    if (now.difference(stableSince) < kRecoveryVerificationHold) {
      return false;
    }

    _recoveryVerificationPending = false;
    _recoveryStartedAt = null;
    _recoveryStableSince = null;
    _recoveryStableStationLocalCamera = null;
    _engine.resetStability();
    _smoother.reset();
    _resetGuidanceHistory();
    _recordTrustedStationReference(
      anchorId: expectedAnchorId,
      anchor: anchor,
      stationLocalCamera: stationLocalCamera,
    );
    debugPrint(
      'ARCORE_RECOVERY: station reference verified after '
      '${kRecoveryVerificationHold.inMilliseconds} ms; '
      'station=${_formatVec(frame.origin)} anchor=${_formatVec(anchor.position)} '
      'cameraLocal=${_formatVec(stationLocalCamera)}. '
      'A new capture stability window is required.',
    );
    _traceStationDiag(
      'trusted',
      'reference verified; the frozen station and all 26 target positions '
      'are unchanged.',
    );
    return true;
  }

  /// A camera can move from one allowed edge of the 20 cm station circle to
  /// the other, plus the existing 3 cm positional-stability allowance, without
  /// proving that ARCore changed coordinate frames. Anything larger between
  /// consecutive trusted samples is not a plausible in-station movement and is
  /// treated as a VIO pose discontinuity rather than as user drift.
  double get _maximumPlausibleRecoveryCameraJump =>
      state.geometry.stationDriftWarningM * 2 +
      state.geometry.maxPositionDriftM;

  /// Explicitly abandons a reference that no longer has a demonstrably
  /// continuous ARCore world frame. This is intentionally not a recalibration:
  /// the user must press Calibrate after the old scene has been released.
  void _invalidateStationReference(String message) {
    if (_stationReferenceInvalidated || _stationInvalidationInFlight) return;
    _stationReferenceInvalidated = true;
    _stationInvalidationInFlight = true;
    final frozenCount = _frozenNodePositions.length;
    final capturedCount = state.capturedCount;
    final retainedTargets = state.targets;
    final station = _frame;
    debugPrint(
      'ARCORE_RECOVERY: station invalidated; '
      'origin=${station == null ? 'none' : _formatVec(station.origin)} '
      'frozenTargets=$frozenCount. $message',
    );
    _traceStationDiag(
      'invalidated',
      'the frozen reference was abandoned; recalibration is required. '
      'Captured photographs and records are kept.',
    );
    _placement = null;
    _clearStation();
    // These methods synchronously clear the service's local anchor maps before
    // their first await, so a later explicit calibration cannot reuse a stale
    // station anchor. The UI is disabled until both host requests settle.
    final release = _anchors.releaseAll();
    final clearScene = _anchors.clearScene();
    unawaited(_finishStationInvalidation(release, clearScene));
    _emit(
      state.copyWith(
        phase: CapturePhase.scanning,
        clearStation: true,
        // Keep the saved target statuses visible while the former AR world
        // frame is unavailable.  This is display-only: `station == null`,
        // `activeTargetId == 0`, and the scanning phase keep guidance and
        // capture disabled until the user explicitly recalibrates.  Clearing
        // this list here made a partially captured but recoverable session
        // look as if every successfully persisted photograph had been lost.
        targets: retainedTargets,
        activeTargetId: 0,
        clearAlignment: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchored: false,
        activeAnchorTracking: false,
        activeSurfaceType: null,
        driftM: 0,
        stationDriftWarningActive: false,
        trackingRecovering: false,
        message: capturedCount == 0
            ? message
            : '$message $capturedCount of ${retainedTargets.length} '
                  'captured photographs are safely saved; use '
                  '“Recalibrate & continue” to resume.',
      ),
    );
  }

  Future<void> _finishStationInvalidation(
    Future<void> release,
    Future<void> clearScene,
  ) async {
    try {
      await _session.setActiveAnchor(null);
      await clearScene;
      await release;
    } catch (error) {
      debugPrint('ARCORE_RECOVERY: invalid station cleanup failed: $error');
    } finally {
      _stationInvalidationInFlight = false;
      if (!_disposed) _emit(state);
    }
  }

  /// Drops every piece of guidance history that belongs to the previous target
  /// or the previous pose authority.
  ///
  /// Called on a target change, on a tracking loss, and on calibration. The
  /// pointer filter would otherwise blend the old target's direction into the
  /// new one for a few frames, and the hysteresis would hold an `aligned`
  /// indicator across a target the user is no longer aiming at — the two
  /// contradictions the brief calls out by name.
  void _resetGuidanceHistory() {
    _pointerFilter.reset();
    _lastGuidance = null;
    _alignedHeldMs = 0;
    _alignedSinceNs = null;
    _autoStage = AutoCaptureStage.idle;
    _autoBlockReason = null;
    // The *trace* latches are deliberately left alone: they exist to suppress
    // repeats, and clearing them here would re-print the same transition on
    // every frame of a tracking gap.
  }

  /// How full the green ring is: the hold's elapsed time over the window.
  ///
  /// The clamp is not cosmetic. A hold whose oldest sample arrived 700 ms ago
  /// reads `700/600`, and a ring drawn from a value above 1 overflows whatever
  /// it is painted into. The ring filling to 100% and the window elapsing are
  /// therefore the same instant, by the same division.
  double _stabilityProgressOf(vio.FrameGateEvaluation gate) {
    final windowMs = state.geometry.stabilityWindow.inMilliseconds;
    if (windowMs <= 0) return gate.isPoseSteady ? 1 : 0;
    return (gate.windowHeldMs / windowMs).clamp(0.0, 1.0);
  }

  /// The user must hold the actual target inside the strict alignment cone as
  /// well as keep the phone stable. The engine owns the motion half; this
  /// controller-owned half prevents an old, off-target stable window from
  /// instantly firing when the reticle first crosses the target.
  double _alignmentHoldProgressOf() {
    final windowMs = state.geometry.stabilityWindow.inMilliseconds;
    if (windowMs <= 0) return 1;
    return (_alignedHeldMs / windowMs).clamp(0.0, 1.0);
  }

  /// The ring is a capture hold, not merely a motion hold. It may advance only
  /// as far as both the raw VIO stability window and the strict alignment hold
  /// have advanced.
  double _captureHoldProgressOf(vio.FrameGateEvaluation gate) => math.min(
    _stabilityProgressOf(gate),
    _alignmentHoldProgressOf(),
  ).toDouble();

  void _resetStationDriftWarningLatch() {
    _stationDriftBelowRecoverySince = null;
  }

  /// Publishes the station-distance warning from the raw gate with hysteresis.
  /// The raw [CaptureState.driftM] is never filtered and remains the capture
  /// authority. A display warning enters immediately at 0.20 m, but cannot
  /// clear until the raw reading has remained at or below 0.18 m for 500 ms.
  /// This keeps a noisy threshold crossing from repeatedly repainting the
  /// marker amber/green while making the recovery more conservative, never
  /// more permissive, than the existing raw capture gate.
  CaptureState _withStationDriftWarning({
    required CaptureState next,
    required bool trackingValid,
  }) {
    if (!trackingValid) {
      _resetStationDriftWarningLatch();
      if (!next.stationDriftWarningActive) return next;
      return next.copyWith(stationDriftWarningActive: false);
    }

    // **Temporary bypass** (`kStationDriftGateBypassed`): the distance no
    // longer blocks capture and the warning is hidden, but the raw crossing
    // must stay visible for the drift investigation. The latch below can
    // therefore never activate — which suppresses the warning text, the chip
    // and the tint through their existing readers — and each *change* of the
    // raw crossing state is reported here instead, transition-based.
    if (kStationDriftGateBypassed) {
      final crossed = next.driftM > next.geometry.stationDriftWarningM;
      if (crossed != _bypassedDriftCrossingLogged) {
        _bypassedDriftCrossingLogged = crossed;
        debugPrint(
          crossed
              ? 'ARCORE_STATION_DIAG: drift gate bypassed; '
                  'raw=${next.driftM.toStringAsFixed(3)} m exceeds the '
                  '${next.geometry.stationDriftWarningM.toStringAsFixed(2)} m '
                  'limit — capture allowed by the temporary bypass.'
              : 'ARCORE_STATION_DIAG: drift gate bypassed; '
                  'raw=${next.driftM.toStringAsFixed(3)} m back inside the '
                  '${next.geometry.stationDriftWarningM.toStringAsFixed(2)} m '
                  'limit.',
        );
      }
      if (!next.stationDriftWarningActive) return next;
      return next.copyWith(stationDriftWarningActive: false);
    }

    if (next.stationDrifted) {
      _resetStationDriftWarningLatch();
      if (next.stationDriftWarningActive) return next;
      debugPrint(
        'ARCORE_STATION: station drift warning entered; '
        'raw=${next.driftM.toStringAsFixed(3)} m '
        '(limit ${next.geometry.stationDriftWarningM.toStringAsFixed(2)} m).',
      );
      return next.copyWith(stationDriftWarningActive: true);
    }

    if (!next.stationDriftWarningActive) return next;
    final recoveryThreshold =
        next.geometry.stationDriftWarningM -
        kStationDriftWarningRecoveryMarginM;
    if (next.driftM > recoveryThreshold) {
      _resetStationDriftWarningLatch();
      return next;
    }

    final now = _clock();
    final belowSince = _stationDriftBelowRecoverySince ??= now;
    if (now.difference(belowSince) < kStationDriftWarningRecoveryHold) {
      return next;
    }

    debugPrint(
      'ARCORE_STATION: station drift warning cleared after '
      '${kStationDriftWarningRecoveryHold.inMilliseconds} ms; '
      'raw=${next.driftM.toStringAsFixed(3)} m '
      '(recovery threshold ${recoveryThreshold.toStringAsFixed(2)} m).',
    );
    _resetStationDriftWarningLatch();
    return next.copyWith(stationDriftWarningActive: false);
  }

  /// The 26 planned directions, fixed and shared by every station.
  final List<CaptureTargetSpec> _specs;

  /// Whether to move to the next pending target after a successful capture.
  final bool autoAdvance;

  /// Who fires the shutter. See [CaptureMode].
  ///
  /// **Mutable, unlike [autoAdvance]**, because the brief asks for a mode
  /// selector the user can flip mid-session. It is the one setting the
  /// automatic-capture machine consults first: in [CaptureMode.manual] the
  /// machine never arms, and the button is the only path to [_shoot].
  ///
  /// Manual is the safe default for a new station. Automatic capture remains
  /// available from the mode selector, but a user must opt in after ARCore has
  /// settled the room; this prevents a newly placed or refined target from
  /// firing and advancing without an operator's confirmation.
  CaptureMode get mode => _mode;
  CaptureMode _mode;

  /// The automatic-capture machine's published stage.
  AutoCaptureStage get autoStage => _autoStage;
  AutoCaptureStage _autoStage = AutoCaptureStage.idle;

  /// The machine-readable reason automatic capture is blocked, or `null`.
  String? get autoBlockReason => _autoBlockReason;
  String? _autoBlockReason;

  /// The target a shutter request is currently in flight for, or `0`.
  ///
  /// The per-target half of the duplicate guard: [_captureInFlight] says "some
  /// capture is running", this says *which* target owns it. A pose event for a
  /// different target can then be refused without waiting for the first
  /// transaction to finish.
  int _capturingTargetId = 0;

  /// The guidance state the previous frame settled on, for the indicator's
  /// hysteresis. Reset whenever the active target or the pose authority
  /// changes.
  GuidanceState? _lastGuidance;

  /// How long the alignment error has been continuously inside tolerance,
  /// measured on ARCore's frame clock so it cannot span a wall-clock change or
  /// a stale event delivery.
  double _alignedHeldMs = 0;
  int? _alignedSinceNs;

  /// Display-only smoothing of the arrow's direction. Never read by the gate.
  // A gentler display-only low-pass removes small VIO/anchor refinements from
  // the aim arrow. Its existing 4° deviation bound still snaps immediately to
  // a real turn, and the raw direction remains the only capture authority.
  final DisplayDirectionFilter _pointerFilter = DisplayDirectionFilter(
    alpha: 0.25,
  );

  /// The frozen world positions of the 26 targets, captured at calibration.
  ///
  /// Automatic capture requires that "the target world positions remain
  /// unchanged"; this is the snapshot that requirement is checked against, so a
  /// silent rewrite of the sphere (a second `calibrateWithReference`, a
  /// re-placement that moved a node) blocks the shutter instead of quietly
  /// aiming at a different layout.
  List<Vec3> _frozenNodePositions = const <Vec3>[];

  /// What the screen paints.
  CaptureState state;

  StreamSubscription<dynamic>? _poseSub;
  StreamSubscription<dynamic>? _diagSub;

  StationFrame? _frame;

  /// Pre-loaded meshes, when the caller supplied them; otherwise loaded from the
  /// asset bundle on boot.
  GlbMeshSource? _meshes;
  AnchoredTarget? _placement;
  DateTime _lastAnchorRefresh = DateTime.fromMillisecondsSinceEpoch(0);

  /// Whether the "waiting for AR to stabilise the target" line is already up
  /// for the current pause: `1` once it has been said, `0` while tracking.
  ///
  /// It used to count refreshes towards a re-placement. It does not any more:
  /// a pause is a tracking state to wait out, not a countdown to destroying
  /// and rebuilding an anchor that was about to be fine.
  int _anchorNotTracking = 0;

  /// How many times the current target has been re-placed because ARCore
  /// would not hold it. Bounded by [kAnchorReplacementsMax].
  int _anchorReplacements = 0;

  /// Set when the recovery budget for one target is spent: placement has been
  /// reported as failed and the loop is stopped until the user retries or
  /// picks another target.
  bool _anchorGiveUp = false;

  /// True while a placement — [selectTarget], and therefore `rescanActive` and
  /// the auto-advance — is between releasing the old anchor and publishing
  /// the new one.
  ///
  /// Placement is several awaits deep, and the pose stream keeps delivering
  /// through every one of them. Without this flag, a pose that lands
  /// mid-placement drives [_refreshAnchorIfDue] into its "the anchor is gone"
  /// branch — true only because the placement itself just released it — and
  /// that starts a **second, concurrent placement of the same target**: two
  /// anchors, two scene pushes, and the scene can reference the anchor the
  /// state flags do not describe. A paused anchor in the scene while Dart
  /// reports `activeAnchorTracking: true` is drawn as nothing at all, which is
  /// the "the first GLB disappears and the next one never appears" bug.
  bool _placing = false;

  /// A target the user asked for while another placement was in flight. One
  /// slot: the last request wins, and it runs after the in-flight placement
  /// completes rather than interleaving releases and creations with it.
  int? _pendingSelection;

  /// The target ids the last scene push left **out** of the frame.
  ///
  /// The scene is only re-pushed when this set changes, so a pose stream at
  /// 30–60 Hz does not turn into 30–60 scene pushes a second: hiding and
  /// restoring a target is one push per transition, not one per frame.
  Set<int> _hiddenTargets = const <int>{};

  /// The tracking authority the last scene push drew with.
  ///
  /// Authority needs **both** the active anchor and the camera pose to be
  /// tracked: a paused camera makes every projected world point stale, even
  /// when ARCore still reports the anchor as tracked. Such a target is hidden
  /// until its pose is trustworthy again, so this transition is also a scene
  /// push. Like
  /// [_hiddenTargets], it is compared per pose and pushed once per
  /// transition — never once per frame.
  bool _sceneTracking = true;

  /// Whether the native marker currently carries the stable station-distance
  /// warning. This mirrors [CaptureState.stationDriftWarningActive], not the
  /// raw shutter gate, so a value hovering at 20 cm does not make the marker
  /// flicker amber/idle.
  ///
  /// The AR scene is expensive to publish, so it is updated only on visible
  /// transitions. Without this value a marker could stay green after the
  /// shutter had become blocked merely because no anchor-tracking or hidden-set
  /// transition happened on that pose.
  bool _sceneStationDriftWarningActive = false;

  /// The station-anchor/target identity at the start of the current
  /// trustworthy hold.
  ///
  /// The native renderer uses a live `anchor.pose` each frame. If that pose is
  /// adjusted by ARCore, the reticle correctly follows it, but the previous
  /// 600 ms camera-only stability window is no longer evidence that the camera
  /// has been held on the same physical target. This reference lets the
  /// controller discard that window and require a fresh settled hold.
  String? _anchorStabilityId;

  /// The target world position the current capture hold was validated against;
  /// the baseline `_hasAnchorRefined` measures refinement against. The
  /// identity includes the target because all 26 nodes share one station anchor.
  Vec3? _anchorStabilityReference;

  /// User-facing drift warning hysteresis. Raw drift remains in [state.driftM]
  /// and remains the authority for the 0.20 m capture gate.
  DateTime? _stationDriftBelowRecoverySince;

  /// The last raw-drift crossing state reported under the temporary
  /// `kStationDriftGateBypassed` bypass, so the DIAG line is transition-based.
  bool _bypassedDriftCrossingLogged = false;

  /// Last pose sample written to the optional pose trace, and the tracking
  /// states it described. Used only to keep the trace at 2 Hz while still
  /// printing every transition.
  DateTime? _lastPoseTraceAt;
  TrackingState? _tracedCameraTracking;
  bool? _tracedAnchorTracking;

  /// The last guidance state the log described, so a transition is one line and
  /// a steady aim is none. `null` until the first pose is evaluated.
  GuidanceState? _tracedGuidance;

  /// The last automatic-capture block reason the log described.
  ///
  /// The machine re-evaluates twelve conditions on every pose — 30–60 times a
  /// second — and printing each verdict would bury the transitions that matter.
  /// One line per *change of reason*, plus the trigger itself, is what a device
  /// log needs.
  String? _tracedAutoBlock;

  /// The last stability rejection the log described, so a window state is one
  /// line when it *changes* and none while it holds. `null` matches both
  /// "never traced" and "not rejecting" — the first trace after boot emits
  /// once either way, which is the honest baseline.
  String? _tracedStabilityRejection;

  /// Whether a capture transaction is in flight.
  ///
  /// **Latch, not a check.** `_shoot` sets it synchronously before its first
  /// await, so by the time any other caller — the shutter button, the auto
  /// path on the next pose, a re-entrant callback — can run, the flag is
  /// already true. `PhotoCaptureService.busy` guards the same thing one layer
  /// down; this one exists because the controller has *two* entry points and
  /// one of them used to read `state.capturing`, which is a *published* flag
  /// and can therefore be a frame behind.
  bool _captureInFlight = false;

  bool _disposed = false;
  bool _shuttingDown = false;

  /// Stations that already have photographs on disk, newest first. Read at
  /// boot and after each capture, so the resume entry point always reflects
  /// what a restart would find.
  List<String> _savedStationIds = const <String>[];

  /// The stations with saved photographs, for the resume entry point.
  List<String> get savedStationIds => _savedStationIds;

  /// The station a resume would continue, or `null` when there is none.
  String? get resumableStationId =>
      _savedStationIds.isEmpty ? null : _savedStationIds.first;

  /// Whether saved coordinates can still be trusted: true only while the AR
  /// session that recorded them is the live one. A new session means a new
  /// world origin, so continuing requires [recalibrateAndContinue].
  bool get resumeKeepsWorldFrame =>
      _lastSessionReused && !_stationReferenceInvalidated;

  /// The last surface size the AR texture was created at, so a session restart
  /// can rebuild it at the same size without waiting for another layout pass.
  int _surfaceWidth = 720;
  int _surfaceHeight = 1280;

  /// Whether the last `start()` reused the live session.
  ///
  /// A reused session keeps ARCore's world origin, so a calibrated station is
  /// still valid. A *new* session has a new origin and the station is not.
  bool _lastSessionReused = false;

  /// The 26 planned directions, for the spherical overview.
  List<CaptureTargetSpec> get specs => _specs;

  // ── boot ──────────────────────────────────────────────────────────────

  /// Probes the device, starts the session and uploads the GLB meshes.
  ///
  /// Never throws into the UI: every failure mode — no host, unsupported device,
  /// no ARCore service, camera denied — is a [CapturePhase.unavailable] state
  /// with the sentence the user needs. A stack trace is not a diagnosis.
  Future<void> boot({required int width, required int height}) async {
    _surfaceWidth = width;
    _surfaceHeight = height;
    // A second `boot()` must not leave the first pose subscription alive: two
    // listeners would advance the stability gate twice and could fire the
    // shutter twice for one target.
    await _poseSub?.cancel();
    _poseSub = null;
    await _diagSub?.cancel();
    _diagSub = null;
    _lastPoseReceivedAt = null;
    _lastTrackingEpoch = null;
    _resetStationDriftWarningLatch();
    _emit(
      state.copyWith(
        phase: CapturePhase.checking,
        message: 'Checking AR support…',
        clearError: true,
        clearArFault: true,
        clearCameraQuality: true,
        stationDriftWarningActive: false,
      ),
    );
    try {
      final capability = await _session.probe();
      if (!capability.usable) {
        _emit(
          state.copyWith(
            phase: CapturePhase.unavailable,
            capability: capability,
            message: capability.message,
          ),
        );
        return;
      }

      _emit(
        state.copyWith(phase: CapturePhase.starting, capability: capability),
      );

      await _loadMeshes();
      // Applied on every boot, not only when the assets are loaded: a
      // controller constructed with preloaded meshes skips `_loadMeshes`
      // entirely, and the marker then drew at the authored 1.0 — the
      // oversized disk again.
      _applyMarkerScale();
      final session = await _session.start(width: width, height: height);
      _lastSessionReused = session.reused;
      debugPrint(
        'ARCORE_SESSION: capture session started '
        '${session.reused ? 'reused' : 'fresh'} (render surface '
        '${session.textureId}).',
      );
      await _uploadMeshes();

      _poseSub = _session.poses.listen(_onPose);
      _diagSub = _session.diagnostics.listen(_onDiagnostic);

      // Which stations already have photographs on disk, so the screen can
      // offer "continue where you left off" instead of a silent restart.
      _savedStationIds = await _repository.stationIds();

      _emit(
        state.copyWith(
          phase: CapturePhase.scanning,
          textureId: session.textureId,
          message: 'Move the phone slowly to initialize AR tracking.',
        ),
      );
    } on CaptureFailure catch (f) {
      debugPrint('ARCORE_SESSION: boot failed (CaptureFailure): ${f.message}');
      _emit(
        state.copyWith(phase: CapturePhase.unavailable, message: f.message),
      );
    } on ArCoreFailure catch (f) {
      debugPrint('ARCORE_SESSION: boot failed (ArCoreFailure): ${f.message}');
      _emit(
        state.copyWith(phase: CapturePhase.unavailable, message: f.message),
      );
    } catch (e) {
      debugPrint('ARCORE_SESSION: boot failed (unexpected): $e');
      _emit(
        state.copyWith(
          phase: CapturePhase.unavailable,
          message: 'AR could not start: $e',
        ),
      );
    }
  }

  Future<void> _loadMeshes() async {
    if (_meshes != null) return;
    _meshes = await GlbMeshSource.load();
    _applyMarkerScale();
    _emit(
      state.copyWith(
        assetsMatchSpec: _meshes!.matchesSpec(),
        meshTriangles: _meshes!.targetTriangles + _meshes!.bezelTriangles,
      ),
    );
  }

  /// Uploads the two meshes once. The geometry is the shipped assets' own,
  /// flattened by the Dart glTF parser — no re-authoring, no second loader.
  Future<void> _uploadMeshes() async {
    final meshes = _meshes;
    if (meshes == null) return;
    final target = meshes.targetBuffers;
    final bezel = meshes.bezelBuffers;
    await _session.uploadMesh(
      id: PlaneAnchorService.targetMeshId,
      positions: target.positions,
      colors: target.colors,
    );
    await _session.uploadMesh(
      id: PlaneAnchorService.bezelMeshId,
      positions: bezel.positions,
      colors: bezel.colors,
    );
  }

  /// Evaluates the exact preconditions [calibrateStation] needs immediately
  /// before it freezes the origin and yaw. Receipt time and [_clock] share the
  /// same wall-clock base; ARCore's nanosecond timestamp is intentionally not
  /// subtracted from it.
  CalibrationReadiness get calibrationReadiness {
    if (_disposed || _shuttingDown) {
      return const CalibrationReadiness.blocked(
        'AR is not available for calibration right now.',
      );
    }
    if (_stationInvalidationInFlight) {
      return const CalibrationReadiness.blocked(
        'Cleaning the invalid AR reference. Wait a moment before recalibrating.',
      );
    }
    final pose = _session.latest;
    if (pose == null) {
      return const CalibrationReadiness.blocked(
        'No camera pose yet. Wait for AR tracking.',
      );
    }
    if (pose.tracking != TrackingState.tracking) {
      return CalibrationReadiness.blocked(
        'AR is not tracking yet (${pose.tracking.name}). Move the phone '
        'slowly until it is.',
      );
    }
    final receivedAt = _lastPoseReceivedAt;
    if (receivedAt == null ||
        _clock().difference(receivedAt) > state.geometry.maxMeasurementAge) {
      return const CalibrationReadiness.blocked(
        'The camera pose is stale. Wait for a fresh AR frame before '
        'calibrating.',
      );
    }
    final reference = captureReferenceFromPose(
      cameraForward: pose.orientation.rotate(Vec3.unitZ),
    );
    final orientation = reference.orientation;
    if (!reference.ok || orientation == null) {
      return CalibrationReadiness.blocked(
        reference.reason ??
            'The camera has no usable forward direction yet. Wait for AR '
                'tracking.',
      );
    }
    return CalibrationReadiness.ready(
      pose: pose,
      referenceOrientation: orientation,
    );
  }

  // ── station ───────────────────────────────────────────────────────────

  /// Records the current AR camera pose as the station's reference pose.
  ///
  /// Two things are stored, and the difference matters: the **station** pose is
  /// captured once and never changes; the **camera** pose keeps arriving. Every
  /// target direction, every raycast origin and every drift measurement is
  /// relative to the first, never to the second.
  ///
  /// [completedImages] carries the photographs of an earlier visit to the same
  /// station (target id → saved path) when the user has to calibrate again in a
  /// new AR session. The frame is new, so the targets are placed from the new
  /// station pose; the ids, the statuses and the saved files are kept.
  ///
  /// Returns `true` when the station was recorded; `false` when AR was not
  /// ready or the reference could not be derived, in which case [state] carries
  /// the sentence explaining why.
  Future<bool> calibrateStation({
    String? stationId,
    Map<int, String> completedImages = const <int, String>{},
    DateTime? createdAt,
  }) async {
    // One-time calibration: once a station is calibrated, do not recalibrate
    // (or create a second station anchor) during the same session unless the
    // caller explicitly resets via recalibrateHere()/recalibrateAndContinue().
    if (state.station != null) {
      _emit(state.copyWith(message: 'Centre already calibrated.'));
      return true;
    }
    final readiness = calibrationReadiness;
    if (!readiness.isReady) {
      _emit(state.copyWith(message: readiness.reason));
      return false;
    }
    final pose = readiness.pose!;
    final referenceOrientation = readiness.referenceOrientation!;

    final id = stationId ?? 'station-${DateTime.now().millisecondsSinceEpoch}';
    final frame = StationFrame(
      origin: pose.position,
      // The frozen reference: gravity-aligned, pitch- and roll-free, yaw from
      // the direction the user was facing. Recorded once and never replaced
      // by a later camera pose.
      orientation: referenceOrientation,
    );
    // `_freezeStation` already cleared the guidance history — the pointer
    // filter, the aligned latch and the automatic machine's stage. The display
    // smoother is reset here as well because calibration is the explicit
    // "start over" the brief names.
    _freezeStation(frame);
    _smoother.reset();
    _traceTransition(
      'station $id calibrated: P0=${frame.origin} '
      'reference=${frame.orientation} gravityUp=${Vec3.unitY} '
      'forward=${frame.orientation.rotate(Vec3.unitZ)}',
    );
    _tracePose(
      'station-reference set id=$id origin=${_formatVec(frame.origin)} '
      'q=${_formatQuat(frame.orientation)}',
    );

    final station = CaptureStation(
      stationId: id,
      referencePosition: frame.origin,
      referenceOrientation: frame.orientation,
      createdAt: createdAt ?? _clock(),
      trackingStatus: pose.tracking,
      capturedCount: completedImages.length,
      total: _specs.length,
    );

    final targets = <CaptureTarget>[
      for (final spec in _specs)
        CaptureTarget(
          targetId: spec.targetId,
          stationId: id,
          yawDegrees: spec.yawDegrees,
          pitchDegrees: spec.pitchDegrees,
          stationLocalDirection: spec.direction,
          // Photographs from an earlier session survive a recalibration: the
          // station frame is new, but the target ids and the files are not.
          status: completedImages.containsKey(spec.targetId)
              ? CaptureTargetStatus.captured
              : CaptureTargetStatus.pending,
          imagePath: completedImages[spec.targetId],
        ),
    ];

    var firstPending = 0;
    for (final CaptureTarget t in targets) {
      if (!t.status.isDone) {
        firstPending = t.targetId;
        break;
      }
    }

    await _repository.saveStation(station);
    await _repository.saveTargets(targets);
    // A new station record exists now; the resume list must know about it.
    _savedStationIds = await _repository.stationIds();

    _emit(
      state.copyWith(
        phase: CapturePhase.capturing,
        station: station,
        targets: targets,
        activeTargetId: firstPending,
        message: null,
        clearMessage: true,
        clearAlignment: true,
        // A calibration begins a new station-distance measurement. Do not show
        // a stale value from the previous station while the first live anchor
        // report is still arriving.
        driftM: 0,
        stationDriftWarningActive: false,
      ),
    );

    // Probe all 26 directions first — as *information*, never as a gate. The
    // target sheet then says which directions happen to have a surface, but
    // placement and capture do not depend on it: targets are placed at
    // `station + radius × direction` whether or not any surface exists.
    await probeTargets();

    // Place the complete arrangement, once, before the first photograph. A
    // single station anchor is created here and hosts all 26 placements for
    // the whole sequence — one capture origin, not twenty-six independently
    // refinable anchors — and a photograph never adds, removes or moves a
    // node. `placeAll` is idempotent by target id, so a re-run (a failed
    // calibration retried, a recovery after a drop) cannot produce a second
    // node for one target.
    final outcome = await _withPlacementSlot<PlacementOutcome>(
      () => _anchors.placeAll(station: frame, specs: _specs),
    );
    var placedTargets = state.targets;
    for (final entry in outcome.placed.entries) {
      placedTargets = _withAnchorIn(placedTargets, entry.key, entry.value);
    }
    final missing = outcome.failures.keys.toList()..sort();
    _emit(
      state.copyWith(
        targets: placedTargets,
        message: outcome.allPlaced
            ? 'Placed ${outcome.placedCount} targets.'
            : 'Placed ${outcome.placedCount} of ${_specs.length} targets — '
                  '${missing.join(', ')} could not be anchored.',
      ),
    );
    if (!outcome.allPlaced) {
      _traceTransition('placeAll failures: ${outcome.failures}');
    }

    // Start on the first target that still needs a photograph. A station whose
    // records already cover all 26 — a recalibration after the sequence was
    // finished — is complete, not reset to target 1.
    if (firstPending == 0) {
      _emit(state.copyWith(phase: CapturePhase.complete));
      return true;
    }
    await selectTarget(firstPending);
    return true;
  }

  /// Continues a station that already has photographs on disk.
  ///
  /// **Progress comes from the saved records, not from the target rows**: a
  /// target counts as done only when a [CaptureRecord] exists for it in this
  /// station's store, which is the same evidence the review screen lists. The
  /// station pose itself is *not* re-recorded: it is the stored `P0` + reference,
  /// so every target lands exactly where it was.
  ///
  /// **Only valid inside the AR session that recorded the station.** ARCore's
  /// world origin is chosen per session, so the stored coordinates mean
  /// nothing after the session restarts — the same rule [onAppResumed] already
  /// applies when it discards a station across a restart. In a new session the
  /// caller must use [recalibrateAndContinue] instead.
  ///
  /// Returns `false` when the station cannot be resumed (missing, not one of
  /// this app's 26-target stations, or a new AR world frame) so the caller can
  /// say so rather than silently starting empty.
  Future<bool> resumeStation(String stationId) async {
    if (!_lastSessionReused ||
        _stationReferenceInvalidated ||
        _stationInvalidationInFlight) {
      _emit(
        state.copyWith(
          message:
              'The saved AR world frame is no longer trustworthy, so the '
              'station must be recalibrated. Saved photographs are kept.',
        ),
      );
      return false;
    }
    final station = await _repository.loadStation(stationId);
    if (station == null) return false;
    final saved = await _repository.loadTargets(stationId);
    if (saved.length != _specs.length) return false;
    final records = await _repository.loadRecords(stationId);
    final accepted = <int>{for (final CaptureRecord r in records) r.targetId};
    if (accepted.any((int id) => id < 1 || id > _specs.length)) return false;

    _freezeStation(
      StationFrame(
        origin: station.referencePosition,
        orientation: station.referenceOrientation,
      ),
    );
    _tracePose(
      'station-reference resumed id=$stationId '
      'origin=${_formatVec(station.referencePosition)} '
      'q=${_formatQuat(station.referenceOrientation)}',
    );
    _smoother.reset();

    // Rebuild the target rows from the records: done where a record exists,
    // pending everywhere else. Failed attempts are not records, so they read
    // as pending again — the honest state after a restart. The image path comes
    // from the record, not the row: the record is the evidence the file exists.
    final images = <int, String>{
      for (final CaptureRecord r in records) r.targetId: r.imagePath,
    };
    final targets = <CaptureTarget>[
      for (final spec in _specs)
        saved[spec.targetId - 1].copyWith(
          status: accepted.contains(spec.targetId)
              ? CaptureTargetStatus.captured
              : CaptureTargetStatus.pending,
          imagePath: images[spec.targetId],
          clearImage: !accepted.contains(spec.targetId),
          clearMessage: true,
        ),
    ];
    await _repository.saveTargets(targets);

    final capturedCount = accepted.length;
    await _repository.saveStation(
      station.copyWith(capturedCount: capturedCount),
    );

    _emit(
      state.copyWith(
        phase: CapturePhase.capturing,
        station: station.copyWith(capturedCount: capturedCount),
        targets: targets,
        activeTargetId: 0,
        clearAlignment: true,
        clearMessage: true,
        message: 'Resumed: $capturedCount of ${targets.length} saved.',
        driftM: 0,
        stationDriftWarningActive: false,
      ),
    );

    // Place whatever the controller no longer holds after the restart.
    // `placeAll` is idempotent by target id, so this can only add the missing
    // nodes and start on the first target that still needs a photograph.
    final outcome = await _withPlacementSlot(
      () => _anchors.placeAll(station: _frame!, specs: _specs),
    );
    var placedTargets = state.targets;
    for (final entry in outcome.placed.entries) {
      placedTargets = _withAnchorIn(placedTargets, entry.key, entry.value);
    }
    _emit(state.copyWith(targets: placedTargets));
    _traceTransition(
      'resumed station $stationId with $capturedCount saved frames; '
      'placeAll placed ${outcome.placedCount}, failed ${outcome.failureCount}',
    );

    if (capturedCount >= targets.length) {
      _emit(state.copyWith(phase: CapturePhase.complete));
      return true;
    }
    var next = 0;
    for (final t in targets) {
      if (!t.status.isDone) {
        next = t.targetId;
        break;
      }
    }
    if (next != 0) await selectTarget(next);
    return true;
  }

  /// Continues a saved station in a **new** AR session.
  ///
  /// ARCore's world origin is session-scoped, so the stored target positions
  /// cannot be reused — that is why [resumeStation] refuses a new session. The
  /// photographs and their target ids *can* be reused, so this records a new
  /// station pose at the phone's current position and places the arrangement
  /// around it, keeping every saved frame's status and path.
  ///
  /// Returns `false` when the station is missing or AR is not ready to record a
  /// new pose; the state carries the explanation.
  Future<bool> recalibrateAndContinue(String stationId) async {
    final station = await _repository.loadStation(stationId);
    if (station == null) return false;
    final records = await _repository.loadRecords(stationId);
    final images = <int, String>{
      for (final CaptureRecord r in records)
        if (r.targetId >= 1 && r.targetId <= _specs.length)
          r.targetId: r.imagePath,
    };
    // Only tear the current arrangement down once AR can actually record a new
    // station; a failed recalibration must not leave the user with neither
    // frame. The no-pose / not-tracking branches inside `calibrateStation`
    // emit the same sentence they always do.
    final pose = _session.latest;
    if (pose == null || pose.tracking != TrackingState.tracking) {
      return calibrateStation(
        stationId: stationId,
        completedImages: images,
        createdAt: station.createdAt,
      );
    }
    await _releaseArrangement();
    return calibrateStation(
      stationId: stationId,
      completedImages: images,
      createdAt: station.createdAt,
    );
  }

  /// Continues [resumableStationId] the only safe way the current AR session
  /// allows: reuse the saved station when the world frame is still the same,
  /// recalibrate when it is not.
  Future<bool> continueSavedStation() {
    final id = resumableStationId;
    if (id == null) return Future<bool>.value(false);
    return resumeKeepsWorldFrame
        ? resumeStation(id)
        : recalibrateAndContinue(id);
  }

  /// Starts a **new station at the phone's current position**.
  ///
  /// The recovery for "I have moved away from the station". Re-placing the
  /// arrangement around the station it was recorded from puts the targets back
  /// where the user is no longer standing, so it cannot help. The
  /// honest options are to walk back or to re-anchor here, and the app now
  /// offers both: this is the second.
  ///
  /// It is exactly what Calibrate does — a new station record with a new id —
  /// so the previous station's photographs and records are left untouched on
  /// disk; they are simply no longer the current station.
  Future<void> recalibrateHere() async {
    _traceTransition('recalibrate here: releasing the old arrangement');
    await _releaseArrangement();
    await calibrateStation();
  }

  /// Releases the arrangement and forgets the station frame, leaving the
  /// capture flow ready for a calibration at a new pose.
  ///
  /// The anchors are ARCore's, so they cannot outlive a station whose frame is
  /// about to change: re-placing by target id would otherwise treat the old
  /// nodes as already placed and keep them around the old origin.
  Future<void> _releaseArrangement() async {
    await _anchors.releaseAll();
    await _anchors.clearScene();
    await _session.setActiveAnchor(null);
    _placement = null;
    _clearStation();
    _tracePose('station-reference cleared: arrangement released');
    _hiddenTargets = const <int>{};
    _smoother.reset();
    _emit(
      state.copyWith(
        clearStation: true,
        targets: const <CaptureTarget>[],
        activeTargetId: 0,
        clearAlignment: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchored: false,
        activeAnchorTracking: false,
        activeSurfaceType: null,
        driftM: 0,
        stationDriftWarningActive: false,
        // A new station is a new beginning: the recovery notice belongs to the
        // alignment that was cleared with it, not to the next one.
        trackingRecovering: false,
        clearMessage: true,
      ),
    );
  }

  /// Probes all 26 directions and records the result per target.
  ///
  /// No anchors are created: this is a question, not a commitment. The answer
  /// decides which target to start on and what the target list says.
  Future<void> probeTargets() async {
    final frame = _frame;
    if (frame == null) {
      _emit(state.copyWith(message: 'Calibrate the station before scanning.'));
      return;
    }
    try {
      final results = await _session.probeDirections(
        origin: frame.origin,
        directions: <Vec3>[
          for (final CaptureTargetSpec spec in _specs)
            frame.localToWorld(spec.direction).normalized,
        ],
        maxDistanceM: state.geometry.rayDistanceM,
      );
      if (_disposed) return;
      final probes = <int, SurfaceProbe>{};
      for (var i = 0; i < _specs.length && i < results.length; i++) {
        probes[_specs[i].targetId] = results[i];
      }
      final found = probes.values.where((SurfaceProbe p) => p.found).length;
      _emit(
        state.copyWith(
          probes: probes,
          message: probes.isEmpty
              ? 'The scan returned nothing.'
              : '$found of ${probes.length} directions have a surface.',
        ),
      );
    } catch (e) {
      _emit(state.copyWith(message: 'Could not scan the directions: $e'));
    }
  }

  // ── targets ───────────────────────────────────────────────────────────

  /// Makes [targetId] the active target and highlights its already-placed GLB
  /// node.
  ///
  /// **No node is created, moved or released here.** The whole 26-target
  /// arrangement is placed once, at calibration (see
  /// [PlaneAnchorService.placeAll]); selecting a target moves the highlight
  /// and aims the guidance, nothing else. The only placement that happens
  /// after calibration is the bounded recovery of an anchor ARCore has
  /// dropped, and it re-places that one target.
  Future<void> selectTarget(int targetId) async {
    final specIndex = targetId - 1;
    if (specIndex < 0 || specIndex >= _specs.length) return;

    // A completed target is not guidance: its photograph is saved, its marker
    // is out of the scene, and making it active again would put the arrow and
    // the ring back over a place the user is done with. Retake returns the
    // target to `pending` *before* selecting it, so an explicit retake is the
    // one path that can legitimately re-activate it.
    final existing = specIndex < state.targets.length
        ? state.targets[specIndex]
        : null;
    if (existing != null && existing.status.isDone) {
      _traceTransition('target $targetId is captured; selection refused');
      return;
    }

    if (_placing) {
      // A recovery placement is in flight. Queue rather than interleave: two
      // concurrent placements would fight over the scene, and the scene can
      // end up referencing whichever push landed last while the state flags
      // describe the other — a marker that is never drawn.
      _pendingSelection = targetId;
      _traceTransition('queued selection $targetId: a placement is in flight');
      return;
    }
    final frame = _frame;
    final station = state.station;
    if (frame == null || station == null) {
      // This can occur while a photograph is finishing: a tracking recovery
      // may invalidate the station between persisting the successful image and
      // auto-selecting its successor.  A silent return leaves target zero
      // selected with no marker or explanation, which is indistinguishable
      // from a broken auto-advance path.  Do not recreate anything from the
      // current camera pose; make the missing frozen reference explicit and
      // leave automatic capture disarmed.
      _placement = null;
      _engine.resetStability();
      _resetGuidanceHistory();
      _emit(
        state.copyWith(
          targets: _demoteActive(state.targets, except: 0),
          activeTargetId: 0,
          clearAlignment: true,
          stable: false,
          stabilityProgress: 0,
          activeAnchored: false,
          activeAnchorTracking: false,
          retryPending: false,
          autoStage: AutoCaptureStage.waiting,
          autoBlockReason: 'no-station',
          message: 'The station reference is unavailable. Recalibrate at the '
              'station before continuing.',
        ),
      );
      // Keep native rendering and the disarmed Flutter state together. This
      // is deliberately best-effort: if it follows a successfully saved
      // image, an AR teardown error must not turn that saved target into a
      // capture failure.
      try {
        await _pushScene(state);
        await _session.setActiveAnchor(null);
      } catch (e) {
        _traceTransition(
          'station missing while selecting $targetId; scene clear failed: $e',
        );
      }
      return;
    }

    // A genuine target switch starts a fresh recovery budget. The recovery
    // path re-enters this method with the *same* id, and resetting there
    // would make the replacement limit unbounded — the exact hang this class
    // exists to prevent.
    if (targetId != state.activeTargetId) {
      _anchorReplacements = 0;
      _anchorGiveUp = false;
      _anchorNotTracking = 0;
    }

    _engine.resetStability();
    final placement = _anchors.placementFor(targetId);
    _placement = placement;
    _lastAnchorRefresh = _clock();

    // Demote the target that used to be active. Without this, every target
    // the user has ever visited keeps `active` — and the sphere map paints
    // `active` amber, so old targets stay highlighted forever.
    final demoted = _demoteActive(state.targets, except: targetId);

    if (placement == null) {
      // No node for this target: placement was refused at calibration (or the
      // anchor was dropped and not yet recovered). Say so, with the retry
      // action — never claim a marker that is not in the scene.
      debugPrint(
        'ARCORE_TARGET: target $targetId selected but not placed; '
        'no station anchor exists for it.',
      );
      _emit(
        state.copyWith(
          activeTargetId: targetId,
          clearAlignment: true,
          stable: false,
          stabilityProgress: 0,
          activeAnchored: false,
          activeAnchorTracking: false,
          activeSurfaceType: null,
          message: 'This target is not placed — recalibrate the station.',
          // A different target is a different episode: whatever the last
          // attempt did, this one starts unlatched.
          retryPending: false,
        ),
      );
      await _pushScene(state);
      // Nothing to aim at: the target has no node, so the host must not keep
      // reporting the previous target's pose as the active one.
      await _session.setActiveAnchor(null);
      return;
    }

    // A placed anchor is not necessarily a *tracked* one yet: ARCore commonly
    // starts a world anchor paused. It is drawn either way, so the honest state
    // here is "waiting for ARCore to refine it" rather than "not drawn" — and
    // the shutter stays blocked until it does.
    final tracking = placement.isTracking;
    _anchorNotTracking = tracking ? 0 : 1;
    debugPrint(
      'ARCORE_TARGET: target $targetId active; '
      'world=${_formatVec(placement.worldPosition)} '
      'anchor=${placement.anchor.id} tracking=$tracking.',
    );
    // A different target means every piece of guidance state belongs to the
    // wrong target: the pointer filter's estimate, the aligned latch, and the
    // automatic machine's stage. Cleared *before* the emit, so the first frame
    // of the new target cannot inherit the old one's `aligned`.
    _resetGuidanceHistory();
    // Target selection waits for a matching live station anchor before it
    // publishes another distance. Do not carry the previous target's warning
    // through that short hand-off.
    _emit(
      state.copyWith(
        targets: _withAnchorIn(demoted, targetId, placement, setActive: true),
        activeTargetId: targetId,
        clearAlignment: true,
        stable: false,
        stabilityProgress: 0,
        driftM: 0,
        stationDriftWarningActive: false,
        activeAnchored: true,
        activeAnchorTracking: tracking,
        activeSurfaceType: placement.surfaceType,
        message: tracking ? null : 'Waiting for AR to stabilise the target…',
        clearMessage: tracking,
        // A freshly selected target has never been attempted, so it is never
        // waiting on a retry.
        retryPending: false,
      ),
    );
    // Pushed **after** the emit, so the scene is built from the state that
    // already names this target: a push assembled before it can be overtaken
    // by a pose-driven push describing the previous selection, and the
    // highlight then stays on a target the user has left.
    await _pushScene(state);
    // The guidance aims at this anchor's *live* pose from here on, so the host
    // has to know which one that is.
    await _session.setActiveAnchor(placement.anchor.id);
  }

  /// [list] with every `active` target other than [except] returned to
  /// `pending`. Captured and failed targets keep their status — only the
  /// highlight moves.
  List<CaptureTarget> _demoteActive(
    List<CaptureTarget> list, {
    required int except,
  }) {
    var changed = false;
    final updated = List<CaptureTarget>.of(list);
    for (var i = 0; i < updated.length; i++) {
      final t = updated[i];
      if (t.targetId != except && t.status == CaptureTargetStatus.active) {
        updated[i] = t.copyWith(status: CaptureTargetStatus.pending);
        changed = true;
      }
    }
    return changed ? updated : list;
  }

  /// Runs [body] as the one in-flight placement and drains anything queued
  /// behind it.
  ///
  /// Every placement in this controller — the calibration pass, a single
  /// target's recovery, the full arrangement rebuild (`rescanActive`) — goes
  /// through this slot, so a
  /// selection that arrives while anchors are being created queues instead of
  /// interleaving with the creates.
  Future<T> _withPlacementSlot<T>(Future<T> Function() body) async {
    _placing = true;
    try {
      return await body();
    } finally {
      _placing = false;
      final pending = _pendingSelection;
      _pendingSelection = null;
      if (pending != null && pending != state.activeTargetId) {
        // The queued selection runs on a clean slate; if it queues another,
        // its own drain handles it.
        _traceTransition('running queued selection $pending');
        selectTarget(pending);
      }
    }
  }

  /// Re-places **one** target's anchor — the bounded recovery for an anchor
  /// ARCore dropped or will not track. The other 29 nodes are untouched, and
  /// the target stays active: the user keeps aiming at the same direction
  /// while its marker is rebuilt.
  Future<void> _recoverTarget(int targetId) async {
    final frame = _frame;
    if (frame == null || _placing) return;
    final specIndex = targetId - 1;
    if (specIndex < 0 || specIndex >= _specs.length) return;

    _traceTransition('re-placing target $targetId alone');
    await _withPlacementSlot(() async {
      await _anchors.release(targetId);
      final outcome = await _anchors.placeAll(
        station: frame,
        specs: <CaptureTargetSpec>[_specs[specIndex]],
      );
      _lastAnchorRefresh = _clock();
      final placement = outcome.placed[targetId];
      _placement = placement;
      if (placement == null) {
        final reason =
            outcome.failures[targetId] ??
            'The target could not be placed. Recalibrate the station.';
        debugPrint(
          'ARCORE_TARGET: re-placement of target $targetId failed: $reason',
        );
        _emit(
          state.copyWith(
            targets: _withTargetMessage(targetId, reason, clearAnchor: true),
            activeAnchored: false,
            activeAnchorTracking: false,
            activeSurfaceType: null,
            message: 'Place target again',
          ),
        );
        return;
      }
      await _pushScene(state);
      _emit(
        state.copyWith(
          targets: _withAnchor(targetId, placement),
          activeAnchored: true,
          activeAnchorTracking: placement.isTracking,
          activeSurfaceType: placement.surfaceType,
          message: placement.isTracking
              ? null
              : 'Waiting for AR to stabilise the target…',
          clearMessage: placement.isTracking,
        ),
      );
      // A recovery made a *new* anchor for this target, so the host must aim at
      // the new handle — the old one is detached and its pose is gone.
      await _session.setActiveAnchor(placement.anchor.id);
    });
  }

  /// Re-arms the session, drops the stale anchors, and re-places the whole
  /// arrangement from the station pose that is still recorded.
  ///
  /// **Not reachable from the interface.** The post-calibration panel's
  /// "Scan again" button was removed when calibration became a one-time
  /// session event; this stays as the bounded rebuild the tests drive, and as
  /// the code path behind a full arrangement re-place.
  ///
  /// Three things it deliberately does **not** do:
  ///
  /// * restart the AR session — that would give ARCore a new world origin and
  ///   invalidate the station pose, and with it all 26 target directions;
  /// * touch any captured photograph or captured target status;
  /// * claim success when the anchor is refused.
  ///
  /// If tracking is not valid it says so instead of silently doing nothing,
  /// because a re-place cannot hold an anchor while ARCore is paused.
  Future<void> rescanActive() async {
    if (_stationReferenceInvalidated || _stationInvalidationInFlight) {
      _emit(
        state.copyWith(
          message:
              'The AR station reference is invalid. Recalibrate at the station '
              'before placing targets again.',
        ),
      );
      return;
    }
    _engine.resetStability();
    _emit(
      state.copyWith(
        clearAlignment: true,
        stable: false,
        stabilityProgress: 0,
        message: 'Re-arming the session…',
      ),
    );

    try {
      await _session.rescan();
    } catch (e) {
      _emit(
        state.copyWith(
          message: 'The AR session could not be re-armed: $e',
          lastError: '$e',
        ),
      );
      return;
    }

    // The host dropped its anchors, so every local handle is stale either way
    // — and so is the state's claim that the active target is placed. Clear
    // both on every early return below, or the UI would offer a shutter for a
    // marker that no longer exists.
    _placement = null;
    await _anchors.releaseAll();
    await _anchors.clearScene();
    await _session.setActiveAnchor(null);
    // Nothing is drawn from here until the arrangement is rebuilt, so the
    // hidden set must not keep claiming a marker that is in the scene.
    _hiddenTargets = const <int>{};

    if (_stationReferenceInvalidated ||
        _stationInvalidationInFlight ||
        _frame == null ||
        state.station == null) {
      _emit(
        state.copyWith(
          activeAnchored: false,
          activeSurfaceType: null,
          targets: _withTargetMessage(
            state.activeTargetId,
            'Calibrate the station before placing a target.',
            clearAnchor: true,
          ),
          message: 'Calibrate the station before placing a target.',
        ),
      );
      return;
    }
    if (state.tracking != TrackingState.tracking) {
      _emit(
        state.copyWith(
          activeAnchored: false,
          activeSurfaceType: null,
          targets: _withTargetMessage(
            state.activeTargetId,
            'AR is not tracking yet. Move the phone slowly to initialize '
            'tracking.',
            clearAnchor: true,
          ),
          message:
              'AR is not tracking yet. Move the phone slowly to initialize '
              'tracking.',
        ),
      );
      return;
    }

    // Re-place the whole arrangement from the same station: `placeAll` is
    // idempotent, and the map was just cleared, so this creates exactly one
    // node per target. The station — and with it the 26 directions — is not
    // re-recorded; only the nodes are rebuilt.
    final outcome = await _withPlacementSlot<PlacementOutcome>(
      () => _anchors.placeAll(station: _frame!, specs: _specs),
    );
    var placedTargets = state.targets;
    for (final entry in outcome.placed.entries) {
      placedTargets = _withAnchorIn(placedTargets, entry.key, entry.value);
    }
    final missing = outcome.failures.keys.toList()..sort();
    _emit(
      state.copyWith(
        targets: placedTargets,
        message: outcome.allPlaced
            ? 'All ${_specs.length} targets placed again.'
            : 'Placed ${outcome.placedCount} of ${_specs.length} targets — '
                  '${missing.join(', ')} could not be anchored.',
      ),
    );

    final id = state.activeTargetId;
    if (id == 0) return;
    await selectTarget(id);
  }

  /// Restarts the AR session from scratch. The recovery action behind a fatal
  /// render error.
  ///
  /// Unlike [rescanActive] this *does* replace the session, so the station pose
  /// and the captured targets are discarded from the live flow: the new session
  /// has a new world origin, and keeping the old station would silently aim every
  /// target at the wrong place. Photographs already on disk are untouched.
  Future<void> retrySession() async {
    final width = _surfaceWidth;
    final height = _surfaceHeight;
    await onAppPaused();
    await _session.setActiveAnchor(null);
    _clearStation();
    _placement = null;
    _emit(
      state.copyWith(
        phase: CapturePhase.idle,
        clearStation: true,
        targets: const <CaptureTarget>[],
        activeTargetId: 0,
        clearAlignment: true,
        clearArFault: true,
        clearError: true,
        clearMessage: true,
        clearCameraQuality: true,
        stable: false,
        stabilityProgress: 0,
        driftM: 0,
        stationDriftWarningActive: false,
      ),
    );
    await boot(width: width, height: height);
  }

  /// Drops the active target's photograph so it can be taken again.
  ///
  /// Only that target is touched: the station's other records are left exactly
  /// as they were, which is the whole reason a record is keyed by target id.
  Future<void> retake(int targetId) async {
    final index = targetId - 1;
    if (index < 0 || index >= state.targets.length) return;
    final updated = List<CaptureTarget>.of(state.targets);
    updated[index] = updated[index].copyWith(
      status: CaptureTargetStatus.pending,
      clearImage: true,
      clearMessage: true,
    );
    await _repository.saveTargets(updated);
    // The station's saved count follows the targets: a retaken target is no
    // longer saved, and a station record that still counts it would report one
    // more photograph than exists on disk.
    final station = state.station;
    if (station != null) {
      final capturedCount = updated
          .where((CaptureTarget t) => t.status.isDone)
          .length;
      final updatedStation = station.copyWith(capturedCount: capturedCount);
      await _repository.saveStation(updatedStation);
      _emit(
        state.copyWith(
          targets: updated,
          station: updatedStation,
          activeTargetId: targetId,
          // Retaking the last photograph leaves the station in `complete`, and
          // that phase refuses every capture. The target is pending again, so
          // the phase has to follow it back to `capturing` — otherwise the
          // retake selects a target whose shutter is still locked by "Every
          // target has a photograph."
          phase: state.phase == CapturePhase.complete
              ? CapturePhase.capturing
              : state.phase,
        ),
      );
    } else {
      _emit(state.copyWith(targets: updated, activeTargetId: targetId));
    }
    await selectTarget(targetId);
  }

  // ── capture ───────────────────────────────────────────────────────────

  /// Legacy API retained for callers compiled against an earlier build. The
  /// product no longer permits manual shuttering: only the automatic gate can
  /// fire after a stable, aligned pose has held for its required duration.
  Future<void> captureNow() async {
    debugPrint('ARCORE_CAPTURE: ignored manual capture request.');
  }

  /// Automatic capture is the sole supported operating mode.
  void setCaptureMode(CaptureMode next) {
    if (!next.isAutomatic || _mode.isAutomatic) return;
    _mode = CaptureMode.automatic;
    _autoStage = AutoCaptureStage.waiting;
    _autoBlockReason = 'awaiting-frame';
    _tracedAutoBlock = null;
    debugPrint(
      'ARCORE_CAPTURE: capture mode ${_mode.label.toLowerCase()} '
      '(target=${state.activeTargetId}).',
    );
    _emit(state.copyWith(mode: _mode));
  }

  /// The capture transaction, from either entry point.
  ///
  /// [requested] is the target the caller believed was active when it decided
  /// to fire. It is checked against the live selection before the latch is
  /// taken, so a pose that arrived between the decision and this call cannot
  /// shoot a target the user has since moved off.
  Future<void> _shoot({
    required CaptureTriggerSource source,
    CaptureTarget? requested,
  }) async {
    final station = state.station;
    final target = state.activeTarget;
    final alignment = state.alignment;
    final pose = _session.latest;
    if (station == null ||
        target == null ||
        alignment == null ||
        pose == null) {
      return;
    }
    if (requested != null && requested.targetId != target.targetId) return;
    // One transaction per target at a time. The latch is taken *before* the
    // first await so it is already true when any other entry point can run:
    // `state.capturing` is published state and can be a frame behind, and a
    // second shutter inside that window is what produced the fire/fail/fail
    // loop and the "Ready ↔ Saving" flicker.
    if (_captureInFlight || _photos.busy) return;
    // A target that already has a photograph, or one a transaction already owns,
    // is not shootable. This is the second half of the duplicate guard: the
    // latch above covers "another capture is running", these cover "this one
    // already happened".
    if (target.status.isDone || target.status.isCapturing) return;
    if (_capturingTargetId == target.targetId) return;

    final index = state.targets.indexWhere(
      (CaptureTarget t) => t.targetId == target.targetId,
    );
    if (index < 0) return;

    _captureInFlight = true;
    _capturingTargetId = target.targetId;

    // ── the atomic waiting → capturing transition ────────────────────────
    //
    // Emitted synchronously, before the first await, so every observer — the
    // button, the status line, the pose callback on the next frame — sees the
    // target as `capturing` while the photograph is in flight. A second frame
    // that reaches [_autoCaptureVerdict] in that window is refused by
    // `in-progress` rather than by a race.
    final marked = List<CaptureTarget>.of(state.targets);
    marked[index] = marked[index].copyWith(
      status: CaptureTargetStatus.capturing,
      clearMessage: true,
    );
    _autoStage = AutoCaptureStage.capturing;
    _autoBlockReason = 'in-progress';
    _emit(
      state.copyWith(
        targets: marked,
        capturing: true,
        autoStage: AutoCaptureStage.capturing,
        autoBlockReason: 'in-progress',
      ),
    );
    debugPrint(
      'ARCORE_CAPTURE: shutter fired [${source.label}] for target '
      '${target.targetId} '
      '(anchored=${_placement != null}, '
      'error=${alignment.angularErrorDeg.toStringAsFixed(2)} deg, '
      'window=${state.stabilityProgress.toStringAsFixed(2)}, '
      'drift=${state.driftM.toStringAsFixed(3)} m).',
    );
    try {
      await _runCapture(
        station: station,
        target: target,
        alignment: alignment,
        pose: pose,
        placement: _placement,
        source: source,
      );
    } finally {
      _captureInFlight = false;
      _capturingTargetId = 0;
    }
  }

  /// The capture transaction itself, with the latch already held by the caller.
  ///
  /// Split out from [_shoot] so the `try`/`finally` that releases the latch has
  /// one body and cannot be bypassed by an early return.
  Future<void> _runCapture({
    required CaptureStation station,
    required CaptureTarget target,
    required AlignmentResult alignment,
    required ArPoseSample pose,
    required AnchoredTarget? placement,
    required CaptureTriggerSource source,
  }) async {
    _emit(state.copyWith(capturing: true));
    try {
      final record = await _photos.capture(
        stationId: station.stationId,
        target: target,
        camera: RigidPose(
          position: pose.position,
          orientation: pose.orientation,
        ),
        alignment: alignment,
        anchored: placement != null,
        anchorIdentifier: placement?.anchor.id,
        anchorSurfaceType: placement?.surfaceType,
        toleranceDeg: state.geometry.alignmentToleranceDeg,
        intrinsics: state.intrinsics,
        // `takeStill` is fulfilled by the following live ARCore frame. Keep
        // that image tied to the frame that armed the shutter: an EventChannel
        // backlog or a render stall must not turn an old aligned pose into a
        // much later photograph that is then counted as this target.
        shutterPoseTimestampNs: pose.raw.timestampNs,
        maxStillFrameLag: state.geometry.maxMeasurementAge,
      );
      debugPrint(
        'ARCORE_CAPTURE: photograph returned for target '
        '${target.targetId} [${source.label}] (${record.imageWidth}×'
        '${record.imageHeight} ${record.imageFormat}).',
      );

      final index = target.targetId - 1;
      // The write is asynchronous. If the station was reset or replaced while
      // it was in flight, this photograph belongs to the old arrangement:
      // recording it against the rows that now exist would mark a target
      // complete in a world it was never taken in.
      final current = index >= 0 && index < state.targets.length
          ? state.targets[index]
          : null;
      if (current == null || state.station?.stationId != station.stationId) {
        _traceTransition(
          'target ${target.targetId}: capture finished after the station '
          'changed; completion discarded',
        );
        _emit(state.copyWith(capturing: false));
        return;
      }
      // A duplicate success event must not count or advance the sequence a
      // second time.
      if (current.status.isDone) {
        _traceTransition(
          'target ${target.targetId}: duplicate success ignored',
        );
        _emit(state.copyWith(capturing: false));
        return;
      }

      final wasActive = state.activeTargetId == target.targetId;
      final updated = List<CaptureTarget>.of(state.targets);
      updated[index] = updated[index].copyWith(
        status: CaptureTargetStatus.captured,
        imagePath: record.imagePath,
        clearMessage: true,
      );
      await _repository.saveTargets(updated);

      final capturedCount = updated
          .where((CaptureTarget t) => t.status.isDone)
          .length;
      debugPrint(
        'ARCORE_CAPTURE: capture success [${source.label}] target '
        '${target.targetId} -> captured; $capturedCount of '
        '${updated.length} recorded.',
      );
      final updatedStation = station.copyWith(capturedCount: capturedCount);
      await _repository.saveStation(updatedStation);
      // The resume list must reflect what is on disk *now*: the next boot
      // offers "continue" from this station with this count.
      _savedStationIds = await _repository.stationIds();

      // The user may have chosen another target while the JPEG was being
      // written. The photograph still belongs to the target that was shot,
      // but the selection — and its guidance — belongs to the user: record the
      // completion and leave the new selection alone.
      final allDone = capturedCount >= updated.length;
      if (!wasActive) {
        _emit(
          state.copyWith(
            targets: updated,
            station: updatedStation,
            capturing: false,
            // The photograph in flight can be the last one even when the user
            // selected something else during the write. A selection made
            // mid-write survives only while there is still work to guide; a
            // complete station has no active target, and leaving `capturing`
            // here would report a full station as capturable.
            phase: allDone ? CapturePhase.complete : state.phase,
            activeTargetId: allDone ? 0 : null,
            clearAlignment: allDone,
            stable: allDone ? false : null,
            stabilityProgress: allDone ? 0 : null,
            activeAnchored: allDone ? false : null,
            activeAnchorTracking: allDone ? false : null,
            retryPending: allDone ? false : null,
            autoStage: AutoCaptureStage.complete,
            clearAutoBlockReason: true,
            lastSavedPath: record.imagePath,
            imageRotationDeg: record.rotationAppliedDeg,
            message:
                'Saved ${record.imageWidth}×${record.imageHeight} '
                '${record.imageFormat.toUpperCase()} for target '
                '${target.targetId}.',
          ),
        );
        if (!allDone) {
          await _pushScene(state, targets: updated);
          return;
        }
        _placement = null;
        await _pushScene(state);
        await _session.setActiveAnchor(null);
        return;
      }

      final next = !allDone && autoAdvance
          ? _nextPending(updated, after: target.targetId)
          : null;

      // Completion and the removal of the completed target's guidance are one
      // transition. The old flow kept the captured target active (with its
      // 100% alignment) in this emit and only moved on afterwards — or never,
      // when auto-advance was off — so the ring and arrow for a saved target
      // could stay on screen.
      _emit(
        state.copyWith(
          targets: updated,
          station: updatedStation,
          capturing: false,
          // No target is active until the next selection below has installed
          // its anchor and reset alignment. The completed target is therefore
          // never the active guidance target, not even for one frame.
          activeTargetId: 0,
          clearAlignment: true,
          stable: false,
          stabilityProgress: 0,
          activeAnchored: false,
          activeAnchorTracking: false,
          // The selected anchor has just been released.  Its previous station
          // distance is no longer evidence about the next target, and keeping
          // it made the neutral post-capture reticle look like a fresh
          // "return to station" failure.  Selection of the next target
          // recomputes drift only once that target's live anchor is tracked.
          driftM: 0,
          stationDriftWarningActive: false,
          retryPending: false,
          phase: allDone ? CapturePhase.complete : state.phase,
          autoStage: AutoCaptureStage.complete,
          clearAutoBlockReason: true,
          lastSavedPath: record.imagePath,
          imageRotationDeg: record.rotationAppliedDeg,
          message:
              'Saved ${record.imageWidth}×${record.imageHeight} '
              '${record.imageFormat.toUpperCase()} for target '
              '${target.targetId}.',
        ),
      );
      _placement = null;

      if (next == null) {
        // There is no subsequent target to install. Push the terminal scene
        // only here: when a next target exists, selectTarget builds and sends
        // its replacement scene itself. Clearing first creates a visible
        // blank-frame gap where native ARCore has no active marker, which can
        // look like lost guidance after a successful automatic capture.
        await _pushScene(state);
        // Nothing is active: the host must stop reporting an active anchor.
        await _session.setActiveAnchor(null);
        return;
      }
      _traceTransition(
        'target ${target.targetId} accepted; next pending is $next',
      );
      // The image and target-completion rows are already persisted above.
      // Activating the next marker is a separate, best-effort AR scene
      // operation: a platform error here must never fall through to the
      // capture-failure handler below and turn an already-saved photograph
      // back into a failed target.
      try {
        await selectTarget(next);
      } catch (e) {
        _traceTransition(
          'target ${target.targetId} saved, but next target $next could not '
          'be activated: $e',
        );
        _placement = null;
        _engine.resetStability();
        _resetGuidanceHistory();
        _emit(
          state.copyWith(
            targets: _demoteActive(state.targets, except: 0),
            activeTargetId: 0,
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
            activeAnchored: false,
            activeAnchorTracking: false,
            retryPending: false,
            autoStage: AutoCaptureStage.waiting,
            autoBlockReason: 'next-target-transition',
            message: 'Photo saved. AR could not prepare target $next — '
                'open Review targets to choose another.',
          ),
        );
        try {
          await _session.setActiveAnchor(null);
        } catch (_) {
          // The user-facing state above is already safe: no target can auto-
          // capture until a fresh scan installs its active anchor.
        }
      }
    } on CaptureFailure catch (f) {
      debugPrint(
        'ARCORE_CAPTURE: capture failed [${source.label}] target '
        '${target.targetId}: ${f.message} — target returned to retryable.',
      );
      final updated = _withTargetMessage(
        state.activeTargetId,
        f.message,
        status: CaptureTargetStatus.failed,
      );
      // Persisted, not just emitted: a failure that lives only in memory
      // reads as "never attempted" after a resume, and the saved-image count
      // then disagrees with the target list on disk.
      await _repository.saveTargets(updated);
      _emit(
        state.copyWith(
          capturing: false,
          retryPending: true,
          lastError: f.message,
          targets: updated,
          autoStage: AutoCaptureStage.failed,
          autoBlockReason: 'failure-latched',
        ),
      );
      // The attempt failed, so the phase is `retry` and the hidden set drops
      // this target: the marker comes back on its own anchor, at its own
      // position, ready to be aimed at again.
      await _pushScene(state, targets: updated);
    } catch (e) {
      debugPrint(
        'ARCORE_CAPTURE: capture failed [${source.label}] target '
        '${target.targetId}: $e — target returned to retryable.',
      );
      final message = 'Capture failed: $e';
      final updated = _withTargetMessage(
        state.activeTargetId,
        message,
        status: CaptureTargetStatus.failed,
      );
      await _repository.saveTargets(updated);
      _emit(
        state.copyWith(
          capturing: false,
          retryPending: true,
          lastError: message,
          targets: updated,
          autoStage: AutoCaptureStage.failed,
          autoBlockReason: 'failure-latched',
        ),
      );
      // Same restoration as the typed failure above: the marker comes back,
      // and the same latch stops the shutter re-firing on the next pose.
      await _pushScene(state, targets: updated);
    }
  }

  int? _nextPending(List<CaptureTarget> targets, {required int after}) {
    // Position-based, not id-arithmetic. The list order *is* the capture
    // order; stepping by `(after + i) % length` silently assumes
    // `targetId == index + 1`, an invariant nothing enforces — one reorder of
    // the spec list and the sequence would skip or repeat targets. Find the
    // position of `after` and step from there instead.
    final start = targets.indexWhere((CaptureTarget t) => t.targetId == after);
    if (start < 0) return null;
    for (var step = 1; step <= targets.length; step++) {
      final candidate = targets[(start + step) % targets.length];
      if (!candidate.status.isDone) return candidate.targetId;
    }
    return null;
  }

  // ── pose stream ───────────────────────────────────────────────────────

  void _onPose(ArPoseSample pose) {
    if (_disposed || _shuttingDown) return;
    final epochChanged = _didTrackingEpochChange(pose);
    final trackingChanged = pose.tracking != state.tracking;
    final trackingLeft =
        state.tracking == TrackingState.tracking &&
        pose.tracking != TrackingState.tracking;
    final trackingResumed =
        state.tracking != TrackingState.tracking &&
        pose.tracking == TrackingState.tracking;
    // A paused sample is never allowed to keep a pre-pause freshness stamp.
    // The next TRACKING event establishes a new stamp before readiness can
    // become true again.
    _lastPoseReceivedAt = pose.tracking == TrackingState.tracking
        ? _clock()
        : null;

    var next = state.copyWith(
      tracking: pose.tracking,
      trackingFailureReason: pose.failureReason,
      planesTracked: pose.planesTracked,
      planeCount: pose.planeCount,
      walls: pose.walls,
      floors: pose.floors,
      ceilings: pose.ceilings,
      intrinsics: pose.intrinsics ?? state.intrinsics,
      imageAxis: _session.imageAxis ?? state.imageAxis,
      displayProjectionMatrix:
          pose.displayProjectionMatrix ?? state.displayProjectionMatrix,
      // The camera config arrives on exactly one event per session. Reading it
      // from the service as well means a missed event in the state copy still
      // lands: the number the status detail shows must not depend on which
      // particular frame the host happened to attach it to.
      cameraQuality: _session.cameraQuality ?? state.cameraQuality,
    );

    // Native continuity epochs cover the gap that ordinary event coalescing
    // used to hide: PAUSED -> TRACKING can happen entirely between two Flutter
    // deliveries. A state transition remains the compatibility fallback for
    // older hosts that do not publish an epoch yet.
    if (trackingChanged || pose.trackingTransition || epochChanged) {
      debugPrint(
        'ARCORE_RECOVERY: camera=${state.tracking.name}->${pose.tracking.name} '
        'nativeTransition=${pose.trackingTransition} '
        'epoch=${pose.trackingEpoch?.toString() ?? 'unreported'} '
        'epochChanged=$epochChanged.',
      );
    }
    final recoveryBoundary =
        epochChanged || trackingLeft || trackingResumed ||
        (pose.trackingTransition &&
            state.tracking == TrackingState.tracking);
    if (recoveryBoundary && next.phase == CapturePhase.capturing) {
      _beginRecoveryVerification(
        reason: epochChanged
            ? 'native continuity epoch changed'
            : trackingLeft
            ? 'camera tracking left TRACKING'
            : 'camera tracking resumed',
        epoch: pose.trackingEpoch,
      );
      next = next.copyWith(
        trackingRecovering: true,
        clearAlignment: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchorTracking: false,
        stationDriftWarningActive: false,
      );
    } else if (!_recoveryVerificationPending &&
        next.trackingRecovering &&
        pose.tracking == TrackingState.tracking) {
      // The previous frame completed recovery verification. The engine was
      // reset there, so this frame starts a fresh normal 600 ms capture hold.
      next = next.copyWith(trackingRecovering: false);
    }

    // The recovery grace is measured from the **resume**, not from the
    // boundary. The device log shows why the distinction matters: a ~2.4 s
    // tracking pause left only 0.606 s after `paused->tracking` before the
    // 3 s timeout — measured from the boundary — expired, and the station was
    // invalidated before ARCore had any realistic chance to re-report the
    // anchor. Relocalisation after an epoch change takes seconds. Capture
    // stays blocked for the whole window either way, so a full post-resume
    // budget weakens nothing.
    if (trackingResumed && _recoveryVerificationPending) {
      _recoveryStartedAt = _clock();
      _traceStationDiag(
        'recovering:resumed',
        'grace restarted at resume; '
        'anchored=${state.activeAnchored} '
        'placement=${_placement == null ? 'none' : 'held'} '
        'trustedCameraLocal='
        '${_lastTrustedStationLocalCamera == null ? 'none' : _formatVec(_lastTrustedStationLocalCamera!)}.',
      );
    }

    if (next.phase == CapturePhase.starting &&
        pose.tracking == TrackingState.tracking) {
      next = next.copyWith(phase: CapturePhase.scanning);
    }
    // "Move the phone slowly…" is a *boot* message. Once ARCore is tracking it
    // is stale, and leaving it up is what made a working session look stuck.
    if (next.phase == CapturePhase.scanning &&
        pose.tracking == TrackingState.tracking &&
        !_stationReferenceInvalidated) {
      next = next.copyWith(clearMessage: true);
    }

    final frame = _frame;
    if (frame != null && next.phase == CapturePhase.capturing) {
      final now = _clock();

      // A tracking gap invalidates the filter's state: resuming from the old
      // estimate would smear a pose from before the interruption into the
      // present, which is exactly the "smoothing through a discontinuity" the
      // brief forbids.
      //
      // Resetting is not enough on its own — filtering the *paused* sample would
      // immediately re-seed the filter and the next good sample would be
      // interpolated from it. So while tracking is down the filter is bypassed
      // entirely, and the display shows the raw pose because there is nothing
      // better to show.
      final trackingOk = pose.tracking == TrackingState.tracking;
      if (!trackingOk) {
        _smoother.reset();
        _engine.resetStability();
        _resetStationDriftWarningLatch();
        // A paused stream supplies no station-distance sample, so the raw
        // predicate cannot hold a warning: clearing it here keeps the marker
        // and the sentence from carrying a pre-pause walk-away into recovery.
        next = next.copyWith(stationDriftWarningActive: false);
      }
      // Recorded before any branching so every `ARCORE_STATION_DIAG` line can
      // name the frame it describes, and a paused frame's failure reason —
      // `INSUFFICIENT_FEATURES`, `EXCESSIVE_MOTION` — travels with it.
      _lastPoseFrameTsMs = pose.raw.timestampNs == null
          ? null
          : pose.raw.timestampNs! ~/ 1000000;
      _lastPoseFailureReason = pose.failureReason;

      final rawCamera = RigidPose(
        position: pose.position,
        orientation: pose.orientation,
      );

      // The sample the engine consumes: the **raw** ARCore pose, in ARCore's own
      // convention — `(x, y, z, w)`, optical axis on `−Z` — which is the frame
      // the sphere was built in. `pose.orientation` is this app's mirrored
      // capture frame and is deliberately not used here; see
      // `lib/core/math/vio_sphere_math.dart` for why the two must not be mixed.
      final timing = _poseTiming(timestampNs: pose.raw.timestampNs, now: now);
      final rawVioSample = _sampleFrom(pose, timing.sampleNanoseconds);

      // The measurement age is the gap between this pose and the previous one,
      // which is what "how stale is this reading" means for an event stream. A
      // first pose has no predecessor and is reported as fresh.
      final age = _lastPoseAt.millisecondsSinceEpoch == 0
          ? Duration.zero
          : now.difference(_lastPoseAt);
      _lastPoseAt = now;

      final sample = MotionSample(
        orientation: pose.orientation,
        at: Duration(milliseconds: now.millisecondsSinceEpoch),
        position: pose.position,
        angularRateDegPerSec: pose.raw.angularRateDegPerSec,
        measurementAge: age,
      );
      final smoothed = trackingOk ? _smoother.filter(sample) : pose.orientation;

      // A pose ARCore has flagged as paused is its *last known* pose, and ARCore
      // says so itself. Nothing downstream may treat it as current, so two
      // things hold their last trustworthy value instead of being re-derived
      // from it:
      //
      // * **the drift figure** — which the app uses to tell the user they have
      //   walked away from the station. Growing it from a pose ARCore does not
      //   trust blames the user for the tracker's own uncertainty, and the
      //   "walk back to it" sentence then sends them somewhere they never left;
      // * **the alignment** — no turn instruction, ring fill or arrow is
      //   computed from a pose that may be stale.
      //
      // The 26 targets are untouched by any of this: their positions come from
      // the frozen station, and the renderer keeps drawing them from their
      // anchors regardless of what the camera is doing.
      final placement = _placement;
      final target = next.activeTarget;
      // A bare recovered camera pose has no reliable reference for station
      // distance after ARCore has paused or relocalised. In particular, when a
      // completed target leaves no active anchor, treating a new world-map
      // origin as user movement turns the next marker orange before the user
      // has selected it. Preserve the last *trusted* distance until an active
      // tracked anchor can supply the map-correction delta below.
      final uncorrectedDrift =
          trackingOk && target != null && next.activeAnchorTracking
          ? frame.driftOf(rawCamera)
          : state.driftM;

      // The placement may only be used while it belongs to the *active*
      // target. During a transition `activeAnchored` is false and `_placement`
      // still describes the previous target while `activeTargetId` has already
      // moved on: computing alignment from it aims the arrow at a marker that
      // was just released — the "the arrow points at the vanished first
      // target" half of the transition bug. While it is false the honest
      // display is no alignment at all, which the branch below emits.
      // `trackingOk` is part of the condition, not an inner check: with a paused
      // pose there is no honest alignment to compute, and the auto shutter —
      // which lives inside this branch — becomes structurally unreachable
      // rather than merely gated.
      final placementIsActive = next.activeAnchored && trackingOk;
      // The three null checks are spelled out in the `if` rather than folded
      // into [placementIsActive]: Dart promotes a local only from a condition
      // it can see, not from a `bool` variable, and everything below
      // dereferences `target` and `placement`.
      if (placementIsActive &&
          target != null &&
          placement != null &&
          !target.status.isDone) {
        // A native pose can be queued while the previous target is active.
        // Never let that old anchor steer the new arrow: accepting a position
        // merely because it arrived on the latest camera frame is how guidance
        // pointed back to the target that was just captured.
        final live = pose.activeAnchorPose;
        final matchingLive = live != null &&
                pose.activeAnchorId == placement.anchor.id
            ? live
            : null;
        if (matchingLive != null) {
          final anchorTracking =
              matchingLive.tracking == TrackingState.tracking;
          // The frozen reference vs the transform the renderer is drawing:
          // exactly the pair an alignment complaint needs. Printed once per
          // tracking transition, never per frame.
          if (anchorTracking != state.activeAnchorTracking) {
            debugPrint(
              'ARCORE_POSE: active anchor tracking '
              '${state.activeAnchorTracking ? 'held' : 'not held'} -> '
              '${anchorTracking ? 'held' : 'not held'}; reference '
              '${_formatVec(placement.worldPosition)} rendered '
              '${_formatVec(matchingLive.position)}.',
            );
          }
          next = next.copyWith(activeAnchorTracking: anchorTracking);
        } else if (next.activeAnchorTracking) {
          // A missing or mismatched report cannot confirm the previous
          // tracking answer. The shutter may only open on a pose that names
          // the selected tracked anchor; the selected placement is only a
          // short-lived arrow fallback while the new report is on its way.
          next = next.copyWith(activeAnchorTracking: false);
        }
        final liveAnchorTracked =
            matchingLive != null &&
            matchingLive.tracking == TrackingState.tracking;
        if (!liveAnchorTracked &&
            state.activeAnchorTracking &&
            !_recoveryVerificationPending) {
          // The station anchor is the only map reference shared by all 26
          // targets. Losing it is a recovery boundary, not an ordinary target
          // wait: do not keep aiming with the fallback world coordinate.
          _beginRecoveryVerification(
            reason: matchingLive == null
                ? 'active station-anchor pose was not reported'
                : 'active station anchor left TRACKING',
            epoch: pose.trackingEpoch,
          );
          next = next.copyWith(
            trackingRecovering: true,
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
            stationDriftWarningActive: false,
          );
        }

        // ARCore can sometimes keep reporting TRACKING across a map correction.
        // In that case there is no PAUSED event to begin recovery, but a large
        // station-anchor transform change would still make every marker jump on
        // screen.  Do not quietly accept that new transform as a new target
        // position.  Hide guidance, preserve the frozen station-local layout,
        // and make the existing continuity verifier prove that the camera and
        // station still describe the same physical reference.
        final anchorDiscontinuity =
            !_recoveryVerificationPending && liveAnchorTracked
            ? _stationAnchorTransformDiscontinuity(
                anchorId: placement.anchor.id,
                anchor: matchingLive!,
              )
            : null;
        if (anchorDiscontinuity != null) {
          _beginRecoveryVerification(
            reason: 'station anchor transform jumped '
                '${anchorDiscontinuity.positionM.toStringAsFixed(3)} m / '
                '${anchorDiscontinuity.rotationDeg.toStringAsFixed(1)}°',
            epoch: pose.trackingEpoch,
          );
          next = next.copyWith(
            trackingRecovering: true,
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
            stationDriftWarningActive: false,
          );
        }
        // ── the target position: renderer parity ─────────────────────────
        // The native renderer reads `anchor.pose` every frame, so guidance must
        // aim at that same live position whenever ARCore tracks this selected
        // anchor. Before the event arrives, the placement is an honest fallback;
        // another target's live pose never is.
        final targetPosition = matchingLive != null &&
                matchingLive.tracking == TrackingState.tracking
            ? matchingLive.position +
                  matchingLive.orientation.rotate(placement.stationLocalPosition)
            : placement.worldPosition;

        // A shared ARCore map correction can legitimately move both the camera
        // and every anchor. We compensate that translation for the station
        // distance below, but must not let it preserve an already-filled
        // capture window: the rendered target has moved, so require another
        // uninterrupted hold against its newly refined pose.
        final anchorRefined = matchingLive != null &&
            matchingLive.tracking == TrackingState.tracking &&
            _hasAnchorRefined(
              // All 26 nodes share the calibrated station anchor.  Its id on
              // its own is therefore not a stability identity: reusing the
              // previous target's position as this target's reference makes
              // the ordinary ~0.52 m separation between adjacent nodes look
              // like a sudden anchor correction.  Track the live anchor and
              // the selected frozen node as one stability episode instead.
              stabilityId: '${placement.anchor.id}:target-${target.targetId}',
              targetPosition: targetPosition,
            );
        if (anchorRefined) {
          _engine.resetStability();
          _alignedSinceNs = null;
          _alignedHeldMs = 0;
          final frozenNode = target.targetId > 0 &&
                  target.targetId <= _frozenNodePositions.length
              ? _frozenNodePositions[target.targetId - 1]
              : null;
          final guideDelta = frozenNode == null
              ? null
              : (targetPosition - frozenNode).length;
          debugPrint(
            'ARCORE_STABILITY: active anchor refined; restarting the raw '
            'capture hold for target ${target.targetId}. The display pointer '
            'keeps its bounded filter so a small map refinement does not jump '
            'the guidance arrow.',
          );
          debugPrint(
            'ARCORE_GEOMETRY: target ${target.targetId} frozen='
            '${frozenNode == null ? 'n/a' : _formatVec(frozenNode)} '
            'guidance=${_formatVec(targetPosition)} '
            'delta=${guideDelta == null ? 'n/a' : '${guideDelta.toStringAsFixed(4)}m'}; '
            'the frozen target geometry was not rewritten.',
          );
        }

        // The station was originally saved as a camera coordinate. ARCore may
        // later refine its world map even while the user remains physically at
        // that spot; comparing the live camera directly with that frozen
        // coordinate then misreads a map correction as a 20+ cm walk-away.
        //
        // The active target is already a world anchor. Its delta from the
        // original placement is the same world-map correction the renderer is
        // using, so subtracting it from the camera gives the engine a
        // station-relative pose. This affects only station drift and the
        // stability window. Alignment deliberately continues to use the raw
        // camera and the live target, which is the actual geometry on screen.
        final liveStationPose = matchingLive != null &&
                matchingLive.tracking == TrackingState.tracking
            ? matchingLive
            : null;
        final stationLocalCamera = liveStationPose == null
            ? null
            : liveStationPose.orientation.conjugate.rotate(
                pose.position - liveStationPose.position,
              );
        final poseFresh =
            timing.nowNanoseconds - timing.sampleNanoseconds <=
            state.geometry.maxMeasurementAge.inMicroseconds * 1000;

        // A VIO reset can occasionally retain TRACKING long enough that no
        // paused event reaches Flutter.  Detect that independently from the
        // native epoch: this is a station-local camera displacement, so a
        // shared ARCore map correction (camera and station anchor move
        // together) does not falsely trip it.  The previous trusted sample was
        // accepted only inside the raw station limit; an impossible one-frame
        // displacement is therefore coordinate-frame evidence, never a reason
        // to translate the frozen station or targets.
        final trustedLocalCamera = _lastTrustedStationLocalCamera;
        if (!_recoveryVerificationPending &&
            trackingOk &&
            poseFresh &&
            stationLocalCamera != null &&
            trustedLocalCamera != null) {
          final localCameraJumpM =
              (stationLocalCamera - trustedLocalCamera).length;
          if (localCameraJumpM > _maximumPlausibleRecoveryCameraJump) {
            _beginRecoveryVerification(
              reason: 'station-local camera pose jumped '
                  '${localCameraJumpM.toStringAsFixed(3)} m',
              epoch: pose.trackingEpoch,
            );
            next = next.copyWith(
              trackingRecovering: true,
              clearAlignment: true,
              stable: false,
              stabilityProgress: 0,
              stationDriftWarningActive: false,
            );
          }
        }
        final gatePose = liveStationPose == null
            ? rawVioSample
            : vio.VioPoseSample(
                // Bring the camera back into the frozen station coordinate
                // system. This compensates both a shared translation and a
                // shared rotation in ARCore's refined world map.
                position: () {
                  final local = stationLocalCamera!;
                  return vio.Vec3(
                    frame.origin.x + local.x,
                    frame.origin.y + local.y,
                    frame.origin.z + local.z,
                  );
                }(),
                // Position and orientation must be expressed in the same
                // frozen station frame. Leaving the raw world quaternion here
                // made the VIO gate compare a relocalised camera direction to
                // an old-world target direction after anchor rotation.
                rotation: (() {
                  final anchorInverse = vio.Quat(
                    liveStationPose.orientation.x,
                    liveStationPose.orientation.y,
                    liveStationPose.orientation.z,
                    liveStationPose.orientation.w,
                  ).conjugate;
                  return (anchorInverse * rawVioSample.rotation).normalized;
                })(),
                timestampNanoseconds: rawVioSample.timestampNanoseconds,
              );

        // One alignment, and it is the whole app's. The pointer is de-jittered
        // only *after* every gating quantity has been computed, and the result
        // travels in a separate display-only field, so a smoothed value can
        // never arm the shutter or change the wording.
        var alignment = _calculator.compute(
          camera: rawCamera,
          targetPosition: targetPosition,
          toleranceDeg: state.geometry.alignmentToleranceDeg,
          isPole: target.isPole,
        );
        if (alignment.usable) {
          alignment = alignment.withDisplayDirection(
            _pointerFilter.filter(alignment.directionInCamera),
          );
        } else {
          // No answer this frame: the next one must not be blended with an
          // estimate that described a different geometry.
          _pointerFilter.reset();
        }
        // The 600 ms hold, the drift gate and the pose validation are the
        // engine's. One implementation, asserted by the plain-`dart` suite,
        // rather than a second copy of the same arithmetic living here — the
        // exact duplication that let the old 450 ms monitor and the 600 ms
        // specification drift apart.
        //
        // `isSavingInProgress` and `isFailureLatched` are folded in so that
        // [vio.FrameGateEvaluation.canTriggerShutter] is the specification's own
        // expression, in one place, rather than a conjunction re-derived at each
        // call site.
        final gate = _engine.evaluateFrame(
          pose: gatePose,
          trackingValid: trackingOk,
          activeTargetIndex: target.targetId - 1,
          isSavingInProgress: state.capturing || _captureInFlight,
          isFailureLatched: next.retryPending,
          enforceStationDrift: !kStationDriftGateBypassed,
          // A slight excursion through the 3° edge may keep the *motion*
          // history through the configured hysteresis band, but only a strict
          // in-tolerance pose advances the visible/capturable alignment hold.
          alignmentWindowValid:
              alignment.usable &&
              alignment.angularErrorDeg <=
                  state.geometry.alignmentToleranceDeg +
                      state.geometry.alignmentHysteresisDeg,
          nowNanoseconds: timing.nowNanoseconds,
        );
        final stable = gate.isPoseSteady;

        final recoveryVerified = _verifyRecoveryReference(
          now: now,
          frame: frame,
          expectedAnchorId: placement.anchor.id,
          anchor: liveStationPose,
          stationLocalCamera: stationLocalCamera,
          poseFresh: poseFresh,
        );
        if (_stationReferenceInvalidated) return;
        if (recoveryVerified &&
            liveStationPose != null &&
            stationLocalCamera != null &&
            poseFresh) {
          _recordTrustedStationReference(
            anchorId: placement.anchor.id,
            anchor: liveStationPose,
            stationLocalCamera: stationLocalCamera,
          );
        }
        // A recovery that was verified on this exact frame still leaves its
        // state latch true until the following frame. That forces a brand-new
        // engine stability window rather than reusing a pre-verification gate.
        final suppressRecoveredGuidance =
            _recoveryVerificationPending || next.trackingRecovering;

        // How long this aim has been continuously inside the **strict** cone.
        // This uses ARCore's timestamp rather than wall time and is required
        // alongside the engine's motion hold before automatic capture may arm.
        if (alignment.usable && alignment.withinTolerance) {
          final startedNs = _alignedSinceNs ??= timing.sampleNanoseconds;
          final heldNs = timing.sampleNanoseconds - startedNs;
          _alignedHeldMs = heldNs <= 0 ? 0 : heldNs / 1000000.0;
        } else {
          _alignedSinceNs = null;
          _alignedHeldMs = 0;
        }

        if (suppressRecoveredGuidance) {
          _pointerFilter.reset();
          _alignedSinceNs = null;
          _alignedHeldMs = 0;
          _resetStationDriftWarningLatch();
          next = next.copyWith(
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
            measurementFresh: gate.stabilityRejection != 'measurement-age',
            angularRateDegPerSec: gate.angularVelocityDegPerSec,
            // The raw station-relative reading remains observable for logs and
            // diagnostics, but not actionable until verification completes.
            driftM: liveStationPose == null ? state.driftM : gate.stationDriftM,
            stationDriftWarningActive: false,
          );
        } else {
          next = next.copyWith(
            alignment: alignment,
            alignedHeldMs: _alignedHeldMs,
            // The previous frame's answer is what makes the hysteresis band
            // actually apply; it was being dropped on the floor before.
            previousGuidance: _lastGuidance,
            // The engine names the failure it rejected on, so "stale" and
            // "discontinuous" stay distinguishable: only the age gate means the
            // reading itself is too old.
            measurementFresh: gate.stabilityRejection != 'measurement-age',
            angularRateDegPerSec: gate.angularVelocityDegPerSec,
            stabilityRejection: gate.stabilityRejection,
            stable: stable,
            stabilityProgress: _captureHoldProgressOf(gate),
            // A station-distance verdict is valid only when the current camera
            // frame and the current target anchor agree on the same ARCore map.
            // Until then the shutter is already blocked by `activeAnchorTracking`;
            // retaining the last trusted value avoids a relocalisation jump being
            // presented as “walk back to station”.
            driftM:
                liveStationPose == null ? state.driftM : gate.stationDriftM,
          );
          next = _withStationDriftWarning(
            next: next,
            // Do not resurrect a warning from a pre-pause drift value. A warning
            // may re-enter only once this frame names a currently tracked station
            // anchor and therefore supplies a fresh station-relative distance.
            trackingValid: trackingOk && liveStationPose != null,
          );
        }
        // Crossings of the walk-back limit are already logged once, by
        // `_withStationDriftWarning`, from the same raw distance and limit.
        _tracePoseSample(
          now: now,
          pose: pose,
          live: live,
          placement: placement,
          target: target,
          guidanceTargetPosition: targetPosition,
          alignment: alignment,
          gate: gate,
          next: next,
          poseAgeMs:
              ((timing.nowNanoseconds - timing.sampleNanoseconds) ~/
                  1000000),
        );
        diagnostics.add(
          MotionDiagnosticsEntry(
            at: sample.at,
            raw: pose.orientation,
            filtered: smoothed,
            angularRateDegPerSec: gate.angularVelocityDegPerSec,
            targetId: target.targetId,
            errorDeg: alignment.angularErrorDeg,
            stable: stable,
            phase: next.shotPhase.name,
          ),
        );

        _refreshAnchorIfDue();

        // ── the automatic-capture machine ────────────────────────────────
        //
        // Every one of the twelve conditions is evaluated here, from *this*
        // frame's data. Nothing is remembered between frames except the
        // engine's own 600 ms window, which is the specification's hold and not
        // a verdict: a previous success cannot authorise the next shot, because
        // the target has changed and every condition is recomputed.
        //
        // The block reason is a token, not a sentence: `ARCORE_CAPTURE` prints
        // one line per change of reason, which is the only way a device log
        // answers "why did it not fire?" without a line per frame.
        final verdict = _autoCaptureVerdict(
          next: next,
          gate: gate,
          trackingOk: trackingOk,
          target: target,
          anchorRefined: anchorRefined,
        );
        _autoBlockReason = verdict.block;
        _autoStage = verdict.stage;
        _traceAutoCaptureTransition(next: next, verdict: verdict);

        if (verdict.eligible) {
          // Publish the armed state *before* the shutter so the status line and
          // the log agree with the frame that caused it.
          _emit(
            next.copyWith(
              autoStage: AutoCaptureStage.armed,
              autoBlockReason: null,
              clearAutoBlockReason: true,
            ),
          );
          // Not awaited: this is the pose callback, and blocking it for the
          // length of a JPEG write would stall the stream the stability window
          // is measured from. `_shoot` takes its latch synchronously before its
          // first await, so no second frame can enter it.
          _shoot(source: CaptureTriggerSource.automatic, requested: target);
          return;
        }
      } else {
        // A recovery can also lack an active live anchor entirely (for
        // example, ARCore dropped it while tracking was paused). Do not time
        // out a merely paused camera, but once TRACKING has returned it must
        // supply the selected station anchor within the same bounded recovery
        // interval as the normal verification path. Otherwise guidance would
        // remain indefinitely attached to a reference that cannot be proved.
        if (_recoveryVerificationPending && trackingOk && target != null) {
          final startedAt = _recoveryStartedAt ?? now;
          _recoveryStartedAt ??= now;
          // Do not merely wait out the grace: **attempt the restoration**.
          // The invalidation message says "could not restore", but before
          // this call nothing ever tried — the re-place machinery only ran
          // on the healthy branch. `_refreshAnchorIfDue` is rate-limited and
          // bounded: if the host still holds the anchor it is a no-op, and
          // if ARCore dropped it the existing `gone → re-place at the frozen
          // station position` path runs, after which verification proceeds
          // against the re-placed anchor's geometry.
          _refreshAnchorIfDue();
          if (now.difference(startedAt) >= kRecoveryVerificationTimeout) {
            _invalidateStationReference(
              'AR could not restore the selected station anchor after '
              'tracking recovered. Recalibrate at the station to continue.',
            );
            return;
          }
        }
        // No alignment is computed here — not from a paused pose, and not from
        // a placement that is not the active target's. The stability progress
        // goes with it: a ring that keeps its fill through a tracking gap is a
        // ring that lies about how steady the phone has been.
        next = next.copyWith(
          clearAlignment: true,
          driftM: uncorrectedDrift,
          stable: false,
          stabilityProgress: 0,
        );
        // Nothing may be aimed with, so nothing may be captured. The machine is
        // parked in `waiting` with the reason that applies, and the guidance
        // history is dropped: a filter or a latch carried across a tracking gap
        // would describe a pose that no longer exists.
        _resetGuidanceHistory();
        _autoBlockReason = !trackingOk
            ? 'tracking'
            : _recoveryVerificationPending || next.trackingRecovering
            ? 'tracking-recovery'
            : 'no-anchor';
        _autoStage = mode.isAutomatic
            ? AutoCaptureStage.waiting
            : AutoCaptureStage.idle;
      }
    }

    // A failed attempt is re-armed only by re-aiming, and "re-aiming" means the
    // same deliberate move that breaks the stability window — an error past
    // `tolerance + hysteresis`. Not `!withinTolerance`: the raw error is a
    // strict 3°, so a hand tremor would clear the latch on the very next frame
    // and the loop would be back.
    if (next.retryPending) {
      final g = state.geometry;
      final aimed =
          next.alignment != null &&
          next.alignment!.usable &&
          next.alignment!.angularErrorDeg <=
              g.alignmentToleranceDeg + g.alignmentHysteresisDeg;
      if (!aimed) next = next.copyWith(retryPending: false);
    }

    // The active target is hidden while its ring is complete and restored when
    // the hold is lost. Comparing the sets keeps that to **one scene push per
    // transition**: the pose stream runs at 30–60 Hz and pushing per frame
    // would be 60 host calls a second for a change that happens once. Anchor
    // tracking and the station-distance warning ride the same transition check:
    // an untracked anchor clears the scene, and a blocked target becomes amber,
    // once rather than once per frame.
    final hidden = _hiddenTargetsIn(next, next.targets);
    final sceneTrackingChanged = _sceneTracked(next) != _sceneTracking;
    final stationDriftWarningChanged =
        next.stationDriftWarningVisible != _sceneStationDriftWarningActive;
    if (sceneTrackingChanged ||
        stationDriftWarningChanged ||
        !_sameHidden(hidden, _hiddenTargets)) {
      _pushScene(next);
    }

    // The machine's verdict travels with the frame: the status line and the
    // "why is it waiting" sentence read these two fields rather than
    // re-deriving the twelve conditions in the widget tree.
    next = next.copyWith(
      autoStage: _autoStage,
      autoBlockReason: _autoBlockReason,
      clearAutoBlockReason: _autoBlockReason == null,
    );

    // The indicator's hysteresis needs this frame's answer next frame.
    _lastGuidance = next.guidance;

    _traceGuidanceTransition(next: next);
    _traceStabilityTransition(next);
    _emit(next);
  }

  /// One `ARCORE_STABILITY` line each time the engine's rejection reason
  /// changes — tracking loss, a stale pose, a sample gap, a timestamp
  /// regression, an angular-rate breach — and one when the window completes.
  ///
  /// The rejection is the engine's own label from
  /// `FrameGateEvaluation.stabilityRejection`, carried here verbatim: the
  /// reason a capture is blocked should be the same string in the status line
  /// and the log, not two re-derivations that can drift apart.
  void _traceStabilityTransition(CaptureState next) {
    final rejection = next.stabilityRejection;
    if (rejection == _tracedStabilityRejection) return;
    _tracedStabilityRejection = rejection;
    if (rejection == null) {
      debugPrint(
        'ARCORE_STABILITY: window '
        '${next.stable ? 'complete' : 'filling'} '
        'progress=${next.stabilityProgress.toStringAsFixed(0)}% '
        'target=${next.activeTargetId} '
        'rate=${next.angularRateDegPerSec?.toStringAsFixed(2) ?? 'n/a'}deg/s '
        'fresh=${next.measurementFresh}.',
      );
      return;
    }
    debugPrint(
      'ARCORE_STABILITY: window rejected [$rejection] '
      'target=${next.activeTargetId} '
      'tracking=${next.tracking.name} '
      'progress=${next.stabilityProgress.toStringAsFixed(0)}% '
      'drift=${next.driftM.toStringAsFixed(3)}m '
      'rate=${next.angularRateDegPerSec?.toStringAsFixed(2) ?? 'n/a'}deg/s.',
    );
  }

  /// Whether the frozen station reference and the 26 target positions are still
  /// the ones automatic capture was authorised against.
  ///
  /// A defensive check, not a repair: nothing in this class rewrites them, and
  /// the brief forbids moving targets or recalibrating to make a gate pass. If
  /// they ever disagree the shutter is blocked and `ARCORE_STATION` says so,
  /// which is the honest outcome — the alternative is capturing from a
  /// geometry the app no longer understands.
  bool _stationReferenceIntact() {
    if (_stationReferenceInvalidated || _stationInvalidationInFlight) {
      return false;
    }
    final frame = _frame;
    if (frame == null) return false;
    final origin = _engine.stationOrigin;
    if (origin == null || _engine.stationYawRad == null) return false;
    if ((origin.x - frame.origin.x).abs() > 1e-6 ||
        (origin.y - frame.origin.y).abs() > 1e-6 ||
        (origin.z - frame.origin.z).abs() > 1e-6) {
      return false;
    }
    final nodes = _engine.nodes;
    if (nodes.length != _frozenNodePositions.length) return false;
    for (var i = 0; i < nodes.length; i++) {
      final a = nodes[i].worldPosition;
      final b = _frozenNodePositions[i];
      if ((a.x - b.x).abs() > 1e-6 ||
          (a.y - b.y).abs() > 1e-6 ||
          (a.z - b.z).abs() > 1e-6) {
        return false;
      }
    }
    return true;
  }

  /// The twelve automatic-capture conditions, evaluated against this frame.
  ///
  /// Ordered so the first failing condition is the one the user should act on:
  /// tracking, then station, then aim, then steadiness, then bookkeeping. The
  /// order matters for the status line as much as for the log — "waiting for
  /// tracking" is more useful than "hold still" when ARCore has no pose.
  ({bool eligible, AutoCaptureStage stage, String? block}) _autoCaptureVerdict({
    required CaptureState next,
    required vio.FrameGateEvaluation gate,
    required bool trackingOk,
    required CaptureTarget target,
    required bool anchorRefined,
  }) {
    // 1. Mode.
    if (!mode.isAutomatic) {
      return (eligible: false, stage: AutoCaptureStage.idle, block: 'mode-manual');
    }
    // 2. ARCore tracking.
    if (!trackingOk || next.tracking != TrackingState.tracking) {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'tracking');
    }
    if (_stationReferenceInvalidated || _stationInvalidationInFlight) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'station-invalidated',
      );
    }
    if (_recoveryVerificationPending || next.trackingRecovering) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'tracking-recovery',
      );
    }
    // 3. Pose freshness — the engine's own 250 ms age gate.
    if (gate.stabilityRejection == 'measurement-age') {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'pose-age');
    }
    // 4. Station calibrated, and 5. the frozen reference unchanged.
    if (next.station == null || !_engine.isCalibrated) {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'no-station');
    }
    if (!_stationReferenceIntact()) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'station-changed',
      );
    }
    // 6. Three-dimensional station drift.
    //
    // **Bypassed while `kStationDriftGateBypassed` is `true`** — the distance
    // is still measured, published and logged, but does not block automatic
    // capture during the temporary testing period.
    if (!gate.isWithinStation && !kStationDriftGateBypassed) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'station-drift',
      );
    }
    // The raw gate above remains authoritative. This extra latch only makes
    // recovery after crossing 20 cm conservative for the short 0.18 m / 500 ms
    // display-confirmation period; it can never allow an out-of-range pose.
    // Inert while the bypass is active: the latch cannot engage (see
    // `_withStationDriftWarning`), and the check is gated explicitly so the
    // block reason cannot reappear from a pre-bypass latch.
    if (next.stationDriftWarningActive && !kStationDriftGateBypassed) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'station-drift-recovering',
      );
    }
    // 12a. Target bookkeeping, before the geometric conditions: a target that
    // already has a photograph, or one a shutter is already in flight for, must
    // not be re-evaluated as capturable even for one frame.
    if (target.status.isDone) {
      return (
        eligible: false,
        stage: AutoCaptureStage.idle,
        block: 'already-captured',
      );
    }
    if (_captureInFlight ||
        _photos.busy ||
        _capturingTargetId == target.targetId ||
        target.status.isCapturing) {
      return (
        eligible: false,
        stage: AutoCaptureStage.capturing,
        block: 'in-progress',
      );
    }
    // 12b. A failed attempt must be re-aimed before it is retried.
    if (next.retryPending) {
      return (
        eligible: false,
        stage: AutoCaptureStage.failed,
        block: 'failure-latched',
      );
    }
    // The target must be in the scene, and ARCore must be holding its anchor.
    if (!next.activeAnchored) {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'no-anchor');
    }
    if (!next.activeAnchorTracking) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'anchor-paused',
      );
    }
    if (anchorRefined) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'anchor-moving',
      );
    }
    // 7. The actual angular alignment error.
    final alignment = next.alignment;
    if (alignment == null || !alignment.usable) {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'alignment');
    }
    if (!alignment.withinTolerance || !gate.isAligned) {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'alignment');
    }
    // The phone may have been stable while aimed away from the target. Require
    // a new strict-alignment hold before arming, so entering the 3° cone cannot
    // trigger an immediate photograph from that earlier history.
    if (!next.alignmentHoldComplete) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'alignment-window',
      );
    }
    // 8–11. The 600 ms window, its orientation and position drift, and the
    // angular velocity. All four are the engine's single steady verdict, and
    // its rejection token names which one refused.
    if (!gate.isPoseSteady) {
      final rejection = gate.stabilityRejection;
      final block = switch (rejection) {
        'orientation-drift' => 'orientation-drift',
        'position-drift' => 'position-drift',
        'angular-rate' => 'angular-velocity',
        'sample-gap' => 'sample-gap',
        'invalid-pose' => 'invalid-pose',
        'timestamp' => 'timestamp',
        _ => 'window',
      };
      return (eligible: false, stage: AutoCaptureStage.waiting, block: block);
    }
    // Every condition holds. The last guard is the specification's own
    // expression, kept as a conjunction so a future change to either authority
    // can only make the shutter more conservative.
    if (!gate.canTriggerShutter) {
      return (eligible: false, stage: AutoCaptureStage.waiting, block: 'gate');
    }
    return (eligible: true, stage: AutoCaptureStage.armed, block: null);
  }

  /// Shortest rotational distance between two equivalent unit quaternions.
  /// `q` and `-q` are the same ARCore orientation, so the absolute dot avoids
  /// classifying an innocuous sign flip as a 360° anchor jump.
  double _quaternionAngleDeg(Quat a, Quat b) {
    final dot = a
        .normalized
        .dot(b.normalized)
        .abs()
        .clamp(0.0, 1.0)
        .toDouble();
    return 2 * math.acos(dot) * 180 / math.pi;
  }

  /// Detects a discontinuous update of the one anchor that defines the frozen
  /// station-local target layout.
  ///
  /// Normal ARCore refinement is allowed to be small and continuous.  A larger
  /// single update is different: using it immediately would visibly relocate
  /// every marker even though neither the station nor the stored target geometry
  /// has changed.  The existing recovery thresholds are deliberately reused so
  /// recovery is not stricter than the rest of the station-continuity policy.
  ({double positionM, double rotationDeg})?
  _stationAnchorTransformDiscontinuity({
    required String anchorId,
    required ({Vec3 position, Quat orientation, TrackingState tracking}) anchor,
  }) {
    final previous = _lastTrustedStationAnchor;
    if (previous == null || previous.id != anchorId) return null;
    final positionM = (anchor.position - previous.position).length;
    final rotationDeg = _quaternionAngleDeg(anchor.orientation, previous.orientation);
    if (positionM <= kRecoveryAnchorPositionDiscontinuityM &&
        rotationDeg <= kRecoveryAnchorOrientationDiscontinuityDeg) {
      return null;
    }
    debugPrint(
      'ARCORE_RECOVERY: station-anchor discontinuity while TRACKING '
      '(${positionM.toStringAsFixed(3)} m, '
      '${rotationDeg.toStringAsFixed(1)}°); suppressing guidance until '
      'station continuity is verified.',
    );
    return (positionM: positionM, rotationDeg: rotationDeg);
  }

  /// Returns true when ARCore has moved or rotated the selected shared station
  /// anchor by a meaningful amount since the current capture hold began. A new
  /// anchor establishes its own baseline; only movement of that same anchor
  /// invalidates the hold.
  /// Returns true when the **target's world position** has moved enough since
  /// the current capture hold began to invalidate the camera-to-target aim.
  ///
  /// The measure is deliberately the computed target position, not the
  /// anchor's own translation and rotation. ARCore refines its shared station
  /// anchor constantly — the device log shows a 0.015 m / 3.50° refinement —
  /// and an anchor *rotation* can change by degrees without moving the target
  /// meaningfully (a rotation about the target's own bearing moves it not at
  /// all). The previous 0.01 m / 0.6° anchor thresholds were stricter than the
  /// hold's own stability tolerances, so routine map refinements restarted the
  /// hold faster than a person could refill it: "aligned, but the ring never
  /// completes". The hold's own position allowance
  /// (`maxPositionDriftM`) is the coherent threshold — a movement the
  /// hold would tolerate cannot justify restarting it, and one it would reject
  /// must.
  bool _hasAnchorRefined({
    required String stabilityId,
    required Vec3 targetPosition,
  }) {
    final reference = _anchorStabilityReference;
    if (_anchorStabilityId != stabilityId || reference == null) {
      // A new target on the shared anchor — or the first evaluation —
      // establishes its own baseline; only movement while aiming at that same
      // target invalidates the hold.
      _anchorStabilityId = stabilityId;
      _anchorStabilityReference = targetPosition;
      return false;
    }
    final movedM = (targetPosition - reference).length;
    if (movedM <= state.geometry.maxPositionDriftM) {
      return false;
    }
    _anchorStabilityReference = targetPosition;
    debugPrint(
      'ARCORE_STABILITY: station anchor refinement moved the target '
      '${movedM.toStringAsFixed(3)} m (> '
      '${state.geometry.maxPositionDriftM.toStringAsFixed(3)} m allowance); '
      'restarting hold.',
    );
    return true;
  }

  /// One `ARCORE_CAPTURE` line per change of block reason, plus the trigger.
  ///
  /// A steady blocked state logs once, not 60 times a second. The eligibility
  /// transition to `armed` is logged by [_shoot] instead, so the trigger and the
  /// start of the transaction are the same line.
  void _traceAutoCaptureTransition({
    required CaptureState next,
    required ({bool eligible, AutoCaptureStage stage, String? block}) verdict,
  }) {
    if (verdict.eligible) return;
    if (_tracedAutoBlock == verdict.block) return;
    _tracedAutoBlock = verdict.block;
    final a = next.alignment;
    debugPrint(
      'ARCORE_CAPTURE: auto blocked by ${verdict.block} '
      '(mode=${mode.label} target=${next.activeTargetId} '
      'stage=${verdict.stage.name} '
      'error=${a == null ? 'n/a' : '${a.angularErrorDeg.toStringAsFixed(2)}°'} '
      'window=${next.stabilityProgress.toStringAsFixed(2)} '
      'drift=${next.driftM.toStringAsFixed(3)}m '
      'tracking=${next.tracking.name} '
      'anchorTracking=${next.activeAnchorTracking}).',
    );
  }

  /// Writes one line every 500 ms, and one immediately whenever the camera or
  /// anchor tracking state changes.
  ///
  /// The fields are the minimum needed to tell the three failure stories
  /// apart: `cam`/`camQ` move while `anchor` stays put means the target is
  /// world-fixed and the camera is not; `anchor` changes while `placement`
  /// does not is ARCore refining a tracked node; `anchor=unreported` explains
  /// why the gate fell back to `placement`. `station`/`stationQ` identify the
  /// frame the placement was computed from, so a target aimed at the wrong
  /// place can be traced to a changed station reference rather than an anchor.
  /// `frozenTarget` and `guidanceTarget` then make it explicit whether ARCore
  /// merely refined the rendered station anchor or any authoritative target
  /// geometry was rewritten.
  void _tracePoseSample({
    required DateTime now,
    required ArPoseSample pose,
    required ({Vec3 position, Quat orientation, TrackingState tracking})? live,
    required AnchoredTarget placement,
    required CaptureTarget target,
    required Vec3 guidanceTargetPosition,
    required AlignmentResult alignment,
    required vio.FrameGateEvaluation gate,
    required CaptureState next,
    required int poseAgeMs,
  }) {
    final changed =
        _tracedCameraTracking != pose.tracking ||
        _tracedAnchorTracking != next.activeAnchorTracking;
    final due =
        _lastPoseTraceAt == null ||
        now.difference(_lastPoseTraceAt!) >= const Duration(milliseconds: 500);
    if (!changed && !due) return;

    _lastPoseTraceAt = now;
    _tracedCameraTracking = pose.tracking;
    _tracedAnchorTracking = next.activeAnchorTracking;

    final frame = _frame;
    final anchorPosition = live == null
        ? 'unreported'
        : _formatVec(live.position);
    final anchorOrientation = live == null
        ? 'unreported'
        : _formatQuat(live.orientation);
    final anchorTracking = live?.tracking.name ?? 'unreported';
    final dc = alignment.directionInCamera;
    final proj = projectWithArCoreProjection(
      dc,
      1,
      1,
      next.displayProjectionMatrix,
      fallbackHfovDeg: next.displayFov.hfovDeg,
      fallbackVfovDeg: next.displayFov.vfovDeg,
    );
    // The engine and the guidance answer the same question — how far the optical
    // axis is from this target — from the same pose. Printing the pair is what
    // turns "the arrow points wrong but the shutter says aligned" into a number:
    // a `angleGap` above a hundredth of a degree means the two are reading
    // different target positions, not different clocks.
    final angleGapDeg =
        (gate.alignmentErrorDeg - alignment.angularErrorDeg).abs();
    // How far ARCore's anchor sits from where this app placed it: the line
    // between "the target moved" and "the world estimate refined under it".
    final anchorDeltaM = live == null
        ? null
        : (live.position - placement.worldPosition).length;
    // The engine's frozen node is the authoritative geometry captured at
    // calibration. `guidanceTargetPosition` is the live renderer-parity point
    // for this frame: it can differ during an ARCore map refinement, but that
    // must never mutate the frozen node. Logging both distinguishes a changing
    // camera-to-target direction from an accidental target rewrite.
    final frozenTarget = target.targetId > 0 &&
            target.targetId <= _frozenNodePositions.length
        ? _frozenNodePositions[target.targetId - 1]
        : null;
    final frozenToGuidanceDeltaM = frozenTarget == null
        ? null
        : (guidanceTargetPosition - frozenTarget).length;
    final frozenToPlacementDeltaM = frozenTarget == null
        ? null
        : (placement.worldPosition - frozenTarget).length;
    _tracePose(
      't=${now.millisecondsSinceEpoch} '
      'cam=${_formatVec(pose.position)} '
      'camQ=${_formatQuat(pose.orientation)} '
      'camTracking=${pose.tracking.name} '
      'epoch=${pose.trackingEpoch?.toString() ?? 'unreported'} '
      'transition=${pose.trackingTransition} '
      'recoveryPending=$_recoveryVerificationPending '
      'anchorId=${placement.anchor.id} '
      'anchor=$anchorPosition '
      'anchorQ=$anchorOrientation '
      'anchorTracking=$anchorTracking '
      'anchorDelta=${anchorDeltaM == null ? 'n/a' : '${anchorDeltaM.toStringAsFixed(4)}m'} '
      'gate=${next.activeAnchorTracking} '
      'placement=${_formatVec(placement.worldPosition)} '
       'frozenTarget=${frozenTarget == null ? 'n/a' : _formatVec(frozenTarget)} '
       'guidanceTarget=${_formatVec(guidanceTargetPosition)} '
       'frozenToPlacement=${frozenToPlacementDeltaM == null ? 'n/a' : '${frozenToPlacementDeltaM.toStringAsFixed(4)}m'} '
       'frozenToGuidance=${frozenToGuidanceDeltaM == null ? 'n/a' : '${frozenToGuidanceDeltaM.toStringAsFixed(4)}m'} '
      'target=${target.targetId} '
      'station=${_formatVec(frame?.origin ?? Vec3.zero)} '
      'stationQ=${_formatQuat(frame?.orientation ?? Quat.identity)} '
      'dc=(${dc.x.toStringAsFixed(3)},${dc.y.toStringAsFixed(3)},${dc.z.toStringAsFixed(3)}) '
      'displayDc=(${alignment.pointerDirection.x.toStringAsFixed(3)},'
      '${alignment.pointerDirection.y.toStringAsFixed(3)},'
      '${alignment.pointerDirection.z.toStringAsFixed(3)}) '
      'proj=(${proj.u.toStringAsFixed(3)},${proj.v.toStringAsFixed(3)}) '
      'onScreen=${proj.onScreen} '
      'error=${alignment.angularErrorDeg.toStringAsFixed(2)} '
      'gateError=${gate.alignmentErrorDeg.toStringAsFixed(2)} '
      'angleGap=${angleGapDeg.toStringAsFixed(2)} '
      'drift=${next.driftM.toStringAsFixed(3)} '
       'driftWarning=${next.stationDriftWarningActive} '
      // Pose freshness and the stability window, the two facts the 250 ms and
      // 600 ms gates are built on. `poseAgeMs` is the same number the engine's
      // age gate consumed, so a stale-pose capture can be traced to the gate
      // rather than argued about.
      'poseAgeMs=$poseAgeMs '
      'poseTsMs=${pose.raw.timestampNs == null ? 'n/a' : pose.raw.timestampNs! ~/ 1000000} '
      'fresh=${next.measurementFresh} '
      'stable=${next.stable} '
      'stabProg=${next.stabilityProgress.toStringAsFixed(0)}% '
      'rejection=${gate.stabilityRejection ?? 'none'} '
      'rate=${gate.angularVelocityDegPerSec?.toStringAsFixed(2) ?? 'n/a'}deg/s '
      'ready=${next.readyToCapture} '
      'block=${next.captureBlockReason ?? 'ready'} '
      'guidance=${next.guidance.name}',
    );
  }

  /// One line each time the guidance answer changes, with the numbers that
  /// produced it.
  ///
  /// The state itself is a coarse label — the arrow is drawn from the projected
  /// vector, not from this — so the line carries the camera-frame direction, the
  /// normalised screen offset and the gating reason alongside it. Transitions
  /// only: a steady aim logs nothing, and a 60 Hz stream of "right" would bury
  /// the one transition that matters.
  void _traceGuidanceTransition({required CaptureState next}) {
    final from = _tracedGuidance;
    if (from == null) {
      _tracedGuidance = next.guidance;
      return;
    }
    if (from == next.guidance) return;
    _tracedGuidance = next.guidance;

    final a = next.alignment;
    final dc = a?.directionInCamera;
    final dcText = dc == null
        ? 'none'
        : '(${dc.x.toStringAsFixed(3)},${dc.y.toStringAsFixed(3)},'
              '${dc.z.toStringAsFixed(3)})';
    final screen = dc == null
        ? 'none'
        : () {
            final p = projectWithArCoreProjection(
              dc,
              1,
              1,
              next.displayProjectionMatrix,
              fallbackHfovDeg: next.displayFov.hfovDeg,
              fallbackVfovDeg: next.displayFov.vfovDeg,
            );
            return '(${p.u.toStringAsFixed(3)},${p.v.toStringAsFixed(3)}'
                '${p.onScreen ? '' : ' off-screen'})';
          }();
    debugPrint(
      'ARCORE_GUIDANCE: guidance ${from.name} -> ${next.guidance.name} '
      'target=${next.activeTargetId} '
      'dc=$dcText '
      'screen=$screen '
      'error=${a == null ? 'n/a' : '${a.angularErrorDeg.toStringAsFixed(2)}°'} '
      'held=${next.alignedHeldMs.toStringAsFixed(0)}ms '
      'drift=${next.driftM.toStringAsFixed(3)}m '
      'tracking=${next.tracking.name} '
      'block=${next.captureBlockReason ?? 'ready'}',
    );
  }

  /// Re-reads the **active** target's anchor about twice a second: ARCore keeps
  /// refining it as it maps more of the room, and the alignment is only as good
  /// as the position it is aimed at. The other 29 anchors are left alone —
  /// their positions are the station's arithmetic, and ARCore refines them in
  /// place; re-reading all 26 twice a second would be 60 host calls a second
  /// for no gain.
  ///
  /// **This is also where a lost anchor is recovered from.** "Placed" and
  /// "still held by ARCore" are different facts, and this method is the only
  /// place they are reconciled. A paused anchor is not drawn from its stale
  /// pose; it is kept and allowed to resume, so only a host-missing anchor
  /// needs placement recovery:
  ///
  /// * `gone` — the anchor is **gone** (tracking lost, session restarted).
  ///   Re-place that one target: the position is recomputed from the station,
  ///   the anchor recreated, the marker reappears. The other 29 nodes are
  ///   untouched. Bounded by [kAnchorReplacementsMax] — a placement ARCore
  ///   will not hold is a failure to report, not a retry storm.
  /// * placed but **paused** — wait, and say so. ARCore pauses a world anchor
  ///   whenever its own tracking degrades, and resumes it by itself; its frozen
  ///   placement remains stored but its marker stays hidden until tracking is
  ///   back. Re-placing a paused anchor used to be the bug that made targets
  ///   disappear on a turn: the replacement starts paused as well, two rounds
  ///   of it hit the give-up latch, and the marker never came back.
  void _refreshAnchorIfDue() {
    final now = _clock();
    if (now.difference(_lastAnchorRefresh) <
        const Duration(milliseconds: 500)) {
      return;
    }
    // A recovery placement is in flight: it owns the target's anchor, so
    // there is nothing to refresh — and a "gone" answer here would describe
    // the recovery, not a loss.
    if (_placing) return;
    final requestedTarget = state.activeTargetId;
    if (requestedTarget == 0) return;
    if (_anchors.placementFor(requestedTarget) == null) return;
    _lastAnchorRefresh = now;
    _anchors.refresh(requestedTarget).then((AnchorRefreshOutcome outcome) {
      if (_disposed || _shuttingDown) return;
      // A refresh is only ever about the target that was active when it
      // started. If the active target moved on, the answer applies to a
      // target the user is no longer aiming at — discard it rather than
      // write its pose, or its recovery, into the new target's row.
      if (outcome.stale || state.activeTargetId != requestedTarget) {
        _traceTransition(
          'discarded a refresh about target $requestedTarget '
          '(active is ${state.activeTargetId})',
        );
        return;
      }
      final refreshed = outcome.placement;
      if (refreshed == null) {
        // The single shared station anchor is gone. Replacing just one target
        // would create a new reference for that target alone and reintroduce
        // the drifting-target bug. Keep the frozen geometry intact and require
        // a deliberate recalibration instead.
        debugPrint(
          'ARCORE_ANCHOR: station reference lost for target $requestedTarget; '
          'frozen geometry is no longer usable; explicit recalibration '
          'required.',
        );
        _invalidateStationReference(
          'AR lost the shared station anchor. Recalibrate at the station to '
          'continue.',
        );
        return;
      }
      _placement = refreshed;
      _emit(
        state.copyWith(
          targets: _withAnchor(state.activeTargetId, refreshed),
          // A fresh, host-confirmed placement for the active target is
          // exactly what `activeAnchored` asserts. Restoring it here matters
          // during a recovery: if the flag was lost across a tracking pause,
          // the pose handler would otherwise stay in the no-placement branch
          // and time the station out without ever re-entering verification.
          // This cannot fake readiness — capture still requires the anchor
          // to be *tracked*, the reference to be verified, and the stability
          // window to refill.
          activeAnchored: true,
        ),
      );
      if (refreshed.isTracking) {
        if (_anchorNotTracking > 0) {
          // Tracking recovered (or held): clear the wait and give the next
          // target a full recovery budget.
          debugPrint(
            'ARCORE_ANCHOR: target $requestedTarget anchor '
            '${refreshed.anchor.id} tracking recovered; gate open.',
          );
          _anchorNotTracking = 0;
          _anchorReplacements = 0;
          _anchorGiveUp = false;
          _emit(state.copyWith(activeAnchorTracking: true, clearMessage: true));
        }
        return;
      }

      // Placed, and the anchor is merely **paused**. ARCore pauses a world
      // anchor whenever its own tracking degrades — a fast turn, a featureless
      // wall, the first moments after creation — and resumes it on its own.
      // It is not a lost position: the marker keeps the world position this
      // app computed and is drawn from the anchor's pose every frame, so the
      // honest answer is to wait and say what is happening. Re-placing here is
      // what made targets disappear on a turn: the replacement starts paused
      // too, two rounds of it hit the give-up latch, and the marker never
      // came back.
      if (_anchorGiveUp) return;
      if (_anchorNotTracking == 0) {
        debugPrint(
          'ARCORE_ANCHOR: target $requestedTarget anchor '
          '${refreshed.anchor.id} paused; waiting for ARCore to resume it.',
        );
        _anchorNotTracking = 1;
        _emit(
          state.copyWith(
            activeAnchorTracking: false,
            message: 'Waiting for AR to stabilise the target…',
          ),
        );
      }
    });
  }

  void _onDiagnostic(ArCoreEvent event) {
    if (_disposed) return;
    if (event.type == ArCoreEventType.error) {
      final message = event.message ?? 'The AR renderer reported an error.';
      _emit(
        state.copyWith(
          lastError: message,
          // A fatal render failure means the preview cannot work at all, so it
          // gets its own banner and a Retry action rather than a diagnostics
          // line. This is the path the `Must use a native order direct buffer`
          // exception travels now — with its class name and its first frame.
          arFault: event.fatal ? message : null,
          arFaultFrame: event.frame,
        ),
      );
    }
    if (event.type == ArCoreEventType.imageOrientation) {
      _emit(state.copyWith(imageAxis: _session.imageAxis));
    }
  }

  // ── lifecycle ─────────────────────────────────────────────────────────

  /// The in-flight [onAppPaused] teardown, observed rather than re-run by a
  /// second pause request while the first is mid-await.
  Future<void>? _pauseInFlight;

  /// The in-flight [onAppResumed] boot, observed rather than re-run by a
  /// second resume request while the first is mid-await.
  Future<void>? _resumeInFlight;

  Future<void> resize({required int width, required int height}) {
    _surfaceWidth = width;
    _surfaceHeight = height;
    return _session.resize(width: width, height: height);
  }

  /// The app went to the background: tear the AR session down rather than
  /// leaving a camera running behind a screen that is not visible.
  ///
  /// Two pause requests must not run the teardown twice: the first pass
  /// empties the anchor maps across awaits, and a second re-entry used to be
  /// the concurrent-modification crash. A queued request observes the
  /// in-flight teardown instead of running its own. Errors propagate so the
  /// platform layer can react to a failed stop.
  Future<void> onAppPaused() async {
    final existing = _pauseInFlight;
    if (existing != null) {
      return existing;
    }
    final inFlight = _runPauseTeardown();
    _pauseInFlight = inFlight;
    try {
      await inFlight;
    } finally {
      // Both resets happen on every path: a failed `stop` must not leave the
      // flag raised for the next pause, and the slot must free so a later
      // pause can run again after resume.
      _shuttingDown = false;
      _pauseInFlight = null;
    }
  }

  /// The teardown body of [onAppPaused], run at most once.
  Future<void> _runPauseTeardown() async {
    _shuttingDown = true;
    await _poseSub?.cancel();
    _poseSub = null;
    await _diagSub?.cancel();
    _diagSub = null;
    await _anchors.dispose();
    await _session.stop();
    _shuttingDown = false;
    _emit(
      state.copyWith(
        clearAlignment: true,
        stable: false,
        stationDriftWarningActive: false,
      ),
    );
  }

  Future<void> onAppResumed({required int width, required int height}) async {
    if (state.phase == CapturePhase.idle ||
        state.phase == CapturePhase.unavailable ||
        state.phase == CapturePhase.failed) {
      return;
    }

    final existing = _resumeInFlight;
    if (existing != null) {
      return existing;
    }
    final inFlight = _runResume(width: width, height: height);
    _resumeInFlight = inFlight;
    try {
      await inFlight;
    } finally {
      _resumeInFlight = null;
    }
  }

  /// The resume body of [onAppResumed], run at most once at a time.
  ///
  /// A queued pause is joined first: starting an AR session while a teardown
  /// is mid-flight is the camera/state mix pausing exists to destroy. Its
  /// failure is logged, not propagated — the resume decides from the refreshed
  /// phase what is actually possible.
  Future<void> _runResume({required int width, required int height}) async {
    final pauseInFlight = _pauseInFlight;
    if (pauseInFlight != null) {
      try {
        await pauseInFlight;
      } catch (error) {
        debugPrint(
          'ARCORE_SESSION: pending pause errored before resume: $error',
        );
      }
    }

    // The joined pause may have moved the phase; re-check before booting into
    // a flow that is no longer there.
    if (state.phase == CapturePhase.idle ||
        state.phase == CapturePhase.unavailable ||
        state.phase == CapturePhase.failed) {
      return;
    }

    final station = state.station;
    final activeId = state.activeTargetId;
    await boot(width: width, height: height);
    if (state.arFault != null || state.phase == CapturePhase.unavailable) {
      return;
    }
    if (station == null) return;

    if (_lastSessionReused) {
      // Same AR session, same world origin: the station pose is still valid, so
      // restore the flow the user was in rather than dropping them back to
      // "starting".
      _freezeStation(
        StationFrame(
          origin: station.referencePosition,
          orientation: station.referenceOrientation,
        ),
      );
      _tracePose(
        'station-reference restored id=${station.stationId} '
        'origin=${_formatVec(station.referencePosition)} '
        'q=${_formatQuat(station.referenceOrientation)}',
      );
      _emit(
        state.copyWith(
          phase: CapturePhase.capturing,
          clearAlignment: true,
          clearMessage: true,
        ),
      );
      if (activeId != 0) await selectTarget(activeId);
      return;
    }

    // A new session means a new world origin. Keeping the old station would aim
    // every one of the 26 targets at the wrong place, so it is discarded and the
    // user is told why. Captured photographs on disk are untouched.
    _clearStation();
    _tracePose('station-reference cleared: new AR session');
    _emit(
      state.copyWith(
        phase: CapturePhase.scanning,
        clearStation: true,
        targets: const <CaptureTarget>[],
        activeTargetId: 0,
        clearAlignment: true,
        driftM: 0,
        stationDriftWarningActive: false,
        message:
            'AR restarted with a new world origin. Recalibrate at the station '
            'to continue — the saved photographs are kept.',
      ),
    );
  }

  @override
  void dispose() {
    if (_disposed) return;
    _disposed = true;
    _poseSub?.cancel();
    _diagSub?.cancel();
    _anchors.dispose();
    _session.stop();
    super.dispose();
  }

  // ── helpers ───────────────────────────────────────────────────────────

  List<CaptureTarget> _withAnchor(int targetId, AnchoredTarget placement) =>
      _withAnchorIn(state.targets, targetId, placement, setActive: true);

  /// [list] with [targetId]'s row updated to the placement's anchor, position
  /// and surface. Takes the list explicitly so a placement pass over all 26
  /// can chain without re-reading `state` between updates.
  ///
  /// [setActive] is what separates *placing* a node from *activating* a
  /// target, and it defaults to false on purpose: the placement pass runs over
  /// all 26 targets, and marking each of them `active` there is what made the
  /// sphere map paint every dot amber at "0 of 26 captured" — the map reads
  /// `active` as "the target you are on". Placing is not activating; only
  /// [selectTarget] and the recovery mark the one target the user is aiming
  /// at.
  List<CaptureTarget> _withAnchorIn(
    List<CaptureTarget> list,
    int targetId,
    AnchoredTarget placement, {
    bool setActive = false,
  }) {
    if (targetId <= 0 || list.isEmpty) return list;
    final updated = List<CaptureTarget>.of(list);
    final index = targetId - 1;
    if (index < 0 || index >= updated.length) return list;
    final existing = updated[index];
    updated[index] = existing.copyWith(
      anchorIdentifier: placement.anchor.id,
      anchorWorldPosition: placement.worldPosition,
      anchorSurfaceType: placement.surfaceType,
      status: existing.status.isDone
          ? existing.status
          : setActive
          ? CaptureTargetStatus.active
          : existing.status,
      clearMessage: true,
    );
    return updated;
  }

  List<CaptureTarget> _withTargetMessage(
    int targetId,
    String message, {
    CaptureTargetStatus? status,
    bool clearAnchor = false,
  }) {
    if (targetId <= 0 || state.targets.isEmpty) return state.targets;
    final updated = List<CaptureTarget>.of(state.targets);
    final index = targetId - 1;
    if (index < 0 || index >= updated.length) return state.targets;
    updated[index] = updated[index].copyWith(
      status: status ?? updated[index].status,
      message: message,
      clearAnchor: clearAnchor,
    );
    return updated;
  }

  /// The targets whose meshes must **not** be drawn for [s]: every completed
  /// one, plus the active target while its alignment ring is complete.
  ///
  /// **Visibility is not readiness, and this is where they are kept apart.**
  /// Four separate facts, each with its own trigger:
  ///
  /// * the **ring** is complete — [CaptureState.ringComplete], i.e. aimed and
  ///   steady. Both halves are hysteretic, so this does not flap;
  /// * a **transaction** is in flight — `capturing`;
  /// * the target is **accepted** — `isDone`, which stays hidden for the rest
  ///   of the station;
  /// * and, deliberately absent, *readiness*: tracking, measurement freshness
  ///   and drift. Those flip on their own — a pose gap over 250 ms, a
  ///   momentary ARCore pause — and folding them in here is what made the
  ///   marker flash on and off while the phone was held still.
  ///
  /// The active target comes back on `align`/`settle`/`retry`, which is what
  /// restores it when a hold is lost or a photograph fails.
  Set<int> _hiddenTargetsIn(CaptureState s, List<CaptureTarget> targets) {
    final hidden = <int>{
      // A captured target stays hidden. Its record, its anchor and its world
      // position all survive — only the geometry leaves the frame.
      for (final CaptureTarget t in targets)
        if (t.status.isDone) t.targetId,
    };
    final id = s.activeTargetId;
    if (id != 0 && (s.ringComplete || s.capturing)) {
      hidden.add(id);
    }
    return hidden;
  }

  /// One scene push for [s]: the whole arrangement, the active target
  /// highlighted, and the hidden ones omitted.
  ///
  /// Every push goes through here so the tint, the done set and the hidden set
  /// cannot disagree with each other — a finished marker still drawn at full
  /// opacity, or a target hidden while another call site republished the active
  /// tint, were both possible while each site assembled its own draw list.
  Future<void> _pushScene(CaptureState s, {List<CaptureTarget>? targets}) {
    // One state for the whole push. The tint, the done set, the hidden set and
    // the phase all have to describe the *same* target list — a push that read
    // the tint from the old rows and the done set from the new ones is how a
    // failed target can end up wearing the idle bezel.
    final next = targets == null ? s : s.copyWith(targets: targets);
    final active = next.activeTarget;
    final tint = active == null
        ? kBezelIdleTint
        : active.status == CaptureTargetStatus.failed
        ? kBezelFailedTint
        : active.status.isDone
        ? kBezelDoneTint
        : next.stationDriftWarningVisible && kStationDriftWarningVisualsEnabled
        ? kBezelStationDriftTint
        : kBezelIdleTint;
    final hidden = _hiddenTargetsIn(next, next.targets);
    final tracking = _sceneTracked(next);
    final wasTracking = _sceneTracking;
    _hiddenTargets = hidden;
    _sceneTracking = tracking;
    _sceneStationDriftWarningActive = next.stationDriftWarningVisible;

    // Previously `activeTracking: false` only lowered marker opacity. The
    // native renderer could still re-read `anchor.pose` every AR frame; during
    // a VIO pause or recovery that pose is ARCore's last/refining estimate, so
    // the marker could visibly slide while guidance and capture were correctly
    // blocked.
    //
    // Do not "correct" frozen target geometry from an untrusted pose. Hide the
    // native scene while retaining every placement, station frame and target
    // row. Once `_sceneTracked` becomes true after recovery verification, the
    // ordinary transition below republishes the same frozen 26-node layout.
    if (!tracking) {
      if (wasTracking) {
        debugPrint(
          'ARCORE_SCENE: marker rendering suppressed; '
          'camera=${next.tracking.name} '
          'anchorTracking=${next.activeAnchorTracking} '
          'recovery=${next.trackingRecovering}.',
        );
      }
      return _anchors.clearScene();
    }
    if (!wasTracking) {
      debugPrint(
        'ARCORE_SCENE: marker rendering restored from frozen station '
        'geometry; target=${next.activeTargetId}.',
      );
    }
    return _anchors.pushScene(
      activeTargetId: next.activeTargetId,
      bezelTint: tint,
      // There is no native active marker after a successful capture.  Do not
      // carry an old marker's warning tint into that neutral state.
      activeTargetTint:
          active != null &&
              next.stationDriftWarningVisible &&
              kStationDriftWarningVisualsEnabled
          ? kBezelStationDriftTint
          : null,
      activeTracking: tracking,
      debugMarker: kGlbDebugMarker,
      doneTargets: <int>{
        for (final CaptureTarget t in next.targets)
          if (t.status.isDone) t.targetId,
      },
      hiddenTargets: hidden,
    );
  }

  /// Whether the active target may be presented as authoritative.
  ///
  /// The anchor says the *target* is known in the world; the camera says the
  /// *view* is known. A paused/recovering camera exposes only a last or
  /// refining pose, so the native scene is hidden until both are trustworthy.
  bool _sceneTracked(CaptureState s) =>
      s.activeAnchorTracking &&
      s.tracking == TrackingState.tracking &&
      !s.trackingRecovering;

  /// Whether two hidden sets are the same. `Set` identity is reference
  /// equality, and the comparison exists so a 60 Hz pose stream costs one
  /// scene push per transition rather than one per frame.
  bool _sameHidden(Set<int> a, Set<int> b) {
    if (a.length != b.length) return false;
    for (final int id in a) {
      if (!b.contains(id)) return false;
    }
    return true;
  }

  void _emit(CaptureState next) {
    if (_disposed) return;
    state = next;
    notifyListeners();
  }
}
