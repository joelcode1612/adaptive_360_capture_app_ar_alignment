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
import 'package:adaptive_360_capture_app/features/capture/data/services/ar_tracking_trace.dart';
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
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_sources.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/sensor_pose_source.dart';
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
/// `_refreshAnchorIfDue` runs twice a second. A **paused** shared station
/// anchor is never re-placed: ARCore resumes a world anchor when its tracking
/// recovers. If the host loses that one anchor, the frozen reference is
/// invalidated and the user explicitly recalibrates; making a new anchor for
/// one target would split the 26 targets across coordinate systems.

/// A recovered ARCore frame must prove it still belongs to the frozen station
/// before guidance resumes. The confirmation duration deliberately matches the
/// existing 600 ms capture window; capture itself starts a *new* window after
/// this verification, so it is never relaxed by the recovery check.
const Duration kRecoveryVerificationHold = Duration(milliseconds: 600);

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

/// The accepted evidence retained while the user photographs one target again.
/// This is deliberately memory-only: the durable manifest remains on the old
/// photograph until the replacement capture publishes a new complete snapshot.
class _RetakeSnapshot {
  const _RetakeSnapshot({
    required this.targetId,
    required this.target,
    required this.station,
    required this.phase,
  });

  final int targetId;
  final CaptureTarget target;
  final CaptureStation? station;
  final CapturePhase phase;
}

/// Relates the arbitrary sensor-attitude frame to the AR camera frame at the
/// instant the station was calibrated.
///
/// This is deliberately ephemeral: it is not a station pose, is never
/// persisted, and cannot alter P0, q, an anchor, or any target coordinate. It
/// only lets the UI draw a labelled 2D orientation hint while ARCore restores
/// its world reference.
class _RecoveryGuidanceCalibration {
  const _RecoveryGuidanceCalibration({
    required this.sensorAtCalibration,
    required this.arCameraAtCalibration,
  });

  final Quat sensorAtCalibration;
  final Quat arCameraAtCalibration;
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
    ArPoseSource? recoveryGuidanceSource,
    ArTrackingTrace? trackingTrace,
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
       _recoveryGuidanceSource = recoveryGuidanceSource,
       _trace = trackingTrace ?? ArTrackingTrace.disabled(),
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

  /// Optional attitude-only source for a visibly non-authoritative recovery
  /// cue. It never replaces [_session] and is never passed into the capture
  /// gate.
  final ArPoseSource? _recoveryGuidanceSource;

  /// The on-disk tracking trace. Defaults to a trace that records nothing, so
  /// every existing caller and test is unaffected until one is passed in.
  final ArTrackingTrace _trace;

  /// The camera config arrives on exactly one pose event per session, so it is
  /// recorded once rather than on every frame that carries the cached value.
  bool _cameraConfigTraced = false;

  /// One drift-jump line per continuity epoch: a flapping diagnostic must not
  /// fill the trace file.
  int? _driftJumpTracedEpoch;

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

  /// Native-session provenance captured with the original shared station
  /// anchor.  A tracking epoch detects a break *inside* a session, but it
  /// resets for a newly-created ARCore world, so it cannot prove that the
  /// recovered frame belongs to this station on its own.
  String? _stationSessionId;
  String? _stationAnchorId;

  /// Recovery is separate from ordinary capture stability. While it is pending
  /// the target geometry stays frozen, the arrow is neutral, and automatic
  /// capture is blocked until the station reference has been verified again.
  bool _recoveryVerificationPending = false;

  /// The configurable deadline begins only after fresh camera tracking returns.
  /// Time while Android has deliberately paused the ARCore camera is not a
  /// failed recovery attempt.
  DateTime? _recoveryStartedAt;
  DateTime? _recoveryStableSince;
  Vec3? _recoveryStableStationLocalCamera;
  Timer? _recoveryDeadlineTimer;
  int _recoveryDeadlineGeneration = 0;

  /// Monotonic within a calibrated station. It distinguishes one genuinely new
  /// recovery boundary from the ordinary per-frame polling that follows it.
  int _recoveryAttempt = 0;

  /// One explanation per recovery episode is enough. In particular, a blank
  /// wall can leave ARCore PAUSED for much longer than a timer-based recovery
  /// policy; that is not evidence that the calibrated station disappeared.
  bool _recoveryAnchorWaitLogged = false;

  /// A map correction can be present on every pose while recovery verifies it.
  /// Keep one diagnostic for that episode rather than flooding Logcat at the
  /// pose-stream rate and obscuring the actual tracking transitions.
  bool _recoveryWorldCorrectionLogged = false;

  /// One diagnostic is sufficient when the phone naturally moved while ARCore
  /// was unavailable.  That movement is never proof that the station anchor
  /// belongs to another world frame.
  bool _recoveryCameraMovementLogged = false;

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
    _stationSessionId = null;
    _stationAnchorId = null;
    _recoveryAttempt = 0;
    ++_anchorRefreshGeneration;
    _resetRecoveryVerification(clearTrustedReference: true);
    _clearRecoveryGuidanceCalibration();
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
    _retakeSnapshot = null;
    _stationSessionId = null;
    _stationAnchorId = null;
    _recoveryAttempt = 0;
    ++_anchorRefreshGeneration;
    _frame = null;
    _engine.clearStation();
    _frozenNodePositions = const <Vec3>[];
    _resetRecoveryVerification(clearTrustedReference: true);
    _clearRecoveryGuidanceCalibration();
    _lastStationTrustDiag = null;
    _anchorStabilityId = null;
    _anchorStabilityReference = null;
    _anchorStabilityStationLocalCamera = null;
    _resetGuidanceHistory();
    debugPrint('ARCORE_STATION: station cleared.');
  }

  /// Binds the frozen station to the one native anchor deliberately created by
  /// calibration (or by an explicit full re-scan).  Recovery never calls this:
  /// a different id while recovering is evidence that the reference changed,
  /// not an invitation to adopt a replacement anchor.
  void _bindStationReference(
    AnchoredTarget placement, {
    required String source,
  }) {
    final sessionId = placement.anchor.sessionId ?? _session.sessionId;
    _stationAnchorId = placement.anchor.id;
    _stationSessionId = sessionId;
    debugPrint(
      'ARCORE_STATION: reference bound source=$source '
      'session=${sessionId ?? 'unreported'} '
      'anchor=${placement.anchor.id}.',
    );
    _trace.record('stationReferenceBound', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'source': source,
      'sessionId': sessionId,
      'anchorId': placement.anchor.id,
      'stationId': state.station?.stationId,
    });
  }

  bool _hasStationSessionMismatch(String? frameSessionId) {
    final expected = _stationSessionId;
    if (expected == null) return false;
    final active = _session.sessionId;
    return (frameSessionId != null && frameSessionId != expected) ||
        (active != null && active != expected);
  }

  bool _hasVerifiedStationSession(String? frameSessionId) {
    final expected = _stationSessionId;
    if (expected == null || frameSessionId == null) return false;
    final active = _session.sessionId;
    return frameSessionId == expected && (active == null || active == expected);
  }

  void _clearRecoveryGuidanceCalibration() {
    _recoveryGuidanceExpiry?.cancel();
    _recoveryGuidanceExpiry = null;
    _recoveryGuidanceCalibration = null;
    _lastRecoveryGuidancePublishedAt = null;
    _lastRecoveryGuidanceTargetId = null;
  }

  /// Removes a temporary sensor-only cue as soon as its source is no longer
  /// fresh. A held-over arrow would be misleading during an ARCore outage,
  /// even though it can never satisfy a capture gate.
  void _clearUnverifiedGuidanceIfPresent([CaptureState? snapshot]) {
    _recoveryGuidanceExpiry?.cancel();
    _recoveryGuidanceExpiry = null;
    if (_disposed) return;
    final current = snapshot ?? state;
    if (current.unverifiedGuidance != null) {
      _emit(current.copyWith(clearUnverifiedGuidance: true));
    }
  }

  static bool _usableOrientation(Quat orientation) =>
      orientation.x.isFinite &&
      orientation.y.isFinite &&
      orientation.z.isFinite &&
      orientation.w.isFinite &&
      orientation.length > 1e-6;

  /// Captures a one-session bridge between the sensor attitude and the ARCore
  /// camera at calibration time. It is intentionally not reconstructed on a
  /// later frame: doing so would let a tracking recovery redefine the visual
  /// reference the user calibrated.
  void _captureRecoveryGuidanceCalibration(ArPoseSample arPose) {
    final sensor = _recoveryGuidanceSource?.latest;
    if (sensor == null || sensor.tracking != TrackingState.tracking) return;
    final sensorAtCalibration = sensor.captureWorld;
    final arCameraAtCalibration = arPose.raw.captureWorld;
    if (!_usableOrientation(sensorAtCalibration) ||
        !_usableOrientation(arCameraAtCalibration)) {
      return;
    }
    _recoveryGuidanceCalibration = _RecoveryGuidanceCalibration(
      sensorAtCalibration: sensorAtCalibration.normalized,
      arCameraAtCalibration: arCameraAtCalibration.normalized,
    );
    debugPrint(
      'ARCORE_RECOVERY_CUE: sensor bridge frozen at calibration; '
      'it is display-only and cannot arm capture.',
    );
  }

  /// Starts the optional attitude stream without making AR startup depend on
  /// sensors. A missing or failed sensor source simply means no recovery cue.
  Future<void> _startRecoveryGuidance() async {
    final source = _recoveryGuidanceSource;
    if (source == null) return;
    final generation = ++_recoveryGuidanceGeneration;
    await _recoveryGuidanceSub?.cancel();
    if (generation != _recoveryGuidanceGeneration ||
        _disposed ||
        _shuttingDown) {
      return;
    }
    _recoveryGuidanceSub = source.poses.listen(
      (ArPose pose) {
        if (generation == _recoveryGuidanceGeneration) {
          _onRecoveryGuidancePose(pose);
        }
      },
      onError: (Object error, StackTrace stackTrace) {
        if (generation != _recoveryGuidanceGeneration || _disposed) return;
        debugPrint('ARCORE_RECOVERY_CUE: sensor hint stopped: $error');
        _clearUnverifiedGuidanceIfPresent();
      },
    );
    try {
      await source.start();
      if (generation != _recoveryGuidanceGeneration ||
          _disposed ||
          _shuttingDown) {
        return;
      }
      final latest = source.latest;
      if (latest != null) _onRecoveryGuidancePose(latest);
    } catch (error) {
      if (generation != _recoveryGuidanceGeneration || _disposed) return;
      debugPrint('ARCORE_RECOVERY_CUE: sensor hint unavailable: $error');
      _clearUnverifiedGuidanceIfPresent();
    }
  }

  /// Stops the optional sensor source. Its attitude is not valid across an app
  /// pause, so no cue may survive this call.
  Future<void> _stopRecoveryGuidance() async {
    ++_recoveryGuidanceGeneration;
    await _recoveryGuidanceSub?.cancel();
    _recoveryGuidanceSub = null;
    _clearRecoveryGuidanceCalibration();
    _clearUnverifiedGuidanceIfPresent();
    final source = _recoveryGuidanceSource;
    if (source == null) return;
    try {
      await source.stop();
    } catch (error) {
      debugPrint('ARCORE_RECOVERY_CUE: could not stop sensor hint: $error');
    }
  }

  bool _needsUnverifiedGuidance(CaptureState snapshot) =>
      !_stationReferenceInvalidated &&
      !_stationInvalidationInFlight &&
      snapshot.phase == CapturePhase.capturing &&
      snapshot.station != null &&
      snapshot.activeTarget != null &&
      (snapshot.tracking != TrackingState.tracking ||
          snapshot.trackingRecovering ||
          (snapshot.activeAnchored && !snapshot.activeAnchorTracking));

  void _onRecoveryGuidancePose(ArPose sensorPose) {
    if (_disposed || _shuttingDown) return;
    _refreshUnverifiedGuidance(sensorPose);
  }

  /// Publishes only a camera-frame direction, never a reconstructed target
  /// world position. The cue is therefore safe across a temporary ARCore loss:
  /// it cannot translate P0, q, the shared anchor, or any of the 26 targets.
  void _refreshUnverifiedGuidance([ArPose? sample]) {
    final snapshot = state;
    if (!_needsUnverifiedGuidance(snapshot)) {
      _clearUnverifiedGuidanceIfPresent(snapshot);
      return;
    }
    final bridge = _recoveryGuidanceCalibration;
    final frame = _frame;
    final target = snapshot.activeTarget;
    final sensor = sample ?? _recoveryGuidanceSource?.latest;
    if (bridge == null ||
        frame == null ||
        target == null ||
        sensor == null ||
        sensor.tracking != TrackingState.tracking) {
      _clearUnverifiedGuidanceIfPresent(snapshot);
      return;
    }

    final sensorNow = sensor.captureWorld;
    if (!_usableOrientation(sensorNow)) {
      _clearUnverifiedGuidanceIfPresent(snapshot);
      return;
    }

    // Station local -> frozen AR world -> calibration camera -> current sensor
    // camera. The only sensor operation is a *relative rotation*; the sensor's
    // arbitrary global heading never mixes with ARCore's world frame.
    final stationDirectionWorld = frame.orientation
        .rotate(target.stationLocalDirection)
        .normalized;
    final atCalibrationCamera = bridge.arCameraAtCalibration.conjugate
        .rotate(stationDirectionWorld)
        .normalized;
    final relativeCamera =
        (sensorNow.conjugate * bridge.sensorAtCalibration).normalized;
    final directionInCamera = relativeCamera
        .rotate(atCalibrationCamera)
        .normalized;
    if (directionInCamera.length <= 1e-6 ||
        !directionInCamera.x.isFinite ||
        !directionInCamera.y.isFinite ||
        !directionInCamera.z.isFinite) {
      _clearUnverifiedGuidanceIfPresent(snapshot);
      return;
    }

    final now = _clock();
    final last = _lastRecoveryGuidancePublishedAt;
    if (last != null &&
        _lastRecoveryGuidanceTargetId == target.targetId &&
        now.difference(last) < const Duration(milliseconds: 33)) {
      return;
    }
    _lastRecoveryGuidancePublishedAt = now;
    final targetChanged = _lastRecoveryGuidanceTargetId != target.targetId;
    _lastRecoveryGuidanceTargetId = target.targetId;
    if (targetChanged) {
      debugPrint(
        'ARCORE_RECOVERY_CUE: target ${target.targetId}; '
        'AR position unverified, cue cannot capture.',
      );
    }
    _emit(
      snapshot.copyWith(
        unverifiedGuidance: UnverifiedGuidance(
          targetId: target.targetId,
          directionInCamera: directionInCamera,
        ),
      ),
    );
    _recoveryGuidanceExpiry?.cancel();
    final cueGeneration = _recoveryGuidanceGeneration;
    _recoveryGuidanceExpiry = Timer(snapshot.geometry.maxMeasurementAge, () {
      if (_disposed || cueGeneration != _recoveryGuidanceGeneration) return;
      final current = state;
      final cue = current.unverifiedGuidance;
      if (cue == null || cue.targetId != target.targetId) return;
      debugPrint(
        'ARCORE_RECOVERY_CUE: sensor sample expired; hiding the '
        'non-authoritative direction cue.',
      );
      _clearUnverifiedGuidanceIfPresent(current);
    });
  }

  /// Drops recovery-only evidence. This never alters the station or the 26
  /// frozen nodes; only [_clearStation] or explicit calibration may do that.
  void _resetRecoveryVerification({bool clearTrustedReference = false}) {
    _cancelRecoveryDeadline();
    _recoveryVerificationPending = false;
    _recoveryStartedAt = null;
    _recoveryStableSince = null;
    _recoveryStableStationLocalCamera = null;
    _recoveryWorldCorrectionLogged = false;
    _recoveryAnchorWaitLogged = false;
    _recoveryCameraMovementLogged = false;
    if (clearTrustedReference) {
      _lastTrustedStationAnchor = null;
      _lastTrustedStationLocalCamera = null;
    }
  }

  void _cancelRecoveryDeadline() {
    ++_recoveryDeadlineGeneration;
    _recoveryDeadlineTimer?.cancel();
    _recoveryDeadlineTimer = null;
  }

  /// Starts the bounded recovery period only once camera tracking has resumed.
  /// A timer is required in addition to pose-driven checks: a stalled native
  /// frame stream must still give the operator an explicit next action.
  void _startRecoveryDeadline(DateTime startedAt) {
    if (!_recoveryVerificationPending || _recoveryDeadlineTimer != null) return;
    _recoveryStartedAt = startedAt;
    final timeout = state.geometry.stationRecoveryTimeout;
    final generation = ++_recoveryDeadlineGeneration;
    debugPrint(
      'ARCORE_RECOVERY: verification deadline started '
      'session=${_stationSessionId ?? 'unreported'} '
      'anchor=${_stationAnchorId ?? 'none'} '
      'attempt=$_recoveryAttempt '
      'timeoutMs=${timeout.inMilliseconds}.',
    );
    _traceStationDiag(
      'recovering:deadline',
      'fresh camera tracking returned; waiting up to '
          '${timeout.inMilliseconds} ms for the retained station anchor.',
    );
    _trace.record('recoveryStart', {
      'wallMs': startedAt.millisecondsSinceEpoch,
      'timeoutMs': timeout.inMilliseconds,
      'sessionId': _session.sessionId,
      'expectedSessionId': _stationSessionId,
      'stationId': state.station?.stationId,
      'anchorId': _stationAnchorId,
      'attempt': _recoveryAttempt,
      'cameraTracking': state.tracking.name,
      'anchorTracking': _placement?.anchor.tracking.name,
      'automaticCapturePermitted':
          state.mode.isAutomatic && state.readyToCapture,
      'epoch': _lastTrackingEpoch,
    });
    _recoveryDeadlineTimer = Timer(timeout, () {
      if (_disposed ||
          _shuttingDown ||
          generation != _recoveryDeadlineGeneration ||
          !_recoveryVerificationPending) {
        return;
      }
      _recoveryDeadlineTimer = null;
      _invalidateStationReference(
        'AR could not verify the original station reference within '
        '${timeout.inSeconds} seconds. Recalibrate at the station to continue.',
        recoveryFailureReason: 'recovery-timeout',
      );
    });
  }

  void _beginRecoveryVerification({
    required String reason,
    required int? epoch,
  }) {
    if (_stationReferenceInvalidated) return;
    if (_recoveryVerificationPending) {
      // A second interruption while verification was pending gets a new budget
      // from the next fresh TRACKING frame; a paused camera cannot consume it.
      ++_recoveryAttempt;
      _cancelRecoveryDeadline();
      _recoveryStartedAt = null;
      _recoveryStableSince = null;
      _recoveryStableStationLocalCamera = null;
      return;
    }
    ++_recoveryAttempt;
    _recoveryVerificationPending = true;
    _recoveryStartedAt = null;
    _recoveryStableSince = null;
    _recoveryStableStationLocalCamera = null;
    _recoveryWorldCorrectionLogged = false;
    _recoveryAnchorWaitLogged = false;
    _recoveryCameraMovementLogged = false;
    _smoother.reset();
    _engine.resetStability();
    _resetGuidanceHistory();
    debugPrint(
      'ARCORE_RECOVERY: continuity boundary reason=$reason '
      'attempt=$_recoveryAttempt '
      'epoch=${epoch?.toString() ?? 'unreported'} '
      'trustedAnchor=${_lastTrustedStationAnchor?.id ?? 'none'} '
      'trustedAnchorPose=${_lastTrustedStationAnchor == null ? 'none' : _formatVec(_lastTrustedStationAnchor!.position)} '
      'trustedAnchorQ=${_lastTrustedStationAnchor == null ? 'none' : _formatQuat(_lastTrustedStationAnchor!.orientation)} '
      'trustedCameraLocal='
      '${_lastTrustedStationLocalCamera == null ? 'none' : _formatVec(_lastTrustedStationLocalCamera!)}. '
      'Guidance and automatic capture are blocked pending verification.',
    );
    _traceStationDiag(
      'recovering:boundary',
      'continuity boundary: $reason. Capture and guidance are blocked.',
    );
    final trustedAnchor = _lastTrustedStationAnchor;
    final trustedCamera = _lastTrustedStationLocalCamera;
    final station = _frame;
    _trace.record('recoveryBoundary', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'reason': reason,
      'attempt': _recoveryAttempt,
      'sessionId': _session.sessionId,
      'expectedSessionId': _stationSessionId,
      'stationId': state.station?.stationId,
      'stationPosition': station == null
          ? null
          : <double>[station.origin.x, station.origin.y, station.origin.z],
      'anchorId': _stationAnchorId,
      'trustedAnchorId': trustedAnchor?.id,
      'trustedAnchorPosition': trustedAnchor == null
          ? null
          : <double>[
              trustedAnchor.position.x,
              trustedAnchor.position.y,
              trustedAnchor.position.z,
            ],
      'trustedAnchorOrientation': trustedAnchor == null
          ? null
          : <double>[
              trustedAnchor.orientation.x,
              trustedAnchor.orientation.y,
              trustedAnchor.orientation.z,
              trustedAnchor.orientation.w,
            ],
      'trustedCameraStationLocal': trustedCamera == null
          ? null
          : <double>[trustedCamera.x, trustedCamera.y, trustedCamera.z],
      'epoch': epoch,
    });
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
    final frame = _frame;
    final anchor = _placement?.anchor;
    final renderAllowed = _sceneTracked(state);
    debugPrint(
      'ARCORE_STATION_DIAG: $trustState '
      'wallMs=${_clock().millisecondsSinceEpoch} '
      'frameTsMs=${_lastPoseFrameTsMs?.toString() ?? 'n/a'} '
      'failureReason=${_lastPoseFailureReason ?? 'none'} '
      'session=${_session.sessionId ?? 'unreported'} '
      'expectedSession=${_stationSessionId ?? 'none'} '
      'epoch=${_lastTrackingEpoch?.toString() ?? 'unreported'} '
      'attempt=$_recoveryAttempt '
      'stationId=${state.station?.stationId ?? 'none'} '
      'station=${frame == null ? 'none' : _formatVec(frame.origin)} '
      'anchor=${anchor?.id ?? 'none'} '
      'expectedAnchor=${_stationAnchorId ?? 'none'} '
      'cameraTracking=${state.tracking.name} '
      'anchorTracking=${anchor?.tracking.name ?? 'unreported'} '
      'anchored=${state.activeAnchored} '
      'recovery=${state.stationRecoveryPhase.name} '
      'render=$renderAllowed captureReady=${state.readyToCapture} '
      'autoPermitted=${state.mode.isAutomatic && state.readyToCapture} '
      'invalidated=$_stationReferenceInvalidated. $detail',
    );
    _trace.record('stationTrust', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'trust': trustState,
      'frameTsMs': _lastPoseFrameTsMs,
      'failureReason': _lastPoseFailureReason,
      'sessionId': _session.sessionId,
      'expectedSessionId': _stationSessionId,
      'epoch': _lastTrackingEpoch,
      'attempt': _recoveryAttempt,
      'stationId': state.station?.stationId,
      'stationPosition': frame == null
          ? null
          : <double>[frame.origin.x, frame.origin.y, frame.origin.z],
      'anchorId': anchor?.id,
      'expectedAnchorId': _stationAnchorId,
      'cameraTracking': state.tracking.name,
      'anchorTracking': anchor?.tracking.name,
      'anchored': state.activeAnchored,
      'recoveryPhase': state.stationRecoveryPhase.name,
      'renderAllowed': renderAllowed,
      'captureReady': state.readyToCapture,
      'automaticCapturePermitted':
          state.mode.isAutomatic && state.readyToCapture,
      'invalidated': _stationReferenceInvalidated,
      'driftM': _metres(state.driftM),
      'gyroHz': _achievedSensorHz,
    });
  }

  /// The gyroscope rate actually being delivered, when the guidance source is
  /// the real sensor-backed one.
  ///
  /// `null` on hosts without a sensor source and before the first accepted
  /// sample: an absent rate is honest, `0.0` would blame the sensor for a stream
  /// that never started.
  double? get _achievedSensorHz {
    final source = _recoveryGuidanceSource;
    return source is SensorPoseSource ? source.achievedSampleHz : null;
  }

  /// Records the camera ARCore selected, once per session, with the rate the
  /// sensors are actually delivering.
  ///
  /// Both belong together because the two remaining VIO hypotheses — weak
  /// visual input and platform pressure — are told apart by exactly this pair:
  /// a `fixed` focus mode or a 640x480 still stream points at the image, and a
  /// delivered sensor rate far below the requested one points at the platform.
  void _traceCameraConfig(ArCameraQuality quality) {
    if (_cameraConfigTraced) return;
    _cameraConfigTraced = true;
    debugPrint(
      'ARCORE_TRACE: camera ${quality.stillWidth}x${quality.stillHeight} still, '
      '${quality.previewWidth}x${quality.previewHeight} preview, '
      'focus=${quality.focusMode}, gyroHz=${_achievedSensorHz?.toStringAsFixed(1) ?? 'n/a'}.',
    );
    _trace.record('cameraConfig', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'stillWidth': quality.stillWidth,
      'stillHeight': quality.stillHeight,
      'previewWidth': quality.previewWidth,
      'previewHeight': quality.previewHeight,
      'focusMode': quality.focusMode,
      'epoch': _lastTrackingEpoch,
      'attempt': _recoveryAttempt,
      'gyroHz': _achievedSensorHz,
    });
  }

  /// Records a metres-scale drift reading, naming which definition produced it.
  ///
  /// `driftM` carries two incompatible meanings in this controller — the
  /// anchor-compensated station distance and the raw world-to-frozen-origin
  /// distance. After a map correction the second can read metres for a phone
  /// that never moved, and without this line the two are indistinguishable in
  /// the file. At most one line per continuity epoch.
  void _traceDriftJump({
    required double publishedM,
    required bool compensated,
    required double? rawWorldM,
    required bool anchorTracked,
  }) {
    if (publishedM < 1.0) return;
    final epoch = _lastTrackingEpoch;
    if (epoch != null && epoch == _driftJumpTracedEpoch) return;
    _driftJumpTracedEpoch = epoch;
    _trace.record('driftJump', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'publishedM': _metres(publishedM),
      'definition': compensated ? 'anchorCompensated' : 'rawWorldMinusP0',
      'rawWorldM': _metres(rawWorldM),
      'anchorTracked': anchorTracked,
      'tracking': state.tracking.name,
      'failureReason': _lastPoseFailureReason,
      'epoch': epoch,
      'gyroHz': _achievedSensorHz,
    });
  }

  /// Metres at millimetre resolution, which is as fine as this file needs to
  /// be and small enough to read.
  static double? _metres(double? value) =>
      value == null ? null : double.parse(value.toStringAsFixed(3));

  /// Creates continuity evidence while the already-calibrated station is live
  /// and tracked. Camera-to-station movement remains diagnostic evidence; it
  /// does not replace session and anchor continuity verification.
  void _recordTrustedStationReference({
    required String anchorId,
    required ({Vec3 position, Quat orientation, TrackingState tracking}) anchor,
    required Vec3 stationLocalCamera,
  }) {
    if (_recoveryVerificationPending ||
        _stationReferenceInvalidated ||
        anchor.tracking != TrackingState.tracking) {
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
    required String? frameSessionId,
    required String expectedAnchorId,
    required ({Vec3 position, Quat orientation, TrackingState tracking})?
    anchor,
    required Vec3? stationLocalCamera,
    required bool poseFresh,
  }) {
    if (!_recoveryVerificationPending) return true;

    final stationSessionId = _stationSessionId;
    final stationAnchorId = _stationAnchorId;
    if (_hasStationSessionMismatch(frameSessionId)) {
      _invalidateStationReference(
        'AR resumed in a different session, so the calibrated world frame '
        'cannot be continued. Recalibrate at the station to continue.',
        recoveryFailureReason: 'session-mismatch',
      );
      return false;
    }
    if (stationSessionId == null ||
        stationAnchorId == null ||
        !_hasVerifiedStationSession(frameSessionId)) {
      _recoveryStableSince = null;
      _recoveryStableStationLocalCamera = null;
      if (!_recoveryAnchorWaitLogged) {
        _recoveryAnchorWaitLogged = true;
        final reason = stationSessionId == null || stationAnchorId == null
            ? 'the calibrated session or anchor identity was not recorded'
            : 'the recovered frame has no matching native session identity';
        debugPrint(
          'ARCORE_RECOVERY: retaining frozen station geometry while waiting; '
          '$reason. Guidance and capture remain blocked.',
        );
        _traceStationDiag('recovering:session', reason);
      }
      return false;
    }
    if (expectedAnchorId != stationAnchorId) {
      _invalidateStationReference(
        'The active station anchor no longer matches the calibrated anchor. '
        'Recalibrate at the station to continue.',
        recoveryFailureReason: 'anchor-id-mismatch',
      );
      return false;
    }

    // ARCore's PAUSED state is recoverable: it retains the native session and
    // the shared station anchor while it searches for visual features again.
    // STOPPED is different — that world reference cannot safely be continued.
    if (anchor?.tracking == TrackingState.stopped) {
      _invalidateStationReference(
        'AR stopped tracking the station anchor. Recalibrate at the station '
        'to continue.',
        recoveryFailureReason: 'anchor-stopped',
      );
      return false;
    }

    if (!poseFresh ||
        anchor == null ||
        anchor.tracking != TrackingState.tracking ||
        stationLocalCamera == null) {
      _recoveryStableSince = null;
      _recoveryStableStationLocalCamera = null;
      if (!_recoveryAnchorWaitLogged) {
        _recoveryAnchorWaitLogged = true;
        final reason = !poseFresh
            ? 'the recovered camera pose is stale'
            : anchor == null
            ? 'the station anchor was not reported'
            : 'the station anchor is ${anchor.tracking.name}';
        debugPrint(
          'ARCORE_RECOVERY: retaining frozen station geometry while waiting; '
          '$reason. Guidance and capture remain blocked.',
        );
        _traceStationDiag(
          'recovering:waiting',
          'the frozen station and all targets are retained while $reason.',
        );
      }
      return false;
    }

    final trustedAnchor = _lastTrustedStationAnchor;
    if (trustedAnchor != null) {
      final sameAnchor = trustedAnchor.id == expectedAnchorId;
      if (!sameAnchor) {
        _invalidateStationReference(
          'The recovered station anchor is not the calibrated anchor. '
          'Recalibrate at the station to continue.',
          recoveryFailureReason: 'anchor-id-mismatch',
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

    // A phone may legitimately move while tracking is unavailable (for example,
    // while the user turns back towards visual features).  The old recovery
    // policy treated that ordinary camera-to-anchor translation as evidence of
    // a new world frame and invalidated a still-live original anchor.  Keep the
    // delta as a diagnostic only: session provenance, original anchor identity,
    // current TRACKING states, and fresh multi-frame evidence above are the
    // continuity proof.
    final trustedCamera = _lastTrustedStationLocalCamera;
    final localCameraShiftM = trustedCamera == null
        ? null
        : (stationLocalCamera - trustedCamera).length;
    if (localCameraShiftM != null &&
        localCameraShiftM > kRecoveryAnchorPositionDiscontinuityM &&
        !_recoveryCameraMovementLogged) {
      _recoveryCameraMovementLogged = true;
      debugPrint(
        'ARCORE_RECOVERY: recovered camera moved '
        '${localCameraShiftM.toStringAsFixed(3)} m relative to the retained '
        'station anchor; recorded as user motion, not reference loss.',
      );
    }

    final recoveryDuration = _recoveryStartedAt == null
        ? null
        : now.difference(_recoveryStartedAt!);
    _recoveryVerificationPending = false;
    _cancelRecoveryDeadline();
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
      '${recoveryDuration?.inMilliseconds ?? kRecoveryVerificationHold.inMilliseconds} ms; '
      'session=$stationSessionId '
      'station=${_formatVec(frame.origin)} anchor=${_formatVec(anchor.position)} '
      'cameraLocal=${_formatVec(stationLocalCamera)}. '
      'A new capture stability window is required.',
    );
    _trace.record('recoveryVerified', {
      'wallMs': now.millisecondsSinceEpoch,
      'durationMs':
          recoveryDuration?.inMilliseconds ??
          kRecoveryVerificationHold.inMilliseconds,
      'sessionId': frameSessionId,
      'expectedSessionId': stationSessionId,
      'stationId': state.station?.stationId,
      'anchorId': expectedAnchorId,
      'cameraTracking': TrackingState.tracking.name,
      'anchorTracking': anchor.tracking.name,
      'stationPosition': <double>[
        frame.origin.x,
        frame.origin.y,
        frame.origin.z,
      ],
      'anchorPosition': <double>[
        anchor.position.x,
        anchor.position.y,
        anchor.position.z,
      ],
      'cameraStationLocal': <double>[
        stationLocalCamera.x,
        stationLocalCamera.y,
        stationLocalCamera.z,
      ],
      'epoch': _lastTrackingEpoch,
      'attempt': _recoveryAttempt,
      'automaticCapturePermitted':
          state.mode.isAutomatic && state.readyToCapture,
    });
    _traceStationDiag(
      'trusted',
      'reference verified; the frozen station and all 26 target positions '
          'are unchanged.',
    );
    return true;
  }

  /// Explicitly abandons a reference that no longer has a demonstrably
  /// continuous ARCore world frame. This is intentionally not a recalibration:
  /// the user must press Calibrate after the old scene has been released.
  void _invalidateStationReference(
    String message, {
    String? recoveryFailureReason,
  }) {
    if (_stationReferenceInvalidated || _stationInvalidationInFlight) return;
    final failureReason = recoveryFailureReason ?? message;
    _stationReferenceInvalidated = true;
    _stationInvalidationInFlight = true;
    final frozenCount = _frozenNodePositions.length;
    final capturedCount = state.capturedCount;
    final retainedTargets = state.targets;
    final station = _frame;
    debugPrint(
      'ARCORE_RECOVERY: station invalidated; '
      'session=${_stationSessionId ?? 'unreported'} '
      'anchor=${_stationAnchorId ?? 'none'} '
      'origin=${station == null ? 'none' : _formatVec(station.origin)} '
      'frozenTargets=$frozenCount. $message',
    );
    _trace.record('recoveryFailed', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'reason': failureReason,
      'message': message,
      'sessionId': _session.sessionId,
      'expectedSessionId': _stationSessionId,
      'stationId': state.station?.stationId,
      'anchorId': _placement?.anchor.id,
      'expectedAnchorId': _stationAnchorId,
      'cameraTracking': state.tracking.name,
      'anchorTracking': _placement?.anchor.tracking.name,
      'stationPosition': station == null
          ? null
          : <double>[station.origin.x, station.origin.y, station.origin.z],
      'epoch': _lastTrackingEpoch,
      'attempt': _recoveryAttempt,
      'renderAllowed': _sceneTracked(state),
      'captureReady': state.readyToCapture,
      'automaticCapturePermitted':
          state.mode.isAutomatic && state.readyToCapture,
    });
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
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.anchorRecoveryRequired,
        recoveryFailureReason: failureReason,
        message: capturedCount == 0
            ? message
            : '$message $capturedCount of ${retainedTargets.length} '
                  'captured photographs are safely saved. Recalibrate at the '
                  'station to continue.',
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
  double _captureHoldProgressOf(vio.FrameGateEvaluation gate) => math
      .min(_stabilityProgressOf(gate), _alignmentHoldProgressOf())
      .toDouble();

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

  /// Last frame's mode-agnostic capture eligibility (all conditions except the
  /// operating-mode check). Lets [captureNow] honour the same steady / aligned /
  /// anchored / non-refining gate that arms automatic capture without
  /// recomputing the stability window from a synthetic frame.
  bool _lastCaptureEligible = false;

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
  StreamSubscription<ArPose>? _recoveryGuidanceSub;

  /// The calibration-relative bridge for the temporary amber 2D cue. A null
  /// bridge means no sensor hint is shown; it never weakens the AR-only gates.
  _RecoveryGuidanceCalibration? _recoveryGuidanceCalibration;
  DateTime? _lastRecoveryGuidancePublishedAt;
  int? _lastRecoveryGuidanceTargetId;
  Timer? _recoveryGuidanceExpiry;

  /// Changes whenever the optional sensor source is stopped or restarted, so
  /// an old asynchronous start cannot publish a cue into a newer AR session.
  int _recoveryGuidanceGeneration = 0;

  StationFrame? _frame;

  /// The complete station-calibration transaction, when one is already
  /// running.  A placement slot only protects individual anchor operations;
  /// without this future, two quick calibration taps can both observe
  /// `state.station == null` before either shared-anchor request completes.
  /// Joining the first transaction preserves one station id, one anchor, and
  /// one manifest publication.
  Future<bool>? _calibrationInFlight;

  /// Pre-loaded meshes, when the caller supplied them; otherwise loaded from the
  /// asset bundle on boot.
  GlbMeshSource? _meshes;
  AnchoredTarget? _placement;
  DateTime _lastAnchorRefresh = DateTime.fromMillisecondsSinceEpoch(0);

  /// At most one native refresh may be pending.  The generation makes a late
  /// result harmless after a target switch, reference reset, or new session.
  bool _anchorRefreshInFlight = false;
  int _anchorRefreshGeneration = 0;
  int? _activeAnchorRefreshGeneration;

  /// Whether the "waiting for AR to stabilise the target" line is already up
  /// for the current pause: `1` once it has been said, `0` while tracking.
  ///
  /// It used to count refreshes towards a re-placement. It does not any more:
  /// a pause is a tracking state to wait out, not a countdown to destroying
  /// and rebuilding an anchor that was about to be fine.
  int _anchorNotTracking = 0;

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

  /// Why the most recently published scene could not draw. This is a
  /// transition latch, not a gate: it lets a device Logcat identify a
  /// persistent missing marker without turning a 15 Hz pose stream into noise.
  String? _lastSceneBlocker;

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

  /// The station-relative camera captured with [_anchorStabilityReference].
  /// World-frame deltas alone cannot distinguish a shared ARCore world
  /// correction (camera and anchor move together — the scene the user sees is
  /// unchanged) from an anchor-only jump (the scene really relocated). This
  /// witness is invariant under shared rigid corrections and moves by exactly
  /// the anchor-only or camera-only part, so the two events become separable.
  /// Stored only with the baseline, never refreshed on ordinary frames.
  Vec3? _anchorStabilityStationLocalCamera;

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

  /// Set only during an explicit retake. It prevents a failed replacement from
  /// turning an already accepted target into a failed/pending durable row.
  _RetakeSnapshot? _retakeSnapshot;

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
    await _stopRecoveryGuidance();
    _lastPoseReceivedAt = null;
    _lastTrackingEpoch = null;
    // A new session gets a new camera config and a new epoch zero, so both
    // once-per-session trace latches have to reopen with it.
    _cameraConfigTraced = false;
    _driftJumpTracedEpoch = null;
    _emit(
      state.copyWith(
        phase: CapturePhase.checking,
        message: 'Checking AR support…',
        clearError: true,
        clearArFault: true,
        clearCameraQuality: true,
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
      // A sensor hint may fail or take up to its own first-sample timeout. It
      // is advisory only, so ARCore's normal boot must not wait for it.
      unawaited(_startRecoveryGuidance());

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
  }) {
    final existing = _calibrationInFlight;
    if (existing != null) return existing;

    final transaction = _calibrateStation(
      stationId: stationId,
      completedImages: completedImages,
      createdAt: createdAt,
    );
    _calibrationInFlight = transaction;
    unawaited(
      transaction.then<void>(
        (_) => _clearCalibrationInFlight(transaction),
        onError: (Object _, StackTrace _) =>
            _clearCalibrationInFlight(transaction),
      ),
    );
    return transaction;
  }

  void _clearCalibrationInFlight(Future<bool> transaction) {
    if (identical(_calibrationInFlight, transaction)) {
      _calibrationInFlight = null;
    }
  }

  Future<bool> _calibrateStation({
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

    // Place the complete arrangement before publishing or persisting anything.
    // A single station anchor hosts all 26 placements for the whole sequence —
    // one capture origin, not twenty-six independently refinable anchors —
    // and a photograph never adds, removes or moves a node. `placeAll` is
    // idempotent by target id, so a retry after a failed calibration cannot
    // produce a second node for one target.
    final outcome = await _withPlacementSlot<PlacementOutcome>(
      () => _anchors.placeAll(station: frame, specs: _specs),
    );
    // The native anchor request crosses an asynchronous boundary.  A camera
    // that was TRACKING when the user tapped Calibrate may have paused (or the
    // ARCore session may have been recreated) before the native host answers.
    // Do not freeze or persist a station from that stale pre-placement pose.
    //
    // The current Kotlin host supplies all three identities.  The nullable
    // comparisons preserve compatibility with an older host that did not yet
    // report session provenance, while still rejecting every positive mismatch
    // rather than treating it as a recovered world frame.
    final postPlacementReadiness = calibrationReadiness;
    final expectedSessionId = pose.sessionId ?? _session.sessionId;
    final liveSessionId = _session.sessionId;
    final anchorSessionId = outcome.placed.isEmpty
        ? null
        : outcome.placed.values.first.anchor.sessionId;
    final sessionChanged =
        expectedSessionId != null &&
        ((liveSessionId != null && liveSessionId != expectedSessionId) ||
            (anchorSessionId != null && anchorSessionId != expectedSessionId));
    final placementSucceeded =
        outcome.allPlaced && outcome.placedCount == _specs.length;
    if (!placementSucceeded ||
        !postPlacementReadiness.isReady ||
        sessionChanged) {
      final missing = <int>{
        ...outcome.failures.keys,
        for (final spec in _specs)
          if (!outcome.placed.containsKey(spec.targetId)) spec.targetId,
      }.toList()..sort();
      _traceTransition(
        'calibration placement failed: ${outcome.placedCount} of '
        '${_specs.length} targets; failures=${outcome.failures}',
      );

      // `placeAll` may have created the one shared station anchor before a
      // later placement failed. It was never committed, so discard that
      // transient arrangement rather than retaining a partial reference for a
      // future retry. None of the repository methods have run at this point:
      // previously accepted photos and records stay intact.
      await _anchors.releaseAll();
      await _anchors.clearScene();
      await _session.setActiveAnchor(null);
      _placement = null;
      _clearStation();
      _hiddenTargets = const <int>{};
      _smoother.reset();
      _emit(
        state.copyWith(
          phase: CapturePhase.scanning,
          clearStation: true,
          targets: const <CaptureTarget>[],
          activeTargetId: 0,
          probes: const <int, SurfaceProbe>{},
          clearAlignment: true,
          clearUnverifiedGuidance: true,
          stable: false,
          stabilityProgress: 0,
          capturing: false,
          retryPending: false,
          autoStage: AutoCaptureStage.idle,
          clearAutoBlockReason: true,
          measurementFresh: false,
          activeAnchored: false,
          activeAnchorTracking: false,
          driftM: 0,
          trackingRecovering: false,
          stationRecoveryPhase: StationRecoveryPhase.normalTracking,
          clearRecoveryFailureReason: true,
          message: !placementSucceeded
              ? (missing.isEmpty
                    ? 'AR could not establish the shared station anchor. Move '
                          'slowly and try calibration again.'
                    : 'AR could not place all targets (${missing.join(', ')}). '
                          'Move slowly and try calibration again.')
              : sessionChanged
              ? 'AR restarted while placing the station anchor. No station '
                    'was saved; wait for tracking and calibrate again.'
              : 'AR tracking changed while placing the station anchor. No '
                    'station was saved; wait for tracking and calibrate again.',
        ),
      );
      return false;
    }

    var placedTargets = targets;
    for (final entry in outcome.placed.entries) {
      placedTargets = _withAnchorIn(placedTargets, entry.key, entry.value);
    }

    // `_freezeStation` clears the guidance history — the pointer filter, the
    // aligned latch and the automatic machine's stage. Do it only after the
    // shared station anchor and every local target transform exist, so an
    // anchor refusal cannot leave an in-memory or durable phantom station.
    _freezeStation(frame);
    _captureRecoveryGuidanceCalibration(pose);
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

    final existing = await _repository.loadCaptureState(id);
    await _repository.saveCaptureState(
      station: station,
      targets: placedTargets,
      // A recalibration can retain verified photos from a previous AR session.
      // Include their records in this first manifest rather than reintroducing
      // the old station/targets/records three-file crash window.
      records: existing?.records ?? const <CaptureRecord>[],
    );
    // A new station record exists now; the resume list must know about it.
    _savedStationIds = await _repository.stationIds();

    _emit(
      state.copyWith(
        phase: CapturePhase.capturing,
        station: station,
        targets: placedTargets,
        activeTargetId: firstPending,
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.normalTracking,
        clearRecoveryFailureReason: true,
        message: 'Placed ${outcome.placedCount} targets.',
        clearAlignment: true,
        clearUnverifiedGuidance: true,
        // A calibration begins a new station-displacement diagnostic. Do not
        // show a stale value from the previous station while the first live
        // anchor report is still arriving.
        driftM: 0,
      ),
    );
    _bindStationReference(outcome.placed.values.first, source: 'calibration');

    // Probe all 26 directions as information, never as a gate. The target
    // sheet says which directions happen to have a surface, but placement and
    // capture do not depend on it: targets are placed at `station + radius ×
    // direction` whether or not any surface exists.
    await probeTargets();

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
    final savedState = await _repository.loadCaptureState(stationId);
    if (savedState == null) return false;
    final station = savedState.station;
    final saved = savedState.targets;
    if (saved.length != _specs.length) return false;
    final records = savedState.records;
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
    final capturedCount = accepted.length;
    final resumedStation = station.copyWith(capturedCount: capturedCount);
    await _repository.saveCaptureState(
      station: resumedStation,
      targets: targets,
      records: records,
    );

    _emit(
      state.copyWith(
        phase: CapturePhase.capturing,
        station: resumedStation,
        targets: targets,
        activeTargetId: 0,
        clearAlignment: true,
        clearMessage: true,
        message: 'Resumed: $capturedCount of ${targets.length} saved.',
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.normalTracking,
        clearRecoveryFailureReason: true,
        driftM: 0,
      ),
    );

    // Place whatever the controller no longer holds after the restart.
    // `placeAll` is idempotent by target id, so this can only add the missing
    // nodes and start on the first target that still needs a photograph.
    final outcome = await _withPlacementSlot(
      () => _anchors.placeAll(station: _frame!, specs: _specs),
    );
    if (outcome.placed.isNotEmpty) {
      _bindStationReference(
        outcome.placed.values.first,
        source: 'saved-station-resume',
      );
    }
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
    final savedState = await _repository.loadCaptureState(stationId);
    if (savedState == null) return false;
    final station = savedState.station;
    final records = savedState.records;
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
  /// arrangement around the station it was recorded from cannot be trusted,
  /// so an explicit recalibration is required.
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
        clearUnverifiedGuidance: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchored: false,
        activeAnchorTracking: false,
        activeSurfaceType: null,
        driftM: 0,
        // A new station is a new beginning: the recovery notice belongs to the
        // alignment that was cleared with it, not to the next one.
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.normalTracking,
        clearRecoveryFailureReason: true,
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
  /// and aims the guidance, nothing else. Losing the shared station anchor
  /// invalidates the reference and requires an explicit recalibration.
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
          clearUnverifiedGuidance: true,
          stable: false,
          stabilityProgress: 0,
          activeAnchored: false,
          activeAnchorTracking: false,
          retryPending: false,
          autoStage: AutoCaptureStage.waiting,
          autoBlockReason: 'no-station',
          message:
              'The station reference is unavailable. Recalibrate at the '
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

    // A genuine target switch gets a fresh anchor-tracking observation. It
    // does not create a replacement anchor: every target must remain attached
    // to the one calibrated station anchor.
    if (targetId != state.activeTargetId) {
      _anchorNotTracking = 0;
      // An in-flight refresh describes the previous active target. It may still
      // finish, but its generation can no longer publish into this selection.
      ++_anchorRefreshGeneration;
    }

    _engine.resetStability();
    final placement = _anchors.placementFor(targetId);
    _placement = placement;
    _lastAnchorRefresh = _clock();
    if (placement != null && _stationAnchorId == null) {
      _bindStationReference(placement, source: 'active-target-$targetId');
    }

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
          clearUnverifiedGuidance: true,
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
        clearUnverifiedGuidance: true,
        stable: false,
        stabilityProgress: 0,
        driftM: 0,
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
    _refreshUnverifiedGuidance();
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
    _stationAnchorId = null;
    _stationSessionId = null;
    ++_anchorRefreshGeneration;
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
    if (outcome.placed.isNotEmpty) {
      _bindStationReference(
        outcome.placed.values.first,
        source: 'explicit-rescan',
      );
    }
    var placedTargets = state.targets;
    for (final entry in outcome.placed.entries) {
      placedTargets = _withAnchorIn(placedTargets, entry.key, entry.value);
    }
    final missing = outcome.failures.keys.toList()..sort();
    _emit(
      state.copyWith(
        targets: placedTargets,
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.normalTracking,
        clearRecoveryFailureReason: true,
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
    // This is an explicit fatal-error retry, not an ordinary lifecycle pause.
    // Dispose and stop here so the next boot gets a genuinely new ARCore world
    // origin; normal backgrounding deliberately does neither.
    await _anchors.dispose();
    await _session.stop();
    _lifecycleSessionPaused = false;
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
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.normalTracking,
        clearRecoveryFailureReason: true,
        stable: false,
        stabilityProgress: 0,
        driftM: 0,
      ),
    );
    await boot(width: width, height: height);
  }

  /// Prepares one accepted target for a replacement photograph.
  ///
  /// The current JPEG, record and manifest stay committed until the new JPEG
  /// has passed verification and its replacement snapshot is published. A
  /// crash or failed retake therefore resumes on the old accepted photograph,
  /// not a blank target with the only evidence overwritten.
  Future<void> retake(int targetId) async {
    final index = targetId - 1;
    if (index < 0 || index >= state.targets.length) return;
    if (_retakeSnapshot != null) {
      _emit(
        state.copyWith(
          message: 'Finish or retry the current replacement photograph first.',
        ),
      );
      return;
    }
    final original = state.targets[index];
    if (!original.status.isDone) {
      await selectTarget(targetId);
      return;
    }
    _retakeSnapshot = _RetakeSnapshot(
      targetId: targetId,
      target: original,
      station: state.station,
      phase: state.phase,
    );
    final updated = List<CaptureTarget>.of(state.targets);
    updated[index] = updated[index].copyWith(
      status: CaptureTargetStatus.pending,
      clearMessage: true,
    );
    // This is deliberately an in-memory preview only. Persisting `pending`
    // here would make a process death or failed write discard the previously
    // accepted manifest before a replacement existed.
    _emit(
      state.copyWith(
        targets: updated,
        activeTargetId: targetId,
        phase: state.phase == CapturePhase.complete
            ? CapturePhase.capturing
            : state.phase,
        message:
            'Retaking target $targetId — the previous photograph is kept '
            'until the replacement is verified.',
      ),
    );
    await selectTarget(targetId);
  }

  // ── capture ───────────────────────────────────────────────────────────

  /// Legacy API retained for callers compiled against an earlier build. The
  /// product no longer permits manual shuttering: only the automatic gate can
  /// fire after a stable, aligned pose has held for its required duration.
  /// Manual shutter.
  ///
  /// Fires only when the same alignment, tracking, station, anchor-immobility
  /// and 600 ms-steadiness gate that arms automatic capture is currently
  /// satisfied, so an explicit tap can never accept a mis-aimed, blurry,
  /// drifting or freshly-relocalised frame. The eligibility flag is refreshed
  /// by every pose frame (see [_captureVerdict]); with no recent frame it is
  /// `false`, which is also the safe answer. Fire-and-forget, like the
  /// automatic path, so the in-flight latch in [_shoot] is what serialises a
  /// double tap into one photograph.
  Future<void> captureNow() async {
    if (!_lastCaptureEligible) {
      debugPrint(
        'ARCORE_CAPTURE: manual capture request ignored — frame is not '
        'aligned/steady enough to accept ($_autoBlockReason).',
      );
      return;
    }
    _shoot(source: CaptureTriggerSource.manual);
  }

  /// Switches between manual and automatic capture.
  ///
  /// Manual gives the user an explicit shutter ([captureNow]) and stops the
  /// automatic machine from firing or advancing on its own; automatic fires
  /// and advances once the stability gate is held. Switching modes resets the
  /// machine to a neutral stage so a stale block reason from the other mode is
  /// never displayed.
  void setCaptureMode(CaptureMode next) {
    if (next == _mode) return;
    _mode = next;
    _autoStage = next.isAutomatic
        ? AutoCaptureStage.waiting
        : AutoCaptureStage.idle;
    _autoBlockReason = next.isAutomatic ? 'awaiting-frame' : 'mode-manual';
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
    // A failed capture can leave the last pose-frame verdict armed until the
    // next pose arrives.  Do not let a second manual tap reuse that stale
    // verdict: a failure is retryable only after the operator deliberately
    // breaks and re-establishes the aim window.  The automatic path already
    // sees this in [_captureVerdict], but the transaction boundary must enforce
    // it too because [captureNow] is fire-and-forget.
    if (state.retryPending) {
      debugPrint(
        'ARCORE_CAPTURE: manual capture request ignored — re-aim before retry.',
      );
      return;
    }

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
    // that reaches [_captureVerdict] in that window is refused by
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
    CaptureCommitResult? durableCommit;
    try {
      final committed = await _photos.capture(
        station: station,
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
      durableCommit = committed;
      final record = committed.record;
      if (_retakeSnapshot?.targetId == target.targetId) {
        _retakeSnapshot = null;
      }
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

      final capturedCount = updated
          .where((CaptureTarget t) => t.status.isDone)
          .length;
      debugPrint(
        'ARCORE_CAPTURE: capture success [${source.label}] target '
        '${target.targetId} -> captured; $capturedCount of '
        '${updated.length} recorded.',
      );
      // The image, record, target status and station count were committed as
      // one immutable manifest before PhotoCaptureService returned. Rebuild
      // only the live target list here so a user selection made during the
      // file write is not overwritten by an older persisted presentation row.
      final updatedStation = committed.station;
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
            message:
                'Photo saved. AR could not prepare target $next — '
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
      final committed = durableCommit;
      if (committed != null) {
        await _handlePostCommitPresentationFailure(
          committed: committed,
          target: target,
          error: f,
        );
        return;
      }
      debugPrint(
        'ARCORE_CAPTURE: capture failed [${source.label}] target '
        '${target.targetId}: ${f.message} — target returned to retryable.',
      );
      if (await _restoreRetakeAfterFailure(
        target: target,
        message: f.message,
      )) {
        return;
      }
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
      final committed = durableCommit;
      if (committed != null) {
        await _handlePostCommitPresentationFailure(
          committed: committed,
          target: target,
          error: e,
        );
        return;
      }
      debugPrint(
        'ARCORE_CAPTURE: capture failed [${source.label}] target '
        '${target.targetId}: $e — target returned to retryable.',
      );
      final message = 'Capture failed: $e';
      if (await _restoreRetakeAfterFailure(target: target, message: message)) {
        return;
      }
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

  /// Keeps live presentation failures from rewriting a completed manifest.
  ///
  /// Once [PhotoCaptureService] returns, the JPEG, record, target and station
  /// count are already one durable snapshot. Repository refreshes and AR scene
  /// updates are best-effort presentation work; treating any of their failures
  /// as a capture failure would publish a contradictory `failed` target row.
  Future<void> _handlePostCommitPresentationFailure({
    required CaptureCommitResult committed,
    required CaptureTarget target,
    required Object error,
  }) async {
    debugPrint(
      'ARCORE_CAPTURE: photograph for target ${target.targetId} was '
      'durably saved, but presentation work failed: $error',
    );
    if (!_savedStationIds.contains(committed.station.stationId)) {
      _savedStationIds = <String>[
        committed.station.stationId,
        ..._savedStationIds,
      ];
    }

    final sameStation = state.station?.stationId == committed.station.stationId;
    final index = state.targets.indexWhere(
      (CaptureTarget current) => current.targetId == target.targetId,
    );
    if (!sameStation || index < 0) {
      // The user moved to another station while the commit completed. Do not
      // overwrite that newer live state; the committed station will be offered
      // from the saved-station list on the next refresh.
      _emit(state.copyWith(capturing: false));
      return;
    }

    final reconciled = List<CaptureTarget>.of(state.targets);
    reconciled[index] = reconciled[index].copyWith(
      status: CaptureTargetStatus.captured,
      imagePath: committed.record.imagePath,
      clearMessage: true,
    );
    final allDone = reconciled.every(
      (CaptureTarget current) => current.status.isDone,
    );
    final completedTargetWasActive = state.activeTargetId == target.targetId;
    if (completedTargetWasActive) {
      _placement = null;
      _engine.resetStability();
      _resetGuidanceHistory();
    }
    _emit(
      state.copyWith(
        station: committed.station,
        targets: reconciled,
        capturing: false,
        activeTargetId: completedTargetWasActive ? 0 : state.activeTargetId,
        phase: allDone ? CapturePhase.complete : state.phase,
        clearAlignment: completedTargetWasActive,
        clearUnverifiedGuidance: completedTargetWasActive,
        stable: completedTargetWasActive ? false : null,
        stabilityProgress: completedTargetWasActive ? 0 : null,
        activeAnchored: completedTargetWasActive ? false : null,
        activeAnchorTracking: completedTargetWasActive ? false : null,
        retryPending: false,
        autoStage: allDone ? AutoCaptureStage.complete : AutoCaptureStage.idle,
        clearAutoBlockReason: true,
        lastSavedPath: committed.record.imagePath,
        imageRotationDeg: committed.record.rotationAppliedDeg,
        lastError: 'Photo saved, but AR guidance could not update: $error',
        message:
            'Photo saved for target ${target.targetId}. Open Review targets '
            'to continue guidance.',
      ),
    );
    if (!completedTargetWasActive) return;
    try {
      await _pushScene(state, targets: reconciled);
    } catch (sceneError) {
      debugPrint(
        'ARCORE_CAPTURE: retained committed target ${target.targetId}; '
        'scene recovery also failed: $sceneError',
      );
    }
    try {
      await _session.setActiveAnchor(null);
    } catch (anchorError) {
      debugPrint(
        'ARCORE_CAPTURE: retained committed target ${target.targetId}; '
        'active-anchor cleanup failed: $anchorError',
      );
    }
  }

  /// Restores the already accepted target after a replacement attempt failed.
  /// No repository write happens here: the prior immutable manifest is still
  /// authoritative, and writing a failure row would only create a second
  /// crash window around evidence that is already safe on disk.
  Future<bool> _restoreRetakeAfterFailure({
    required CaptureTarget target,
    required String message,
  }) async {
    final saved = _retakeSnapshot;
    if (saved == null || saved.targetId != target.targetId) return false;
    _retakeSnapshot = null;
    final sameStation = state.station?.stationId == saved.station?.stationId;
    final index = state.targets.indexWhere(
      (CaptureTarget current) => current.targetId == target.targetId,
    );
    if (!sameStation || index < 0) return false;

    final restored = List<CaptureTarget>.of(state.targets);
    // Retain the newest live anchor geometry, but restore the accepted status
    // and image path from the durable snapshot.
    restored[index] = restored[index].copyWith(
      status: CaptureTargetStatus.captured,
      imagePath: saved.target.imagePath,
      clearMessage: true,
    );
    final allDone = restored.every(
      (CaptureTarget current) => current.status.isDone,
    );
    _placement = null;
    _engine.resetStability();
    _resetGuidanceHistory();
    _emit(
      state.copyWith(
        phase: allDone && saved.phase == CapturePhase.complete
            ? CapturePhase.complete
            : CapturePhase.capturing,
        station: saved.station,
        targets: restored,
        activeTargetId: 0,
        capturing: false,
        retryPending: false,
        clearAlignment: true,
        clearUnverifiedGuidance: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchored: false,
        activeAnchorTracking: false,
        autoStage: allDone ? AutoCaptureStage.complete : AutoCaptureStage.idle,
        clearAutoBlockReason: true,
        lastError:
            'Replacement failed; the previous photograph was kept. $message',
        message:
            'Replacement failed; the previous photograph for target '
            '${target.targetId} was kept.',
      ),
    );
    await _pushScene(state, targets: restored);
    await _session.setActiveAnchor(null);
    return true;
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

    // One event per session carries ARCore's chosen camera config; the trace
    // has to be opened on that event or the number is gone.
    final incomingQuality = pose.cameraQuality;
    if (incomingQuality != null) _traceCameraConfig(incomingQuality);

    if (_hasStationSessionMismatch(pose.sessionId) &&
        next.phase == CapturePhase.capturing &&
        next.station != null) {
      _invalidateStationReference(
        'AR resumed in a different session, so the calibrated world frame '
        'cannot be continued. Recalibrate at the station to continue.',
        recoveryFailureReason: 'session-mismatch',
      );
      return;
    }

    // ARCore uses STOPPED for a terminal tracking loss. There is no valid
    // world reference to preserve in that state, so do not let the ordinary
    // PAUSED recovery path wait forever. Publish the terminal tracking state
    // first, then use the one explicit recalibration path to release it.
    if (pose.tracking == TrackingState.stopped &&
        next.phase == CapturePhase.capturing &&
        next.station != null) {
      _emit(
        next.copyWith(
          trackingRecovering: false,
          stationRecoveryPhase: StationRecoveryPhase.anchorRecoveryRequired,
          clearAlignment: true,
          stable: false,
          stabilityProgress: 0,
          activeAnchorTracking: false,
        ),
      );
      _invalidateStationReference(
        'AR tracking stopped, so the calibrated station reference cannot be '
        'continued. Recalibrate at the station to continue.',
        recoveryFailureReason: 'camera-stopped',
      );
      return;
    }

    // Native continuity epochs cover the gap that ordinary event coalescing
    // used to hide: PAUSED -> TRACKING can happen entirely between two Flutter
    // deliveries. A state transition remains the compatibility fallback for
    // older hosts that do not publish an epoch yet.
    if (trackingChanged || pose.trackingTransition || epochChanged) {
      debugPrint(
        'ARCORE_RECOVERY: camera=${state.tracking.name}->${pose.tracking.name} '
        'session=${pose.sessionId ?? 'unreported'} '
        'nativeTransition=${pose.trackingTransition} '
        'epoch=${pose.trackingEpoch?.toString() ?? 'unreported'} '
        'epochChanged=$epochChanged.',
      );
      // The failure reason that belongs to *this* boundary, sampled by the host
      // on the same frame the state changed on. This is the line that tells
      // `INSUFFICIENT_LIGHT` from `EXCESSIVE_MOTION` from an ARCore internal
      // VIO reset, which is the question the logcat-only path could not answer
      // after the session closed.
      _trace.record('boundary', {
        'wallMs': _clock().millisecondsSinceEpoch,
        'frameTsMs': pose.raw.timestampNs == null
            ? null
            : pose.raw.timestampNs! ~/ 1000000,
        'cameraFrom': state.tracking.name,
        'cameraTo': pose.tracking.name,
        'failureReason': pose.failureReason,
        'sessionId': pose.sessionId,
        'expectedSessionId': _stationSessionId,
        'stationId': state.station?.stationId,
        'expectedAnchorId': _stationAnchorId,
        'epoch': pose.trackingEpoch,
        'nativeTransition': pose.trackingTransition,
        'epochChanged': epochChanged,
        'anchorTracking': pose.activeAnchorPose?.tracking.name,
        'anchorId': pose.activeAnchorId,
        'cameraPosition': <double>[
          pose.position.x,
          pose.position.y,
          pose.position.z,
        ],
        'cameraOrientation': <double>[
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
        ],
        'anchorPosition': pose.activeAnchorPose == null
            ? null
            : <double>[
                pose.activeAnchorPose!.position.x,
                pose.activeAnchorPose!.position.y,
                pose.activeAnchorPose!.position.z,
              ],
        'anchorOrientation': pose.activeAnchorPose == null
            ? null
            : <double>[
                pose.activeAnchorPose!.orientation.x,
                pose.activeAnchorPose!.orientation.y,
                pose.activeAnchorPose!.orientation.z,
                pose.activeAnchorPose!.orientation.w,
              ],
        'planesTracked': pose.planesTracked,
        'gyroHz': _achievedSensorHz,
      });
    }
    final recoveryBoundary =
        epochChanged ||
        trackingLeft ||
        trackingResumed ||
        (pose.trackingTransition && state.tracking == TrackingState.tracking);
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
        stationRecoveryPhase: pose.tracking == TrackingState.tracking
            ? StationRecoveryPhase.recovering
            : StationRecoveryPhase.trackingInterrupted,
        clearRecoveryFailureReason: true,
        clearAlignment: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchorTracking: false,
      );
    } else if (!_recoveryVerificationPending &&
        next.trackingRecovering &&
        pose.tracking == TrackingState.tracking) {
      // The previous frame completed recovery verification. The engine was
      // reset there, so this frame starts a fresh normal 600 ms capture hold.
      next = next.copyWith(
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.normalTracking,
        clearRecoveryFailureReason: true,
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

      // The recovery deadline starts from a fresh TRACKING frame, never from
      // the preceding PAUSED interval or an app-lifecycle pause.
      if (_recoveryVerificationPending &&
          pose.tracking == TrackingState.tracking &&
          _recoveryStartedAt == null) {
        _startRecoveryDeadline(now);
      }

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
      // * **the drift figure** — a diagnostic measurement. Growing it from a
      //   pose ARCore does not trust would make the diagnostic misleading;
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
        final matchingLive =
            live != null && pose.activeAnchorId == placement.anchor.id
            ? live
            : null;
        // A terminal anchor state is not a temporary camera interruption. The
        // live pose is deliberately ignored below unless it is TRACKING, so
        // handle STOPPED before that conversion can turn this into an ambiguous
        // `null` and leave the operator waiting for the recovery deadline.
        if (matchingLive?.tracking == TrackingState.stopped) {
          _invalidateStationReference(
            'AR stopped tracking the shared station anchor. Recalibrate at the '
            'station to continue.',
            recoveryFailureReason: 'anchor-stopped',
          );
          return;
        }
        if (_stationAnchorId != null &&
            placement.anchor.id != _stationAnchorId) {
          _invalidateStationReference(
            'The active station anchor no longer matches the calibrated anchor. '
            'Recalibrate at the station to continue.',
            recoveryFailureReason: 'anchor-id-mismatch',
          );
          return;
        }
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
        final liveStationPose = matchingLive?.tracking == TrackingState.tracking
            ? matchingLive
            : null;
        final liveAnchorTracked = liveStationPose != null;
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
            stationRecoveryPhase: StationRecoveryPhase.recovering,
            clearRecoveryFailureReason: true,
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
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
        // station-relative pose. This affects the diagnostic displacement and
        // stability window. Alignment deliberately continues to use the raw
        // camera and the live target, which is the actual geometry on screen.
        //
        // Computed once, before the anchor classifiers below, so the anchor
        // discontinuity check, the hold-refinement check, the station jump
        // check and the gate all read one coherent frame snapshot.
        // `stationLocalCamera` is invariant under any shared rigid world
        // correction (camera and anchor move together) and moves by exactly an
        // anchor-only or camera-only jump — which makes it the witness that
        // separates "the world map was corrected" from "the reference really
        // moved".
        Vec3? stationLocalCamera;
        if (liveStationPose != null) {
          stationLocalCamera = liveStationPose.orientation.conjugate.rotate(
            pose.position - liveStationPose.position,
          );
        }
        final poseFresh =
            timing.nowNanoseconds - timing.sampleNanoseconds <=
            state.geometry.maxMeasurementAge.inMicroseconds * 1000;

        // ARCore can sometimes keep reporting TRACKING across a map correction.
        // In that case there is no PAUSED event to begin recovery, but a large
        // station-anchor transform change would still make every marker jump on
        // screen.  Do not quietly accept that new transform as a new target
        // position.  Hide guidance, preserve the frozen station-local layout,
        // and make the existing continuity verifier prove that the camera and
        // station still describe the same physical reference.  A jump the
        // station-relative camera does *not* share is a shared world-map
        // correction instead: the rendered image does not move, so it is
        // logged as a refinement and never starts recovery.
        final anchorDiscontinuity =
            !_recoveryVerificationPending && liveAnchorTracked
            ? _stationAnchorTransformDiscontinuity(
                anchorId: placement.anchor.id,
                anchor: liveStationPose,
                stationLocalCamera: stationLocalCamera,
              )
            : null;
        if (anchorDiscontinuity != null) {
          _beginRecoveryVerification(
            reason:
                'station anchor transform jumped '
                '${anchorDiscontinuity.positionM.toStringAsFixed(3)} m / '
                '${anchorDiscontinuity.rotationDeg.toStringAsFixed(1)}°',
            epoch: pose.trackingEpoch,
          );
          next = next.copyWith(
            trackingRecovering: true,
            stationRecoveryPhase: StationRecoveryPhase.recovering,
            clearRecoveryFailureReason: true,
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
          );
        }
        // ── the target position: renderer parity ─────────────────────────
        // The native renderer reads `anchor.pose` every frame, so guidance must
        // aim at that same live position whenever ARCore tracks this selected
        // anchor. Before the event arrives, the placement is an honest fallback;
        // another target's live pose never is.
        final targetPosition = liveStationPose != null
            ? liveStationPose.position +
                  liveStationPose.orientation.rotate(
                    placement.stationLocalPosition,
                  )
            : placement.worldPosition;

        // A shared ARCore map correction can legitimately move both the camera
        // and every anchor. We compensate that translation for the station
        // distance below, but must not let it preserve an already-filled
        // capture window: the rendered target has moved, so require another
        // uninterrupted hold against its newly refined pose.
        final anchorRefined =
            liveStationPose != null &&
            _hasAnchorRefined(
              // All 26 nodes share the calibrated station anchor.  Its id on
              // its own is therefore not a stability identity: reusing the
              // previous target's position as this target's reference makes
              // the ordinary ~0.52 m separation between adjacent nodes look
              // like a sudden anchor correction.  Track the live anchor and
              // the selected frozen node as one stability episode instead.
              stabilityId: '${placement.anchor.id}:target-${target.targetId}',
              targetPosition: targetPosition,
              stationLocalCamera: stationLocalCamera,
            );
        if (anchorRefined) {
          _engine.resetStability();
          _alignedSinceNs = null;
          _alignedHeldMs = 0;
          final frozenNode =
              target.targetId > 0 &&
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
        // The 600 ms hold and pose validation are the engine's. One
        // implementation, asserted by the plain-`dart` suite,
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
        // The number the user is about to see, with the definition that
        // produced it. `frame.driftOf(rawCamera)` is the same subtraction the
        // uncorrected publish path uses, so a large value here that does *not*
        // appear in `publishedM` is a shared world-map correction rather than a
        // phone that moved.
        _traceDriftJump(
          publishedM: liveStationPose == null
              ? state.driftM
              : gate.stationDriftM,
          compensated: liveStationPose != null,
          rawWorldM: frame.driftOf(rawCamera),
          anchorTracked: liveAnchorTracked,
        );
        final stable = gate.isPoseSteady;

        final wasRecovering = _recoveryVerificationPending;
        final recoveryVerified = _verifyRecoveryReference(
          now: now,
          frame: frame,
          frameSessionId: pose.sessionId,
          expectedAnchorId: placement.anchor.id,
          anchor: liveStationPose,
          stationLocalCamera: stationLocalCamera,
          poseFresh: poseFresh,
        );
        if (_stationReferenceInvalidated) return;
        final verifiedThisFrame =
            wasRecovering && recoveryVerified && !_recoveryVerificationPending;
        if (verifiedThisFrame) {
          next = next.copyWith(
            stationRecoveryPhase: StationRecoveryPhase.referenceVerified,
            clearRecoveryFailureReason: true,
          );
        }
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
          next = next.copyWith(
            clearAlignment: true,
            stable: false,
            stabilityProgress: 0,
            measurementFresh: gate.stabilityRejection != 'measurement-age',
            angularRateDegPerSec: gate.angularVelocityDegPerSec,
            // The raw station-relative reading remains observable for logs and
            // diagnostics, but not actionable until verification completes.
            driftM: liveStationPose == null ? state.driftM : gate.stationDriftM,
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
            // A station-displacement diagnostic is valid only when the current
            // camera frame and the current target anchor agree on the same
            // ARCore map. Until then the shutter is already blocked by
            // `activeAnchorTracking`; retain the last trusted value to avoid a
            // relocalisation jump in diagnostics.
            driftM: liveStationPose == null ? state.driftM : gate.stationDriftM,
          );
        }
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
              ((timing.nowNanoseconds - timing.sampleNanoseconds) ~/ 1000000),
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
        final armed = _captureVerdict(
          next: next,
          gate: gate,
          trackingOk: trackingOk,
          target: target,
          anchorRefined: anchorRefined,
          requireAutomatic: false,
        );
        _lastCaptureEligible = armed.eligible;
        final verdict = mode.isAutomatic
            ? armed
            : (
                eligible: false,
                stage: AutoCaptureStage.idle,
                block: 'mode-manual',
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
        // example, while ARCore is relocalising from a feature-poor view).
        // Keep the frozen reference and wait for the *same* shared anchor;
        // `_refreshAnchorIfDue` performs the destructive path only when the
        // host confirms that anchor is actually gone.
        if (_recoveryVerificationPending && trackingOk && target != null) {
          // Poll the existing shared anchor while its own tracking recovers.
          // This never creates a replacement: a missing shared anchor makes
          // the frozen world reference untrustworthy and is handled by the
          // explicit invalidation below.
          _refreshAnchorIfDue();
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
        _traceDriftJump(
          publishedM: uncorrectedDrift,
          compensated: false,
          rawWorldM: uncorrectedDrift,
          anchorTracked: next.activeAnchorTracking,
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
    // Tracking and hidden-set changes each result in one scene push rather than
    // a push on every pose frame.
    final hidden = _hiddenTargetsIn(next, next.targets);
    final sceneTrackingChanged = _sceneTracked(next) != _sceneTracking;
    final sceneBlockerChanged = _sceneBlocker(next) != _lastSceneBlocker;
    if (sceneTrackingChanged ||
        sceneBlockerChanged ||
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

    // Once ARCore has a tracked camera, a tracked shared anchor, and no
    // recovery verification pending, the native marker and authoritative arrow
    // take over. Remove the sensor cue before publishing that trusted frame.
    if (!_needsUnverifiedGuidance(next)) {
      _recoveryGuidanceExpiry?.cancel();
      _recoveryGuidanceExpiry = null;
      next = next.copyWith(clearUnverifiedGuidance: true);
    }

    // The indicator's hysteresis needs this frame's answer next frame.
    _lastGuidance = next.guidance;

    _traceGuidanceTransition(next: next);
    _traceStabilityTransition(next);
    _emit(next);
    if (_needsUnverifiedGuidance(next)) {
      _refreshUnverifiedGuidance();
    }
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

  /// The capture conditions, evaluated against this frame.
  ///
  /// [requireAutomatic] gates on the operating mode: the automatic machine
  /// passes `true` (so manual mode never auto-fires), while [captureNow] uses
  /// `false` to learn whether the frame would be capturable regardless of mode.
  ///
  /// Ordered so the first failing condition is the one the user should act on:
  /// tracking, then station, then aim, then steadiness, then bookkeeping. The
  /// order matters for the status line as much as for the log — "waiting for
  /// tracking" is more useful than "hold still" when ARCore has no pose.
  ({bool eligible, AutoCaptureStage stage, String? block}) _captureVerdict({
    required CaptureState next,
    required vio.FrameGateEvaluation gate,
    required bool trackingOk,
    required CaptureTarget target,
    required bool anchorRefined,
    required bool requireAutomatic,
  }) {
    // 1. Mode.
    if (requireAutomatic && !mode.isAutomatic) {
      return (
        eligible: false,
        stage: AutoCaptureStage.idle,
        block: 'mode-manual',
      );
    }
    // 2. ARCore tracking.
    if (!trackingOk || next.tracking != TrackingState.tracking) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'tracking',
      );
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
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'pose-age',
      );
    }
    // 4. Station calibrated, and 5. the frozen reference unchanged.
    if (next.station == null || !_engine.isCalibrated) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'no-station',
      );
    }
    if (!_stationReferenceIntact()) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'station-changed',
      );
    }
    // 6. Target bookkeeping, before the geometric conditions: a target that
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
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'no-anchor',
      );
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
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'alignment',
      );
    }
    if (!alignment.withinTolerance || !gate.isAligned) {
      return (
        eligible: false,
        stage: AutoCaptureStage.waiting,
        block: 'alignment',
      );
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
    final dot = a.normalized.dot(b.normalized).abs().clamp(0.0, 1.0).toDouble();
    return 2 * math.acos(dot) * 180 / math.pi;
  }

  /// Detects a discontinuous update of the one anchor that defines the frozen
  /// station-local target layout.
  ///
  /// Normal ARCore refinement is allowed to be small and continuous. A larger
  /// update resets guidance and capture stability so no photograph is accepted
  /// against an unverified frame. It does not rewrite the station or any target
  /// and it does not hide a still-tracked shared anchor: camera motion relative
  /// to that anchor can also be an ordinary user walking back to the station.
  ///
  /// A world-frame jump alone does not say *what* moved. A shared world-map
  /// correction moves the camera and the anchor by the same rigid transform:
  /// the world transform jumps, but the station-relative camera
  /// ([stationLocalCamera]) does not, and the rendered image the user sees is
  /// pixel-identical — recovery would be false. Only a jump the station-relative
  /// camera also feels can relocate the markers relative to the viewer, and
  /// that is the case this method reports.
  ({double positionM, double rotationDeg})?
  _stationAnchorTransformDiscontinuity({
    required String anchorId,
    required ({Vec3 position, Quat orientation, TrackingState tracking}) anchor,
    required Vec3? stationLocalCamera,
  }) {
    final previous = _lastTrustedStationAnchor;
    if (previous == null || previous.id != anchorId) return null;
    final positionM = (anchor.position - previous.position).length;
    final rotationDeg = _quaternionAngleDeg(
      anchor.orientation,
      previous.orientation,
    );
    if (positionM <= kRecoveryAnchorPositionDiscontinuityM &&
        rotationDeg <= kRecoveryAnchorOrientationDiscontinuityDeg) {
      return null;
    }
    final trustedCamera = _lastTrustedStationLocalCamera;
    final cameraJumpM = stationLocalCamera == null || trustedCamera == null
        ? null
        : (stationLocalCamera - trustedCamera).length;
    if (cameraJumpM != null &&
        cameraJumpM <= kRecoveryAnchorPositionDiscontinuityM) {
      // Shared world correction: the station-relative geometry — the thing the
      // user sees and the thing recovery protects — did not change.
      debugPrint(
        'ARCORE_WORLD_REFINEMENT: station anchor world transform corrected by '
        '${positionM.toStringAsFixed(3)} m / '
        '${rotationDeg.toStringAsFixed(1)}° while the station-local camera '
        'moved only ${cameraJumpM.toStringAsFixed(3)} m; shared world-map '
        'correction, rendered geometry unchanged, no recovery.',
      );
      return null;
    }
    debugPrint(
      'ARCORE_RECOVERY: station-anchor discontinuity while TRACKING '
      '(${positionM.toStringAsFixed(3)} m, '
      '${rotationDeg.toStringAsFixed(1)}°, '
      'station-local camera '
      '${cameraJumpM == null ? 'unmeasured' : '${cameraJumpM.toStringAsFixed(3)} m'}); '
      'blocking guidance and capture until station continuity is verified; '
      'frozen marker geometry is retained.',
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
  ///
  /// The world-frame target position alone cannot say whether the *station*
  /// moved: a shared ARCore world correction translates camera and anchor
  /// together, so the target exceeds the allowance while the geometry the user
  /// actually sees — the station-relative camera-to-target relationship — is
  /// unchanged. [stationLocalCamera] is that witness: invariant under a shared
  /// rigid correction, moved by an anchor-only jump. When the world position
  /// exceeds the allowance but the station-relative camera does not, the event
  /// is re-based as a world refinement ([ARCORE_WORLD_REFINEMENT]) and the
  /// uninterrupted hold continues; only a jump the station-relative camera
  /// shares restarts it.
  bool _hasAnchorRefined({
    required String stabilityId,
    required Vec3 targetPosition,
    required Vec3? stationLocalCamera,
  }) {
    final reference = _anchorStabilityReference;
    if (_anchorStabilityId != stabilityId || reference == null) {
      // A new target on the shared anchor — or the first evaluation —
      // establishes its own baseline; only movement while aiming at that same
      // target invalidates the hold. The station-relative camera witness is
      // stored with the baseline, never refreshed on ordinary frames: a
      // per-frame refresh would measure each frame against itself and swallow
      // a slow genuine drift.
      _anchorStabilityId = stabilityId;
      _anchorStabilityReference = targetPosition;
      _anchorStabilityStationLocalCamera = stationLocalCamera;
      return false;
    }
    final movedM = (targetPosition - reference).length;
    if (movedM <= state.geometry.maxPositionDriftM) {
      return false;
    }
    final referenceCamera = _anchorStabilityStationLocalCamera;
    final cameraMovedM = stationLocalCamera == null || referenceCamera == null
        ? null
        : (stationLocalCamera - referenceCamera).length;
    if (cameraMovedM != null &&
        cameraMovedM <= state.geometry.maxPositionDriftM) {
      // Shared world-map correction: both the frozen baseline and the live
      // target moved by the same rigid transform, so the rendered aim is
      // unchanged. Re-base the reference and keep the hold running.
      _anchorStabilityReference = targetPosition;
      _anchorStabilityStationLocalCamera = stationLocalCamera;
      debugPrint(
        'ARCORE_WORLD_REFINEMENT: shared world correction moved the target '
        '${movedM.toStringAsFixed(3)} m while the station-local camera moved '
        'only ${cameraMovedM.toStringAsFixed(3)} m (<= '
        '${state.geometry.maxPositionDriftM.toStringAsFixed(3)} m allowance); '
        're-basing the hold reference without restarting the hold.',
      );
      return false;
    }
    _anchorStabilityReference = targetPosition;
    _anchorStabilityStationLocalCamera = stationLocalCamera;
    debugPrint(
      'ARCORE_STABILITY: station anchor refinement moved the target '
      '${movedM.toStringAsFixed(3)} m (> '
      '${state.geometry.maxPositionDriftM.toStringAsFixed(3)} m allowance, '
      'station-local camera '
      '${cameraMovedM == null ? 'unmeasured' : '${cameraMovedM.toStringAsFixed(3)} m'}); '
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
    final angleGapDeg = (gate.alignmentErrorDeg - alignment.angularErrorDeg)
        .abs();
    // How far ARCore's anchor sits from where this app placed it: the line
    // between "the target moved" and "the world estimate refined under it".
    // Measured against the calibration station origin — the anchor's own
    // creation pose — not against a target, which would print the sphere
    // radius (a constant ~1.7 m) instead of an anchor displacement.
    final anchorDeltaM = live == null || frame == null
        ? null
        : (live.position - frame.origin).length;
    // The engine's frozen node is the authoritative geometry captured at
    // calibration. `guidanceTargetPosition` is the live renderer-parity point
    // for this frame: it can differ during an ARCore map refinement, but that
    // must never mutate the frozen node. Logging both distinguishes a changing
    // camera-to-target direction from an accidental target rewrite.
    final frozenTarget =
        target.targetId > 0 && target.targetId <= _frozenNodePositions.length
        ? _frozenNodePositions[target.targetId - 1]
        : null;
    // The frozen node is a calibration-frame coordinate; the guidance point is
    // current-world. A shared ARCore world correction moves the live point and
    // leaves the frozen coordinate behind, so subtracting them directly prints
    // the correction itself as a "target move" (the 1.797 m device artefact).
    // Map the frozen node into the current world through the live station
    // anchor first — the same transform the renderer applies — so the delta is
    // correction-invariant and answers the real question: how far the rendered
    // target is from its frozen geometry right now.
    final frozenInWorld = frozenTarget == null || live == null || frame == null
        ? null
        : live.position + live.orientation.rotate(frozenTarget - frame.origin);
    final frozenToGuidanceDeltaM = frozenInWorld == null
        ? null
        : (guidanceTargetPosition - frozenInWorld).length;
    final frozenToPlacementDeltaM = frozenTarget == null
        ? null
        : (placement.worldPosition - frozenTarget).length;
    _tracePose(
      't=${now.millisecondsSinceEpoch} '
      'cam=${_formatVec(pose.position)} '
      'camQ=${_formatQuat(pose.orientation)} '
      'camTracking=${pose.tracking.name} '
      'session=${pose.sessionId ?? 'unreported'} '
      'expectedSession=${_stationSessionId ?? 'none'} '
      'epoch=${pose.trackingEpoch?.toString() ?? 'unreported'} '
      'transition=${pose.trackingTransition} '
      'recoveryPending=$_recoveryVerificationPending '
      'recoveryPhase=${next.stationRecoveryPhase.name} '
      'stationId=${next.station?.stationId ?? 'none'} '
      'anchorId=${placement.anchor.id} '
      'anchor=$anchorPosition '
      'anchorQ=$anchorOrientation '
      'anchorTracking=$anchorTracking '
      'anchorDelta=${anchorDeltaM == null ? 'n/a' : '${anchorDeltaM.toStringAsFixed(4)}m'} '
      'gate=${next.activeAnchorTracking} '
      'placement=${_formatVec(placement.worldPosition)} '
      'frozenTarget=${frozenTarget == null ? 'n/a' : _formatVec(frozenTarget)} '
      'frozenToWorld=${frozenInWorld == null ? 'n/a' : _formatVec(frozenInWorld)} '
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
      'render=${_sceneTracked(next)} '
      'ready=${next.readyToCapture} '
      'autoPermitted=${next.mode.isAutomatic && next.readyToCapture} '
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

  /// Re-reads the active target's view of the **one shared station anchor**
  /// about twice a second. All 26 target transforms remain station-local;
  /// this reads ARCore's live shared-anchor pose only to decide whether native
  /// rendering and capture may resume. A paused anchor is allowed to recover.
  /// A missing anchor never triggers per-target re-placement, because that
  /// would split the frozen target geometry across coordinate systems.
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
    if (_anchorRefreshInFlight) return;
    final requestedTarget = state.activeTargetId;
    if (requestedTarget == 0) return;
    final current = _anchors.placementFor(requestedTarget);
    if (current == null) return;
    _lastAnchorRefresh = now;
    final requestGeneration = ++_anchorRefreshGeneration;
    _anchorRefreshInFlight = true;
    _activeAnchorRefreshGeneration = requestGeneration;
    unawaited(
      _refreshActiveAnchor(
        requestedTarget: requestedTarget,
        expectedAnchorId: current.anchor.id,
        expectedSessionId:
            _stationSessionId ?? current.anchor.sessionId ?? _session.sessionId,
        requestGeneration: requestGeneration,
      ),
    );
  }

  /// Applies a native anchor refresh only if it still describes the exact
  /// active target, shared anchor, and session that requested it. In
  /// particular, a delayed answer cannot turn a target switch, recalibration,
  /// or resumed/new session into a false anchor-loss event.
  Future<void> _refreshActiveAnchor({
    required int requestedTarget,
    required String expectedAnchorId,
    required String? expectedSessionId,
    required int requestGeneration,
  }) async {
    try {
      final outcome = await _anchors.refresh(requestedTarget);
      if (_disposed || _shuttingDown) return;
      final stillCurrent =
          requestGeneration == _anchorRefreshGeneration &&
          state.activeTargetId == requestedTarget &&
          _placement?.anchor.id == expectedAnchorId;
      // A refresh is only ever about the target that was active when it
      // started. If the active target moved on, the answer applies to a target
      // the user is no longer aiming at — discard it rather than write its pose
      // or recovery state into the new target's row.
      if (!stillCurrent || outcome.stale) {
        _traceTransition(
          'discarded refresh target=$requestedTarget '
          'generation=$requestGeneration '
          'active=${state.activeTargetId} stale=${outcome.stale}',
        );
        return;
      }

      final activeSessionId = _session.sessionId;
      if ((expectedSessionId != null &&
              activeSessionId != null &&
              activeSessionId != expectedSessionId) ||
          _hasStationSessionMismatch(null)) {
        _invalidateStationReference(
          'AR resumed in a different session, so the calibrated world frame '
          'cannot be continued. Recalibrate at the station to continue.',
          recoveryFailureReason: 'session-mismatch',
        );
        return;
      }

      final refreshed = outcome.placement;
      if (refreshed == null) {
        // `gone` is the service's explicit contract for the current shared
        // anchor disappearing. A non-gone null belongs to an invalidated or
        // concurrent placement and must never be upgraded into a terminal loss.
        if (!outcome.gone) {
          _traceTransition(
            'discarded non-gone null refresh for target $requestedTarget',
          );
          return;
        }
        debugPrint(
          'ARCORE_ANCHOR: station reference lost for target $requestedTarget; '
          'anchor=$expectedAnchorId session=${expectedSessionId ?? 'unreported'}; '
          'explicit recalibration required.',
        );
        _invalidateStationReference(
          'AR lost the shared station anchor. Recalibrate at the station to '
          'continue.',
          recoveryFailureReason: 'anchor-missing',
        );
        return;
      }

      if (refreshed.anchor.id != expectedAnchorId ||
          (_stationAnchorId != null &&
              refreshed.anchor.id != _stationAnchorId)) {
        _invalidateStationReference(
          'The active station anchor no longer matches the calibrated anchor. '
          'Recalibrate at the station to continue.',
          recoveryFailureReason: 'anchor-id-mismatch',
        );
        return;
      }
      if (expectedSessionId != null &&
          refreshed.anchor.sessionId != null &&
          refreshed.anchor.sessionId != expectedSessionId) {
        _invalidateStationReference(
          'AR resumed in a different session, so the calibrated world frame '
          'cannot be continued. Recalibrate at the station to continue.',
          recoveryFailureReason: 'session-mismatch',
        );
        return;
      }

      _placement = refreshed;
      var refreshedState = state.copyWith(
        targets: _withAnchor(requestedTarget, refreshed),
        // A fresh, host-confirmed placement for the active target is exactly
        // what `activeAnchored` asserts. Restoring it here matters during a
        // recovery: if the flag was lost across a tracking pause, the pose
        // handler would otherwise stay in the no-placement branch and time the
        // station out without ever re-entering verification.
        // This cannot fake readiness — capture still requires the anchor to be
        // *tracked*, the reference to be verified, and the stability window to
        // refill.
        activeAnchored: true,
      );
      if (refreshed.isTracking) {
        // A recovery boundary deliberately clears `activeAnchorTracking`.
        // Normally the next EventChannel pose restores it, but a coalesced or
        // late active-anchor event used to leave Flutter permanently false even
        // though this newer native refresh proved the shared anchor TRACKING.
        // The host result is authoritative, so synchronise it here and publish
        // the unchanged frozen scene. Capture remains blocked by the recovery
        // verifier until a fresh matching pose arrives.
        final shouldSyncTracking =
            _anchorNotTracking > 0 || !refreshedState.activeAnchorTracking;
        if (shouldSyncTracking) {
          debugPrint(
            'ARCORE_ANCHOR: target $requestedTarget anchor '
            '${refreshed.anchor.id} tracking recovered; waiting for station '
            'verification before rendering the native marker.',
          );
          _anchorNotTracking = 0;
          refreshedState = refreshedState.copyWith(
            activeAnchorTracking: true,
            clearMessage: true,
          );
        }
        _emit(refreshedState);
        if (shouldSyncTracking && _sceneTracked(refreshedState)) {
          unawaited(_pushScene(refreshedState));
        }
        return;
      }

      // A shared anchor in STOPPED cannot be relocalised in the current world
      // frame. Unlike PAUSED, retaining it would make the frozen geometry look
      // valid when ARCore has already abandoned the reference.
      if (refreshed.anchor.tracking == TrackingState.stopped) {
        _invalidateStationReference(
          'AR stopped tracking the shared station anchor. Recalibrate at the '
          'station to continue.',
          recoveryFailureReason: 'anchor-stopped',
        );
        return;
      }

      _emit(refreshedState);

      // Placed, and the anchor is merely **paused**. ARCore pauses a world
      // anchor whenever its own tracking degrades — a fast turn, a featureless
      // wall, the first moments after creation — and resumes it on its own.
      // It is not a lost position, so wait rather than re-place a subset of the
      // frozen station geometry.
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
    } catch (error) {
      if (!_disposed && !_shuttingDown) {
        debugPrint(
          'ARCORE_ANCHOR: refresh failed for target $requestedTarget '
          'anchor=$expectedAnchorId session=${expectedSessionId ?? 'unreported'}: '
          '$error',
        );
      }
    } finally {
      if (_activeAnchorRefreshGeneration == requestGeneration) {
        _anchorRefreshInFlight = false;
        _activeAnchorRefreshGeneration = null;
      }
    }
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

  /// True only after the native ARCore session has released its camera with
  /// `Session.pause()` while retaining its world map and shared station anchor.
  /// A normal resume must use that same session rather than booting a new one.
  bool _lifecycleSessionPaused = false;

  /// The in-flight [onAppResumed] boot, observed rather than re-run by a
  /// second resume request while the first is mid-await.
  Future<void>? _resumeInFlight;

  Future<void> resize({required int width, required int height}) {
    _surfaceWidth = width;
    _surfaceHeight = height;
    return _session.resize(width: width, height: height);
  }

  /// The app went to the background: release the camera without destroying the
  /// ARCore session or the calibrated station's shared anchor.
  ///
  /// Two pause requests must not run the teardown twice: the first pass
  /// would otherwise overlap lifecycle operations. A queued request observes
  /// the in-flight pause instead of running its own.
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
      // Both resets happen on every path so a failed platform pause cannot
      // leave the controller permanently marked as shutting down.
      _shuttingDown = false;
      _pauseInFlight = null;
    }
  }

  /// The pause body of [onAppPaused], run at most once.
  Future<void> _runPauseTeardown() async {
    _shuttingDown = true;
    await _poseSub?.cancel();
    _poseSub = null;
    await _diagSub?.cancel();
    _diagSub = null;
    await _stopRecoveryGuidance();

    // A paused station remains the same station.  Do not dispose the shared
    // anchor service or close the AR session here: either action destroys the
    // native anchor map and forces a new ARCore world origin on resume.
    try {
      await _session.pause();
      _lifecycleSessionPaused = _session.isPaused;
    } catch (error) {
      debugPrint('ARCORE_SESSION: could not pause the live session: $error');
      _lifecycleSessionPaused = false;
    }

    final preservingStation =
        _lifecycleSessionPaused &&
        state.phase == CapturePhase.capturing &&
        state.station != null;
    if (preservingStation) {
      _beginRecoveryVerification(
        reason: 'app lifecycle paused the AR session',
        epoch: _lastTrackingEpoch,
      );
    }
    _lastPoseReceivedAt = null;
    _lastPoseAt = DateTime.fromMillisecondsSinceEpoch(0);
    _shuttingDown = false;
    _emit(
      state.copyWith(
        tracking: _lifecycleSessionPaused
            ? TrackingState.paused
            : state.tracking,
        trackingRecovering: preservingStation,
        stationRecoveryPhase: preservingStation
            ? StationRecoveryPhase.trackingInterrupted
            : state.stationRecoveryPhase,
        clearRecoveryFailureReason: preservingStation,
        clearAlignment: true,
        clearUnverifiedGuidance: true,
        stable: false,
        stabilityProgress: 0,
        measurementFresh: false,
        activeAnchorTracking: _lifecycleSessionPaused
            ? false
            : state.activeAnchorTracking,
        message: preservingStation
            ? 'AR paused — restoring the station reference when the app resumes.'
            : state.message,
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
  /// A queued pause is joined first. A normal lifecycle resume reopens the
  /// same ARCore session; booting is reserved for an actual new session.
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

    // The joined pause may have moved the phase; re-check before reopening or
    // booting into a flow that is no longer there.
    if (state.phase == CapturePhase.idle ||
        state.phase == CapturePhase.unavailable ||
        state.phase == CapturePhase.failed) {
      return;
    }

    if (_lifecycleSessionPaused || _session.isPaused) {
      await _resumePreservedSession(width: width, height: height);
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
        trackingRecovering: false,
        stationRecoveryPhase: StationRecoveryPhase.anchorRecoveryRequired,
        recoveryFailureReason: 'session-restarted',
        message:
            'AR restarted with a new world origin. Recalibrate at the station '
            'to continue — the saved photographs are kept.',
      ),
    );
  }

  /// Resumes an ARCore session that was paused for the app lifecycle.
  ///
  /// The frozen station, its one shared native anchor, and all 26 local target
  /// transforms deliberately stay in place.  Only fresh pose subscriptions and
  /// recovery verification are restarted; no calibration data is regenerated.
  Future<void> _resumePreservedSession({
    required int width,
    required int height,
  }) async {
    final station = state.station;
    final activeId = state.activeTargetId;
    try {
      _surfaceWidth = width;
      _surfaceHeight = height;
      await _session.resume();
      _lifecycleSessionPaused = false;
      _lastSessionReused = true;
      await _session.resize(width: width, height: height);

      // The subscriptions were deliberately cancelled during pause so that no
      // pre-pause event can satisfy a post-resume gate. Reattach them only
      // after the native camera has resumed.
      await _poseSub?.cancel();
      _poseSub = _session.poses.listen(_onPose);
      await _diagSub?.cancel();
      _diagSub = _session.diagnostics.listen(_onDiagnostic);
      _lastPoseReceivedAt = null;
      _lastPoseAt = DateTime.fromMillisecondsSinceEpoch(0);
      _cameraConfigTraced = false;
      unawaited(_startRecoveryGuidance());

      if (station == null || _frame == null || activeId == 0) {
        _emit(
          state.copyWith(
            tracking: TrackingState.initialising,
            clearAlignment: true,
            clearUnverifiedGuidance: true,
            stable: false,
            stabilityProgress: 0,
            measurementFresh: false,
          ),
        );
        return;
      }

      _beginRecoveryVerification(
        reason: 'app lifecycle resumed the existing AR session',
        epoch: _lastTrackingEpoch,
      );
      _emit(
        state.copyWith(
          phase: CapturePhase.capturing,
          tracking: TrackingState.initialising,
          trackingRecovering: true,
          stationRecoveryPhase: StationRecoveryPhase.recovering,
          clearRecoveryFailureReason: true,
          activeAnchorTracking: false,
          clearAlignment: true,
          clearUnverifiedGuidance: true,
          stable: false,
          stabilityProgress: 0,
          measurementFresh: false,
          message: 'Restoring the station reference…',
        ),
      );

      // The native controller retains this id through Session.pause(). Naming
      // it again makes the first post-resume pose carry the same anchor pose
      // even when Android coalesced the original selection event.
      final placement = _placement;
      if (placement != null) {
        await _session.setActiveAnchor(placement.anchor.id);
      }
    } on ArCoreFailure catch (failure) {
      _handlePreservedSessionResumeFailure(failure.message);
    } catch (error) {
      _handlePreservedSessionResumeFailure('$error');
    }
  }

  /// A session that cannot reopen no longer offers a trustworthy ARCore world
  /// frame. This is terminal (unlike PAUSED), so use the existing explicit
  /// recalibration path rather than silently creating a new station.
  void _handlePreservedSessionResumeFailure(String detail) {
    _lifecycleSessionPaused = false;
    debugPrint(
      'ARCORE_SESSION: could not resume the preserved session: $detail',
    );
    _invalidateStationReference(
      'AR could not resume the preserved station session. Recalibrate at the '
      'station to continue.',
      recoveryFailureReason: 'session-resume-failed',
    );
    _emit(
      state.copyWith(
        phase: CapturePhase.unavailable,
        clearAlignment: true,
        clearUnverifiedGuidance: true,
        stable: false,
        stabilityProgress: 0,
        activeAnchorTracking: false,
        message:
            'AR could not resume the preserved station session ($detail). '
            'Recalibrate at the station to continue.',
      ),
    );
  }

  @override
  void dispose() {
    if (_disposed) return;
    _disposed = true;
    ++_anchorRefreshGeneration;
    _cancelRecoveryDeadline();
    ++_recoveryGuidanceGeneration;
    _poseSub?.cancel();
    _diagSub?.cancel();
    _recoveryGuidanceSub?.cancel();
    _recoveryGuidanceSource?.stop();
    _clearRecoveryGuidanceCalibration();
    _anchors.dispose();
    _session.stop();
    // The trace is append-only and outlives this controller, so a closing line
    // is what separates one session's boundaries from the next one's.
    _trace.record('sessionEnd', {
      'wallMs': _clock().millisecondsSinceEpoch,
      'epoch': _lastTrackingEpoch,
    });
    unawaited(_trace.flush());
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
        : kBezelIdleTint;
    final hidden = _hiddenTargetsIn(next, next.targets);
    final blocker = _sceneBlocker(next);
    final tracking = blocker == null;
    final wasTracking = _sceneTracking;
    final previousBlocker = _lastSceneBlocker;
    _hiddenTargets = hidden;
    _sceneTracking = tracking;
    _lastSceneBlocker = blocker;

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
      if (blocker != previousBlocker) {
        debugPrint(
          'ARCORE_SCENE: marker rendering suppressed; reason=$blocker '
          'camera=${next.tracking.name} '
          'anchorTracking=${next.activeAnchorTracking} '
          'anchored=${next.activeAnchored} '
          'target=${next.activeTargetId} '
          'session=${_session.sessionId ?? 'unreported'} '
          'stationId=${next.station?.stationId ?? 'none'} '
          'anchor=${_placement?.anchor.id ?? 'none'} '
          'expectedAnchor=${_stationAnchorId ?? 'none'} '
          'invalidated=$_stationReferenceInvalidated '
          'invalidating=$_stationInvalidationInFlight '
          'recoveryPending=$_recoveryVerificationPending '
          'recoveryPhase=${next.stationRecoveryPhase.name} '
          'recovering=${next.trackingRecovering} '
          'hiddenActive=${hidden.contains(next.activeTargetId)}.',
        );
        _trace.record('sceneSuppressed', {
          'wallMs': _clock().millisecondsSinceEpoch,
          'reason': blocker,
          'sessionId': _session.sessionId,
          'stationId': next.station?.stationId,
          'anchorId': _placement?.anchor.id,
          'cameraTracking': next.tracking.name,
          'anchorTracking': next.activeAnchorTracking,
          'recoveryPhase': next.stationRecoveryPhase.name,
        });
      }
      return _anchors.clearScene();
    }
    if (!wasTracking || previousBlocker != null) {
      debugPrint(
        'ARCORE_SCENE: marker rendering restored from frozen station '
        'geometry; target=${next.activeTargetId} '
        'session=${_session.sessionId ?? 'unreported'} '
        'stationId=${next.station?.stationId ?? 'none'} '
        'anchor=${_placement?.anchor.id ?? 'none'} '
        'recoveryPending=$_recoveryVerificationPending '
        'recoveryPhase=${next.stationRecoveryPhase.name} '
        'recovering=${next.trackingRecovering}.',
      );
      _trace.record('sceneRestored', {
        'wallMs': _clock().millisecondsSinceEpoch,
        'sessionId': _session.sessionId,
        'stationId': next.station?.stationId,
        'anchorId': _placement?.anchor.id,
        'cameraTracking': next.tracking.name,
        'anchorTracking': next.activeAnchorTracking,
        'recoveryPhase': next.stationRecoveryPhase.name,
        'targetId': next.activeTargetId,
      });
    }
    return _anchors.pushScene(
      activeTargetId: next.activeTargetId,
      bezelTint: tint,
      activeTargetTint: null,
      activeTracking: tracking,
      debugMarker: kGlbDebugMarker,
      doneTargets: <int>{
        for (final CaptureTarget t in next.targets)
          if (t.status.isDone) t.targetId,
      },
      hiddenTargets: hidden,
    );
  }

  /// Why the native 3D target must not be rendered.
  ///
  /// A tracked anchor alone is not enough during recovery: until its relation
  /// to the last trusted camera pose has been verified, its live pose could be
  /// the discontinuity that made the marker jump. The temporary 2D sensor cue
  /// remains available then, but a native 3D marker never implies an
  /// authoritative AR position before verification completes.
  String? _sceneBlocker(CaptureState s) {
    if (_stationReferenceInvalidated || _stationInvalidationInFlight) {
      return _stationInvalidationInFlight
          ? 'station-reference-invalidating'
          : 'station-reference-invalid';
    }
    if (s.stationRecoveryPhase == StationRecoveryPhase.anchorRecoveryRequired) {
      return 'station-reference-recalibration-required';
    }
    if (s.tracking != TrackingState.tracking) {
      return 'camera-${s.tracking.name}';
    }
    if (!s.activeAnchored || s.activeTarget == null) {
      return 'no-active-station-anchor';
    }
    if (!s.activeAnchorTracking) return 'station-anchor-not-tracking';
    if (_recoveryVerificationPending || s.trackingRecovering) {
      return 'station-reference-verifying';
    }
    return null;
  }

  /// Scene visibility is a rendering decision, not a shutter decision. It
  /// requires a tracked camera, the original shared station anchor, and a
  /// verified station reference. Capture safety separately requires fresh pose
  /// data, alignment and stability gates.
  bool _sceneTracked(CaptureState s) => _sceneBlocker(s) == null;

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
