/// The 30-node capture sphere and the rules that decide when the shutter fires.
///
/// Pure Dart, and deliberately separate from the AR screen. Everything the
/// brief pins down — the ring counts, the 1.0 m radius, the 0.20 m drift budget,
/// the 3.0° alignment threshold, the 600 ms stability window, and which node is
/// next — is decided here and asserted by `test/virtual_tour_nodes_test.dart`
/// without a device.
///
/// **The geometry is a new mode, not the capture module's screen.** The app's
/// other sphere (`features/capture/domain/services/target_sphere.dart`) has
/// settled on the same 30-node, 1.0 m layout since its 26-target draft was
/// superseded, but it stays on the module's anchored, controller-driven path: a
/// tour station is a different product decision with no anchors at all,
/// world-locked GLB markers, and horizontal drift with hysteresis rather than
/// the module's full-3D distance gate.
///
/// **Orientation and position have the same contract as the rest of the app.**
/// The start pose is the first tracked ARCore pose converted by
/// `ArPoseSample.orientation`; `startOrientation.rotate(directionFromYawPitch(…))`
/// is therefore a world direction, and the node positions are world points. The
/// layout is built **once**, at the start pose, and never rebuilt: that is what
/// keeps a marker on the wall while the phone turns.
library;

import 'dart:math' as math;

import 'package:adaptive_360_capture_app/features/virtual_tour/domain/guidance/projection.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/scalars.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/services/virtual_tour_stability_monitor.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// One world-locked capture direction.
class VirtualTourNode {
  /// Position in [VirtualTourCaptureEngine.nodes], i.e. the capture order the
  /// completion count refers to. It is **not** the trigger order: the engine
  /// always aims at whichever pending node is nearest the optical axis.
  final int index;

  /// Stable storage key, e.g. `h01`, `u05`, `t01`.
  final String id;
  final String ring;
  final double yawDeg;
  final double pitchDeg;

  /// Unit direction from the start point, in world space.
  final Vec3 direction;

  /// World position, exactly [VirtualTourLayout.radiusM] from the start point.
  final Vec3 worldPosition;

  const VirtualTourNode({
    required this.index,
    required this.id,
    required this.ring,
    required this.yawDeg,
    required this.pitchDeg,
    required this.direction,
    required this.worldPosition,
  });

  String get label =>
      '$ring · yaw ${yawDeg.toStringAsFixed(0)}°, '
      'pitch ${pitchDeg.toStringAsFixed(0)}°';

  @override
  String toString() =>
      'VirtualTourNode($id, ${yawDeg.toStringAsFixed(0)}° / '
      '${pitchDeg.toStringAsFixed(0)}°)';
}

/// The 30 directions, built once from the start pose.
abstract final class VirtualTourLayout {
  /// Twelve around the horizon, eight at +35°, eight at −35°, and the two near
  /// poles. 12 + 8 + 8 + 1 + 1 = 30.
  static const int nodeCount = 30;
  static const double radiusM = 1.0;
  static const int horizontalCount = 12;
  static const int upperCount = 8;
  static const int lowerCount = 8;
  static const double upperPitchDeg = 35;
  static const double lowerPitchDeg = -35;
  static const double topPitchDeg = 85;
  static const double bottomPitchDeg = -85;

  /// Builds all 30 nodes in world space from the first tracked pose.
  ///
  /// Yaw 0 is the user's initial optical axis, so the first target is straight
  /// ahead rather than at an arbitrary compass bearing.
  static List<VirtualTourNode> build({
    required Vec3 startPosition,
    required Quat startOrientation,
  }) {
    final specs = <({String id, String ring, double yaw, double pitch})>[];
    for (var i = 0; i < horizontalCount; i++) {
      specs.add((
        id: 'h${(i + 1).toString().padLeft(2, '0')}',
        ring: 'Horizon',
        yaw: i * 360 / horizontalCount,
        pitch: 0,
      ));
    }
    for (var i = 0; i < upperCount; i++) {
      specs.add((
        id: 'u${(i + 1).toString().padLeft(2, '0')}',
        ring: 'Upper ring',
        yaw: i * 360 / upperCount,
        pitch: upperPitchDeg,
      ));
    }
    for (var i = 0; i < lowerCount; i++) {
      specs.add((
        id: 'l${(i + 1).toString().padLeft(2, '0')}',
        ring: 'Lower ring',
        yaw: i * 360 / lowerCount,
        pitch: lowerPitchDeg,
      ));
    }
    specs
      ..add((id: 't01', ring: 'Zenith', yaw: 0, pitch: topPitchDeg))
      ..add((id: 'b01', ring: 'Nadir', yaw: 0, pitch: bottomPitchDeg));

    assert(specs.length == nodeCount);
    return List<VirtualTourNode>.unmodifiable(<VirtualTourNode>[
      for (var i = 0; i < specs.length; i++)
        _nodeAt(
          index: i,
          id: specs[i].id,
          ring: specs[i].ring,
          yawDeg: specs[i].yaw,
          pitchDeg: specs[i].pitch,
          startPosition: startPosition,
          startOrientation: startOrientation,
        ),
    ]);
  }

  static VirtualTourNode _nodeAt({
    required int index,
    required String id,
    required String ring,
    required double yawDeg,
    required double pitchDeg,
    required Vec3 startPosition,
    required Quat startOrientation,
  }) {
    final local = directionFromYawPitch(yawDeg, pitchDeg);
    final world = startOrientation.rotate(local).normalized;
    return VirtualTourNode(
      index: index,
      id: id,
      ring: ring,
      yawDeg: yawDeg,
      pitchDeg: pitchDeg,
      direction: world,
      worldPosition: startPosition + world * radiusM,
    );
  }
}

/// The explicit capture lifecycle shown to the user.
///
/// The first ten values are the phases in the brief; [completed] is the terminal
/// state. The engine moves between them from evidence, never from a timer alone.
enum VirtualTourCaptureState {
  trackingUnavailable,
  readyForAlignment,
  aligning,
  waitingForStability,
  capturing,
  validatingImage,
  savingImage,
  captureSucceeded,
  captureFailed,
  awaitingTrackingRecovery,
  completed,
}

/// Why the shutter cannot fire right now, in priority order.
enum VirtualTourCaptureBlocker {
  /// Every node has a photograph.
  complete,

  /// AR is not tracking, so there is no dependable camera pose.
  tracking,

  /// The phone left the 0.20 m start sphere.
  drift,

  /// No pending node exists (the same state as [complete], kept explicit for
  /// callers that start from an empty layout).
  noTarget,

  /// The nearest pending node is outside the 3.0° cone.
  alignment,

  /// The motion gate refused: too fast, too much drift across the window, or a
  /// stale pose.
  stability,

  /// The automatic retry budget is spent; the user must ask for another try.
  retry,

  /// A still is being taken, checked or written.
  capturing,

  /// A just-captured (or failed) node is cooling down so the shutter cannot
  /// fire twice for one pass.
  cooldown,

  /// Aligned, but the hold has not completed yet.
  settling,
}

/// A frozen eligibility snapshot for one shutter press.
///
/// The attempt is created from the exact guidance that opened the shutter, so
/// the still can later be checked against the pose that justified it rather than
/// against whatever pose happens to be current when validation runs.
class VirtualTourCaptureAttempt {
  final int id;
  final VirtualTourNode node;
  final Duration startedAt;
  final Vec3 cameraPosition;
  final Quat cameraOrientation;
  final double alignmentDeg;
  final double driftM;
  final double? angularRateDegPerSec;
  final Duration? poseAge;

  /// 1 for the first try, 2 for the first automatic retry, and so on.
  final int attemptNumber;

  /// One of capturing, validatingImage or savingImage.
  final VirtualTourCaptureState state;

  const VirtualTourCaptureAttempt({
    required this.id,
    required this.node,
    required this.startedAt,
    required this.cameraPosition,
    required this.cameraOrientation,
    required this.alignmentDeg,
    required this.driftM,
    required this.angularRateDegPerSec,
    required this.poseAge,
    required this.attemptNumber,
    required this.state,
  });

  VirtualTourCaptureAttempt copyWithState(VirtualTourCaptureState value) =>
      VirtualTourCaptureAttempt(
        id: id,
        node: node,
        startedAt: startedAt,
        cameraPosition: cameraPosition,
        cameraOrientation: cameraOrientation,
        alignmentDeg: alignmentDeg,
        driftM: driftM,
        angularRateDegPerSec: angularRateDegPerSec,
        poseAge: poseAge,
        attemptNumber: attemptNumber,
        state: value,
      );

  @override
  String toString() =>
      'VirtualTourCaptureAttempt(#$id ${node.id}, try $attemptNumber, $state)';
}

/// Everything the HUD needs for one pose, decided in one place.
class VirtualTourGuidance {
  final VirtualTourNode? target;
  final double alignmentDeg;
  final double driftM;
  final bool tracking;
  final bool withinDrift;
  final bool aligned;
  final bool readyToCapture;
  final VirtualTourCaptureBlocker? blocker;
  final VirtualTourCaptureState state;
  final String instruction;
  final int capturedCount;
  final int totalCount;

  /// 0…1 while the stability window fills, 1 when the shutter may fire.
  final double holdProgress;
  final Duration stabilityHeld;
  final double? angularRateDegPerSec;
  final Duration? poseAge;
  final double? windowOrientationDriftDeg;
  final double? windowPositionDriftM;
  final int automaticRetriesRemaining;
  final bool requiresManualRetry;
  final String? lastCaptureError;
  final VirtualTourCaptureAttempt? attempt;

  const VirtualTourGuidance({
    required this.target,
    required this.alignmentDeg,
    required this.driftM,
    required this.tracking,
    required this.withinDrift,
    required this.aligned,
    required this.readyToCapture,
    required this.blocker,
    required this.state,
    required this.instruction,
    required this.capturedCount,
    required this.totalCount,
    required this.holdProgress,
    required this.stabilityHeld,
    required this.angularRateDegPerSec,
    required this.poseAge,
    required this.windowOrientationDriftDeg,
    required this.windowPositionDriftM,
    required this.automaticRetriesRemaining,
    required this.requiresManualRetry,
    required this.lastCaptureError,
    required this.attempt,
  });

  bool get complete => totalCount > 0 && capturedCount >= totalCount;
}

/// The capture plan and its gate.
///
/// One instance per station. It is told the camera pose by the AR screen and
/// answers with a [VirtualTourGuidance]; it never touches a plugin, a file or a
/// widget, which is what makes every threshold testable here.
class VirtualTourCaptureEngine {
  VirtualTourCaptureEngine({
    required this.startPosition,
    required this.startOrientation,
    List<VirtualTourNode>? nodes,
    this.maxDriftM = 0.20,
    this.alignmentToleranceDeg = 3.0,
    this.stabilityWindow = const Duration(milliseconds: 600),
    this.maxAngularVelocityDegPerSec = 3.0,
    this.maxOrientationDriftDeg = 1.0,
    this.maxPositionDriftM = 0.02,
    this.maxPoseAge = const Duration(milliseconds: 150),
    this.cooldown = const Duration(milliseconds: 900),
    this.retryCooldown = const Duration(milliseconds: 1500),
    this.maxAutomaticRetries = 2,
    this.capturePoseToleranceDeg = 5.0,
    this.capturePoseToleranceM = 0.10,
    VirtualTourStabilityMonitor? stabilityMonitor,
  }) : nodes =
           nodes ??
           VirtualTourLayout.build(
             startPosition: startPosition,
             startOrientation: startOrientation,
           ),
       _monitor =
           stabilityMonitor ??
           VirtualTourStabilityMonitor(
             window: stabilityWindow,
             maxAngularVelocityDegPerSec: maxAngularVelocityDegPerSec,
             maxOrientationDriftDeg: maxOrientationDriftDeg,
             maxPositionDriftM: maxPositionDriftM,
             maxPoseAge: maxPoseAge,
           );

  /// Builds the whole layout from the first tracked pose.
  factory VirtualTourCaptureEngine.atStart({
    required Vec3 position,
    required Quat orientation,
  }) => VirtualTourCaptureEngine(
    startPosition: position,
    startOrientation: orientation,
  );

  /// The world point every node is measured from and every marker faces.
  final Vec3 startPosition;
  final Quat startOrientation;
  final List<VirtualTourNode> nodes;

  /// The brief's `< 0.20 m` rule: exactly 0.20 m is already out.
  final double maxDriftM;

  /// The brief's `< 3.0°` rule: exactly 3.0° is not aligned.
  final double alignmentToleranceDeg;
  final Duration stabilityWindow;
  final double maxAngularVelocityDegPerSec;
  final double maxOrientationDriftDeg;
  final double maxPositionDriftM;
  final Duration maxPoseAge;
  final Duration cooldown;
  final Duration retryCooldown;
  final int maxAutomaticRetries;

  /// How closely the still's camera pose must match the attempt's frozen pose.
  final double capturePoseToleranceDeg;
  final double capturePoseToleranceM;

  /// The angle at which the UI switches from "turn toward the marker" to
  /// "almost there". It does not gate the shutter; only the 3.0° tolerance does.
  static const double approachConeDeg = 15.0;

  final VirtualTourStabilityMonitor _monitor;
  final Set<String> _captured = <String>{};
  final Map<String, int> _failures = <String, int>{};
  _AttemptRecord? _attempt;
  VirtualTourGuidance? _lastGuidance;
  bool _hasEverTracked = false;
  int _nextAttemptId = 0;
  Vec3? _lastCameraPosition;
  Quat? _lastCameraOrientation;

  Duration? _cooldownUntil;
  bool _cooldownAfterFailure = false;
  String? _lastSucceededNodeId;
  String? _lastFailedNodeId;
  String? _lastCaptureError;

  /// The node the stability window is currently accumulating for.
  ///
  /// The window is only meaningful for one target: turning from one marker to
  /// the next closes the gate on the way, but switching between two markers
  /// that are both within tolerance must still restart the 600 ms hold rather
  /// than inherit the previous target's samples.
  String? _monitorTargetId;

  Set<String> get capturedIds => Set<String>.unmodifiable(_captured);
  int get capturedCount => _captured.length;
  int get totalCount => nodes.length;
  bool get allCaptured => nodes.isNotEmpty && _captured.length == nodes.length;
  bool get isCapturing => _attempt != null;
  VirtualTourCaptureAttempt? get attempt => _attempt?.snapshot();
  VirtualTourGuidance? get lastGuidance => _lastGuidance;
  String? get lastCaptureError => _lastCaptureError;
  bool get hasEverTracked => _hasEverTracked;

  bool isCaptured(VirtualTourNode node) => _captured.contains(node.id);

  int failureCountFor(VirtualTourNode node) => _failures[node.id] ?? 0;

  bool requiresManualRetryFor(VirtualTourNode node) =>
      failureCountFor(node) > maxAutomaticRetries;

  int automaticRetriesRemainingFor(VirtualTourNode node) {
    final failures = failureCountFor(node);
    if (failures == 0) return maxAutomaticRetries;
    return math.max(0, maxAutomaticRetries + 1 - failures);
  }

  /// Picks the pending node nearest the optical axis and applies every gate.
  ///
  /// The nearest node is chosen by the **current** vector from the camera to
  /// the node, not by the node's original direction. That matters once the user
  /// has drifted: the marker stays on the wall, so the direction the phone must
  /// point is the one to the marker, not the direction it had at the start.
  ///
  /// [now] must be monotonic — an ARCore frame timestamp or an elapsed
  /// [Stopwatch], never `DateTime.now()`. [measurementAge] is how old the pose
  /// was when it arrived, measured on the same clock.
  VirtualTourGuidance evaluate({
    required Vec3 cameraPosition,
    required Quat cameraOrientation,
    required TrackingState tracking,
    required Duration now,
    Duration? measurementAge,
    double? angularRateDegPerSec,
  }) {
    _lastCameraPosition = cameraPosition;
    _lastCameraOrientation = cameraOrientation;
    final driftM = (cameraPosition - startPosition).length;
    final trackingOk = tracking == TrackingState.tracking;
    if (trackingOk) _hasEverTracked = true;
    final forward = cameraOrientation.rotate(Vec3.unitZ);

    VirtualTourNode? target;
    var bestAngle = double.infinity;
    for (final node in nodes) {
      if (_captured.contains(node.id)) continue;
      final angle = _angleDeg(forward, node.worldPosition - cameraPosition);
      if (angle < bestAngle) {
        bestAngle = angle;
        target = node;
      }
    }

    final withinDrift = driftM < maxDriftM;
    final aligned =
        target != null && trackingOk && bestAngle < alignmentToleranceDeg;
    final cooldownActive = _cooldownUntil != null && now < _cooldownUntil!;
    final failedNode = _lastFailedNodeId == null
        ? null
        : _nodeById(_lastFailedNodeId!);
    final failureRelevant = failedNode != null && target?.id == failedNode.id;
    final failedNodeNeedsManual =
        failedNode != null && requiresManualRetryFor(failedNode);
    final requiresManual = failureRelevant && failedNodeNeedsManual;
    final failureCooling =
        failedNode != null && cooldownActive && _cooldownAfterFailure;
    final successCooling =
        cooldownActive &&
        !_cooldownAfterFailure &&
        _lastSucceededNodeId != null;

    final gateOpen =
        trackingOk &&
        target != null &&
        aligned &&
        withinDrift &&
        !cooldownActive &&
        !requiresManual;

    if (target != null && target.id != _monitorTargetId) {
      _monitorTargetId = target.id;
      _monitor.reset();
    }

    final stable = _monitor.update(
      now: now,
      orientation: cameraOrientation,
      position: cameraPosition,
      poseAge: measurementAge,
      angularRateDegPerSec: angularRateDegPerSec,
      gateOpen: gateOpen,
    );

    VirtualTourCaptureState state;
    VirtualTourCaptureBlocker? blocker;
    var ready = false;
    var holdProgress = 0.0;

    final activeAttempt = _attempt;
    if (allCaptured) {
      state = VirtualTourCaptureState.completed;
      blocker = VirtualTourCaptureBlocker.complete;
    } else if (activeAttempt != null) {
      state = activeAttempt.state;
      blocker = VirtualTourCaptureBlocker.capturing;
    } else if (!trackingOk) {
      state = _hasEverTracked
          ? VirtualTourCaptureState.awaitingTrackingRecovery
          : VirtualTourCaptureState.trackingUnavailable;
      blocker = VirtualTourCaptureBlocker.tracking;
    } else if (successCooling) {
      state = VirtualTourCaptureState.captureSucceeded;
      blocker = VirtualTourCaptureBlocker.cooldown;
    } else if (failureCooling || requiresManual) {
      state = VirtualTourCaptureState.captureFailed;
      blocker = requiresManual
          ? VirtualTourCaptureBlocker.retry
          : VirtualTourCaptureBlocker.cooldown;
    } else if (target == null) {
      state = VirtualTourCaptureState.readyForAlignment;
      blocker = VirtualTourCaptureBlocker.noTarget;
    } else if (!withinDrift) {
      state = VirtualTourCaptureState.readyForAlignment;
      blocker = VirtualTourCaptureBlocker.drift;
    } else if (bestAngle > approachConeDeg) {
      state = VirtualTourCaptureState.readyForAlignment;
      blocker = VirtualTourCaptureBlocker.alignment;
    } else if (!aligned) {
      state = VirtualTourCaptureState.aligning;
      blocker = VirtualTourCaptureBlocker.alignment;
    } else {
      state = VirtualTourCaptureState.waitingForStability;
      ready = stable && gateOpen;
      holdProgress = _monitor.progress;
      if (ready) {
        blocker = null;
      } else if (_monitor.lastIssue != null &&
          _monitor.lastIssue != VirtualTourStabilityIssue.window) {
        blocker = VirtualTourCaptureBlocker.stability;
      } else {
        blocker = VirtualTourCaptureBlocker.settling;
      }
    }

    if (state == VirtualTourCaptureState.captureSucceeded ||
        state == VirtualTourCaptureState.captureFailed ||
        state == VirtualTourCaptureState.readyForAlignment ||
        state == VirtualTourCaptureState.aligning ||
        state == VirtualTourCaptureState.trackingUnavailable ||
        state == VirtualTourCaptureState.awaitingTrackingRecovery ||
        state == VirtualTourCaptureState.completed ||
        activeAttempt != null) {
      holdProgress = 0;
    }

    final instruction = _instruction(
      state: state,
      blocker: blocker,
      target: target,
      cameraOrientation: cameraOrientation,
      cameraPosition: cameraPosition,
      alignmentDeg: bestAngle,
      driftM: driftM,
      tracking: tracking,
      failureNode: failedNode,
      requiresManual: failedNodeNeedsManual,
      automaticRetriesRemaining: failedNode == null
          ? 0
          : automaticRetriesRemainingFor(failedNode),
    );

    final guidance = VirtualTourGuidance(
      target: target,
      alignmentDeg: bestAngle,
      driftM: driftM,
      tracking: trackingOk,
      withinDrift: withinDrift,
      aligned: aligned,
      readyToCapture: ready,
      blocker: blocker,
      state: state,
      instruction: instruction,
      capturedCount: capturedCount,
      totalCount: totalCount,
      holdProgress: holdProgress,
      stabilityHeld: _monitor.heldFor,
      angularRateDegPerSec: _monitor.lastAngularRateDegPerSec,
      poseAge: measurementAge ?? _monitor.lastPoseAge,
      windowOrientationDriftDeg: _monitor.lastOrientationDriftDeg,
      windowPositionDriftM: _monitor.lastPositionDriftM,
      automaticRetriesRemaining: failedNode == null
          ? 0
          : automaticRetriesRemainingFor(failedNode),
      requiresManualRetry: requiresManual,
      lastCaptureError: _lastCaptureError,
      attempt: _attempt?.snapshot(),
    );
    _lastGuidance = guidance;
    return guidance;
  }

  /// Drops an accumulated hold when no fresh pose is available.
  ///
  /// Called by the screen once the pose stream has been silent for
  /// [maxPoseAge]. A window built while the stream was alive must not survive a
  /// gap, and readiness frozen in [lastGuidance] must not remain spendable by
  /// [beginCapture]. An attempt already in flight is deliberately untouched:
  /// it carries its own frozen pose and its own validation.
  void invalidateStability() {
    _monitor.reset();
    _lastGuidance = null;
  }

  /// Freezes the last [evaluate]'s eligibility and reserves the shutter.
  ///
  /// Returns `null` unless the *immediately preceding* evaluation was ready for
  /// [node]. That is what makes the shutter single-flight: a duplicate pose
  /// event cannot reserve a second still, and a pose that was merely close does
  /// not qualify.
  VirtualTourCaptureAttempt? beginCapture({
    required VirtualTourNode node,
    required Duration now,
  }) {
    if (_attempt != null) return null;
    final guidance = _lastGuidance;
    if (guidance == null ||
        !guidance.readyToCapture ||
        guidance.target?.id != node.id ||
        _captured.contains(node.id) ||
        allCaptured ||
        requiresManualRetryFor(node)) {
      return null;
    }
    _nextAttemptId++;
    final attempt = VirtualTourCaptureAttempt(
      id: _nextAttemptId,
      node: node,
      startedAt: now,
      cameraPosition: _lastCameraPosition!,
      cameraOrientation: _lastCameraOrientation!,
      alignmentDeg: guidance.alignmentDeg,
      driftM: guidance.driftM,
      angularRateDegPerSec: guidance.angularRateDegPerSec,
      poseAge: guidance.poseAge,
      attemptNumber: failureCountFor(node) + 1,
      state: VirtualTourCaptureState.capturing,
    );
    _attempt = _AttemptRecord(attempt);
    _monitor.reset();
    _lastGuidance = null;
    return attempt;
  }

  /// Advances one attempt through the validation and save phases.
  ///
  /// Stages only move forward. A stale attempt id is ignored, so a late callback
  /// from a timed-out capture cannot move the next attempt's state.
  bool markAttemptState(
    VirtualTourCaptureAttempt attempt,
    VirtualTourCaptureState state,
  ) {
    final record = _attempt;
    if (record == null ||
        record.id != attempt.id ||
        (state != VirtualTourCaptureState.capturing &&
            state != VirtualTourCaptureState.validatingImage &&
            state != VirtualTourCaptureState.savingImage)) {
      return false;
    }
    final current = record.state;
    final allowed =
        (current == VirtualTourCaptureState.capturing &&
            (state == VirtualTourCaptureState.capturing ||
                state == VirtualTourCaptureState.validatingImage)) ||
        (current == VirtualTourCaptureState.validatingImage &&
            (state == VirtualTourCaptureState.validatingImage ||
                state == VirtualTourCaptureState.savingImage)) ||
        (current == VirtualTourCaptureState.savingImage &&
            state == VirtualTourCaptureState.savingImage);
    if (!allowed) return false;
    record.state = state;
    return true;
  }

  /// Records the outcome after the JPEG has actually been validated and saved.
  void completeCapture(
    VirtualTourCaptureAttempt attempt, {
    required bool success,
    required Duration now,
    String? error,
  }) {
    final record = _attempt;
    if (record == null || record.id != attempt.id) return;
    _attempt = null;
    _monitor.reset();
    _lastGuidance = null;

    if (success) {
      _captured.add(record.node.id);
      _failures.remove(record.node.id);
      _lastSucceededNodeId = record.node.id;
      if (_lastFailedNodeId == record.node.id) _lastFailedNodeId = null;
      _lastCaptureError = null;
      _cooldownUntil = now + cooldown;
      _cooldownAfterFailure = false;
    } else {
      final failures = failureCountFor(record.node) + 1;
      _failures[record.node.id] = failures;
      _lastFailedNodeId = record.node.id;
      _lastSucceededNodeId = null;
      _lastCaptureError = error;
      _cooldownUntil = now + retryCooldown;
      _cooldownAfterFailure = true;
    }
  }

  /// Cancels without spending an automatic retry.
  ///
  /// Used when tracking is lost while the still is in flight: there is no
  /// evidence that the photograph was bad, only that the platform could not
  /// deliver it. The screen evaluates again on the next pose, which is what
  /// moves the state to [VirtualTourCaptureState.awaitingTrackingRecovery].
  void cancelCapture(VirtualTourCaptureAttempt attempt) {
    if (_attempt?.id != attempt.id) return;
    _attempt = null;
    _monitor.reset();
    _lastGuidance = null;
  }

  /// Re-arms a target whose automatic retry budget is spent.
  ///
  /// The user must aim and hold still again; this only clears the failure
  /// budget, it does not take a photograph by itself.
  bool manualRetry({required VirtualTourNode node}) {
    if (_attempt != null ||
        _captured.contains(node.id) ||
        !requiresManualRetryFor(node)) {
      return false;
    }
    _failures.remove(node.id);
    if (_lastFailedNodeId == node.id) {
      _lastFailedNodeId = null;
      _lastCaptureError = null;
    }
    _cooldownUntil = null;
    _cooldownAfterFailure = false;
    _monitor.reset();
    _lastGuidance = null;
    return true;
  }

  /// Checks a still's captured camera pose against the frozen attempt.
  ///
  /// Returns `null` when the still was taken from the pose that passed the
  /// gates, otherwise a human-readable reason. This is deliberately looser than
  /// the stability gate: it catches "the phone was already somewhere else when
  /// the shutter finally fired", not normal hand tremor.
  String? validateCapturePose({
    required VirtualTourCaptureAttempt attempt,
    required Vec3 cameraPosition,
    required Quat cameraOrientation,
  }) {
    final angle = _quaternionAngleDeg(
      attempt.cameraOrientation,
      cameraOrientation,
    );
    if (angle > capturePoseToleranceDeg) {
      return 'The camera turned '
          '${angle.toStringAsFixed(1)}° after the shutter was armed '
          '(limit ${capturePoseToleranceDeg.toStringAsFixed(1)}°).';
    }
    final moved = (cameraPosition - attempt.cameraPosition).length;
    if (moved > capturePoseToleranceM) {
      return 'The camera moved '
          '${(moved * 100).toStringAsFixed(1)} cm after the shutter was armed '
          '(limit ${(capturePoseToleranceM * 100).toStringAsFixed(0)} cm).';
    }
    return null;
  }

  String _instruction({
    required VirtualTourCaptureState state,
    required VirtualTourCaptureBlocker? blocker,
    required VirtualTourNode? target,
    required Quat cameraOrientation,
    required Vec3 cameraPosition,
    required double alignmentDeg,
    required double driftM,
    required TrackingState tracking,
    required VirtualTourNode? failureNode,
    required bool requiresManual,
    required int automaticRetriesRemaining,
  }) {
    switch (state) {
      case VirtualTourCaptureState.completed:
        return 'All $totalCount targets are captured. The panorama is next.';
      case VirtualTourCaptureState.trackingUnavailable:
        return tracking.advice ?? 'Waiting for AR tracking.';
      case VirtualTourCaptureState.awaitingTrackingRecovery:
        return tracking.advice ??
            'AR tracking was lost. Move the phone slowly to recover.';
      case VirtualTourCaptureState.captureSucceeded:
        return 'Target captured. Turn to the next marker.';
      case VirtualTourCaptureState.captureFailed:
        final reason = _lastCaptureError;
        if (requiresManual) {
          return '${reason ?? 'That capture failed.'} '
              'No automatic retries remain — tap Retry on this target.';
        }
        final tries = automaticRetriesRemaining == 1
            ? '1 automatic retry'
            : '$automaticRetriesRemaining automatic retries';
        return '${reason ?? 'That capture failed.'} '
            '$tries left — keep aiming and hold still.';
      case VirtualTourCaptureState.capturing:
        return 'Capturing ${target?.label ?? 'the target'}…';
      case VirtualTourCaptureState.validatingImage:
        return 'Checking the photograph…';
      case VirtualTourCaptureState.savingImage:
        return 'Saving the photograph…';
      case VirtualTourCaptureState.readyForAlignment:
        if (blocker == VirtualTourCaptureBlocker.drift) {
          final cm = (driftM * 100).round();
          final limit = (maxDriftM * 100).round();
          return 'You are $cm cm from the start point (limit $limit cm). '
              'Step back toward where you began.';
        }
        if (target == null) return 'No pending target.';
        final toTarget = (target.worldPosition - cameraPosition).normalized;
        return wording(
          cameraOrientation.toCameraFrame(toTarget),
          alignmentDeg,
          approachConeDeg,
        );
      case VirtualTourCaptureState.aligning:
        if (target == null) return 'No pending target.';
        final toTarget = (target.worldPosition - cameraPosition).normalized;
        return wording(
          cameraOrientation.toCameraFrame(toTarget),
          alignmentDeg,
          alignmentToleranceDeg,
        );
      case VirtualTourCaptureState.waitingForStability:
        if (blocker == VirtualTourCaptureBlocker.stability) {
          final issue = _monitor.lastIssue;
          final rate = _monitor.lastAngularRateDegPerSec;
          switch (issue) {
            case VirtualTourStabilityIssue.poseAge:
              return 'The camera reading is stale. Hold the phone still.';
            case VirtualTourStabilityIssue.angularRate:
              return 'Moving too fast'
                  '${rate == null ? '' : ' (${rate.toStringAsFixed(1)}°/s)'} — '
                  'hold still.';
            case VirtualTourStabilityIssue.orientationDrift:
            case VirtualTourStabilityIssue.positionDrift:
              return 'The phone drifted while holding — hold still again.';
            case VirtualTourStabilityIssue.window:
            case null:
              break;
          }
        }
        return 'Target aligned — hold still for '
            '${(stabilityWindow.inMilliseconds / 1000).toStringAsFixed(1)} s.';
    }
  }

  VirtualTourNode? _nodeById(String id) {
    for (final node in nodes) {
      if (node.id == id) return node;
    }
    return null;
  }

  /// Geodesic angle between two directions. Zero-length input is 180°, which
  /// can never pass the 3° gate.
  static double _angleDeg(Vec3 a, Vec3 b) {
    final u = a.normalized;
    final v = b.normalized;
    if (u.length == 0 || v.length == 0) return 180;
    return math.acos(clampDouble(u.dot(v), -1, 1)) * radToDeg;
  }

  static double _quaternionAngleDeg(Quat a, Quat b) {
    final rel = (b * a.conjugate).normalized;
    final v = math.sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
    return 2 * math.atan2(v, rel.w.abs()) * radToDeg;
  }
}

class _AttemptRecord {
  final int id;
  final VirtualTourNode node;
  final Duration startedAt;
  final Vec3 cameraPosition;
  final Quat cameraOrientation;
  final double alignmentDeg;
  final double driftM;
  final double? angularRateDegPerSec;
  final Duration? poseAge;
  final int attemptNumber;
  VirtualTourCaptureState state;

  _AttemptRecord(VirtualTourCaptureAttempt attempt)
    : id = attempt.id,
      node = attempt.node,
      startedAt = attempt.startedAt,
      cameraPosition = attempt.cameraPosition,
      cameraOrientation = attempt.cameraOrientation,
      alignmentDeg = attempt.alignmentDeg,
      driftM = attempt.driftM,
      angularRateDegPerSec = attempt.angularRateDegPerSec,
      poseAge = attempt.poseAge,
      attemptNumber = attempt.attemptNumber,
      state = attempt.state;

  VirtualTourCaptureAttempt snapshot() => VirtualTourCaptureAttempt(
    id: id,
    node: node,
    startedAt: startedAt,
    cameraPosition: cameraPosition,
    cameraOrientation: cameraOrientation,
    alignmentDeg: alignmentDeg,
    driftM: driftM,
    angularRateDegPerSec: angularRateDegPerSec,
    poseAge: poseAge,
    attemptNumber: attemptNumber,
    state: state,
  );
}
