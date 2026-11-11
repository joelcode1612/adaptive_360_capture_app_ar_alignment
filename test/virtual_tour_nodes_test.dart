import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/scalars.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour_capture_engine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

const Vec3 origin = Vec3.zero;
const double eps = 1e-9;
const Duration t0 = Duration.zero;
const Duration hold = Duration(milliseconds: 600);

VirtualTourCaptureEngine engineAtStart({
  Vec3 position = origin,
  Quat orientation = Quat.identity,
}) => VirtualTourCaptureEngine.atStart(
  position: position,
  orientation: orientation,
);

VirtualTourNode nodeById(VirtualTourCaptureEngine engine, String id) =>
    engine.nodes.firstWhere((VirtualTourNode n) => n.id == id);

/// A pose whose optical axis is [direction].
Quat facing(Vec3 direction) {
  final d = direction.normalized;
  final axis = Vec3.unitZ.cross(d);
  if (axis.length < 1e-9) {
    return d.z >= 0 ? Quat.identity : Quat.axisAngle(180, Vec3.unitX);
  }
  final angle = math.acos(clampDouble(d.z, -1, 1)) * radToDeg;
  return Quat.axisAngle(angle, axis).normalized;
}

VirtualTourGuidance aim(
  VirtualTourCaptureEngine engine,
  VirtualTourNode node, {
  Vec3? cameraPosition,
  Duration at = t0,
  Quat? orientation,
  TrackingState tracking = TrackingState.tracking,
  Duration? measurementAge,
  double? angularRateDegPerSec,
}) {
  final position = cameraPosition ?? engine.startPosition;
  return engine.evaluate(
    cameraPosition: position,
    cameraOrientation: orientation ?? facing(node.worldPosition - position),
    tracking: tracking,
    now: at,
    measurementAge: measurementAge,
    angularRateDegPerSec: angularRateDegPerSec,
  );
}

/// Fills one full 600 ms window and returns the final guidance.
VirtualTourGuidance holdSteady(
  VirtualTourCaptureEngine engine,
  VirtualTourNode node, {
  required Duration from,
  Vec3? cameraPosition,
  Quat? orientation,
}) {
  aim(
    engine,
    node,
    cameraPosition: cameraPosition,
    at: from,
    orientation: orientation,
  );
  return aim(
    engine,
    node,
    cameraPosition: cameraPosition,
    at: from + hold,
    orientation: orientation,
  );
}

VirtualTourCaptureAttempt captureOnce(
  VirtualTourCaptureEngine engine,
  VirtualTourNode node,
  Duration from, {
  bool success = true,
}) {
  final ready = holdSteady(engine, node, from: from);
  expect(ready.readyToCapture, isTrue, reason: node.id);
  final attempt = engine.beginCapture(node: node, now: from + hold);
  expect(attempt, isNotNull, reason: node.id);
  engine.completeCapture(
    attempt!,
    success: success,
    now: from + hold,
    error: success ? null : 'the test capture failed',
  );
  return attempt;
}

void main() {
  group('layout', () {
    test('is 30 nodes: 12 horizon, 8 upper, 8 lower, zenith and nadir', () {
      final engine = engineAtStart();
      expect(engine.nodes.length, VirtualTourLayout.nodeCount);
      expect(engine.nodes.length, 30);

      final byRing = <String, List<VirtualTourNode>>{};
      for (final node in engine.nodes) {
        byRing.putIfAbsent(node.ring, () => <VirtualTourNode>[]).add(node);
      }
      expect(byRing['Horizon']!.length, 12);
      expect(byRing['Upper ring']!.length, 8);
      expect(byRing['Lower ring']!.length, 8);
      expect(byRing['Zenith']!.length, 1);
      expect(byRing['Nadir']!.length, 1);

      expect(byRing['Horizon']!.every((n) => n.pitchDeg == 0), isTrue);
      expect(byRing['Upper ring']!.every((n) => n.pitchDeg == 35), isTrue);
      expect(byRing['Lower ring']!.every((n) => n.pitchDeg == -35), isTrue);
      expect(byRing['Zenith']!.single.pitchDeg, 85);
      expect(byRing['Nadir']!.single.pitchDeg, -85);

      expect(byRing['Horizon']!.map((n) => n.yawDeg).toList(), <double>[
        0,
        30,
        60,
        90,
        120,
        150,
        180,
        210,
        240,
        270,
        300,
        330,
      ]);
      expect(byRing['Upper ring']!.map((n) => n.yawDeg).toList(), <double>[
        0,
        45,
        90,
        135,
        180,
        225,
        270,
        315,
      ]);
      expect(byRing['Lower ring']!.map((n) => n.yawDeg).toList(), <double>[
        0,
        45,
        90,
        135,
        180,
        225,
        270,
        315,
      ]);

      final ids = engine.nodes.map((n) => n.id).toList();
      expect(ids.toSet().length, 30);
      expect(ids.first, 'h01');
      expect(ids.where((id) => id.startsWith('h')).length, 12);
      expect(ids.where((id) => id.startsWith('u')).length, 8);
      expect(ids.where((id) => id.startsWith('l')).length, 8);
      expect(ids, containsAll(<String>['t01', 'b01']));
    });

    test('every node is exactly 1.0 m from the start point', () {
      final engine = engineAtStart(
        position: const Vec3(2.5, 1.4, -3.2),
        orientation: Quat.fromYawPitch(73, 8),
      );
      for (final node in engine.nodes) {
        expect(
          (node.worldPosition - engine.startPosition).length,
          closeTo(VirtualTourLayout.radiusM, 1e-9),
          reason: node.id,
        );
        expect(node.direction.length, closeTo(1, 1e-12), reason: node.id);
      }
    });

    test('yaw 0 is the start optical axis and yaw +90 is to its right', () {
      final engine = engineAtStart();
      expect(nodeById(engine, 'h01').direction.x, closeTo(0, eps));
      expect(nodeById(engine, 'h01').direction.y, closeTo(0, eps));
      expect(nodeById(engine, 'h01').direction.z, closeTo(1, eps));
      // 90° yaw is +X in this right-handed, +Y-up world.
      expect(nodeById(engine, 'h04').direction.x, closeTo(1, eps));
      expect(nodeById(engine, 'h04').direction.z, closeTo(0, eps));
      // The pole node is 5° off the zenith, not 90°.
      expect(
        nodeById(engine, 't01').direction.y,
        closeTo(math.sin(85 * degToRad), eps),
      );
    });

    test('the layout is built in the start pose world frame, once', () {
      final start = const Vec3(2, 1, -3);
      final orientation = Quat.fromYawPitch(90, 0);
      final engine = engineAtStart(position: start, orientation: orientation);

      // h01 lies one metre along the start pose's optical axis, which is +X.
      final h01 = nodeById(engine, 'h01');
      expect(h01.direction.x, closeTo(1, 1e-9));
      expect(h01.worldPosition.x, closeTo(start.x + 1, 1e-9));
      expect(h01.worldPosition.y, closeTo(start.y, 1e-9));
      expect(h01.worldPosition.z, closeTo(start.z, 1e-9));
      // h04 (local yaw +90) is now −Z.
      final h04 = nodeById(engine, 'h04');
      expect(h04.direction.x, closeTo(0, 1e-9));
      expect(h04.direction.z, closeTo(-1, 1e-9));

      final before = engine.nodes.map((n) => n.worldPosition).toList();
      aim(engine, h01, at: t0, orientation: Quat.fromYawPitch(200, -20));
      final after = engine.nodes.map((n) => n.worldPosition).toList();
      for (var i = 0; i < before.length; i++) {
        expect(after[i], before[i]);
      }
    });

    test('the engine starts un-captured and picks the node under the aim', () {
      final engine = engineAtStart();
      expect(engine.capturedCount, 0);
      expect(engine.allCaptured, isFalse);

      expect(aim(engine, nodeById(engine, 'h01')).target?.id, 'h01');
      // A fixed index order is not used: aim at 90° and the target is h04.
      expect(aim(engine, nodeById(engine, 'h04')).target?.id, 'h04');
      expect(aim(engine, nodeById(engine, 't01')).target?.id, 't01');
    });
  });

  group('gates', () {
    test('station displacement remains diagnostic at the former threshold', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      final atLimit = aim(engine, h01, cameraPosition: const Vec3(0.20, 0, 0));
      expect(atLimit.driftM, closeTo(0.20, 1e-12));
      expect(atLimit.aligned, isTrue);
      expect(atLimit.blocker, VirtualTourCaptureBlocker.settling);
    });

    test('a far camera can capture after alignment and stability', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');
      final camera = const Vec3(5, 0, 0);

      final guidance = holdSteady(
        engine,
        h01,
        from: t0,
        cameraPosition: camera,
      );
      expect(guidance.driftM, closeTo(5, 1e-12));
      expect(guidance.aligned, isTrue);
      expect(guidance.readyToCapture, isTrue);
      expect(guidance.blocker, isNull);
      expect(
        engine.beginCapture(node: h01, now: hold),
        isNotNull,
      );
    });

    test('the 3.0° alignment gate is strict', () {
      final engine = engineAtStart();

      VirtualTourGuidance aimed(double yawDeg) => aim(
        engine,
        nodeById(engine, 'h01'),
        orientation: Quat.fromYawPitch(yawDeg, 0),
      );

      final inside = aimed(2.9);
      expect(inside.target?.id, 'h01');
      expect(inside.alignmentDeg, closeTo(2.9, 0.05));
      expect(inside.aligned, isTrue);

      final outside = aimed(3.1);
      expect(outside.target?.id, 'h01');
      expect(outside.aligned, isFalse);
      expect(outside.blocker, VirtualTourCaptureBlocker.alignment);
    });

    test('the shutter waits for the full 600 ms stability window', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      final first = aim(engine, h01);
      expect(first.state, VirtualTourCaptureState.waitingForStability);
      expect(first.blocker, VirtualTourCaptureBlocker.settling);
      expect(first.readyToCapture, isFalse);
      expect(first.holdProgress, 0);

      final nearly = aim(engine, h01, at: const Duration(milliseconds: 599));
      expect(nearly.readyToCapture, isFalse);
      expect(nearly.holdProgress, closeTo(599 / 600, 1e-3));

      final held = aim(engine, h01, at: const Duration(milliseconds: 600));
      expect(held.readyToCapture, isTrue);
      expect(held.blocker, isNull);
      expect(held.holdProgress, 1);
      expect(held.stabilityHeld, hold);
    });

    test('3 °/s is a hard angular-speed limit', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      // A constant 4 °/s turn through the target: no drift is visible against
      // the first sample alone, but the rate is unambiguous.
      aim(engine, h01, at: t0, orientation: Quat.identity);
      final moving = aim(
        engine,
        h01,
        at: hold,
        orientation: Quat.fromYawPitch(2.4, 0),
      );
      expect(moving.aligned, isTrue);
      expect(moving.readyToCapture, isFalse);
      expect(moving.blocker, VirtualTourCaptureBlocker.stability);
      expect(moving.angularRateDegPerSec, greaterThan(3.5));
    });

    test('orientational drift across the window is capped at 1.0°', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      aim(engine, h01, at: t0, orientation: Quat.identity);
      final drifting = aim(
        engine,
        h01,
        at: hold,
        orientation: Quat.fromYawPitch(1.2, 0),
      );
      expect(drifting.aligned, isTrue);
      expect(drifting.readyToCapture, isFalse);
      expect(drifting.blocker, VirtualTourCaptureBlocker.stability);
      expect(drifting.windowOrientationDriftDeg, greaterThan(1.0));
      expect(drifting.angularRateDegPerSec, lessThan(3.0));
    });

    test('positional drift across the window is capped at 0.02 m', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      aim(engine, h01, at: t0, orientation: Quat.identity);
      final moving = aim(
        engine,
        h01,
        at: hold,
        cameraPosition: const Vec3(0.025, 0, 0),
        orientation: Quat.identity,
      );
      expect(moving.aligned, isTrue);
      expect(moving.readyToCapture, isFalse);
      expect(moving.blocker, VirtualTourCaptureBlocker.stability);
      expect(moving.windowPositionDriftM, greaterThan(0.02));
    });

    test('switching to another aligned target restarts the window', () {
      VirtualTourNode nodeAt(int index, String id, double yawDeg) {
        final direction = directionFromYawPitch(yawDeg, 0);
        return VirtualTourNode(
          index: index,
          id: id,
          ring: 'Test',
          yawDeg: yawDeg,
          pitchDeg: 0,
          direction: direction,
          worldPosition: direction * VirtualTourLayout.radiusM,
        );
      }

      // Half a degree apart: both sit inside the 3° gate, so the gate never
      // closes on the way from one to the other. Only the explicit target
      // change can prevent A's hold from being credited to B.
      final a = nodeAt(0, 'a', 0);
      final b = nodeAt(1, 'b', 0.5);
      final engine = VirtualTourCaptureEngine(
        startPosition: origin,
        startOrientation: Quat.identity,
        nodes: <VirtualTourNode>[a, b],
      );

      aim(engine, a, at: t0);
      aim(engine, a, at: const Duration(milliseconds: 200));
      final switched = aim(engine, b, at: const Duration(milliseconds: 500));
      expect(switched.target?.id, 'b');
      expect(switched.aligned, isTrue);

      // 800 ms after A's first sample, but only 300 ms after the switch.
      final tooEarly = aim(engine, b, at: const Duration(milliseconds: 800));
      expect(tooEarly.readyToCapture, isFalse);
      expect(tooEarly.holdProgress, closeTo(300 / 600, 1e-3));

      final ready = aim(engine, b, at: const Duration(milliseconds: 1100));
      expect(ready.readyToCapture, isTrue);
    });

    test('invalidating a stalled stream drops the hold and the readiness', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      expect(holdSteady(engine, h01, from: t0).readyToCapture, isTrue);
      engine.invalidateStability();
      expect(engine.lastGuidance, isNull);
      expect(
        engine.beginCapture(node: h01, now: hold),
        isNull,
        reason: 'a readiness frozen before the gap must not spend',
      );

      aim(engine, h01, at: const Duration(milliseconds: 700));
      final stillShort = aim(
        engine,
        h01,
        at: const Duration(milliseconds: 1100),
      );
      expect(stillShort.readyToCapture, isFalse);

      final recovered = aim(
        engine,
        h01,
        at: const Duration(milliseconds: 1300),
      );
      expect(recovered.readyToCapture, isTrue);
    });

    test('a pose older than 150 ms closes the shutter', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      aim(engine, h01, at: t0);
      final stale = aim(
        engine,
        h01,
        at: hold,
        measurementAge: const Duration(milliseconds: 200),
      );
      expect(stale.readyToCapture, isFalse);
      expect(stale.blocker, VirtualTourCaptureBlocker.stability);
      expect(stale.poseAge, const Duration(milliseconds: 200));
      expect(stale.state, VirtualTourCaptureState.waitingForStability);
    });

    test('tracking loss is explicit before and after the first fix', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      final cold = aim(engine, h01, tracking: TrackingState.paused);
      expect(cold.state, VirtualTourCaptureState.trackingUnavailable);
      expect(cold.blocker, VirtualTourCaptureBlocker.tracking);

      aim(engine, h01);
      final lost = aim(
        engine,
        h01,
        at: const Duration(milliseconds: 1000),
        tracking: TrackingState.paused,
      );
      expect(lost.state, VirtualTourCaptureState.awaitingTrackingRecovery);
      expect(lost.tracking, isFalse);
    });

    test('an empty layout has no target rather than a complete tour', () {
      final engine = VirtualTourCaptureEngine(
        startPosition: origin,
        startOrientation: Quat.identity,
        nodes: const <VirtualTourNode>[],
      );
      final guidance = engine.evaluate(
        cameraPosition: origin,
        cameraOrientation: Quat.identity,
        tracking: TrackingState.tracking,
        now: t0,
      );
      expect(engine.allCaptured, isFalse);
      expect(guidance.blocker, VirtualTourCaptureBlocker.noTarget);
      expect(guidance.state, VirtualTourCaptureState.readyForAlignment);
      expect(guidance.complete, isFalse);
    });
  });

  group('capture lifecycle', () {
    test('one pass walks capturing → validating → saving → saved', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      final ready = holdSteady(engine, h01, from: t0);
      expect(ready.readyToCapture, isTrue);
      expect(ready.state, VirtualTourCaptureState.waitingForStability);

      final attempt = engine.beginCapture(node: h01, now: hold);
      expect(attempt, isNotNull);
      expect(attempt!.id, 1);
      expect(attempt.node.id, 'h01');
      expect(attempt.attemptNumber, 1);
      expect(attempt.state, VirtualTourCaptureState.capturing);
      expect(attempt.cameraPosition, engine.startPosition);
      expect(attempt.alignmentDeg, lessThan(3.0));

      // Single-flight: a duplicate pose cannot reserve a second still.
      expect(engine.beginCapture(node: h01, now: hold), isNull);
      final busy = aim(
        engine,
        h01,
        at: hold + const Duration(milliseconds: 20),
      );
      expect(busy.state, VirtualTourCaptureState.capturing);
      expect(busy.readyToCapture, isFalse);
      expect(busy.blocker, VirtualTourCaptureBlocker.capturing);

      expect(
        engine.markAttemptState(
          attempt,
          VirtualTourCaptureState.validatingImage,
        ),
        isTrue,
      );
      expect(
        aim(engine, h01, at: hold + const Duration(milliseconds: 40)).state,
        VirtualTourCaptureState.validatingImage,
      );

      expect(
        engine.markAttemptState(attempt, VirtualTourCaptureState.savingImage),
        isTrue,
      );
      expect(
        aim(engine, h01, at: hold + const Duration(milliseconds: 60)).state,
        VirtualTourCaptureState.savingImage,
      );

      engine.completeCapture(
        attempt,
        success: true,
        now: hold + const Duration(milliseconds: 80),
      );
      expect(engine.capturedIds, <String>{'h01'});
      expect(engine.capturedCount, 1);

      final saved = aim(
        engine,
        h01,
        at: hold + const Duration(milliseconds: 100),
      );
      expect(saved.state, VirtualTourCaptureState.captureSucceeded);
      expect(saved.blocker, VirtualTourCaptureBlocker.cooldown);
      expect(saved.readyToCapture, isFalse);
      // The capture gate cannot be re-armed without a fresh evaluation.
      expect(engine.beginCapture(node: h01, now: hold), isNull);
    });

    test('attempt phases cannot jump or go backwards', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');
      holdSteady(engine, h01, from: t0);
      final attempt = engine.beginCapture(node: h01, now: hold)!;

      expect(
        engine.markAttemptState(attempt, VirtualTourCaptureState.savingImage),
        isFalse,
      );
      expect(
        engine.markAttemptState(
          attempt,
          VirtualTourCaptureState.validatingImage,
        ),
        isTrue,
      );
      expect(
        engine.markAttemptState(attempt, VirtualTourCaptureState.capturing),
        isFalse,
      );
      expect(
        engine.markAttemptState(attempt, VirtualTourCaptureState.savingImage),
        isTrue,
      );
      expect(
        engine.markAttemptState(
          attempt,
          VirtualTourCaptureState.validatingImage,
        ),
        isFalse,
      );
    });

    test('a failure leaves the node pending and retries automatically', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');
      final captureAt = hold;

      holdSteady(engine, h01, from: t0);
      final attempt = engine.beginCapture(node: h01, now: captureAt)!;
      engine.completeCapture(
        attempt,
        success: false,
        now: captureAt,
        error: 'The disk was full.',
      );

      expect(engine.capturedCount, 0);
      expect(engine.capturedIds, isEmpty);
      expect(engine.isCapturing, isFalse);
      expect(engine.failureCountFor(h01), 1);
      expect(engine.requiresManualRetryFor(h01), isFalse);
      expect(engine.automaticRetriesRemainingFor(h01), 2);

      final cooling = aim(
        engine,
        h01,
        at: captureAt + const Duration(milliseconds: 500),
      );
      expect(cooling.state, VirtualTourCaptureState.captureFailed);
      expect(cooling.blocker, VirtualTourCaptureBlocker.cooldown);
      expect(cooling.readyToCapture, isFalse);
      expect(cooling.instruction, contains('disk was full'));

      // At 1.5 s the retry window opens and a fresh 600 ms hold starts.
      final retryStart = captureAt + const Duration(milliseconds: 1500);
      final retryAligned = aim(engine, h01, at: retryStart);
      expect(retryAligned.state, VirtualTourCaptureState.waitingForStability);
      expect(retryAligned.blocker, VirtualTourCaptureBlocker.settling);

      final retryReady = holdSteady(
        engine,
        h01,
        from: retryStart + const Duration(milliseconds: 10),
      );
      expect(retryReady.readyToCapture, isTrue);
      expect(retryReady.target?.id, 'h01');
      final retry = engine.beginCapture(
        node: h01,
        now: retryStart + const Duration(milliseconds: 610),
      );
      expect(retry, isNotNull);
      expect(retry!.attemptNumber, 2);
      engine.completeCapture(
        retry,
        success: true,
        now: retryStart + const Duration(milliseconds: 620),
      );
      expect(engine.capturedIds, <String>{'h01'});
      expect(engine.requiresManualRetryFor(h01), isFalse);
    });

    test('two automatic retries, then a manual retry is required', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      // Three failed attempts: the first try plus two automatic retries.
      captureOnce(engine, h01, t0, success: false);
      expect(engine.automaticRetriesRemainingFor(h01), 2);
      expect(engine.requiresManualRetryFor(h01), isFalse);

      captureOnce(
        engine,
        h01,
        const Duration(milliseconds: 2200),
        success: false,
      );
      expect(engine.automaticRetriesRemainingFor(h01), 1);
      expect(engine.requiresManualRetryFor(h01), isFalse);

      captureOnce(
        engine,
        h01,
        const Duration(milliseconds: 4400),
        success: false,
      );
      expect(engine.failureCountFor(h01), 3);
      expect(engine.automaticRetriesRemainingFor(h01), 0);
      expect(engine.requiresManualRetryFor(h01), isTrue);

      final blockedAt = const Duration(milliseconds: 6600);
      final blocked = aim(engine, h01, at: blockedAt);
      expect(blocked.state, VirtualTourCaptureState.captureFailed);
      expect(blocked.blocker, VirtualTourCaptureBlocker.retry);
      expect(blocked.requiresManualRetry, isTrue);
      expect(blocked.instruction, contains('No automatic retries remain'));

      final blockedReady = holdSteady(
        engine,
        h01,
        from: blockedAt + const Duration(milliseconds: 100),
      );
      expect(blockedReady.readyToCapture, isFalse);
      expect(
        engine.beginCapture(
          node: h01,
          now: blockedAt + const Duration(milliseconds: 710),
        ),
        isNull,
      );

      expect(engine.manualRetry(node: h01), isTrue);
      expect(engine.failureCountFor(h01), 0);
      expect(engine.requiresManualRetryFor(h01), isFalse);

      final retried = holdSteady(
        engine,
        h01,
        from: blockedAt + const Duration(milliseconds: 800),
      );
      expect(retried.readyToCapture, isTrue);
      expect(retried.requiresManualRetry, isFalse);
    });

    test('a successful capture cools down and is not re-armed', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');
      captureOnce(engine, h01, t0);

      final justAfter = aim(
        engine,
        h01,
        at: hold + const Duration(milliseconds: 300),
      );
      expect(justAfter.state, VirtualTourCaptureState.captureSucceeded);
      expect(justAfter.target?.id, isNot('h01'));
      expect(justAfter.aligned, isFalse);

      final next = aim(
        engine,
        nodeById(engine, 'h02'),
        at: const Duration(seconds: 2),
      );
      expect(next.target?.id, 'h02');
      expect(next.capturedCount, 1);
    });

    test('the attempt freezes the pose that opened the shutter', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');
      holdSteady(engine, h01, from: t0);
      final attempt = engine.beginCapture(node: h01, now: hold)!;

      expect(
        engine.validateCapturePose(
          attempt: attempt,
          cameraPosition: attempt.cameraPosition,
          cameraOrientation: attempt.cameraOrientation,
        ),
        isNull,
      );
      expect(
        engine.validateCapturePose(
          attempt: attempt,
          cameraPosition: attempt.cameraPosition,
          cameraOrientation: Quat.fromYawPitch(6, 0),
        ),
        isNotNull,
      );
      expect(
        engine.validateCapturePose(
          attempt: attempt,
          cameraPosition: const Vec3(0.11, 0, 0),
          cameraOrientation: attempt.cameraOrientation,
        ),
        isNotNull,
      );
    });

    test('stale attempt ids cannot move or complete the next attempt', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');

      final first = captureOnce(engine, h01, t0, success: false);
      final secondStart = const Duration(milliseconds: 2200);
      holdSteady(engine, h01, from: secondStart);
      final second = engine.beginCapture(node: h01, now: secondStart + hold)!;
      expect(second.id, greaterThan(first.id));

      expect(
        engine.markAttemptState(first, VirtualTourCaptureState.validatingImage),
        isFalse,
      );
      engine.completeCapture(first, success: true, now: secondStart + hold);
      expect(engine.isCapturing, isTrue);
      expect(engine.capturedIds, isEmpty);

      engine.completeCapture(second, success: true, now: secondStart + hold);
      expect(engine.capturedIds, <String>{'h01'});
    });

    test('cancelling a capture does not spend an automatic retry', () {
      final engine = engineAtStart();
      final h01 = nodeById(engine, 'h01');
      holdSteady(engine, h01, from: t0);
      final attempt = engine.beginCapture(node: h01, now: hold)!;

      engine.cancelCapture(attempt);
      expect(engine.isCapturing, isFalse);
      expect(engine.failureCountFor(h01), 0);
      expect(engine.automaticRetriesRemainingFor(h01), 2);

      final trackingLost = aim(
        engine,
        h01,
        at: hold + const Duration(milliseconds: 100),
        tracking: TrackingState.paused,
      );
      expect(
        trackingLost.state,
        VirtualTourCaptureState.awaitingTrackingRecovery,
      );

      final ready = holdSteady(
        engine,
        h01,
        from: hold + const Duration(milliseconds: 200),
      );
      expect(ready.readyToCapture, isTrue);
    });

    test(
      'capturing all 30 nodes reports completion and refuses another shot',
      () {
        final engine = engineAtStart();
        for (var i = 0; i < engine.nodes.length; i++) {
          final node = engine.nodes[i];
          final at = Duration(seconds: i * 3);
          final guidance = aim(engine, node, at: at);
          expect(guidance.target?.id, node.id, reason: node.id);

          final settled = holdSteady(
            engine,
            node,
            from: at + const Duration(milliseconds: 10),
          );
          expect(settled.readyToCapture, isTrue, reason: node.id);
          final attempt = engine.beginCapture(
            node: node,
            now: at + const Duration(milliseconds: 610),
          );
          expect(attempt, isNotNull, reason: node.id);
          engine.completeCapture(
            attempt!,
            success: true,
            now: at + const Duration(milliseconds: 620),
          );
        }

        expect(engine.capturedCount, 30);
        expect(engine.allCaptured, isTrue);
        final done = engine.evaluate(
          cameraPosition: origin,
          cameraOrientation: Quat.identity,
          tracking: TrackingState.tracking,
          now: const Duration(minutes: 5),
        );
        expect(done.complete, isTrue);
        expect(done.blocker, VirtualTourCaptureBlocker.complete);
        expect(done.state, VirtualTourCaptureState.completed);
        expect(engine.beginCapture(node: engine.nodes.first, now: t0), isNull);
      },
    );
  });
}
