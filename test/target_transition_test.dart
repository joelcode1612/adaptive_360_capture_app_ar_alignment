import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';

import 'capture_flow_test.dart' show bootHarness, calibratedHarness, Harness;

/// The accept → next-target lifecycle.
///
/// The bug this suite pins: after target 1 is accepted, its GLB disappears
/// (correct — only the active target is drawn) but the next one never appears.
/// The cause is in the transition, not in rendering: `selectTarget` releases
/// the old anchor and then *awaits* the creation of the new one, and a pose
/// that arrives inside that window drives `_refreshAnchorIfDue` into its
/// "anchor is gone" branch, which starts a **second, concurrent placement**
/// of the same target. Two anchors, two scene pushes, and the scene can end up
/// referencing the anchor the state flags do not describe — a paused anchor
/// in the scene while Dart reports `activeAnchorTracking: true`, which the
/// native renderer answers by drawing nothing at all.
///
/// None of this was visible before because the auto-advance path had no test:
/// `bootHarness` hardcoded `autoAdvance: false`, so the transition had never
/// been executed by the suite.
void main() {
  final radius = const CaptureGeometryConfig().targetRadiusM;

  /// Aims at the active target long enough to satisfy the stability gate and
  /// fires the shutter through the real button path, then waits for the write
  /// to land. The wait matters: the shutter is asynchronous file IO, and a
  /// fixed flush makes the assertion about the machine's load rather than
  /// about the outcome.
  Future<void> captureActive(Harness harness, {required int id}) async {
    expect(harness.controller.state.activeTargetId, id);
    await harness.emitStable();
    // Wait for the emitted poses to be processed before pressing: the stream
    // delivers asynchronously, and on a loaded machine a fixed flush can
    // leave the last poses queued — the gate would then read "not stable" and
    // the iteration would silently capture nothing.
    await Harness.waitFor(() => harness.controller.state.stable);
    await harness.controller.captureNow();
    await Harness.waitFor(
      () =>
          harness.controller.state.targets[id - 1].status !=
          CaptureTargetStatus.active,
    );
  }

  final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);

  Vec3 expectedPosition(CaptureTargetSpec spec) =>
      frame.localToWorld(spec.direction).normalized * radius;

  /// The highest opacity any draw for [targetId]'s node carries in the scene.
  /// The whole arrangement is always in the scene; the active target is the
  /// opaque one and everything else is a ghost, so this is how the test sees
  /// which target is highlighted without reading the controller's mind.
  double activeOpacityFor(Harness harness, int targetId) {
    final anchorId =
        harness.controller.state.targets[targetId - 1].anchorIdentifier;
    if (anchorId == null) return -1;
    var max = 0.0;
    for (final draw in harness.gateway.lastScene) {
      if (draw.anchorId == anchorId && draw.opacity > max) {
        max = draw.opacity;
      }
    }
    return max;
  }

  /// The world direction the arrow must aim at for [spec]: the station frame
  /// is a mirror (station-local +X is the physical right, `−q·x̂`), so the
  /// expected vector goes through the same transform placement uses.
  Vec3 expectedDirection(CaptureTargetSpec spec) =>
      frame.localToWorld(spec.direction).normalized;

  group('the 30-target arrangement', () {
    test('ids are unique and 1…30, one of each ring', () {
      final specs = buildCaptureSphere();
      expect(specs.length, kCaptureTargetCount);
      expect(
        specs.map((CaptureTargetSpec s) => s.targetId).toList(),
        List<int>.generate(kCaptureTargetCount, (int i) => i + 1),
      );
      final rings = <String, int>{};
      for (final spec in specs) {
        rings[spec.ringName] = (rings[spec.ringName] ?? 0) + 1;
      }
      // 12 horizontal / 8 upper / 8 lower / 1 zenith / 1 nadir.
      expect(rings[kMiddleRingName], 12);
      expect(rings[kUpperRingName], 8);
      expect(rings[kLowerRingName], 8);
      expect(rings[kZenithRingName], 1);
      expect(rings[kNadirRingName], 1);
      expect(specs.where((CaptureTargetSpec s) => s.isZenith).length, 1);
      expect(specs.where((CaptureTargetSpec s) => s.isNadir).length, 1);
    });

    test('every position is unique and sits on the target sphere', () {
      final specs = buildCaptureSphere();
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final positions = <Vec3>[
        for (final spec in specs)
          frame.origin + frame.localToWorld(spec.direction).normalized * radius,
      ];
      for (var i = 0; i < positions.length; i++) {
        // Radius: P0 + Q0.rotate(dir × radius), never an arbitrary origin.
        expect(positions[i].length, closeTo(radius, 1e-9));
        for (var j = i + 1; j < positions.length; j++) {
          final d = positions[i] - positions[j];
          expect(
            d.length,
            greaterThan(1e-3),
            reason:
                'targets ${specs[i].targetId} and ${specs[j].targetId} '
                'share a position',
          );
        }
      }
    });

    test('zenith is up and nadir is down', () {
      // The near-poles sit at ±85°, not the exact poles: y = ±sin(85°), within
      // 5° of the vertical.
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final poleY = math.sin(85 * math.pi / 180);
      for (final spec in buildCaptureSphere()) {
        final dir = frame.localToWorld(spec.direction).normalized;
        if (spec.isZenith) expect(dir.y, closeTo(poleY, 1e-9));
        if (spec.isNadir) expect(dir.y, closeTo(-poleY, 1e-9));
      }
    });

    test(
      'calibration places all 30 nodes at once, each at its own position',
      () async {
        // The whole arrangement is created in one pass, before the first
        // photograph: 30 anchors, 30 distinct positions, no raycast, no shared
        // node — and none of it re-done when a target is selected.
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: false,
        );
        final specs = buildCaptureSphere();

        expect(harness.gateway.anchorCreations, kCaptureTargetCount);
        expect(harness.gateway.anchors.length, kCaptureTargetCount);
        expect(
          harness.gateway.raycastCalls,
          0,
          reason: 'placement must not raycast',
        );
        expect(
          harness.gateway.probeCalls,
          1,
          reason: 'the probe runs once, at calibration',
        );
        // Bezel + target for every placed node.
        expect(harness.gateway.lastScene.length, kCaptureTargetCount * 2);

        final ids = <String>{};
        for (final spec in specs) {
          final target = harness.controller.state.targets[spec.targetId - 1];
          final expected = expectedPosition(spec);
          expect(target.anchorIdentifier, isNotNull);
          expect(target.anchorWorldPosition!.x, closeTo(expected.x, 1e-9));
          expect(target.anchorWorldPosition!.y, closeTo(expected.y, 1e-9));
          expect(target.anchorWorldPosition!.z, closeTo(expected.z, 1e-9));
          expect(target.anchorSurfaceType, 'WORLD');
          ids.add(target.anchorIdentifier!);
        }
        expect(ids.length, kCaptureTargetCount, reason: '30 distinct nodes');
      },
    );

    test('selecting a target touches no node', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: false,
      );
      final before = harness.gateway.anchorCreations;
      final sceneBefore = harness.gateway.lastScene;

      await harness.controller.selectTarget(9);
      await Harness.flush();

      expect(harness.controller.state.activeTargetId, 9);
      expect(
        harness.gateway.anchorCreations,
        before,
        reason: 'selection is a highlight, never a placement',
      );
      expect(harness.gateway.anchors.length, kCaptureTargetCount);
      // The scene changed only in which target is highlighted: same node set,
      // target 9 now the opaque one.
      expect(harness.gateway.lastScene.length, sceneBefore.length);
      expect(activeOpacityFor(harness, 9), 1);
      expect(activeOpacityFor(harness, 1), lessThan(1));
    });
  });

  group('accepting target N activates target N+1', () {
    test(
      'capturing target 1 activates target 2, placed at its own position',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        expect(harness.controller.state.activeTargetId, 1);
        final firstAnchor =
            harness.controller.state.targets[0].anchorIdentifier!;
        final creations = harness.gateway.anchorCreations;

        await captureActive(harness, id: 1);

        final state = harness.controller.state;
        expect(state.activeTargetId, 2);
        expect(state.targets[0].status, CaptureTargetStatus.captured);
        expect(state.targets[0].imagePath, isNotNull);
        expect(state.capturedCount, 1);
        expect(harness.gateway.stillRequests, 1);

        // Target 2 was already placed at calibration and keeps its own node:
        // accepting target 1 neither recreated nor released anything.
        final second = state.targets[1];
        expect(second.anchorIdentifier, isNotNull);
        expect(second.anchorIdentifier, isNot(firstAnchor));
        expect(second.anchorWorldPosition, isNotNull);
        final expected = expectedPosition(buildCaptureSphere()[1]);
        expect(second.anchorWorldPosition!.x, closeTo(expected.x, 1e-9));
        expect(second.anchorWorldPosition!.y, closeTo(expected.y, 1e-9));
        expect(second.anchorWorldPosition!.z, closeTo(expected.z, 1e-9));
        // The highlight moved: target 2 is the opaque node now, and target 1
        // — finished — is omitted from the scene entirely rather than left
        // drawn over the photograph it just took.
        expect(
          harness.gateway.anchorCreations,
          creations,
          reason: 'a photograph must not create a node',
        );
        expect(
          harness.gateway.anchors.length,
          kCaptureTargetCount,
          reason: 'nor release one',
        );
        expect(activeOpacityFor(harness, 2), 1);
        expect(activeOpacityFor(harness, 1), 0);
        expect(activeOpacityFor(harness, 3), lessThan(1));
        expect(state.activeAnchored, isTrue);
        expect(state.activeAnchorTracking, isTrue);
      },
    );

    test('the arrow aims at the new target, not at the released one', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 2);

      await harness.emit();
      final alignment = harness.controller.state.alignment;
      expect(alignment, isNotNull);
      expect(alignment!.usable, isTrue);
      final expectedDir = expectedDirection(buildCaptureSphere()[1]);
      expect(alignment.directionWorld.x, closeTo(expectedDir.x, 1e-9));
      expect(alignment.directionWorld.y, closeTo(expectedDir.y, 1e-9));
      expect(alignment.directionWorld.z, closeTo(expectedDir.z, 1e-9));
    });

    test('capturing target 2 activates target 3', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 2);
      await captureActive(harness, id: 2);

      final state = harness.controller.state;
      expect(state.activeTargetId, 3);
      expect(state.targets[1].status, CaptureTargetStatus.captured);
      expect(state.targets[2].anchorIdentifier, isNotNull);
      expect(state.capturedCount, 2);
      expect(harness.gateway.anchors.length, kCaptureTargetCount);
    });

    test(
      'all 30 captures complete on one session, one anchor each',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        for (var id = 1; id <= kCaptureTargetCount; id++) {
          expect(
            harness.controller.state.activeTargetId,
            id,
            reason: 'target $id must be active before it is captured',
          );
          await captureActive(harness, id: id);
        }

        final state = harness.controller.state;
        expect(state.phase, CapturePhase.complete);
        expect(state.capturedCount, kCaptureTargetCount);
        expect(harness.gateway.stillRequests, kCaptureTargetCount);
        // One anchor per target for the whole sequence — the arrangement was
        // created once at calibration and never rebuilt — and the last target
        // does not wrap back to target 1.
        expect(harness.gateway.anchorCreations, kCaptureTargetCount);
        expect(harness.gateway.anchors.length, kCaptureTargetCount);
        expect(state.station, isNotNull);

        // The last completion leaves no active target, no alignment and no
        // marker: there is nothing left to guide and nothing left to draw.
        expect(state.activeTargetId, 0);
        expect(state.activeTarget, isNull);
        expect(state.guidanceTarget, isNull);
        expect(state.alignment, isNull);
        expect(state.activeAnchored, isFalse);
        expect(harness.gateway.lastScene, isEmpty);

        // A completed target cannot be selected again as guidance; only the
        // explicit reset returns the station to a capturable state.
        await harness.controller.selectTarget(kCaptureTargetCount);
        await Harness.flush();
        expect(harness.controller.state.activeTargetId, 0);

        await harness.controller.retake(kCaptureTargetCount);
        await Harness.flush();
        final reopened = harness.controller.state;
        expect(reopened.phase, CapturePhase.capturing);
        expect(reopened.activeTargetId, kCaptureTargetCount);
        expect(reopened.guidanceTarget?.targetId, kCaptureTargetCount);
        expect(reopened.targets.last.status, CaptureTargetStatus.active);
        expect(reopened.capturedCount, kCaptureTargetCount - 1);
      },
      // 30 frames each write and read back a real JPEG. Alone that is a few
      // seconds; the default 30 s budget expires when the whole suite runs the
      // files in parallel on a loaded machine.
      timeout: const Timeout(Duration(minutes: 3)),
    );
  });

  // ── the transition race ────────────────────────────────────────────────
  //
  // A device delivers poses at 30–60 Hz while `selectTarget` is between its
  // awaits. These tests hold the placement open at exactly that point and
  // push a pose through, which is the only way to reach the interleaving
  // that makes the next marker invisible on hardware.
  // ── the placement race ────────────────────────────────────────────────
  //
  // A device delivers poses at 30–60 Hz while `placeAll` is between its
  // awaits, and a full arrangement rebuild (`rescanActive`) runs under the
  // same hazard.
  // These tests hold a placement open at exactly that point and push a pose
  // through, which is the only way to reach the interleaving that makes
  // markers invisible on hardware.
  group('a pose that arrives mid-placement', () {
    test('neither duplicates a node nor loses the arrangement', () async {
      // Freeze the calibration pass at its very first anchor creation.
      final harness = await bootHarness(autoCapture: false);
      await harness.emit();
      final gate = Completer<void>();
      harness.gateway.anchorGate = gate.future;
      final calibrating = harness.controller.calibrateStation(stationId: 's1');
      await Harness.flush();
      expect(
        harness.gateway.anchorCreations,
        0,
        reason: 'the pass must park before creating anything',
      );

      // The pose a 60 Hz stream would deliver right now, with the refresh
      // interval elapsed so the twice-a-second re-read fires too.
      harness.now = harness.now.add(const Duration(milliseconds: 600));
      harness.gateway.emitPose(
        position: Vec3.zero,
        worldOrientation: Quat.identity,
      );
      await Harness.flush();

      gate.complete();
      await calibrating;
      await Harness.flush();

      // Exactly 26 nodes, no duplicates, nothing lost — and the mid-flight
      // pose started no placement of its own.
      expect(harness.gateway.anchorCreations, kCaptureTargetCount);
      expect(harness.gateway.anchors.length, kCaptureTargetCount);
      final state = harness.controller.state;
      expect(state.activeTargetId, 1);
      expect(state.activeAnchored, isTrue);
      expect(state.activeAnchorTracking, isTrue);
      for (final spec in buildCaptureSphere()) {
        final target = state.targets[spec.targetId - 1];
        final expected = expectedPosition(spec);
        expect(target.anchorIdentifier, isNotNull);
        expect(target.anchorWorldPosition!.x, closeTo(expected.x, 1e-9));
        expect(target.anchorWorldPosition!.y, closeTo(expected.y, 1e-9));
        expect(target.anchorWorldPosition!.z, closeTo(expected.z, 1e-9));
      }
    });

    test(
      'a selection during a recovery queues, and the later one wins',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        final before = harness.gateway.anchorCreations;

        // ARCore drops the active target's anchor; the refresh starts the
        // bounded recovery, which parks on the gate.
        final activeId =
            harness.controller.state.activeTarget!.anchorIdentifier!;
        harness.gateway.dropAnchor(activeId);
        final gate = Completer<void>();
        harness.gateway.anchorGate = gate.future;
        harness.now = harness.now.add(const Duration(milliseconds: 600));
        harness.gateway.emitPose(
          position: Vec3.zero,
          worldOrientation: Quat.identity,
        );
        await Harness.flush();

        // The user picks another target while the recovery is in flight.
        final selection = harness.controller.selectTarget(3);
        await Harness.flush();

        gate.complete();
        await selection;
        await Harness.flush();

        final state = harness.controller.state;
        // The recovery re-placed exactly the one dropped target…
        expect(harness.gateway.anchorCreations, before + 1);
        expect(harness.gateway.anchors.length, kCaptureTargetCount);
        // …the queued selection then ran, and the later request won.
        expect(state.activeTargetId, 3);
        expect(state.activeAnchored, isTrue);
        expect(activeOpacityFor(harness, 3), 1);
      },
    );

    test('a stale refresh answer is discarded, not written onto the new '
        'target', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      // Open a refresh of the active target's anchor, as the twice-a-second
      // re-read would, and leave it unanswered while the user moves on.
      final refreshGate = Completer<void>();
      harness.gateway.refreshGate = refreshGate.future;
      harness.now = harness.now.add(const Duration(milliseconds: 600));
      harness.gateway.emitPose(
        position: Vec3.zero,
        worldOrientation: Quat.identity,
      );
      await Harness.flush();
      expect(harness.gateway.refreshCalls, greaterThan(0));

      // The capture completes and target 2 takes over while the refresh is
      // still open.
      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 2);

      // The host now answers about the *old* anchor, still claiming it
      // tracks — the answer a lagging detach can produce.
      final second = harness.controller.state.targets[1];
      refreshGate.complete();
      await Harness.flush();

      final state = harness.controller.state;
      expect(
        state.targets[1].anchorIdentifier,
        second.anchorIdentifier,
        reason:
            'a stale answer must not re-point the new target at the '
            'old anchor',
      );
      expect(
        harness.gateway.anchorCreations,
        kCaptureTargetCount,
        reason: 'no recovery was started by the stale answer',
      );
      expect(harness.gateway.anchors.length, kCaptureTargetCount);
      expect(state.activeAnchored, isTrue);
      expect(state.activeAnchorTracking, isTrue);

      await harness.emit();
      final expectedDir = expectedDirection(buildCaptureSphere()[1]);
      final alignment = harness.controller.state.alignment!;
      expect(alignment.directionWorld.x, closeTo(expectedDir.x, 1e-9));
      expect(alignment.directionWorld.y, closeTo(expectedDir.y, 1e-9));
      expect(alignment.directionWorld.z, closeTo(expectedDir.z, 1e-9));
    });

    test(
      'two overlapping selections place nothing and highlight the later one',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        final creations = harness.gateway.anchorCreations;

        // Two selections in the same event-loop turn: neither places anything,
        // and the later request is the one that ends up active.
        final first = harness.controller.selectTarget(2);
        final second = harness.controller.selectTarget(3);
        await first;
        await second;
        await Harness.flush();

        final state = harness.controller.state;
        expect(state.activeTargetId, 3, reason: 'the later request wins');
        expect(harness.gateway.anchorCreations, creations);
        expect(harness.gateway.anchors.length, kCaptureTargetCount);
        expect(activeOpacityFor(harness, 3), 1);
        expect(activeOpacityFor(harness, 2), lessThan(1));
      },
    );
  });

  // ── failures keep the current target ───────────────────────────────────
  group('a failed capture or validation', () {
    test('keeps the target active and does not advance', () async {
      final harness = await calibratedHarness(
        stillError: const ArCoreFailure('Fake: no camera image.'),
        autoCapture: false,
        autoAdvance: true,
      );
      final anchor = harness.controller.state.targets[0].anchorIdentifier;

      await captureActive(harness, id: 1);

      final state = harness.controller.state;
      expect(harness.gateway.stillRequests, 1);
      expect(state.targets[0].status, CaptureTargetStatus.failed);
      expect(state.targets[0].imagePath, isNull);
      expect(state.activeTargetId, 1, reason: 'no advance on failure');
      expect(state.capturedCount, 0);
      expect(state.activeAnchored, isTrue);
      expect(
        harness.gateway.anchors.length,
        kCaptureTargetCount,
        reason: 'the whole arrangement survives a failed capture',
      );
      expect(
        harness.gateway.anchorCreations,
        kCaptureTargetCount,
        reason: 'no re-placement was started',
      );
      // The GLB is still in the scene, on the same anchor.
      expect(
        harness.gateway.lastScene.any((d) => d.anchorId == anchor),
        isTrue,
      );
      expect(state.lastError, isNotNull);
    });

    test('bytes that are not a photograph do not advance either', () async {
      final harness = await calibratedHarness(
        stillBytes: Uint8List.fromList(const <int>[1, 2, 3]),
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 1);
      expect(harness.controller.state.capturedCount, 0);
      expect(harness.gateway.anchorCreations, kCaptureTargetCount);
    });

    test('a retry replaces the anchor once and then advances', () async {
      final harness = await calibratedHarness(
        stillError: const ArCoreFailure('Fake: no camera image.'),
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 1);
      final creations = harness.gateway.anchorCreations;

      // Fix the host, retake the same target, capture again. A retake re-uses
      // the node that is already placed: no re-placement at all.
      harness.gateway.stillError = null;
      await harness.controller.retake(1);
      await Harness.flush();
      expect(
        harness.gateway.anchorCreations,
        creations,
        reason: 'a retry re-uses the placed node',
      );
      expect(harness.gateway.anchors.length, kCaptureTargetCount);

      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 2);
      expect(harness.controller.state.capturedCount, 1);
      expect(harness.gateway.anchorCreations, creations);
    });
  });

  // ── persistence follows the flow ──────────────────────────────────────
  //
  // A failure that lives only in memory reads as "never attempted" after a
  // resume, and a station record that still counts a retaken target reports
  // one more photograph than exists on disk. Both are persistence bugs, not
  // display bugs, so both are asserted against the repository.
  group('persistence follows the flow', () {
    test('a failed capture is persisted, not only shown', () async {
      final harness = await calibratedHarness(
        stillError: const ArCoreFailure('Fake: no camera image.'),
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      await Harness.flush();

      final onDisk = harness.repository.targets['s1']!;
      expect(onDisk[0].status, CaptureTargetStatus.failed);
      expect(harness.repository.stations['s1']!.capturedCount, 0);
    });

    test('a retake persists the decremented station count', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      expect(harness.repository.stations['s1']!.capturedCount, 1);

      await harness.controller.retake(1);
      await Harness.flush();
      expect(
        harness.repository.targets['s1']![0].status,
        CaptureTargetStatus.pending,
      );
      expect(
        harness.repository.stations['s1']!.capturedCount,
        0,
        reason: 'a retaken target is not a saved photograph',
      );
    });
  });

  // ── the plate faces the station at every ring ─────────────────────────
  group('target orientation', () {
    test('every plate faces the capture centre, poles included', () {
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      for (final spec in buildCaptureSphere()) {
        final worldDirection = frame.localToWorld(spec.direction).normalized;
        final facing = (-worldDirection).normalized;
        final rotation = facingRotation(facing);

        expect(rotation.x.isFinite, isTrue);
        expect(rotation.y.isFinite, isTrue);
        expect(rotation.z.isFinite, isTrue);
        expect(rotation.w.isFinite, isTrue);
        expect(rotation.length, closeTo(1, 1e-9));

        // The model's own +Z — the plate's face normal — points at the
        // station, from every yaw and pitch.
        final face = rotation.rotate(Vec3.unitZ);
        expect(face.x, closeTo(facing.x, 1e-9));
        expect(face.y, closeTo(facing.y, 1e-9));
        expect(face.z, closeTo(facing.z, 1e-9));

        if (spec.isZenith || spec.isNadir) {
          // Not the degenerate identity rotation: a near-pole plate faces
          // almost straight down (zenith) or up (nadir) — a ~85° turn about a
          // horizontal axis — so its face is within 5° of ±Y and nowhere near
          // the +Z a flat-on plate would use.
          final up = spec.isZenith ? -1.0 : 1.0;
          expect(face.y * up, greaterThan(0.99));
          expect(face.dot(Vec3.unitZ).abs(), lessThan(0.09));
        }
      }
    });
  });
}
