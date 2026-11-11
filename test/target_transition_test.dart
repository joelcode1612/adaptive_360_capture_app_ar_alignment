import 'dart:async';
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/plane_anchor_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';

import 'capture_flow_test.dart' show bootHarness, calibratedHarness, Harness;

/// The accept → next-target lifecycle.
///
/// A calibrated station has exactly one ARCore anchor. Its 26 targets are
/// immutable local transforms below that reference, so advancing a target must
/// only move the highlighted draw: it must not create, release, or replace an
/// ARCore anchor. The tests below pin that contract through normal progression,
/// pause/recovery boundaries, and overlapping UI requests.
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
          !harness.controller.state.capturing &&
          !harness.controller.state.targets[id - 1].status.isCapturing &&
          harness.controller.state.targets[id - 1].status !=
              CaptureTargetStatus.active,
    );
  }

  final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);

  Vec3 expectedPosition(CaptureTargetSpec spec) =>
      frame.localToWorld(spec.direction).normalized * radius;

  /// The highest opacity for [targetId]'s local transform in the scene.
  ///
  /// All targets intentionally share one anchor id, so anchor id alone cannot
  /// identify a draw. The model-matrix translation is the frozen local target
  /// transform below that station anchor, which is exactly what the native
  /// renderer uses to distinguish the 26 targets.
  double activeOpacityFor(Harness harness, int targetId) {
    final state = harness.controller.state;
    final station = state.station;
    final target = state.targets[targetId - 1];
    final worldPosition = target.anchorWorldPosition;
    if (station == null || worldPosition == null) return 0;
    final localPosition = worldPosition - station.referencePosition;
    var max = 0.0;
    for (final draw in harness.gateway.lastScene) {
      final matrix = draw.modelMatrix;
      final matchesTarget =
          draw.meshId == PlaneAnchorService.targetMeshId &&
          draw.useModelTranslation &&
          matrix != null &&
          (matrix[12] - localPosition.x).abs() < 1e-5 &&
          (matrix[13] - localPosition.y).abs() < 1e-5 &&
          (matrix[14] - localPosition.z).abs() < 1e-5;
      if (matchesTarget && draw.opacity > max) {
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

  group('the 26-target arrangement', () {
    test('ids are unique and 1…26, one of each ring', () {
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
      // 8 horizontal / 8 upper / 8 lower / 1 zenith / 1 nadir.
      expect(rings[kMiddleRingName], 8);
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
      // The poles sit at the exact ±90°: y = ±1, straight up and straight down.
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final poleY = 1.0;
      for (final spec in buildCaptureSphere()) {
        final dir = frame.localToWorld(spec.direction).normalized;
        if (spec.isZenith) expect(dir.y, closeTo(poleY, 1e-9));
        if (spec.isNadir) expect(dir.y, closeTo(-poleY, 1e-9));
      }
    });

    test(
      'calibration binds one station anchor to 26 frozen target transforms',
      () async {
        // The arrangement is created in one pass before the first photograph:
        // one shared station anchor plus 26 distinct local transforms, with no
        // raycast and no independent target anchors.
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: false,
        );
        final specs = buildCaptureSphere();

        expect(harness.gateway.anchorCreations, 1);
        expect(harness.gateway.anchors.length, 1);
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
        expect(ids, hasLength(1), reason: 'one shared station anchor');
      },
    );

    test(
      'selecting a target keeps the shared anchor and transforms intact',
      () async {
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
        expect(harness.gateway.anchors.length, 1);
        // The scene changed only in which frozen transform is highlighted:
        // target 9 is opaque while target 1 remains a ghost.
        expect(harness.gateway.lastScene.length, sceneBefore.length);
        expect(activeOpacityFor(harness, 9), 1);
        expect(activeOpacityFor(harness, 1), lessThan(1));
      },
    );
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

        // Target 2 was already placed at calibration on the same station
        // anchor. Accepting target 1 neither recreates nor releases anything.
        final second = state.targets[1];
        expect(second.anchorIdentifier, isNotNull);
        expect(second.anchorIdentifier, firstAnchor);
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
        expect(harness.gateway.anchors.length, 1, reason: 'nor release one');
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
      expect(harness.gateway.anchors.length, 1);
    });

    test(
      'all 26 captures complete on one session and one shared station anchor',
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
        // One station anchor hosts every frozen target transform for the whole
        // sequence. The arrangement is created once and never rebuilt, and the
        // last target does not wrap back to target 1.
        expect(harness.gateway.anchorCreations, 1);
        expect(harness.gateway.anchors.length, 1);
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
      // 26 frames each write and read back a real JPEG. Alone that is a few
      // seconds; the default 30 s budget expires when the whole suite runs the
      // files in parallel on a loaded machine.
      timeout: const Timeout(Duration(minutes: 3)),
    );
  });

  // ── the placement race ────────────────────────────────────────────────
  //
  // A device can deliver a pose while calibration is creating the one shared
  // station anchor. That event must not create a second reference or rewrite
  // any of the 26 frozen target transforms.
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

      // Exactly one station anchor, 26 frozen transforms, and no duplicate
      // placement started by the mid-flight pose.
      expect(harness.gateway.anchorCreations, 1);
      expect(harness.gateway.anchors.length, 1);
      final state = harness.controller.state;
      expect(state.activeTargetId, 1);
      expect(state.activeAnchored, isTrue);
      expect(state.activeAnchorTracking, isTrue);
      for (final spec in buildCaptureSphere()) {
        final target = state.targets[spec.targetId - 1];
        final expected = expectedPosition(spec);
        expect(target.anchorIdentifier, state.targets.first.anchorIdentifier);
        expect(target.anchorWorldPosition!.x, closeTo(expected.x, 1e-9));
        expect(target.anchorWorldPosition!.y, closeTo(expected.y, 1e-9));
        expect(target.anchorWorldPosition!.z, closeTo(expected.z, 1e-9));
      }
    });

    test(
      'a missing shared station anchor requires explicit recalibration',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        final before = harness.gateway.anchorCreations;

        // ARCore dropped the only station anchor. Unlike PAUSED tracking, a
        // missing anchor has no fresh pose to verify, so selecting another target
        // must never create a replacement at the camera's current pose.
        final activeId =
            harness.controller.state.activeTarget!.anchorIdentifier!;
        harness.gateway.dropAnchor(activeId);
        harness.now = harness.now.add(const Duration(milliseconds: 600));
        harness.gateway.emitPose(
          position: Vec3.zero,
          worldOrientation: Quat.identity,
        );
        await Harness.waitFor(() => harness.controller.state.station == null);

        // A later selection can only surface the blocked/recalibration state. It
        // must not revive the old reference or create another native anchor.
        await harness.controller.selectTarget(3);
        await Harness.flush();

        final state = harness.controller.state;
        expect(harness.gateway.anchorCreations, before);
        expect(harness.gateway.anchors, isEmpty);
        expect(state.station, isNull);
        expect(state.phase, CapturePhase.scanning);
        expect(state.activeTargetId, 0);
        expect(
          state.stationRecoveryPhase,
          StationRecoveryPhase.anchorRecoveryRequired,
        );
        expect(state.message, contains('Recalibrate at the station'));
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
        1,
        reason: 'no recovery was started by the stale answer',
      );
      expect(harness.gateway.anchors.length, 1);
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
        expect(harness.gateway.anchors.length, 1);
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
        1,
        reason: 'the whole arrangement survives a failed capture',
      );
      expect(
        harness.gateway.anchorCreations,
        1,
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
      expect(harness.gateway.anchorCreations, 1);
    });

    test('a retry keeps the shared anchor and then advances', () async {
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
        reason: 'a retry reuses the shared station anchor',
      );
      expect(harness.gateway.anchors.length, 1);

      await captureActive(harness, id: 1);
      expect(harness.controller.state.activeTargetId, 2);
      expect(harness.controller.state.capturedCount, 1);
      expect(harness.gateway.anchorCreations, creations);
    });
  });

  // ── persistence follows the flow ──────────────────────────────────────
  //
  // A failure that lives only in memory reads as "never attempted" after a
  // resume. Conversely, a retake must not write a decrement before a
  // replacement is durably accepted: the previous evidence remains the
  // restart authority until then. Both are persistence bugs, not display
  // bugs, so both are asserted against the repository.
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

    test('a failed retake keeps the prior durable record and count', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await captureActive(harness, id: 1);
      final originalPath = harness.controller.state.targets[0].imagePath;
      expect(harness.repository.stations['s1']!.capturedCount, 1);

      await harness.controller.retake(1);
      await Harness.flush();
      expect(harness.controller.state.targets[0].imagePath, originalPath);
      expect(
        harness.repository.targets['s1']![0].status,
        CaptureTargetStatus.captured,
      );
      expect(
        harness.repository.stations['s1']!.capturedCount,
        1,
        reason: 'the accepted image remains the restart authority',
      );

      harness.gateway.stillError = const ArCoreFailure(
        'Fake: replacement failed.',
      );
      await captureActive(harness, id: 1);

      final state = harness.controller.state;
      expect(state.targets[0].status, CaptureTargetStatus.captured);
      expect(state.targets[0].imagePath, originalPath);
      expect(state.capturedCount, 1);
      expect(
        harness.repository.targets['s1']![0].status,
        CaptureTargetStatus.captured,
      );
      expect(harness.repository.targets['s1']![0].imagePath, originalPath);
      expect(harness.repository.stations['s1']!.capturedCount, 1);
      final records = await harness.repository.loadRecords('s1');
      expect(records, hasLength(1));
      expect(records.single.imagePath, originalPath);
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
          // Not the degenerate identity rotation: a pole plate faces straight
          // down (zenith) or up (nadir) — a full 90° turn about a horizontal
          // axis — so its face is ±Y and nowhere near the +Z a flat-on plate
          // would use.
          final up = spec.isZenith ? -1.0 : 1.0;
          expect(face.y * up, greaterThan(0.99));
          expect(face.dot(Vec3.unitZ).abs(), lessThan(0.09));
        }
      }
    });
  });
}
