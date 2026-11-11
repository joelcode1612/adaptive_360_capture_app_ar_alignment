import 'dart:async';

import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/plane_anchor_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

import 'capture_flow_test.dart' show calibratedHarness, Harness;

/// Completion is a photograph, not an alignment reading.
///
/// These pin the guidance contract: the arrow and ring belong to the one
/// incomplete active target, a successfully saved target leaves the active
/// guidance immediately, and the next target is chosen from the actual list of
/// incomplete targets. A ring at 100% without a saved photograph must not
/// advance anything, and a late or repeated completion must not skip a target.
void main() {
  /// Selects [id], holds the aim until the stability window elapses, and fires
  /// the shutter through the controller's real capture path.
  ///
  /// `captureNow` deliberately arms a fire-and-forget transaction, so wait for
  /// the verified disk commit before asserting completion or the next target.
  Future<void> aimAndCapture(Harness harness, int id) async {
    await harness.controller.selectTarget(id);
    await Harness.flush();
    await harness.emitStable(count: 20);
    await Harness.waitFor(() => harness.controller.state.stable);
    await harness.controller.captureNow();
    await Harness.waitForCaptureSettlement(harness.controller);
  }

  group('alignment is not completion', () {
    test('a full ring without a photograph does not advance', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await harness.controller.selectTarget(1);
      await harness.emitStable(count: 20);
      await Harness.waitFor(() => harness.controller.state.stable);

      final state = harness.controller.state;
      expect(state.alignment, isNotNull);
      expect(state.alignment!.withinTolerance, isTrue);
      expect(state.stable, isTrue);
      expect(state.ringComplete, isTrue);
      expect(state.readyToCapture, isTrue);

      // Ready to shoot is still the same target, with no photograph and no
      // advancement.
      expect(state.activeTargetId, 1);
      expect(state.guidanceTarget?.targetId, 1);
      expect(state.targets[0].status, CaptureTargetStatus.active);
      expect(state.capturedCount, 0);
      expect(harness.gateway.stillRequests, 0);
    });

    test('a failed photograph keeps the target active and guided', () async {
      final harness = await calibratedHarness(
        stillError: const ArCoreFailure('Fake: no camera image.'),
        autoCapture: false,
        autoAdvance: true,
      );

      await aimAndCapture(harness, 1);

      final state = harness.controller.state;
      expect(harness.gateway.stillRequests, 1);
      expect(state.targets[0].status, CaptureTargetStatus.failed);
      expect(state.targets[0].imagePath, isNull);
      expect(state.capturedCount, 0);
      expect(state.activeTargetId, 1, reason: 'no advance on failure');
      expect(state.guidanceTarget?.targetId, 1);
      expect(state.instruction, isNot(contains('Select a target')));

      // The same target can be aimed at and shot again; success then advances.
      harness.gateway.stillError = null;
      await aimAndCapture(harness, 1);
      expect(harness.controller.state.capturedCount, 1);
      expect(harness.controller.state.activeTargetId, 2);
    });
  });

  group('a saved target leaves the guidance', () {
    test(
      'the completed target is replaced by the next incomplete one',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        await aimAndCapture(harness, 1);

        final state = harness.controller.state;
        expect(state.targets[0].status, CaptureTargetStatus.captured);
        expect(state.targets[0].imagePath, isNotNull);
        expect(state.capturedCount, 1);
        expect(state.activeTargetId, 2);
        expect(state.guidanceTarget?.targetId, 2);

        // Every target shares the station anchor, so identify the completed
        // target by its frozen local transform rather than by anchor id. Its
        // target draw is gone from the renderer's scene, while target 2 is
        // the one that remains placed and guided.
        final station = state.station!;
        final completedLocalPosition =
            state.targets[0].anchorWorldPosition! - station.referencePosition;
        expect(
          harness.gateway.lastScene.any((draw) {
            final matrix = draw.modelMatrix;
            return draw.meshId == PlaneAnchorService.targetMeshId &&
                draw.useModelTranslation &&
                matrix != null &&
                (matrix[12] - completedLocalPosition.x).abs() < 1e-5 &&
                (matrix[13] - completedLocalPosition.y).abs() < 1e-5 &&
                (matrix[14] - completedLocalPosition.z).abs() < 1e-5;
          }),
          isFalse,
          reason: 'a captured target must not keep its active marker',
        );
        expect(state.activeAnchored, isTrue);
        expect(state.activeAnchorTracking, isTrue);

        // The new target starts clean: the previous target's 0° alignment and
        // full stability window are not carried over.
        expect(state.alignment, isNull);
        expect(state.stable, isFalse);
        expect(state.stabilityProgress, 0);
        expect(state.targets[1].status, CaptureTargetStatus.active);

        // The completion is persisted, and the target cannot be selected again
        // as active guidance.
        expect(
          harness.repository.targets['s1']![0].status,
          CaptureTargetStatus.captured,
        );
        await harness.controller.selectTarget(1);
        await Harness.flush();
        expect(harness.controller.state.activeTargetId, 2);
        expect(harness.controller.state.guidanceTarget?.targetId, 2);
      },
    );

    test('the arrow is aimed at the new target', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await aimAndCapture(harness, 1);
      expect(harness.controller.state.activeTargetId, 2);

      await harness.emit();
      final alignment = harness.controller.state.alignment;
      expect(alignment, isNotNull);
      expect(alignment!.usable, isTrue);

      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final expected = frame
          .localToWorld(buildCaptureSphere()[1].direction)
          .normalized;
      expect(alignment.directionWorld.x, closeTo(expected.x, 1e-9));
      expect(alignment.directionWorld.y, closeTo(expected.y, 1e-9));
      expect(alignment.directionWorld.z, closeTo(expected.z, 1e-9));
      // And it is not the direction of the target just saved.
      final previous = frame
          .localToWorld(buildCaptureSphere()[0].direction)
          .normalized;
      expect(alignment.directionWorld.x, isNot(closeTo(previous.x, 1e-6)));
    });

    test('next selection skips completed targets in list order', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );

      await aimAndCapture(harness, 2);
      expect(harness.controller.state.activeTargetId, 3);

      // The user goes back to the target that was skipped. Once it is saved,
      // the next target is the first incomplete one in list order — target 2
      // is done now and must not become active again.
      await aimAndCapture(harness, 1);
      final state = harness.controller.state;
      expect(state.capturedCount, 2);
      expect(state.targets[1].status, CaptureTargetStatus.captured);
      expect(state.activeTargetId, 3);
      expect(state.guidanceTarget?.targetId, 3);
    });

    test(
      'retake makes a target selectable while retaining its accepted image',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        await aimAndCapture(harness, 1);
        await aimAndCapture(harness, 2);
        expect(harness.controller.state.capturedCount, 2);
        final originalPath = harness.controller.state.targets[0].imagePath;

        await harness.controller.retake(1);
        await Harness.flush();

        final state = harness.controller.state;
        expect(state.targets[0].status, CaptureTargetStatus.active);
        expect(state.targets[0].imagePath, originalPath);
        expect(state.activeTargetId, 1);
        expect(state.guidanceTarget?.targetId, 1);
        expect(state.capturedCount, 1);
        expect(
          harness.repository.targets['s1']![0].status,
          CaptureTargetStatus.captured,
        );
        expect(harness.repository.targets['s1']![0].imagePath, originalPath);
        expect(harness.repository.stations['s1']!.capturedCount, 2);

        await aimAndCapture(harness, 1);
        expect(harness.controller.state.capturedCount, 2);
        expect(harness.controller.state.activeTargetId, 3);
      },
    );
  });

  group('duplicate and interrupted events', () {
    test(
      'a second shutter press inside the save does not skip a target',
      () async {
        final harness = await calibratedHarness(
          autoCapture: false,
          autoAdvance: true,
        );
        await harness.controller.selectTarget(1);
        await harness.emitStable(count: 20);
        await Harness.waitFor(() => harness.controller.state.stable);

        // Both presses land inside the same transaction. The latch must let
        // exactly one reach the camera and advance by exactly one target.
        await Future.wait<void>([
          harness.controller.captureNow(),
          harness.controller.captureNow(),
        ]);
        await Harness.waitForCaptureSettlement(harness.controller);

        final state = harness.controller.state;
        expect(harness.gateway.stillRequests, 1);
        expect(state.capturedCount, 1);
        expect(state.targets[0].status, CaptureTargetStatus.captured);
        expect(state.activeTargetId, 2);
        expect(state.targets[1].status, CaptureTargetStatus.active);
      },
    );

    test('losing tracking neither completes nor advances', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await harness.controller.selectTarget(1);
      await harness.emitStable(count: 20);
      await Harness.waitFor(() => harness.controller.state.stable);
      expect(harness.controller.state.readyToCapture, isTrue);

      await harness.emit(tracking: TrackingState.paused);

      final state = harness.controller.state;
      expect(state.targets[0].status, CaptureTargetStatus.active);
      expect(state.activeTargetId, 1);
      expect(state.guidanceTarget?.targetId, 1);
      expect(state.capturedCount, 0);
      expect(state.alignment, isNull);
      expect(harness.gateway.stillRequests, 0);
      await harness.controller.captureNow();
      await Harness.flush();
      expect(harness.gateway.stillRequests, 0);
      expect(harness.controller.state.activeTargetId, 1);
    });

    test('a selection made during the save survives the completion', () async {
      final harness = await calibratedHarness(
        autoCapture: false,
        autoAdvance: true,
      );
      await harness.controller.selectTarget(1);
      await harness.emitStable(count: 20);
      await Harness.waitFor(() => harness.controller.state.stable);

      // Hold the photograph in flight. This is the window in which the user
      // can still operate the target sheet.
      final gate = Completer<void>();
      harness.gateway.stillGate = gate.future;
      final capture = harness.controller.captureNow();
      await Harness.flush();
      expect(harness.gateway.stillRequests, 1);

      await harness.controller.selectTarget(10);
      await Harness.flush();
      expect(harness.controller.state.activeTargetId, 10);

      gate.complete();
      await capture;
      await Harness.flush();

      final state = harness.controller.state;
      // The photograph is recorded against the target that was shot…
      expect(state.targets[0].status, CaptureTargetStatus.captured);
      expect(state.capturedCount, 1);
      // …and the newer selection keeps the guidance; it is not replaced by
      // the completed target or by an automatic advance.
      expect(state.activeTargetId, 10);
      expect(state.guidanceTarget?.targetId, 10);
      expect(state.targets[9].status, CaptureTargetStatus.active);
      expect(state.alignment, isNull);
    });
  });
}
