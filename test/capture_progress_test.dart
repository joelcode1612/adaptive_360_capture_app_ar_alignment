import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';

import 'capture_flow_test.dart' show bootHarness, calibratedHarness, Harness;

/// Progress, guidance-start and the status→colour contract of the sphere map.
///
/// The regression this suite pins came from a screenshot: the mini-map showed
/// all 26 dots amber while the header read "0 of 26 captured". Placing a node
/// had stamped `CaptureTargetStatus.active` onto every target, and the map
/// paints `active` amber — so a fully-placed, fully-uncaptured arrangement
/// looked like a finished one. The count itself was always taken from saved
/// photographs; what was wrong was the *status*, and therefore the map.
void main() {
  group('the first target after calibration', () {
    test('is the front horizontal target, at yaw 0 / pitch 0', () async {
      final harness = await calibratedHarness(autoCapture: false);
      final state = harness.controller.state;
      expect(state.activeTargetId, 1);

      final front = state.targets[0];
      expect(front.yawDegrees, 0);
      expect(front.pitchDegrees, 0);
      expect(front.stationLocalDirection.y, closeTo(0, 1e-12));
      expect(front.stationLocalDirection.z, closeTo(1, 1e-12));
      // The world position is the forward horizontal direction at the target
      // radius (1.0 m): straight ahead of the station, not above it.
      expect(front.anchorWorldPosition!.y, closeTo(0, 1e-9));
      expect(
        front.anchorWorldPosition!.z,
        closeTo(const CaptureGeometryConfig().targetRadiusM, 1e-9),
      );
    });

    test('the instruction names it', () async {
      final harness = await calibratedHarness(autoCapture: false);
      // Not aimed yet: the guidance says where to go, not just which way to
      // turn.
      await harness.emit(orientation: Quat.axisAngle(180, Vec3.unitY));
      expect(harness.controller.state.instruction, contains('front target'));
    });
  });

  group('the sphere map\u2019s status contract', () {
    test('exactly one target is active, the rest are pending', () async {
      final harness = await calibratedHarness(autoCapture: false);
      final statuses = harness.controller.state.targets
          .map((CaptureTarget t) => t.status)
          .toList();

      expect(
        statuses
            .where((CaptureTargetStatus s) => s == CaptureTargetStatus.active)
            .length,
        1,
        reason: 'placing all 26 nodes must not mark all 26 targets active',
      );
      expect(
        statuses
            .where((CaptureTargetStatus s) => s == CaptureTargetStatus.captured)
            .length,
        0,
        reason: 'nothing is captured before the first photograph',
      );
      // Every dot the map paints grey except the one amber one.
      expect(statuses[0], CaptureTargetStatus.active);
      for (var i = 1; i < statuses.length; i++) {
        expect(statuses[i], CaptureTargetStatus.pending);
      }
    });

    test('moving to another target moves the single active flag', () async {
      final harness = await calibratedHarness(autoCapture: false);
      await harness.controller.selectTarget(17);
      await Harness.flush();

      final statuses = harness.controller.state.targets
          .map((CaptureTarget t) => t.status)
          .toList();
      expect(statuses[16], CaptureTargetStatus.active);
      expect(
        statuses
            .where((CaptureTargetStatus s) => s == CaptureTargetStatus.active)
            .length,
        1,
      );
    });
  });

  group('the scene draws the configured physical size', () {
    test(
      'plate and bezel use the normalised scale, not the authored 1.0',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        final scene = harness.gateway.lastScene;
        final scale = const CaptureGeometryConfig().markerScaleFor(0.54);
        // 0.22 m / 0.54 m authored — the oversized-disk regression.
        expect(scale, closeTo(0.4074, 1e-3));
        expect(scene.length, kCaptureTargetCount * 2);
        for (final draw in scene) {
          expect(draw.scale, closeTo(scale, 1e-9));
        }
        // What that scale means in metres: the plate is 0.22 m across at the
        // configured radius, i.e. ~8.4 degrees of the view.
        expect(0.54 * scene[1].scale, closeTo(0.22, 1e-6));
      },
    );
  });

  group('progress comes from saved frames only', () {
    test('starts at 0 / 26 and rises once per accepted photograph', () async {
      final harness = await calibratedHarness(autoCapture: false);
      expect(harness.controller.state.capturedCount, 0);

      await capture(harness, 1);
      expect(harness.controller.state.capturedCount, 1);

      await capture(harness, 2);
      expect(harness.controller.state.capturedCount, 2);

      final records = await harness.repository.loadRecords('s1');
      expect(records.length, 2);
      expect(harness.controller.state.station!.capturedCount, 2);
    });

    test(
      'a repeated capture of the same target does not count twice',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        await capture(harness, 1);
        await capture(harness, 1);
        await Harness.flush();

        expect(harness.controller.state.capturedCount, 1);
        expect(harness.controller.state.targets[0].imagePath, isNotNull);
        final records = await harness.repository.loadRecords('s1');
        expect(records.length, 1, reason: 'one record per frame index');
      },
    );

    test(
      'a failed capture leaves the count and the target untouched',
      () async {
        final harness = await calibratedHarness(
          stillError: const ArCoreFailure('Fake: no camera image.'),
          autoCapture: false,
        );
        await capture(harness, 1);
        await Harness.flush();

        final state = harness.controller.state;
        expect(state.capturedCount, 0);
        expect(state.activeTargetId, 1, reason: 'no advance on failure');
        expect(state.targets[0].status, CaptureTargetStatus.failed);
        expect(await harness.repository.loadRecords('s1'), isEmpty);
      },
    );

    test(
      '26 / 26 only when all 26 frames are accepted',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        for (var id = 1; id <= kCaptureTargetCount; id++) {
          expect(harness.controller.state.capturedCount, id - 1);
          await capture(harness, id);
        }
        final state = harness.controller.state;
        expect(state.capturedCount, kCaptureTargetCount);
        expect(state.phase, CapturePhase.complete);
        expect(
          (await harness.repository.loadRecords('s1')).length,
          kCaptureTargetCount,
        );
      },
      // 26 frames each write and read back a real JPEG. Alone that is a few
      // seconds; the default 30 s budget expires when the whole suite runs the
      // files in parallel on a loaded machine.
      timeout: const Timeout(Duration(minutes: 3)),
    );
  });

  group('recovering from a walk-away', () {
    test(
      'recalibrate here anchors a NEW station at the current pose',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        final firstStation = harness.controller.state.station!.stationId;

        // The user follows the old "Move to target" wording and walks to the
        // target — 2 m from the station, exactly the screenshot.
        await harness.emit(position: const Vec3(0, 0, 2));
        expect(harness.controller.state.stationDrifted, isTrue);
        expect(harness.controller.state.readyToCapture, isFalse);
        expect(
          harness.controller.state.captureBlockReason,
          contains('Recalibrate here'),
        );

        await harness.controller.recalibrateHere();
        await Harness.flush();

        final state = harness.controller.state;
        // A new station record, at the new spot: drift is measured from the
        // phone, so it is zero again and the shutter is live.
        expect(state.station, isNotNull);
        expect(state.station!.stationId, isNot(firstStation));
        expect(state.driftM, lessThan(0.01));
        expect(state.stationDrifted, isFalse);
        expect(state.capturedCount, 0);
        expect(state.activeTargetId, 1);
        // The whole arrangement exists again around the new station.
        expect(state.targets.length, kCaptureTargetCount);
        expect(
          state.targets.every((CaptureTarget t) => t.anchorIdentifier != null),
          isTrue,
        );
        expect(harness.gateway.anchors.length, kCaptureTargetCount);
        // The old station's records are untouched: a new station is a new
        // record, not a migration.
        expect(harness.controller.savedStationIds, contains(firstStation));
      },
    );

    test('the guidance never tells the user to walk to a target', () async {
      final harness = await calibratedHarness(autoCapture: false);
      await harness.emit(orientation: Quat.axisAngle(180, Vec3.unitY));
      final instruction = harness.controller.state.instruction;
      expect(instruction, contains('Aim at'));
      expect(instruction, isNot(contains('Move to')));
    });
  });

  group('resuming a stored station', () {
    test(
      'restores progress from the saved records, not from the node count',
      () async {
        // Same AR session: `resumeStation` is only allowed while the world
        // frame that recorded the station is still the live one.
        final harness = await calibratedHarness(
          autoCapture: false,
          reusedSession: true,
        );
        await capture(harness, 1);
        await capture(harness, 2);
        await capture(harness, 3);
        expect(harness.controller.state.capturedCount, 3);
        expect(harness.controller.resumableStationId, 's1');

        // Re-enter the same station: the state is rebuilt from what is on
        // disk — the records, not the node count.
        final ok = await harness.controller.resumeStation('s1');
        expect(ok, isTrue);

        final state = harness.controller.state;
        expect(state.station!.stationId, 's1');
        expect(
          state.capturedCount,
          3,
          reason: 'progress is the number of saved frames',
        );
        expect(state.phase, CapturePhase.capturing);
        // One node per target, and no duplicates: re-placing an arrangement
        // that is already placed creates nothing new.
        expect(harness.gateway.anchorCreations, kCaptureTargetCount);
        expect(harness.gateway.anchors.length, kCaptureTargetCount);
        // …and it starts on the first target that still needs a photograph.
        expect(state.activeTargetId, 4);
        expect(state.guidanceTarget?.targetId, 4);
        expect(state.targets[0].status, CaptureTargetStatus.captured);
        expect(state.targets[3].status, CaptureTargetStatus.active);
        expect(state.targets[3].anchorIdentifier, isNotNull);
        // The station record agrees with the records on disk.
        expect(state.station!.capturedCount, 3);
        expect((await harness.repository.loadRecords('s1')).length, 3);
      },
    );

    test(
      'a new station does not inherit another station\u2019s progress',
      () async {
        final first = await calibratedHarness(autoCapture: false);
        await capture(first, 1);
        await capture(first, 2);
        expect(first.controller.state.capturedCount, 2);

        // A different station, calibrated fresh in the same store.
        final second = await bootHarness(autoCapture: false);
        await second.emit();
        await second.controller.calibrateStation(stationId: 's2');
        await Harness.flush();

        final state = second.controller.state;
        expect(state.station!.stationId, 's2');
        expect(state.capturedCount, 0);
        expect(
          state.targets.every(
            (CaptureTarget t) => t.status != CaptureTargetStatus.captured,
          ),
          isTrue,
        );
        expect(await second.repository.loadRecords('s2'), isEmpty);
        // The first station's records are untouched.
        expect((await first.repository.loadRecords('s1')).length, 2);
      },
    );

    test('an unknown or malformed station refuses to resume', () async {
      final harness = await bootHarness(autoCapture: false);
      expect(await harness.controller.resumeStation('nope'), isFalse);
      expect(harness.controller.state.station, isNull);
    });

    test('a new AR session refuses stale coordinates and says why', () async {
      // The default harness is a *new* ARCore session, so its world origin is
      // not the one the station was recorded in — even though the station was
      // calibrated by this very controller.
      final harness = await calibratedHarness(autoCapture: false);
      await capture(harness, 1);
      final creations = harness.gateway.anchorCreations;

      final ok = await harness.controller.resumeStation('s1');

      expect(ok, isFalse);
      expect(
        harness.controller.state.message,
        contains('Recalibrate'),
        reason: 'a refusal without the recovery sentence is a dead end',
      );
      // Refusing must not touch anything: the old station and its nodes stay.
      expect(harness.controller.state.station!.stationId, 's1');
      expect(harness.gateway.anchorCreations, creations);
      expect((await harness.repository.loadRecords('s1')).length, 1);
    });

    test('recalibrate and continue keeps every saved frame', () async {
      final harness = await calibratedHarness(autoCapture: false);
      await capture(harness, 1);
      await capture(harness, 2);
      await capture(harness, 3);
      final before = await harness.repository.loadRecords('s1');
      expect(before.length, 3);
      final createdAt = harness.controller.state.station!.createdAt;

      // A new station pose, as a restart would require: standing somewhere
      // else in the room, with a fresh world frame.
      await harness.emit(position: const Vec3(2, 1.5, 3));
      final ok = await harness.controller.recalibrateAndContinue('s1');
      await Harness.flush();

      expect(ok, isTrue);
      final state = harness.controller.state;
      expect(state.station!.stationId, 's1');
      expect(
        state.station!.createdAt,
        createdAt,
        reason: 'continuing a station must not rewrite its identity',
      );
      expect(state.station!.capturedCount, 3);
      expect(state.capturedCount, 3);
      expect(state.phase, CapturePhase.capturing);
      expect(state.activeTargetId, 4);
      expect(state.guidanceTarget?.targetId, 4);
      expect(state.targets[0].status, CaptureTargetStatus.captured);
      expect(state.targets[1].status, CaptureTargetStatus.captured);
      expect(state.targets[2].status, CaptureTargetStatus.captured);
      expect(state.targets[0].imagePath, before[0].imagePath);
      expect(state.targets[2].imagePath, before[2].imagePath);
      expect(state.targets[3].status, CaptureTargetStatus.active);
      expect(state.targets[3].anchorWorldPosition, isNotNull);

      // The arrangement was rebuilt around the new pose: one node per target,
      // all of them new — the old world coordinates were not reused.
      expect(harness.gateway.anchors.length, kCaptureTargetCount);
      expect(state.targets[3].anchorWorldPosition!.x, isNot(closeTo(0, 1e-6)));
      // The saved evidence is untouched.
      expect((await harness.repository.loadRecords('s1')).length, 3);
    });
  });
}

/// Aims at [targetId], fires the shutter and waits for the write to land.
Future<void> capture(Harness harness, int targetId) async {
  await harness.controller.selectTarget(targetId);
  await Harness.flush();
  await harness.emitStable();
  await Harness.waitFor(() => harness.controller.state.stable);
  await harness.controller.captureNow();
  await Harness.waitFor(
    () =>
        harness.controller.state.targets[targetId - 1].status !=
        CaptureTargetStatus.active,
  );
}
