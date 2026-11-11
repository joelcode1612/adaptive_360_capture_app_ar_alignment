import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/plane_anchor_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/shot_phase.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

import 'capture_flow_test.dart' show bootHarness, calibratedHarness, Harness;
import 'capture_targets_test.dart' show orientationFor;

/// World-space target placement: the brief's §8, §9 and §11 requirements as
/// tests.
///
/// The bug this suite guards against is the original one — a target that
/// followed the camera or was placed wherever a raycast happened to hit. These
/// tests assert the opposite on every axis that matters: the position is
/// `station + radius × direction`, it survives the user walking away, and no
/// amount of missing surfaces stops it.
void main() {
  /// A state that satisfies every gate except the one under test. Shared by
  /// the "no surfaces" group and the station-drift group, because both are
  /// about a *single* condition standing between a ready state and the
  /// shutter — the assertion that matters is that the reason names it and
  /// nothing else changes.
  CaptureState readyState({int planesTracked = 0, double driftM = 0}) =>
      CaptureState(
        geometry: const CaptureGeometryConfig(),
        phase: CapturePhase.capturing,
        textureId: 1,
        tracking: TrackingState.tracking,
        measurementFresh: true,
        driftM: driftM,
        station: CaptureStation(
          stationId: 's1',
          referencePosition: Vec3.zero,
          referenceOrientation: Quat.identity,
          createdAt: DateTime.utc(2026, 1, 1),
          trackingStatus: TrackingState.tracking,
        ),
        targets: const <CaptureTarget>[
          CaptureTarget(
            targetId: 1,
            stationId: 's1',
            yawDegrees: 0,
            pitchDegrees: 45,
            stationLocalDirection: Vec3.unitZ,
            anchorIdentifier: 'a1',
            anchorWorldPosition: Vec3.zero,
            anchorSurfaceType: 'WORLD',
          ),
        ],
        activeTargetId: 1,
        activeAnchored: true,
        activeAnchorTracking: true,
        activeSurfaceType: 'WORLD',
        alignment: const AlignmentResult(
          directionWorld: Vec3.unitZ,
          directionInCamera: Vec3.unitZ,
          distanceM: 1.5,
          yawCorrectionDeg: 0,
          pitchCorrectionDeg: 0,
          angularErrorDeg: 0,
          toleranceDeg: 3,
        ),
        alignedHeldMs: const CaptureGeometryConfig().stabilityWindow.inMilliseconds
            .toDouble(),
        stable: true,
        planesTracked: planesTracked,
        planeCount: planesTracked,
      );

  group('the placed position', () {
    test(
      'is station + radius × direction for a station at the origin',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        final target = harness.controller.state.targets.first;
        final station = harness.controller.state.station!;
        final radius = const CaptureGeometryConfig().targetRadiusM;
        final spec = buildCaptureSphere().first;

        // The persisted station carries the calibrated pose; the frame is the
        // same pose in the placement maths' own type.
        final frame = StationFrame(
          origin: station.referencePosition,
          orientation: station.referenceOrientation,
        );
        final expected =
            frame.origin +
            frame.localToWorld(spec.direction).normalized * radius;
        expect(target.anchorWorldPosition, isNotNull);
        expect(target.anchorWorldPosition!.x, closeTo(expected.x, 1e-9));
        expect(target.anchorWorldPosition!.y, closeTo(expected.y, 1e-9));
        expect(target.anchorWorldPosition!.z, closeTo(expected.z, 1e-9));
      },
    );

    test('uses the station\u2019s own axes, not the world axes', () async {
      // A station calibrated facing world +X, standing a metre up. If placement
      // used the world axes (or the camera pose) instead of the station frame,
      // target 1 would land straight ahead along world +Z.
      final harness = await bootHarness(autoCapture: false);
      await harness.emit(
        position: const Vec3(0, 1, 0),
        orientation: orientationFor(const Vec3(1, 0, 0)),
      );
      await harness.controller.calibrateStation(stationId: 's-rot');
      await Harness.flush();

      final target = harness.controller.state.targets.first;
      final station = harness.controller.state.station!;
      final spec = buildCaptureSphere().first;
      final radius = const CaptureGeometryConfig().targetRadiusM;
      final frame = StationFrame(
        origin: station.referencePosition,
        orientation: station.referenceOrientation,
      );
      final expected =
          frame.origin + frame.localToWorld(spec.direction).normalized * radius;

      expect(target.anchorWorldPosition!.x, closeTo(expected.x, 1e-9));
      expect(target.anchorWorldPosition!.y, closeTo(expected.y, 1e-9));
      expect(target.anchorWorldPosition!.z, closeTo(expected.z, 1e-9));
      // The world-axes answer would put it a metre off; anything but the
      // station-frame answer is visible here.
      expect(target.anchorWorldPosition!.x, greaterThan(0.5));
    });
  });

  group('the target stays put', () {
    test('walking away and turning does not move or re-create it', () async {
      final harness = await calibratedHarness(autoCapture: false);
      await harness.controller.selectTarget(1);
      await Harness.flush();

      final placed =
          harness.controller.state.targets.first.anchorWorldPosition!;
      final anchorId = harness.controller.state.targets.first.anchorIdentifier;
      final creations = harness.gateway.anchorCreations;
      final refreshes = harness.gateway.refreshCalls;

      // Walk three metres to the side, turn around, and then look back.
      await harness.emit(
        position: const Vec3(3, 1.5, -2),
        orientation: orientationFor(const Vec3(0, 0, -1)),
      );
      await Harness.flush();

      final target = harness.controller.state.targets.first;
      expect(
        target.anchorWorldPosition,
        placed,
        reason:
            'the anchor pose is ARCore\u2019s, never re-derived from the camera',
      );
      expect(target.anchorIdentifier, anchorId);
      expect(
        harness.gateway.anchorCreations,
        creations,
        reason: 'placement is not recomputed per frame',
      );
      // Refreshes are allowed — ARCore refines an anchor in place — but they
      // may not invent a new one.
      expect(harness.gateway.refreshCalls, greaterThanOrEqualTo(refreshes));
      expect(harness.gateway.anchorCreations, creations);
    });
  });

  group('readiness does not depend on surfaces', () {
    test('zero planes, everything else ready → the shutter is live', () {
      final state = readyState();
      expect(state.planesTracked, 0);
      expect(state.readyToCapture, isTrue);
      expect(state.captureBlockReason, isNull);
    });

    test('the block reason never names a plane or a surface', () {
      for (final state in <CaptureState>[
        readyState(),
        readyState().copyWith(activeAnchored: false),
        readyState().copyWith(tracking: TrackingState.paused),
        readyState().copyWith(clearStation: true),
      ]) {
        final reason = state.captureBlockReason;
        if (reason == null) continue;
        expect(reason.toLowerCase(), isNot(contains('plane')));
        expect(reason.toLowerCase(), isNot(contains('surface')));
        expect(reason.toLowerCase(), isNot(contains('wall')));
      }
    });
  });

  // ── the invisible-marker regression ────────────────────────────────────
  //
  // The bug: the arrow aimed at the stored anchor position (always correct)
  // while the native renderer refused to draw the marker because ARCore had
  // not tracked the anchor yet — and Dart never read the tracking state it was
  // already being sent. A placed-but-untracked anchor must read as "waiting",
  // never as drawn.
  group('anchor tracking decides whether the marker is drawn', () {
    /// One pose with the injected clock a refresh-interval further on, so
    /// `_refreshAnchorIfDue` fires exactly once per call.
    Future<void> refresh(Harness harness) async {
      harness.now = harness.now.add(const Duration(milliseconds: 600));
      await harness.emit();
      await Harness.flush();
    }

    test(
      'a placed-but-paused anchor is a wait, not an anchored target',
      () async {
        // ARCore commonly starts a world anchor paused, so the *creation*
        // reports paused — which is why the flag is armed before calibration,
        // not after it.
        final harness = await bootHarness(autoCapture: false);
        harness.gateway.anchorTracking = TrackingState.paused;
        await harness.emit();
        await harness.controller.calibrateStation(stationId: 's1');
        await Harness.flush();

        // Placed: the position exists and the arrow can use it.
        expect(harness.controller.state.activeAnchored, isTrue);
        // Not tracked: the UI says "waiting" instead of claiming the marker is
        // authoritative, and the renderer keeps the frozen pose only as a
        // ghost — visible, so it does not blink out on rotation, but dimmed so
        // it cannot be mistaken for a marker ARCore stands behind.
        expect(harness.controller.state.activeAnchorTracking, isFalse);
        final anchorId =
            harness.controller.state.activeTarget!.anchorIdentifier!;
        final draws = harness.gateway.lastScene
            .where((draw) => draw.anchorId == anchorId)
            .toList();
        expect(
          draws,
          isNotEmpty,
          reason: 'a paused marker is a ghost, not gone',
        );
        expect(
          draws.every(
            (draw) => draw.opacity == PlaneAnchorService.kInactiveTargetOpacity,
          ),
          isTrue,
        );
        expect(
          harness.controller.state.message,
          'Waiting for AR to stabilise the target…',
        );
        expect(
          harness.controller.state.instruction,
          'Waiting for AR to stabilise the target…',
        );
        expect(harness.controller.state.readyToCapture, isFalse);
        expect(
          harness.controller.state.captureBlockReason,
          contains('stabilise'),
        );
      },
    );

    test(
      'tracking arriving later clears the wait and arms the shutter',
      () async {
        final harness = await bootHarness(autoCapture: false);
        harness.gateway.anchorTracking = TrackingState.paused;
        await harness.emit();
        await harness.controller.calibrateStation(stationId: 's1');
        await Harness.flush();
        expect(harness.controller.state.activeAnchorTracking, isFalse);

        // ARCore catches up.
        harness.gateway.anchorTracking = TrackingState.tracking;
        await refresh(harness);

        expect(harness.controller.state.activeAnchorTracking, isTrue);
        expect(harness.controller.state.message, isNull);
        // Aimed and steady — the remaining prereqs the shutter also needs.
        await harness.emitStable();
        expect(harness.controller.state.readyToCapture, isTrue);
      },
    );

    test('a per-pose pause blocks the shutter on that very pose', () async {
      final harness = await calibratedHarness(autoCapture: false);
      await harness.controller.selectTarget(1);
      await Harness.flush();
      await harness.emitStable();
      expect(harness.controller.state.readyToCapture, isTrue);
      final placed =
          harness.controller.state.activeTarget!.anchorWorldPosition!;
      double activeOpacity() {
        final id = harness.controller.state.activeTarget!.anchorIdentifier!;
        var max = 0.0;
        for (final draw in harness.gateway.lastScene) {
          if (draw.anchorId == id && draw.opacity > max) max = draw.opacity;
        }
        return max;
      }

      // ARCore pauses the anchor between two 2 Hz refreshes. The gate must
      // follow the pose event, not the refresh: otherwise the shutter stays
      // armed for up to half a second on a pose ARCore does not trust.
      await harness.emit(
        activeAnchorPose: (
          position: placed,
          orientation: Quat.identity,
          tracking: TrackingState.paused,
        ),
      );
      expect(harness.controller.state.activeAnchored, isTrue);
      expect(harness.controller.state.activeAnchorTracking, isFalse);
      expect(harness.controller.state.readyToCapture, isFalse);
      // The same pose ghosts the marker: the ring is no longer complete, so
      // the target comes back out of hiding — visible, but not authoritative.
      expect(activeOpacity(), PlaneAnchorService.kInactiveTargetOpacity);
      // The instruction and the blocker update on this pose; the banner
      // sentence follows on the next 2 Hz refresh, which owns user messaging.
      expect(
        harness.controller.state.instruction,
        'Waiting for AR to stabilise the target…',
      );
      expect(
        harness.controller.state.captureBlockReason,
        contains('stabilise'),
      );

      // And the very next pose that reports tracking re-arms it.
      await harness.emit(
        activeAnchorPose: (
          position: placed,
          orientation: Quat.identity,
          tracking: TrackingState.tracking,
        ),
      );
      expect(harness.controller.state.activeAnchorTracking, isTrue);
      expect(harness.controller.state.readyToCapture, isTrue);
    });

    test(
      'a camera pause ghosts the target even while its anchor still tracks',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        await harness.controller.selectTarget(1);
        await Harness.flush();
        await harness.emitStable();
        expect(harness.controller.state.readyToCapture, isTrue);

        final placed =
            harness.controller.state.activeTarget!.anchorWorldPosition!;
        final anchorId =
            harness.controller.state.activeTarget!.anchorIdentifier!;
        final station = harness.controller.state.station!;
        final positions = <int, Vec3?>{
          for (final target in harness.controller.state.targets)
            target.targetId: target.anchorWorldPosition,
        };
        final creations = harness.gateway.anchorCreations;
        double activeOpacity() {
          var max = 0.0;
          for (final draw in harness.gateway.lastScene) {
            if (draw.anchorId == anchorId && draw.opacity > max) {
              max = draw.opacity;
            }
          }
          return max;
        }

        // The field recording's frame: ARCore loses the camera pose on a
        // featureless wall while still reporting the world anchor as tracked.
        // The anchor is fine, but the view is a last-known estimate — the
        // marker may not keep claiming to be authoritative through it.
        await harness.emit(
          tracking: TrackingState.paused,
          activeAnchorPose: (
            position: placed,
            orientation: Quat.identity,
            tracking: TrackingState.tracking,
          ),
        );

        final paused = harness.controller.state;
        expect(
          paused.activeAnchorTracking,
          isTrue,
          reason: 'only the camera paused; ARCore still tracks the anchor',
        );
        expect(paused.readyToCapture, isFalse);
        expect(paused.captureBlockReason, contains('AR tracking'));
        expect(
          activeOpacity(),
          PlaneAnchorService.kInactiveTargetOpacity,
          reason: 'a paused view cannot present the marker as authoritative',
        );
        expect(
          harness.gateway.anchorCreations,
          creations,
          reason: 'a camera pause must not churn anchors',
        );
        expect(paused.activeTarget!.anchorWorldPosition, placed);
        expect(paused.activeTarget!.anchorIdentifier, anchorId);
        // A camera pause is not a recalibration: the station and the other 25
        // placed directions are frozen exactly where calibration put them.
        expect(paused.station!.referencePosition, station.referencePosition);
        expect(
          paused.station!.referenceOrientation,
          station.referenceOrientation,
        );
        expect(paused.targets.length, positions.length);
        for (final target in paused.targets) {
          expect(target.anchorWorldPosition, positions[target.targetId]);
        }

        // The shutter refuses while the view is paused.
        await harness.controller.captureNow();
        await Harness.flush();
        expect(harness.controller.state.capturedCount, 0);
        expect(harness.controller.state.lastSavedPath, isNull);

        // Tracking returns: authority returns on that pose, readiness only
        // once the aim is held stable again.
        await harness.emit();
        expect(harness.controller.state.activeAnchorTracking, isTrue);
        expect(
          activeOpacity(),
          greaterThan(PlaneAnchorService.kInactiveTargetOpacity),
        );
        await harness.emitStable();
        expect(harness.controller.state.readyToCapture, isTrue);
      },
    );

    test(
      'a paused anchor aims from its placement, not its stale pose',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        await harness.controller.selectTarget(1);
        await Harness.flush();
        final placed =
            harness.controller.state.activeTarget!.anchorWorldPosition!;
        final moved = placed + const Vec3(0.75, 0, 0);

        // Tracking: the live anchor pose is the position the renderer draws, so
        // it is the one the guidance aims at.
        await harness.emit(
          activeAnchorPose: (
            position: moved,
            orientation: Quat.identity,
            tracking: TrackingState.tracking,
          ),
        );
        final tracked = harness.controller.state.alignment!;
        final expectedTracked = moved.normalized;
        expect(tracked.directionWorld.x, closeTo(expectedTracked.x, 1e-9));
        expect(tracked.directionWorld.y, closeTo(expectedTracked.y, 1e-9));
        expect(tracked.directionWorld.z, closeTo(expectedTracked.z, 1e-9));

        // Paused: the anchor's last pose is stale, so the frozen placement is
        // the honest target — and readiness is blocked either way.
        await harness.emit(
          activeAnchorPose: (
            position: moved,
            orientation: Quat.identity,
            tracking: TrackingState.paused,
          ),
        );
        final paused = harness.controller.state.alignment!;
        final expectedPlaced = placed.normalized;
        expect(paused.directionWorld.x, closeTo(expectedPlaced.x, 1e-9));
        expect(paused.directionWorld.y, closeTo(expectedPlaced.y, 1e-9));
        expect(paused.directionWorld.z, closeTo(expectedPlaced.z, 1e-9));
        expect(harness.controller.state.readyToCapture, isFalse);
      },
    );

    test('a stale prior-anchor event cannot steer the selected target',
        () async {
      final harness = await calibratedHarness(autoCapture: false);
      await harness.controller.selectTarget(1);
      await Harness.flush();

      final active = harness.controller.state.activeTarget!;
      final placed = active.anchorWorldPosition!;
      final priorAnchorId =
          harness.controller.state.targets[1].anchorIdentifier!;
      final stalePosition = placed + const Vec3(0.75, 0, 0);

      await harness.emit(
        activeAnchorPose: (
          position: stalePosition,
          orientation: Quat.identity,
          tracking: TrackingState.tracking,
        ),
        reportedActiveAnchorId: priorAnchorId,
      );

      final alignment = harness.controller.state.alignment!;
      final expected = placed.normalized;
      expect(alignment.directionWorld.x, closeTo(expected.x, 1e-9));
      expect(alignment.directionWorld.y, closeTo(expected.y, 1e-9));
      expect(alignment.directionWorld.z, closeTo(expected.z, 1e-9));
      expect(harness.controller.state.activeAnchorTracking, isFalse);
    });

    test('a dropped shared station anchor requires explicit recalibration',
        () async {
      final harness = await calibratedHarness(autoCapture: false);
      await harness.controller.selectTarget(1);
      await Harness.flush();
      expect(harness.controller.state.activeAnchored, isTrue);
      final before = harness.gateway.anchorCreations;

      // The one active handle is the shared station anchor. Replacing it only
      // for target 1 would put the 26 frozen local transforms under different
      // references, so recovery must invalidate rather than silently rebuild.
      final activeId = harness.controller.state.activeTarget!.anchorIdentifier!;
      harness.gateway.dropAnchor(activeId);
      await refresh(harness);
      await Harness.flush();

      expect(
        harness.gateway.anchorCreations,
        before,
        reason: 'ordinary tracking recovery must not create a replacement',
      );
      expect(harness.controller.state.station, isNull);
      expect(harness.controller.state.activeTargetId, 0);
      expect(harness.controller.state.message, contains('Recalibrate'));
    });

    test('a persistently paused anchor is waited on, not re-placed', () async {
      final harness = await calibratedHarness(autoCapture: false);
      harness.gateway.anchorTracking = TrackingState.paused;
      await harness.controller.selectTarget(1);
      await Harness.flush();
      final placed = harness.gateway.anchorCreations;

      // Many refresh periods with ARCore still reporting the anchor paused:
      // the policy is to wait, not to churn anchors or declare failure. A
      // world anchor that ARCore paused has not lost its position.
      for (var i = 0; i < 24; i++) {
        await refresh(harness);
      }

      expect(
        harness.gateway.anchorCreations,
        placed,
        reason: 'a paused anchor is still a placed anchor',
      );
      expect(harness.controller.state.activeAnchored, isTrue);
      expect(harness.controller.state.activeAnchorTracking, isFalse);
      expect(harness.controller.state.readyToCapture, isFalse);
      expect(
        harness.controller.state.message,
        'Waiting for AR to stabilise the target…',
      );

      // The wait ends when ARCore resumes tracking — no re-placement needed.
      harness.gateway.anchorTracking = TrackingState.tracking;
      await refresh(harness);
      expect(harness.controller.state.activeAnchorTracking, isTrue);
      expect(harness.controller.state.message, isNull);
    });

    test(
      'a paused refresh cannot unfreeze the placement or arm the shutter',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        await harness.controller.selectTarget(1);
        await Harness.flush();

        final placed =
            harness.controller.state.activeTarget!.anchorWorldPosition!;
        final anchorId =
            harness.controller.state.activeTarget!.anchorIdentifier!;
        final creations = harness.gateway.anchorCreations;

        // ARCore pauses the anchor, and its last-known pose drifts half a
        // metre from where this app placed it. Two refresh periods — each
        // longer than the 500 ms interval — must not adopt that pose: a paused
        // sample is last-known geometry, not a measurement of the present.
        harness.gateway.anchorTracking = TrackingState.paused;
        harness.gateway.refinedAnchorPositions[anchorId] =
            placed + const Vec3(0.5, 0, 0);
        await refresh(harness);
        await refresh(harness);

        final paused = harness.controller.state;
        expect(
          harness.gateway.refreshCalls,
          greaterThanOrEqualTo(2),
          reason: 'the interval elapsed; the refresh was not skipped',
        );
        expect(paused.activeTarget!.anchorIdentifier, anchorId);
        expect(
          paused.activeTarget!.anchorWorldPosition,
          placed,
          reason:
              'the frozen placement is the fallback until ARCore tracks '
              'the anchor again',
        );
        expect(paused.activeAnchorTracking, isFalse);
        // The guidance aims at the frozen placement, not at the paused pose.
        final aim = paused.alignment!.directionWorld;
        final expectedAim = placed.normalized;
        expect(aim.x, closeTo(expectedAim.x, 1e-9));
        expect(aim.y, closeTo(expectedAim.y, 1e-9));
        expect(aim.z, closeTo(expectedAim.z, 1e-9));
        expect(paused.readyToCapture, isFalse);
        expect(paused.captureBlockReason, contains('stabilise'));
        expect(
          harness.gateway.anchorCreations,
          creations,
          reason: 'waiting on a paused anchor must not churn anchors',
        );

        // A full stable window aimed at the frozen placement is still not
        // evidence that ARCore holds the anchor, and the button refuses.
        await harness.emitStable();
        expect(harness.controller.state.activeAnchorTracking, isFalse);
        expect(harness.controller.state.readyToCapture, isFalse);
        await harness.controller.captureNow();
        await Harness.flush();
        expect(harness.controller.state.capturedCount, 0);

        // Tracking returns: only now is the refined pose adopted.
        harness.gateway.anchorTracking = TrackingState.tracking;
        await refresh(harness);
        final resumed = harness.controller.state;
        expect(resumed.activeAnchorTracking, isTrue);
        expect(
          resumed.activeTarget!.anchorWorldPosition!.x,
          closeTo(placed.x + 0.5, 1e-9),
        );
        await harness.emitStable();
        expect(harness.controller.state.readyToCapture, isTrue);
      },
    );

    test(
      'a pose without the active anchor invalidates the tracking gate',
      () async {
        final harness = await calibratedHarness(autoCapture: false);
        await harness.controller.selectTarget(1);
        await Harness.flush();
        final placed =
            harness.controller.state.activeTarget!.anchorWorldPosition!;
        final aim = facingRotation(placed.normalized);

        await harness.emitStable();
        expect(harness.controller.state.activeAnchorTracking, isTrue);
        expect(harness.controller.state.readyToCapture, isTrue);

        // The camera pose is tracked and still aimed at the target; only the
        // active-anchor report is missing. The earlier "tracking" answer does
        // not describe this event, so it may not be carried forward.
        await harness.emit(orientation: aim, includeActiveAnchorPose: false);

        final state = harness.controller.state;
        expect(
          state.activeAnchorTracking,
          isFalse,
          reason: 'a pose that cannot name the anchor cannot confirm it',
        );
        expect(state.readyToCapture, isFalse);
        expect(state.captureBlockReason, contains('stabilise'));
        // The frozen placement still aims the guidance — blocked, but not
        // blind.
        final direction = state.alignment!.directionWorld;
        final expected = placed.normalized;
        expect(direction.x, closeTo(expected.x, 1e-9));
        expect(direction.y, closeTo(expected.y, 1e-9));
        expect(direction.z, closeTo(expected.z, 1e-9));

        // And the shutter refuses on this state.
        await harness.controller.captureNow();
        await Harness.flush();
        expect(harness.controller.state.capturedCount, 0);
        expect(harness.controller.state.lastSavedPath, isNull);

        // The next pose that names a tracked anchor re-arms the gate.
        await harness.emitStable();
        expect(harness.controller.state.activeAnchorTracking, isTrue);
        expect(harness.controller.state.readyToCapture, isTrue);
      },
    );
  });

  // ── station displacement is diagnostic ───────────────────────────────
  //
  // A fixed optical centre is still encouraged by the short-window
  // `position-drift` stability check, but a static camera position is not a
  // capture rejection merely because it is far from the calibrated station.
  group('station displacement', () {
    test('is reported without changing a ready state', () {
      for (final drift in <double>[0, 0.20, 0.2001, 2, 5]) {
        final state = readyState(driftM: drift);
        expect(state.driftM, drift);
        expect(state.readyToCapture, isTrue);
        expect(state.captureBlockReason, isNull);
        expect(state.shotPhase, ShotPhase.capture);
      }
    });

    test('the automatic shutter captures a stable, aligned far pose', () async {
      final harness = await calibratedHarness(); // automatic capture on
      await harness.emitStable(position: const Vec3(5, 0, 0));
      await Harness.waitFor(() => harness.controller.state.capturedCount == 1);

      final state = harness.controller.state;
      expect(state.capturedCount, 1);
      expect(state.lastSavedPath, isNotNull);
      expect(state.targets.first.status, CaptureTargetStatus.captured);
    });

  });
}
