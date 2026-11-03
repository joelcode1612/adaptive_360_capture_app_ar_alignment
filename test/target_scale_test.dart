import 'dart:io';
import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/glb/glb_model.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// Marker size, target positions and the readiness wording.
///
/// The size regression this pins came from a phone screenshot: a white disc
/// with a green centre covering a large part of the camera view. The cause was
/// not the asset — it is authored 0.54 m across, and the bezel 0.70 m — but the
/// **scale**: the draw scale was a bare constant `1.0`, so the marker was drawn
/// at its authored size and, at the 1.5 m capture radius, subtended about 20°.
/// A physical diameter in metres, divided by the measured plate width, is the
/// fix; these tests hold both halves of that.
void main() {
  const geometry = CaptureGeometryConfig();
  const station = StationFrame(origin: Vec3.zero, orientation: Quat.identity);

  group('the marker is sized in metres, not in a magic scale', () {
    GlbModel plate() => parseGlb(
      File('assets/models/ar_capture_target.glb').readAsBytesSync(),
      source: 'ar_capture_target.glb',
    );
    GlbModel bezel() => parseGlb(
      File('assets/models/white_frame_bezel.glb').readAsBytesSync(),
      source: 'white_frame_bezel.glb',
    );

    test('the plate is authored 0.54 m across, front face on +Z', () {
      final model = plate();
      expect(model.widthX, closeTo(0.54, 2e-3));
      expect(model.heightY, closeTo(0.54, 2e-3));
      // The face the camera sees is the +Z one: the disc is thin, and its
      // front surface sits at +0.006 while the bezel's reaches +0.012.
      expect(model.max.z, greaterThan(0));
      expect(model.depthZ, lessThan(0.02));
    });

    test('the draw scale normalises the plate to the configured diameter', () {
      final scale = geometry.markerScaleFor(plate().widthX);
      expect(scale, closeTo(geometry.targetDiameterM / 0.54, 1e-6));
      // 0.22 / 0.54 — not 1.0, which was the bug.
      expect(scale, closeTo(0.4074, 1e-4));
      expect(scale, lessThan(0.6));
    });

    test('at the capture radius the plate subtends a reticle, not a wall', () {
      final scale = geometry.markerScaleFor(plate().widthX);
      final drawnWidthM = plate().widthX * scale;
      expect(drawnWidthM, closeTo(geometry.targetDiameterM, 1e-9));
      final halfAngle = math.atan((drawnWidthM / 2) / geometry.targetRadiusM);
      final subtendedDeg = 2 * halfAngle * 180 / math.pi;
      // A phone's vertical FOV is roughly 60°; a fifth of it is aimable and
      // nowhere near dominating the frame. The unscaled plate (0.54 m at the
      // 1.0 m radius, ~30°) fails this — that was the bug — and so does the old
      // 1.5 m figure the 0.22 m plate was sized for.
      expect(subtendedDeg, closeTo(12.55, 0.05));
      expect(subtendedDeg, lessThan(14));
      expect(subtendedDeg, greaterThan(4));
    });

    test('the bezel keeps its authored ratio to the plate', () {
      final scale = geometry.markerScaleFor(plate().widthX);
      final plateDrawn = plate().widthX * scale;
      final bezelDrawn = bezel().widthX * scale;
      expect(
        bezelDrawn / plateDrawn,
        closeTo(bezel().widthX / plate().widthX, 1e-9),
      );
      // The ring is larger than the plate and still a small object in frame.
      expect(bezelDrawn, greaterThan(plateDrawn));
      expect(bezelDrawn, lessThan(0.5));
    });
  });

  group('where each of the 26 targets lands', () {
    ({Vec3 worldPosition, Vec3 facingNormal, Quat localRotation}) resolve(
      CaptureTargetSpec spec,
    ) => resolveTarget(
      station: station,
      spec: spec,
      radiusM: geometry.targetRadiusM,
    );

    CaptureTargetSpec specFor(double yaw, double pitch) =>
        buildCaptureSphere().firstWhere(
          (CaptureTargetSpec s) =>
              (s.yawDegrees - yaw).abs() < 1e-9 &&
              (s.pitchDegrees - pitch).abs() < 1e-9,
        );

    test('front is forward and back is behind', () {
      final front = resolve(specFor(0, 0)).worldPosition;
      final back = resolve(specFor(180, 0)).worldPosition;
      expect(front.x, closeTo(0, 1e-9));
      expect(front.y, closeTo(0, 1e-9));
      expect(front.z, closeTo(geometry.targetRadiusM, 1e-9));
      expect(back.z, closeTo(-geometry.targetRadiusM, 1e-9));
      expect(front.normalized.dot(back.normalized), closeTo(-1, 1e-9));
    });

    test('yaw +90 is to the camera\u2019s right, yaw -90 to its left', () {
      // The station frame's local +X is the physical right; `localToWorld`
      // applies the documented mirror, so right and left swap world axes but
      // never swap sides relative to the camera.
      final right = resolve(specFor(90, 0)).worldPosition;
      final left = resolve(specFor(-90, 0)).worldPosition;
      expect(right.normalized.dot(left.normalized), closeTo(-1, 1e-9));
      expect(right.x.abs(), closeTo(geometry.targetRadiusM, 1e-9));
      expect(left.x.abs(), closeTo(geometry.targetRadiusM, 1e-9));
    });

    test('upper ring is up, lower ring is down, poles are the poles', () {
      // The rings sit at ±45° and the poles at the exact ±90°: elevation θ
      // puts a target at y = r·sin θ.
      final upperY = math.sin(45 * math.pi / 180) * geometry.targetRadiusM;
      final lowerY = -upperY;
      final poleY = geometry.targetRadiusM;
      for (final spec in buildCaptureSphere()) {
        final p = resolve(spec).worldPosition;
        if (spec.ringName == kUpperRingName) {
          expect(p.y, closeTo(upperY, 1e-9), reason: 'target ${spec.targetId}');
        }
        if (spec.ringName == kLowerRingName) {
          expect(p.y, closeTo(lowerY, 1e-9), reason: 'target ${spec.targetId}');
        }
        if (spec.ringName == kMiddleRingName) {
          expect(p.y, closeTo(0, 1e-9), reason: 'target ${spec.targetId}');
        }
      }
      expect(resolve(specFor(0, 90)).worldPosition.y, closeTo(poleY, 1e-9));
      expect(resolve(specFor(0, -90)).worldPosition.y, closeTo(-poleY, 1e-9));
    });

    test('the sphere map and the AR world agree for all 26', () {
      // The map projects `spec.direction`, station-local. The world position
      // is the same direction through the station frame, so mapping it back
      // must return the direction the map drew.
      for (final spec in buildCaptureSphere()) {
        final resolved = resolve(spec);
        final local = station
            .worldToLocal(resolved.worldPosition - station.origin)
            .normalized;
        expect(
          local.x,
          closeTo(spec.direction.x, 1e-9),
          reason: 'target ${spec.targetId}',
        );
        expect(local.y, closeTo(spec.direction.y, 1e-9));
        expect(local.z, closeTo(spec.direction.z, 1e-9));
        // Every position is exactly the configured radius from the station.
        expect(
          resolved.worldPosition.length,
          closeTo(geometry.targetRadiusM, 1e-9),
        );
      }
    });

    test('every plate faces the station, and every position is fixed', () {
      for (final spec in buildCaptureSphere()) {
        final resolved = resolve(spec);
        final toStation = (station.origin - resolved.worldPosition).normalized;
        expect(resolved.facingNormal.x, closeTo(toStation.x, 1e-9));
        expect(resolved.facingNormal.y, closeTo(toStation.y, 1e-9));
        expect(resolved.facingNormal.z, closeTo(toStation.z, 1e-9));
        // Resolving twice returns the same position: placement is arithmetic.
        expect(
          resolve(spec).worldPosition.x,
          closeTo(resolved.worldPosition.x, 1e-12),
        );
      }
    });
  });

  group('the readiness message names the blocker', () {
    CaptureState stateWith({
      required String? rejection,
      double? rate,
      bool stable = false,
      bool withinTolerance = true,
      double yawCorrectionDeg = 0.2,
      bool activeAnchorTracking = true,
      bool trackingRecovering = false,
      bool stationDriftWarningActive = false,
      TrackingState tracking = TrackingState.tracking,
      String? trackingFailureReason,
    }) {
      final spec = buildCaptureSphere().first;
      final target = CaptureTarget(
        targetId: 1,
        stationId: 's1',
        yawDegrees: 0,
        pitchDegrees: 0,
        stationLocalDirection: spec.direction,
        status: CaptureTargetStatus.active,
      );
      return CaptureState(
        phase: CapturePhase.capturing,
        tracking: tracking,
        trackingFailureReason: trackingFailureReason,
        geometry: geometry,
        station: CaptureStation(
          stationId: 's1',
          referencePosition: Vec3.zero,
          referenceOrientation: Quat.identity,
          createdAt: DateTime.utc(2026, 10, 21),
          trackingStatus: TrackingState.tracking,
        ),
        targets: <CaptureTarget>[target],
        activeTargetId: 1,
        activeAnchored: true,
        activeAnchorTracking: activeAnchorTracking,
        trackingRecovering: trackingRecovering,
        stationDriftWarningActive: stationDriftWarningActive,
        alignment: AlignmentResult(
          directionWorld: Vec3.unitZ,
          directionInCamera: Vec3.unitZ,
          distanceM: geometry.targetRadiusM,
          yawCorrectionDeg: yawCorrectionDeg,
          pitchCorrectionDeg: 0.1,
          angularErrorDeg: withinTolerance ? 0.4 : 9,
          toleranceDeg: 3,
        ),
        stable: stable,
        stabilityRejection: rejection,
        angularRateDegPerSec: rate,
      );
    }

    test('moving fast says so, with the rate', () {
      final state = stateWith(rejection: 'angular-rate', rate: 7.4);
      expect(state.instruction, contains('7.4°/s'));
      expect(state.instruction, contains('moving'));
    });

    test('a stale pose is not described as movement', () {
      expect(
        stateWith(rejection: 'measurement-age').instruction,
        contains('fresh pose'),
      );
    });

    test('off the target says which way to aim', () {
      // Misalignment is not a stability rejection: the engine reports the hold,
      // the alignment reports the aim. The sentence names the target and the
      // turn, rather than hiding a 12° error behind "Hold still…".
      final state = stateWith(
        rejection: null,
        withinTolerance: false,
        yawCorrectionDeg: 12,
      );
      expect(state.instruction, contains('Aim at'));
      expect(state.instruction, contains('Turn right 12°'));
      expect(state.captureBlockReason, isNotNull);
      expect(state.readyToCapture, isFalse);
    });

    test('genuine movement says so', () {
      expect(
        stateWith(rejection: 'position-drift').instruction,
        contains('Stand steady'),
      );
    });

    test('ready only when stable, and then it says capture', () {
      expect(
        stateWith(rejection: null, stable: true).instruction,
        contains('Ready to capture'),
      );
    });

    test('a retained drift value is not painted during reference recovery', () {
      final live = stateWith(rejection: null, stationDriftWarningActive: true);
      expect(live.stationDriftWarningVisible, isTrue);

      final recovering = stateWith(
        rejection: null,
        stationDriftWarningActive: true,
        trackingRecovering: true,
      );
      expect(recovering.stationDriftWarningVisible, isFalse);

      final noLiveAnchor = stateWith(
        rejection: null,
        stationDriftWarningActive: true,
        activeAnchorTracking: false,
      );
      expect(noLiveAnchor.stationDriftWarningVisible, isFalse);
    });

    test('insufficient visual features asks for a stable textured surface', () {
      final paused = stateWith(
        rejection: null,
        tracking: TrackingState.paused,
        trackingFailureReason: 'INSUFFICIENT_FEATURES',
      );

      expect(paused.trackingGuidance, contains('textured, stationary surface'));
      expect(paused.trackingGuidance, contains('Move slowly'));
      expect(paused.trackingGuidance, isNot(contains('plane')));
    });
  });
}
