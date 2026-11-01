import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart'
    show VioSphereConstants;
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

void main() {
  group('the 26 capture directions', () {
    final sphere = buildCaptureSphere();

    test('are exactly 26, with ids 1..26', () {
      expect(sphere.length, kCaptureTargetCount);
      expect(sphere.length, 26);
      for (var i = 0; i < sphere.length; i++) {
        expect(sphere[i].targetId, i + 1);
      }
    });

    // Required case 1: yaw 0, pitch 0 is the forward direction — and it is
    // the FIRST target, because the capture sequence starts at the front.
    test('yaw 0 / pitch 0 is straight forward, and it is target 1', () {
      final forward = sphere.firstWhere(
        (CaptureTargetSpec s) => s.yawDegrees == 0 && s.pitchDegrees == 0,
      );
      expect(forward.targetId, 1, reason: 'the sequence starts at the front');
      expect(sphere.first.targetId, 1);
      expect(forward.direction.x, closeTo(0, 1e-12));
      expect(forward.direction.y, closeTo(0, 1e-12));
      expect(forward.direction.z, closeTo(1, 1e-12));
    });

    // Required case 2: the id → angle mapping is the specified layout, in the
    // capture order: the horizontal ring, upper, lower, then the two poles.
    test('carry the specified yaw and pitch per ring', () {
      // Horizontal ring: 8 targets, yaw every 45° from 0, all at pitch 0.
      expect(VioSphereConstants.horizontalCount, 8);
      for (var i = 0; i < VioSphereConstants.horizontalCount; i++) {
        expect(
          sphere[i].yawDegrees,
          kCaptureYawsDeg[i],
          reason: 'horizontal ring, target ${i + 1}',
        );
        expect(
          sphere[i].pitchDegrees,
          0,
          reason: 'the horizontal ring is level',
        );
        expect(sphere[i].ringName, kMiddleRingName);
        expect(sphere[i].isPole, isFalse);
      }
      // Upper ring: 8 targets, yaw every 45°, pitch +45°.
      expect(VioSphereConstants.upperCount, 8);
      for (var i = 0; i < VioSphereConstants.upperCount; i++) {
        final spec = sphere[VioSphereConstants.upperFirstIndex + i];
        expect(spec.pitchDegrees, VioSphereConstants.upperElevationDeg);
        expect(spec.yawDegrees, kCaptureRingYawsDeg[i]);
        expect(spec.ringName, kUpperRingName);
      }
      // Lower ring: 8 targets, yaw every 45°, pitch −45°.
      expect(VioSphereConstants.lowerCount, 8);
      for (var i = 0; i < VioSphereConstants.lowerCount; i++) {
        final spec = sphere[VioSphereConstants.lowerFirstIndex + i];
        expect(spec.pitchDegrees, VioSphereConstants.lowerElevationDeg);
        expect(spec.yawDegrees, kCaptureRingYawsDeg[i]);
        expect(spec.ringName, kLowerRingName);
      }
      // The two near-poles close the sequence, one at each end of the vertical.
      expect(
        sphere[VioSphereConstants.zenithIndex].pitchDegrees,
        VioSphereConstants.zenithElevationDeg,
      );
      expect(
        sphere[VioSphereConstants.nadirIndex].pitchDegrees,
        VioSphereConstants.nadirElevationDeg,
      );
    });

    test('end with the zenith and the nadir', () {
      final zenith = sphere[kZenithTargetId - 1];
      final nadir = sphere[kNadirTargetId - 1];
      expect(zenith.isZenith, isTrue);
      expect(zenith.pitchDegrees, VioSphereConstants.zenithElevationDeg);
      expect(
        zenith.direction.maxAbsDiff(directionFromYawPitchForTest(0, 90)),
        lessThan(1e-12),
      );
      // The nadir is the zenith seen from below: the same horizontal direction,
      // the opposite vertical. Both sit at the exact poles, so they are ±unitY
      // and yaw is undefined there. That is safe because every consumer
      // recognises a pole by pitch alone (`CaptureTarget.isPole`,
      // `AlignmentCalculator.isPole`) and offers vertical-only guidance.
      expect(nadir.isNadir, isTrue);
      expect(nadir.pitchDegrees, VioSphereConstants.nadirElevationDeg);
      expect(
        nadir.direction.maxAbsDiff(directionFromYawPitchForTest(0, -90)),
        lessThan(1e-12),
      );
      expect(zenith.direction.y, closeTo(1.0, 1e-12));
      expect(nadir.direction.y, closeTo(-1.0, 1e-12));
    });

    test('every direction is a unit vector', () {
      for (final spec in sphere) {
        expect(spec.direction.length, closeTo(1, 1e-12));
      }
    });

    test('match the §7 spherical formula', () {
      for (final spec in sphere) {
        final yaw = spec.yawDegrees * math.pi / 180;
        final pitch = spec.pitchDegrees * math.pi / 180;
        expect(
          spec.direction.x,
          closeTo(math.sin(yaw) * math.cos(pitch), 1e-12),
        );
        expect(spec.direction.y, closeTo(math.sin(pitch), 1e-12));
        expect(
          spec.direction.z,
          closeTo(math.cos(yaw) * math.cos(pitch), 1e-12),
        );
      }
    });

    test('cover the sphere without clustering', () {
      // The tightest pair is two neighbours on the upper ring (or the lower
      // ring): their 45° yaw steps at 45° of elevation span
      // acos(sin²45° + cos²45°·cos45°) ≈ 31.4° of arc. Horizontal neighbours
      // are 45° apart, the same-yaw gap between the rings is 45°, and each pole
      // is 45° from the ring below it.
      var minimum = 180.0;
      for (var i = 0; i < sphere.length; i++) {
        for (var j = i + 1; j < sphere.length; j++) {
          final angle =
              math.acos(
                (sphere[i].direction.dot(sphere[j].direction)).clamp(-1.0, 1.0),
              ) *
              180 /
              math.pi;
          if (angle < minimum) minimum = angle;
        }
      }
      expect(minimum, closeTo(31.4, 0.05));
      expect(minimum, greaterThan(31.0));
    });
  });

  group('StationFrame', () {
    test('localToWorld and worldToLocal are inverses', () {
      final frame = StationFrame(
        origin: const Vec3(1, 1.6, -2),
        orientation: Quat.fromYawPitch(37, -12),
      );
      for (final spec in buildCaptureSphere()) {
        final world = frame.localToWorld(spec.direction);
        final back = frame.worldToLocal(world);
        expect(back.maxAbsDiff(spec.direction), lessThan(1e-12));
      }
    });

    test('station-local +X is the physical right', () {
      // The whole point of the mirror: "yaw +90°" must be to the user's right,
      // not their left. With the camera facing the station's forward, the
      // physical right is -R·x̂ — see the class comment.
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final right = frame.rightAxis;
      final forward = frame.forwardAxis;
      final up = frame.upAxis;
      expect(forward, Vec3.unitZ);
      expect(up, Vec3.unitY);
      // right × up must be *anti*-parallel to forward for a physical camera.
      expect(right.cross(up), -forward);
    });

    test('a station-local yaw of +90 lands to the right of the camera', () {
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final camera = RigidPose(
        position: Vec3.zero,
        orientation: Quat.identity, // facing the station's forward
      );
      final targetWorld = frame.localPointToWorld(
        directionFromYawPitchForTest(90, 0) * 3,
      );
      // Camera basis in the app convention: physical right is -R·x̂ = -X.
      final toTarget = targetWorld - camera.position;
      expect(camera.right.dot(toTarget), greaterThan(0));
    });

    test('cameraInStation is inverse(T_world_station) × T_world_camera', () {
      final frame = StationFrame(
        origin: const Vec3(2, 0, 1),
        orientation: Quat.fromYawPitch(20, 0),
      );
      // 3 m along the station's own forward, which is yaw 20° in world terms.
      final world = frame.localPointToWorld(
        directionFromYawPitchForTest(0, 0) * 3,
      );
      final camera = RigidPose(
        position: world,
        orientation: Quat.fromYawPitch(20, 0),
      );
      final relative = frame.cameraInStation(camera);
      // 3 m along the station's forward, no rotation relative to it.
      expect(relative.position.z, closeTo(3, 1e-9));
      expect(relative.position.x, closeTo(0, 1e-9));
      expect(relative.position.y, closeTo(0, 1e-9));
      expect(
        (relative.orientation.rotate(Vec3.unitZ)).maxAbsDiff(Vec3.unitZ),
        lessThan(1e-9),
      );
    });

    test('headingOf reports a positive yaw as a heading to the right', () {
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      final worldForward = frame.localToWorld(
        directionFromYawPitchForTest(30, 0),
      );
      final heading = frame.headingOf(
        RigidPose(
          position: Vec3.zero,
          orientation: orientationFor(worldForward),
        ),
      );
      expect(heading.yawDeg, closeTo(30, 1e-9));
      expect(heading.pitchDeg, closeTo(0, 1e-9));
    });

    test('driftOf measures how far the phone has walked', () {
      final frame = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
      expect(
        frame.driftOf(
          RigidPose(
            position: const Vec3(0, 0, 0.75),
            orientation: Quat.identity,
          ),
        ),
        closeTo(0.75, 1e-12),
      );
    });
  });
}

/// Builds a unit quaternion whose optical axis is [forward].
Quat orientationFor(Vec3 forward) {
  final f = forward.normalized;
  final yaw = math.atan2(f.x, f.z) * 180 / math.pi;
  final pitch = math.asin(f.y.clamp(-1.0, 1.0)) * 180 / math.pi;
  return Quat.fromYawPitch(yaw, pitch);
}

Vec3 directionFromYawPitchForTest(double yawDeg, double pitchDeg) {
  final y = yawDeg * math.pi / 180;
  final p = pitchDeg * math.pi / 180;
  final cp = math.cos(p);
  return Vec3(cp * math.sin(y), math.sin(p), cp * math.cos(y));
}
