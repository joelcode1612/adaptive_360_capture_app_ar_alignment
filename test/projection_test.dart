import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/guidance/projection.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

/// The screen projection, against known world points.
///
/// The pipeline the brief asks to be consistent has four stages — a target
/// direction, a station-relative world position, a camera pose, and a screen
/// pixel — and each of the first three had tests. This file closes the fourth:
/// a point whose screen position can be computed by hand, and then the same
/// check for all 26 targets through the *real* `resolveTarget` → camera →
/// `project()` chain, so placement, alignment and projection cannot drift apart
/// without a failure.
void main() {
  const station = StationFrame(origin: Vec3.zero, orientation: Quat.identity);
  const geometry = CaptureGeometryConfig();
  const alignment = AlignmentCalculator();

  /// Projects a world point seen by a level camera at the origin: 1000×1000,
  /// 90°×90°, optical axis along `+Z`.
  ScreenProjection level(Vec3 worldPoint) =>
      project(Quat.identity.toCameraFrame(worldPoint), 1000, 1000, 90, 90);

  group('the pinhole model, against hand-computed pixels', () {
    test('straight ahead is the centre', () {
      final p = level(const Vec3(0, 0, 2));
      expect(p.onScreen, isTrue);
      expect(p.u, closeTo(500, 1e-9));
      expect(p.v, closeTo(500, 1e-9));
      // fx = (W/2)/tan(45°) = 500.
      expect(p.fx, closeTo(500, 1e-9));
    });

    // **The horizontal convention, stated once.** A pose maps `x̂` to the
    // camera's own right, but a physical camera's right is the *negative* of
    // that (`Quat.toCameraFrame` negates x), so with an identity pose — which
    // points along `+Z` — the world `−X` side is the screen-right one. The
    // station frame's mirror exists precisely so that a yaw +90° target, which
    // is world `−X` here, still lands on the user's right.
    test('world -X is screen-right for a camera facing +Z', () {
      expect(level(const Vec3(-1, 0, 2)).u, greaterThan(500));
      expect(level(const Vec3(1, 0, 2)).u, lessThan(500));
      expect(level(const Vec3(0, 1, 2)).v, lessThan(500));
      expect(level(const Vec3(0, -1, 2)).v, greaterThan(500));
    });

    test('the exact numbers match u = W/2 + fx·x/z', () {
      // Camera-frame x/z = +0.5 → 250 px right of centre; y/z = −0.5 → 250 px
      // below. World −X is camera-frame +x, so the point is at world x = −1.
      final p = level(const Vec3(-1, -1, 2));
      expect(p.u, closeTo(750, 1e-9));
      expect(p.v, closeTo(750, 1e-9));
    });

    test('a point on the 45° edge lands on the screen edge', () {
      // 45° off-axis on the screen-right side: exactly the right edge at a
      // 90° horizontal field.
      final p = level(const Vec3(-2, 0, 2));
      expect(p.onScreen, isTrue);
      expect(p.u, closeTo(1000, 1e-9));
    });

    test('a point behind the camera has no projection', () {
      expect(level(const Vec3(0, 0, -2)).onScreen, isFalse);
    });

    test('the poles have no projection either', () {
      expect(level(const Vec3(0, 2, 0)).onScreen, isFalse);
      expect(level(const Vec3(0, -2, 0)).onScreen, isFalse);
    });

    test('a near-horizon direction is an edge indicator, never a huge pixel', () {
      final p = project(const Vec3(1, 0, 0.00011), 1080, 1920, 60, 60);
      expect(p.onScreen, isFalse);
      expect(p.u, inInclusiveRange(12, 1068));
      expect(p.v, inInclusiveRange(12, 1908));
    });

    test('non-finite projection input is rejected without reaching Canvas', () {
      final p = project(
        const Vec3(double.infinity, 0, 1),
        1080,
        1920,
        60,
        60,
      );
      expect(p.onScreen, isFalse);
      // `isFinite` as a *matcher* does not exist in this toolchain's matcher
      // (0.12.19), so the property is asserted directly. The intent is
      // unchanged: the safe fallback projection must never emit NaN or
      // Infinity toward Canvas geometry.
      expect(p.u.isFinite, isTrue);
      expect(p.v.isFinite, isTrue);
    });
  });

  group('the 26 targets through placement, alignment and projection', () {
    /// The camera pose that looks along [direction]: `ẑ → direction`.
    Quat lookAlong(Vec3 direction) => facingRotation(direction);

    test('aiming at any target centres it on screen and reads as aligned', () {
      for (final spec in buildCaptureSphere()) {
        final resolved = resolveTarget(
          station: station,
          spec: spec,
          radiusM: geometry.targetRadiusM,
        );
        final direction = resolved.worldPosition.normalized;
        final camera = RigidPose(
          position: Vec3.zero,
          orientation: lookAlong(direction),
        );

        // Projection: the target must land on the centre pixel.
        final dc = camera.orientation.toCameraFrame(direction);
        expect(dc.z, closeTo(1, 1e-9), reason: 'target ${spec.targetId}');
        final p = project(dc, 1080, 1920, 60, 60);
        expect(p.u, closeTo(540, 1e-6), reason: 'target ${spec.targetId}');
        expect(p.v, closeTo(960, 1e-6), reason: 'target ${spec.targetId}');

        // Alignment: the same pose reads zero 3-D error, so the marker and
        // the guidance agree about where the target is.
        final result = alignment.compute(
          camera: camera,
          targetPosition: resolved.worldPosition,
          toleranceDeg: geometry.alignmentToleranceDeg,
        );
        expect(
          result.angularErrorDeg,
          closeTo(0, 1e-6),
          reason: 'target ${spec.targetId}',
        );
        expect(result.withinTolerance, isTrue);
      }
    });

    test('yaw +45 is on the right half, yaw -45 on the left', () {
      // A level camera facing the front target, with a field wide enough to
      // contain the ±45° ring.
      for (final spec in buildCaptureSphere()) {
        if (spec.pitchDegrees != 0) continue;
        if (spec.yawDegrees != 45 && spec.yawDegrees != -45) continue;
        final resolved = resolveTarget(
          station: station,
          spec: spec,
          radiusM: geometry.targetRadiusM,
        );
        final dc = Quat.identity.toCameraFrame(
          resolved.worldPosition.normalized,
        );
        final p = project(dc, 1000, 1000, 140, 140);
        if (spec.yawDegrees == 45) {
          expect(p.u, greaterThan(500), reason: 'yaw +45 must be screen-right');
        } else {
          expect(p.u, lessThan(500), reason: 'yaw -45 must be screen-left');
        }
      }
    });

    test('upper ring is above centre, lower ring below', () {
      for (final spec in buildCaptureSphere()) {
        if (spec.ringName != kUpperRingName &&
            spec.ringName != kLowerRingName) {
          continue;
        }
        if (spec.yawDegrees != 0) continue;
        final resolved = resolveTarget(
          station: station,
          spec: spec,
          radiusM: geometry.targetRadiusM,
        );
        final dc = Quat.identity.toCameraFrame(
          resolved.worldPosition.normalized,
        );
        final p = project(dc, 1000, 1000, 140, 140);
        if (spec.ringName == kUpperRingName) {
          expect(p.v, lessThan(500), reason: 'upper ring must be above centre');
        } else {
          expect(p.v, greaterThan(500), reason: 'lower ring must be below');
        }
      }
    });

    test('turning ten degrees moves the target by the predicted pixels', () {
      final resolved = resolveTarget(
        station: station,
        spec: buildCaptureSphere().first,
        radiusM: geometry.targetRadiusM,
      );
      final direction = resolved.worldPosition.normalized;
      // The camera is turned 10° off the target, so the target must sit
      // fx·tan(10°) = 500·0.1763 ≈ 88 px off centre.
      final camera = RigidPose(
        position: Vec3.zero,
        orientation: lookAlong(
          Quat.axisAngle(10, Vec3.unitY).rotate(direction),
        ),
      );
      final dc = camera.orientation.toCameraFrame(direction);
      final p = project(dc, 1000, 1000, 90, 90);
      expect((p.u - 500).abs(), closeTo(500 * 0.17632698, 1.0));

      final result = alignment.compute(
        camera: camera,
        targetPosition: resolved.worldPosition,
        toleranceDeg: 3,
      );
      expect(result.angularErrorDeg, closeTo(10, 1e-6));
      expect(result.withinTolerance, isFalse);
      // The invariant that keeps the arrow, the wording and the pixel
      // together: "turn right" and "right of centre" are the same side.
      expect((p.u - 500).sign, result.yawCorrectionDeg.sign);
    });
  });
}
