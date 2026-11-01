import 'dart:math' as math;

import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart';
import 'package:flutter_test/flutter_test.dart';

/// Tests for the VIO sphere mathematics, in six groups:
///
/// * **A** geometry and target count
/// * **B** coordinate convention and yaw
/// * **C** station drift
/// * **D** angular alignment and guidance
/// * **E** the 600 ms stability window
/// * **F** the shutter expression and its sequencing
///
/// Every test here runs the production code path — no threshold is duplicated
/// from the implementation, so a constant changed in `VioSphereConstants` and
/// not here shows up as a failure rather than as two numbers quietly agreeing
/// with each other.

// ── helpers ──────────────────────────────────────────────────────────────

/// A camera rotation looking along (yaw, pitch) in the engine's convention:
/// yaw 0 is world `-Z`, positive yaw turns right onto `+X`, positive pitch
/// looks up. Built as a yaw about world `+Y` composed with a pitch about the
/// **camera's own un-yawed right axis** (`+X`), so the resulting forward vector
/// is `(cosθ·sinψ, sinθ, −cosθ·cosψ)` by construction rather than by assertion.
///
/// The pitch axis is `+X` and not the yawed right axis on purpose: the pitch is
/// applied first, so its axis is expressed in the frame *before* the yaw carries
/// it around. Using `(cosψ, 0, sinψ)` here — as an earlier version of this
/// helper did — silently produces a different direction (40°/25° came out as
/// 37.2°), which would have made every "looking straight at it" test pass
/// against a wrong expectation.
///
/// [rollDeg] turns about the camera's own optical axis, which leaves the
/// forward direction (and therefore the alignment) untouched while changing the
/// quaternion — the only way to test orientation drift without also testing
/// misalignment.
Quat lookingAt(double yawDeg, double pitchDeg, {double rollDeg = 0}) {
  final q =
      Quat.axisAngle(-yawDeg, Vec3.unitY) *
      Quat.axisAngle(pitchDeg, Vec3.unitX);
  if (rollDeg == 0) return q;
  return q * Quat.axisAngle(rollDeg, const Vec3(0, 0, -1));
}

VioPoseSample sample(
  Vec3 position,
  Quat rotation,
  int milliseconds, {
  int nowOffsetMs = 0,
}) => VioPoseSample(
  position: position,
  rotation: rotation,
  timestampNanoseconds: milliseconds * 1000000,
);

/// An engine calibrated at the world origin looking along `-Z`.
VioSphereEngine engineAtOrigin({double stationYawRad = 0}) {
  final engine = VioSphereEngine();
  final result = engine.calibrateWithReference(
    stationOrigin: Vec3.zero,
    stationYawRad: stationYawRad,
  );
  expect(result.ok, isTrue, reason: 'the fixture station must calibrate');
  return engine;
}

/// The one evaluation from feeding [count] samples [stepMs] apart, all at the
/// station, returning the state after the last one.
///
/// [rotationFor]/[positionFor] let a test walk the pose through a window;
/// otherwise every sample is identical, which is a perfectly still phone.
FrameGateEvaluation feed(
  VioSphereEngine engine, {
  required int index,
  required Quat rotation,
  required int count,
  int stepMs = 33,
  int startMs = 0,
  Vec3 position = Vec3.zero,
  bool trackingValid = true,
  bool isSavingInProgress = false,
  bool isFailureLatched = false,
  int nowOffsetMs = 0,
  Quat Function(int i)? rotationFor,
  Vec3 Function(int i)? positionFor,
}) {
  late FrameGateEvaluation last;
  for (var i = 0; i < count; i++) {
    final ms = startMs + i * stepMs;
    last = engine.evaluateFrame(
      pose: sample(
        positionFor?.call(i) ?? position,
        rotationFor?.call(i) ?? rotation,
        ms,
      ),
      trackingValid: trackingValid,
      activeTargetIndex: index,
      isSavingInProgress: isSavingInProgress,
      isFailureLatched: isFailureLatched,
      nowNanoseconds: (ms + nowOffsetMs) * 1000000,
    );
  }
  return last;
}

/// One node's world direction, from the specification's closed form.
Vec3 expectedDirection({
  required double elevationDeg,
  required double azimuthDeg,
  required double stationYawRad,
}) {
  final theta = elevationDeg * math.pi / 180;
  final absolute = stationYawRad + azimuthDeg * math.pi / 180;
  return Vec3(
    math.cos(theta) * math.sin(absolute),
    math.sin(theta),
    -math.cos(theta) * math.cos(absolute),
  );
}

double columnDot(List<double> m, int a, int b) =>
    m[a] * m[b] + m[a + 1] * m[b + 1] + m[a + 2] * m[b + 2];

double columnLength(List<double> m, int a) => math.sqrt(columnDot(m, a, a));

void main() {
  // ══ A. geometry and target count ══════════════════════════════════════

  group('A. geometry', () {
    test('the sphere is exactly 26 nodes in the specified index order', () {
      final engine = engineAtOrigin();
      final nodes = engine.nodes;

      expect(nodes.length, 26);
      expect(VioSphereConstants.nodeCount, 26);
      for (var i = 0; i < nodes.length; i++) {
        expect(nodes[i].index, i, reason: 'index $i must be at position $i');
        expect(nodes[i].targetId, i + 1);
      }

      expect(nodes.where((n) => n.ringName == 'Horizontal').length, 8);
      expect(nodes.where((n) => n.ringName == 'Upper').length, 8);
      expect(nodes.where((n) => n.ringName == 'Lower').length, 8);
      expect(nodes.where((n) => n.ringName == 'Zenith').length, 1);
      expect(nodes.where((n) => n.ringName == 'Nadir').length, 1);

      // The index boundaries the specification fixes.
      expect(nodes[0].ringName, 'Horizontal');
      expect(nodes[7].ringName, 'Horizontal');
      expect(nodes[8].ringName, 'Upper');
      expect(nodes[15].ringName, 'Upper');
      expect(nodes[16].ringName, 'Lower');
      expect(nodes[23].ringName, 'Lower');
      expect(nodes[24].isZenith, isTrue);
      expect(nodes[25].isNadir, isTrue);
      expect(nodes.where((n) => n.isPole).length, 2);
    });

    test('every node is exactly the configured radius from the station', () {
      final engine = engineAtOrigin();
      for (final node in engine.nodes) {
        expect(
          Vec3.distance(node.worldPosition, Vec3.zero),
          closeTo(VioSphereConstants.targetRadiusM, 1e-12),
          reason: 'node ${node.index}',
        );
        expect(node.direction.length, closeTo(1, 1e-12));
      }
    });

    test('elevations, azimuths and steps are the specified layout', () {
      final engine = engineAtOrigin();
      final nodes = engine.nodes;

      for (var i = 0; i < 8; i++) {
        expect(nodes[i].elevationDeg, 0);
        expect(nodes[i].azimuthDeg, closeTo(i * 45.0, 1e-12));
      }
      for (var i = 0; i < 8; i++) {
        expect(nodes[8 + i].elevationDeg, 45);
        expect(nodes[8 + i].azimuthDeg, closeTo(i * 45.0, 1e-12));
        expect(nodes[16 + i].elevationDeg, -45);
        expect(nodes[16 + i].azimuthDeg, closeTo(i * 45.0, 1e-12));
      }
      expect(nodes[24].elevationDeg, 90);
      expect(nodes[25].elevationDeg, -90);
    });

    test('the position is P_station + r·d from the closed form', () {
      // A non-trivial reference: not the origin, and a yaw that is neither 0
      // nor a multiple of 90°, so a sign error cannot cancel out.
      const station = Vec3(1.5, 2.25, -3.75);
      const yaw = 0.7;
      final engine = VioSphereEngine();
      engine.calibrateWithReference(stationOrigin: station, stationYawRad: yaw);

      for (final node in engine.nodes) {
        final d = expectedDirection(
          elevationDeg: node.elevationDeg,
          azimuthDeg: node.azimuthDeg,
          stationYawRad: yaw,
        );
        expect(
          node.direction.x,
          closeTo(d.x, 1e-12),
          reason: 'node ${node.index}',
        );
        expect(
          node.direction.y,
          closeTo(d.y, 1e-12),
          reason: 'node ${node.index}',
        );
        expect(
          node.direction.z,
          closeTo(d.z, 1e-12),
          reason: 'node ${node.index}',
        );
        final expected = station + d * VioSphereConstants.targetRadiusM;
        expect(node.worldPosition.x, closeTo(expected.x, 1e-12));
        expect(node.worldPosition.y, closeTo(expected.y, 1e-12));
        expect(node.worldPosition.z, closeTo(expected.z, 1e-12));
      }
    });

    test('a station yaw rotates the whole arrangement, not one ring', () {
      final zero = engineAtOrigin();
      final turned = engineAtOrigin(stationYawRad: math.pi / 2);

      // The horizontal ring's node 0 is the calibrated forward direction: at
      // yaw 0 that is -Z, at yaw +90° it is +X. If only one ring followed the
      // yaw — or if the yaw were applied with the wrong sign — this pair fails.
      expect(zero.nodes[0].direction.z, closeTo(-1, 1e-12));
      expect(turned.nodes[0].direction.x, closeTo(1, 1e-12));
      expect(turned.nodes[0].direction.z, closeTo(0, 1e-12));

      // Both the horizontal and the lower ring turn with it.
      expect(zero.nodes[1].direction.x, closeTo(math.sin(math.pi / 4), 1e-12));
      expect(
        turned.nodes[1].direction.x,
        closeTo(math.sin(math.pi / 2 + math.pi / 4), 1e-12),
      );
      expect(
        turned.nodes[20].direction.y,
        closeTo(math.sin(-45 * math.pi / 180), 1e-12),
      );
    });

    test('zenith and nadir sit at the exact ±90° above the station', () {
      final engine = engineAtOrigin();
      final zenith = engine.nodes[24];
      final nadir = engine.nodes[25];

      expect(zenith.worldPosition.y, closeTo(1, 1e-12));
      expect(nadir.worldPosition.y, closeTo(-1, 1e-12));
      // At the exact poles the horizontal component vanishes.
      expect(zenith.direction.horizontalLength, closeTo(0, 1e-12));
      // A pole node is still a unit direction, and still at the radius.
      expect(zenith.direction.length, closeTo(1, 1e-12));
      expect(
        Vec3.distance(zenith.worldPosition, Vec3.zero),
        closeTo(VioSphereConstants.targetRadiusM, 1e-12),
      );
      // The pole's azimuth is degenerate — the direction is pure ±Y.
      expect(zenith.direction.x, closeTo(0, 1e-12));
      expect(zenith.direction.z, closeTo(0, 1e-12));
      expect(nadir.direction.x, closeTo(0, 1e-12));
      expect(nadir.direction.z, closeTo(0, 1e-12));
    });

    test('the model matrix translates to the target and faces the station', () {
      final engine = engineAtOrigin();
      for (final node in engine.nodes) {
        final m = node.modelMatrix;
        expect(m.length, 16);

        // Column-major: the translation is the last column.
        expect(m[12], closeTo(node.worldPosition.x, 1e-12));
        expect(m[13], closeTo(node.worldPosition.y, 1e-12));
        expect(m[14], closeTo(node.worldPosition.z, 1e-12));
        expect(m[15], 1);

        // The model's +Z (the third column) points from the target back at the
        // station, i.e. along -direction.
        expect(
          m[8],
          closeTo(-node.direction.x, 1e-12),
          reason: 'node ${node.index}',
        );
        expect(
          m[9],
          closeTo(-node.direction.y, 1e-12),
          reason: 'node ${node.index}',
        );
        expect(
          m[10],
          closeTo(-node.direction.z, 1e-12),
          reason: 'node ${node.index}',
        );
        expect(m[11], 0);

        // An orthonormal, right-handed basis: a non-unit or mirrored basis
        // scales the plate or turns it inside out.
        expect(columnLength(m, 0), closeTo(1, 1e-9));
        expect(columnLength(m, 4), closeTo(1, 1e-9));
        expect(columnLength(m, 8), closeTo(1, 1e-9));
        expect(columnDot(m, 0, 4), closeTo(0, 1e-9));
        expect(columnDot(m, 0, 8), closeTo(0, 1e-9));
        expect(columnDot(m, 4, 8), closeTo(0, 1e-9));

        // det = x · (y × z) = +1.
        final x = Vec3(m[0], m[1], m[2]);
        final y = Vec3(m[4], m[5], m[6]);
        final z = Vec3(m[8], m[9], m[10]);
        expect(x.dot(y.cross(z)), closeTo(1, 1e-9));

        for (var i = 0; i < 16; i++) {
          expect(m[i].isFinite, isTrue, reason: 'node ${node.index}, m[$i]');
        }
      }
    });

    test('a target straight above the station keeps a valid, kept basis', () {
      // The degenerate case the exact poles now exercise: `up × Z` collapses
      // here, so the reference up must flip rather than produce a zero axis —
      // and the translation must survive either way, or the plate is drawn at
      // the world origin instead of at the target.
      const station = Vec3(0.5, 0, -0.5);
      const above = Vec3(0.5, 1.0, -0.5);
      final m = VioSphereEngine.buildModelMatrix(
        targetWorldPosition: above,
        stationOriginWorld: station,
      );

      for (var i = 0; i < 16; i++) {
        expect(m[i].isFinite, isTrue, reason: 'm[$i]');
      }
      expect(m[12], closeTo(0.5, 1e-12));
      expect(m[13], closeTo(1.0, 1e-12));
      expect(m[14], closeTo(-0.5, 1e-12));
      expect(columnLength(m, 0), closeTo(1, 1e-9));
      expect(columnDot(m, 0, 4), closeTo(0, 1e-9));
      // The +Z column still points straight back down at the station.
      expect(m[8], closeTo(0, 1e-12));
      expect(m[9], closeTo(-1, 1e-12));
      expect(m[10], closeTo(0, 1e-12));
      // The alternate up is engaged: the plate's up is no longer world up.
      expect(Vec3(m[4], m[5], m[6]).y.abs(), lessThan(1e-9));
    });

    test('the same reference produces bit-identical geometry every time', () {
      const station = Vec3(-2.0, 0.75, 4.0);
      const yaw = -1.1;
      final a = VioSphereEngine()
        ..calibrateWithReference(stationOrigin: station, stationYawRad: yaw);
      final b = VioSphereEngine()
        ..calibrateWithReference(stationOrigin: station, stationYawRad: yaw);

      for (var i = 0; i < 26; i++) {
        expect(a.nodes[i].worldPosition, b.nodes[i].worldPosition);
        for (var k = 0; k < 16; k++) {
          expect(a.nodes[i].modelMatrix[k], b.nodes[i].modelMatrix[k]);
        }
      }
    });

    test('live evaluations never move the station or the 26 targets', () {
      final engine = engineAtOrigin();
      final origin = engine.stationOrigin;
      final yaw = engine.stationYawRad;
      final positions = engine.nodes.map((n) => n.worldPosition).toList();

      // Wildly different camera poses, including ones that drift, lose
      // tracking and point at nothing in particular.
      for (var i = 0; i < 40; i++) {
        engine.evaluateFrame(
          pose: sample(
            Vec3(9.0 + i, 8, 7),
            lookingAt(i * 37.0, (i % 7) * 20.0 - 60.0),
            i * 33,
          ),
          trackingValid: i.isEven,
          activeTargetIndex: i % 26,
        );
      }

      expect(engine.stationOrigin, origin);
      expect(engine.stationYawRad, yaw);
      expect(engine.nodes.map((n) => n.worldPosition).toList(), positions);
    });
  });

  // ══ B. coordinate convention and yaw ══════════════════════════════════

  group('B. coordinates and yaw', () {
    test(
      'identity looks along -Z and the closed form agrees with rotate()',
      () {
        expect(Quat.identity.cameraForward.x, closeTo(0, 1e-15));
        expect(Quat.identity.cameraForward.y, closeTo(0, 1e-15));
        expect(Quat.identity.cameraForward.z, closeTo(-1, 1e-15));

        final cases = <Quat>[
          Quat.identity,
          lookingAt(0, 0),
          lookingAt(37, 12),
          lookingAt(-123, -44),
          lookingAt(179.5, 89),
          Quat.axisAngle(63, const Vec3(0.3, -0.8, 0.5)),
          Quat.axisAngle(-140, Vec3.unitY),
        ];
        for (final q in cases) {
          final viaRotate = q.rotate(const Vec3(0, 0, -1)).normalized;
          final viaClosedForm = q.cameraForward;
          expect(viaClosedForm.x, closeTo(viaRotate.x, 1e-12), reason: '$q');
          expect(viaClosedForm.y, closeTo(viaRotate.y, 1e-12), reason: '$q');
          expect(viaClosedForm.z, closeTo(viaRotate.z, 1e-12), reason: '$q');
        }
      },
    );

    test('the four cardinal facing directions land on the right axes', () {
      // forward
      var f = lookingAt(0, 0).cameraForward;
      expect(f.z, closeTo(-1, 1e-12));
      expect(Quat.yawOf(f), closeTo(0, 1e-9));
      // right
      f = lookingAt(90, 0).cameraForward;
      expect(f.x, closeTo(1, 1e-12));
      expect(Quat.yawOf(f), closeTo(90, 1e-9));
      // rear
      f = lookingAt(180, 0).cameraForward;
      expect(f.z, closeTo(1, 1e-12));
      expect(Quat.yawOf(f).abs(), closeTo(180, 1e-9));
      // left — the sign that inverts every instruction if it is wrong
      f = lookingAt(-90, 0).cameraForward;
      expect(f.x, closeTo(-1, 1e-12));
      expect(Quat.yawOf(f), closeTo(-90, 1e-9));
      // up
      f = lookingAt(0, 90).cameraForward;
      expect(f.y, closeTo(1, 1e-12));
    });

    test('calibration takes the yaw only, and discards pitch and roll', () {
      final engine = VioSphereEngine();
      final fromPitched = engine.extractCalibratedStationYawRad(
        cameraOrientation: lookingAt(40, 25),
      );
      expect(fromPitched.ok, isTrue);
      expect(fromPitched.yawRad * 180 / math.pi, closeTo(40, 1e-9));

      // Same yaw, upside-down roll and a different pitch: the same station yaw.
      final rolled = engine.extractCalibratedStationYawRad(
        cameraOrientation: lookingAt(40, -30, rollDeg: 150),
      );
      expect(rolled.ok, isTrue);
      expect(rolled.yawRad * 180 / math.pi, closeTo(40, 1e-9));

      // A calibration pose with pitch would tilt the whole arrangement if the
      // raw pose were frozen instead of its yaw.
      final calibrated = engine.calibrate(
        pose: sample(Vec3.zero, lookingAt(0, 25), 0),
      );
      expect(calibrated.ok, isTrue);
      expect(engine.nodes[0].direction.y, closeTo(0, 1e-12));
    });

    test('calibration near vertical is refused, not defaulted to yaw 0', () {
      final engine = VioSphereEngine();

      for (final pitch in <double>[85, -85, 89.9, 90, -90]) {
        final attempt = engine.calibrate(
          pose: sample(Vec3.zero, lookingAt(0, pitch), 0),
        );
        expect(attempt.ok, isFalse, reason: 'pitch $pitch');
        expect(attempt.reason, isNotNull);
        expect(attempt.reason!.isNotEmpty, isTrue);
        expect(engine.isCalibrated, isFalse, reason: 'pitch $pitch');
        expect(engine.nodes, isEmpty, reason: 'pitch $pitch');
        expect(engine.stationYawRad, isNull, reason: 'pitch $pitch');
      }

      // Just inside the limit is accepted, so the refusal is a threshold and
      // not "calibration never works".
      final ok = engine.calibrate(pose: sample(Vec3.zero, lookingAt(0, 45), 0));
      expect(ok.ok, isTrue);
      expect(engine.isCalibrated, isTrue);
      expect(engine.nodes.length, 26);
    });

    test(
      'an unusable quaternion is refused rather than treated as identity',
      () {
        final engine = VioSphereEngine();
        final zero = engine.calibrate(
          pose: sample(Vec3.zero, const Quat(0, 0, 0, 0), 0),
        );
        expect(zero.ok, isFalse);
        expect(engine.isCalibrated, isFalse);

        final nan = engine.calibrate(
          pose: sample(const Vec3(double.nan, 0, 0), lookingAt(0, 0), 0),
        );
        expect(nan.ok, isFalse);
        expect(engine.isCalibrated, isFalse);
      },
    );

    test('wrapToMinus180To180 takes the short way round', () {
      expect(wrapToMinus180To180(0), 0);
      expect(wrapToMinus180To180(179 + 2), closeTo(-179, 1e-12));
      expect(wrapToMinus180To180(350), closeTo(-10, 1e-12));
      expect(wrapToMinus180To180(-350), closeTo(10, 1e-12));
      expect(wrapToMinus180To180(180), 180);
      expect(wrapToMinus180To180(-180).abs(), 180);
      expect(wrapToMinus180To180(720 + 45), closeTo(45, 1e-12));
    });

    test('q and -q are the same rotation', () {
      final q = lookingAt(31, -17);
      final negated = Quat(-q.x, -q.y, -q.z, -q.w);
      expect(Quat.angularDistanceDeg(q, negated), closeTo(0, 1e-9));
      expect(Quat.angularDistanceDeg(q, Quat.identity), greaterThan(30));
    });
  });

  // ══ C. station drift ══════════════════════════════════════════════════

  group('C. station drift', () {
    double driftFor(Vec3 position) {
      final engine = engineAtOrigin();
      return engine
          .evaluateFrame(
            pose: sample(position, lookingAt(0, 0), 0),
            trackingValid: true,
            activeTargetIndex: 0,
          )
          .stationDriftM;
    }

    test('0.199 m is inside, and exactly 0.200 m is inside (inclusive)', () {
      final inside = engineAtOrigin();
      final at199 = inside.evaluateFrame(
        pose: sample(const Vec3(0.199, 0, 0), lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(at199.stationDriftM, closeTo(0.199, 1e-12));
      expect(at199.isWithinStation, isTrue);

      final at200 = engineAtOrigin().evaluateFrame(
        pose: sample(const Vec3(0.2, 0, 0), lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      // The comparison is `<=`, so the reported number and the verdict are
      // checked together: whatever the last bit of `0.2` works out to, the
      // gate agrees with the number the UI prints.
      expect(at200.stationDriftM, closeTo(0.2, 1e-12));
      expect(at200.isWithinStation, at200.stationDriftM <= 0.20);
    });

    test('0.2001 m and beyond is refused', () {
      for (final x in <double>[0.2001, 0.21, 0.5, 5.0]) {
        final engine = engineAtOrigin();
        final ev = engine.evaluateFrame(
          pose: sample(Vec3(x, 0, 0), lookingAt(0, 0), 0),
          trackingValid: true,
          activeTargetIndex: 0,
        );
        expect(ev.stationDriftM, closeTo(x, 1e-12));
        expect(ev.isWithinStation, isFalse, reason: '$x m');
        expect(ev.canTriggerShutter, isFalse, reason: '$x m');
      }
    });

    test('drift is a full 3D distance: vertical offsets count', () {
      // 0.19 m sideways and 0.10 m vertically: a horizontal-only measure would
      // call this 0.19 m (inside), the 3D distance is 0.2147 m (outside).
      expect(driftFor(const Vec3(0.19, 0.1, 0)), closeTo(0.214709, 1e-6));
      final engine = engineAtOrigin();
      final ev = engine.evaluateFrame(
        pose: sample(const Vec3(0.19, 0.1, 0), lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.isWithinStation, isFalse);

      // Purely vertical 0.21 m is also outside.
      final up = engineAtOrigin().evaluateFrame(
        pose: sample(const Vec3(0, 0.21, 0), lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(up.isWithinStation, isFalse);
    });

    test('purely vertical 0.199 m is inside', () {
      final engine = engineAtOrigin();
      final ev = engine.evaluateFrame(
        pose: sample(const Vec3(0, -0.199, 0), lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.isWithinStation, isTrue);
    });
  });

  // ══ D. angular alignment and guidance ════════════════════════════════

  group('D. alignment', () {
    /// The error from the station aiming exactly at [index].
    double errorAt(
      VioSphereEngine engine,
      int index,
      double yaw,
      double pitch,
    ) => engine
        .evaluateFrame(
          pose: sample(Vec3.zero, lookingAt(yaw, pitch), 0),
          trackingValid: true,
          activeTargetIndex: index,
        )
        .alignmentErrorDeg;

    test('looking straight at a target is 0° of error, for all 26 of them', () {
      final engine = engineAtOrigin();
      for (final node in engine.nodes) {
        final ev = engine.evaluateFrame(
          pose: sample(
            Vec3.zero,
            lookingAt(node.azimuthDeg, node.elevationDeg),
            0,
          ),
          trackingValid: true,
          activeTargetIndex: node.index,
        );
        expect(
          ev.alignmentErrorDeg,
          lessThan(1e-6),
          reason: 'node ${node.index} (${node.ringName})',
        );
        expect(ev.isAligned, isTrue, reason: 'node ${node.index}');
        expect(ev.targetDistanceM, closeTo(1.0, 1e-9));
      }
    });

    test('the whole 8-way ring and both poles align from the station', () {
      // Eight compass directions at every ring's 45° spacing, plus the two
      // exact poles: the orientations the user actually walks the station
      // through.
      final engine = engineAtOrigin();
      for (var i = 0; i < 8; i++) {
        expect(errorAt(engine, 8 + i, i * 45.0, 45), lessThan(1e-6));
        expect(errorAt(engine, 16 + i, i * 45.0, -45), lessThan(1e-6));
      }
      expect(errorAt(engine, 24, 0, 90), lessThan(1e-6));
      expect(errorAt(engine, 25, 0, -90), lessThan(1e-6));
    });

    test('the tolerance is a threshold, inclusive at 3.0°', () {
      final engine = engineAtOrigin();

      // Target 0 is straight ahead (yaw 0). A camera 2.9° off is inside.
      var ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(2.9, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.alignmentErrorDeg, closeTo(2.9, 1e-6));
      expect(ev.isAligned, isTrue);

      // 3.1° off is outside.
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(3.1, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.alignmentErrorDeg, closeTo(3.1, 1e-6));
      expect(ev.isAligned, isFalse);

      // Exactly at the limit: the reported number and the verdict must agree,
      // which is what "inclusive" means for a value computed through acos.
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(3.0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.alignmentErrorDeg, closeTo(3.0, 1e-6));
      expect(ev.isAligned, ev.alignmentErrorDeg <= 3.0);

      // The boundary is genuinely one-sided: 2.9999° inside, 3.0001° outside.
      final justIn = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(2.9999, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      final justOut = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(3.0001, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(justIn.isAligned, isTrue);
      expect(justOut.isAligned, isFalse);
    });

    test('the error is 3D: an off-axis pitch counts too', () {
      final engine = engineAtOrigin();
      // Target 0 is horizontal (elevation 0). 4° of pitch with no yaw error is
      // 4° of alignment error.
      final ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 4), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.alignmentErrorDeg, closeTo(4, 1e-6));
      expect(ev.isAligned, isFalse);
    });

    test('signed guidance points the short way, right positive', () {
      final engine = engineAtOrigin();

      // Facing forward, the target at azimuth +45 is to the right.
      var ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 1,
      );
      expect(ev.yawGuidanceDeg, closeTo(45, 1e-6));
      expect(ev.instruction, contains('right'));

      // Facing azimuth 40 with the target at 0, the turn is to the left.
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(40, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.yawGuidanceDeg, closeTo(-40, 1e-6));
      expect(ev.instruction, contains('left'));

      // Across the seam: facing -170° with the target at 180° (index 4, the
      // fifth horizontal node) is 10° to the left, not 350° to the right.
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(-170, 0), 0),
        trackingValid: true,
        activeTargetIndex: 4,
      );
      expect(ev.yawGuidanceDeg, closeTo(-10, 1e-6));
      expect(ev.yawGuidanceDeg.abs(), lessThanOrEqualTo(180));
    });

    test('pitch guidance is signed too', () {
      final engine = engineAtOrigin();
      // Upper ring node 8 is 45° up and straight ahead.
      var ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 8,
      );
      expect(ev.pitchGuidanceDeg, closeTo(45, 1e-6));
      expect(ev.instruction.toLowerCase(), contains('up'));

      // Nadir node 25 is straight down.
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 25,
      );
      expect(ev.pitchGuidanceDeg, closeTo(-90, 1e-6));
      expect(ev.instruction.toLowerCase(), contains('down'));
    });

    test('an invalid pose or a zero-length target direction cannot align', () {
      final engine = engineAtOrigin();

      // Sitting on the target: the direction to it has no length at all.
      final onTarget = engine.evaluateFrame(
        pose: sample(engine.nodes[0].worldPosition, lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(onTarget.targetDirection.isUsable, isFalse);
      expect(onTarget.alignmentErrorDeg, 180);
      expect(onTarget.isAligned, isFalse);
      expect(onTarget.canTriggerShutter, isFalse);

      // A zero quaternion, a NaN position and invalid tracking all refuse.
      final cases = <VioPoseSample>[
        sample(Vec3.zero, const Quat(0, 0, 0, 0), 0),
        sample(const Vec3(double.nan, 0, 0), lookingAt(0, 0), 0),
        sample(const Vec3(0, double.infinity, 0), lookingAt(0, 0), 0),
      ];
      for (final pose in cases) {
        final ev = engine.evaluateFrame(
          pose: pose,
          trackingValid: true,
          activeTargetIndex: 0,
        );
        expect(ev.isAligned, isFalse, reason: '$pose');
        expect(ev.alignmentErrorDeg, 180, reason: '$pose');
        expect(ev.isPoseSteady, isFalse, reason: '$pose');
        expect(ev.canTriggerShutter, isFalse, reason: '$pose');
      }

      final untracked = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 0),
        trackingValid: false,
        activeTargetIndex: 0,
      );
      expect(untracked.isAligned, isFalse);
      expect(untracked.canTriggerShutter, isFalse);
    });

    test('an out-of-range target index is refused, not clamped', () {
      final engine = engineAtOrigin();
      for (final index in <int>[-1, 26, 99]) {
        final ev = engine.evaluateFrame(
          pose: sample(Vec3.zero, lookingAt(0, 0), 0),
          trackingValid: true,
          activeTargetIndex: index,
        );
        expect(ev.stabilityRejection, 'no-station');
        expect(ev.canTriggerShutter, isFalse);
      }
    });
  });

  // ══ E. the stability window ═══════════════════════════════════════════

  group('E. the 600 ms window', () {
    test('the constants are the specified ones', () {
      expect(
        VioSphereConstants.stabilityWindow,
        const Duration(milliseconds: 600),
      );
      expect(VioSphereConstants.maxPoseAge, const Duration(milliseconds: 250));
      expect(VioSphereConstants.maxWindowOrientationDriftDeg, 1.5);
      expect(VioSphereConstants.maxWindowPositionDriftM, 0.03);
      expect(VioSphereConstants.maxAngularVelocityDegPerSec, 5.0);
      expect(VioSphereConstants.maxSampleGap.inMilliseconds, greaterThan(33));
      expect(
        VioSphereConstants.maxSampleGap.inMilliseconds,
        lessThan(VioSphereConstants.stabilityWindow.inMilliseconds),
      );
      expect(
        VioSphereConstants.maxPoseAge.inMilliseconds,
        greaterThan(VioSphereConstants.maxSampleGap.inMilliseconds),
      );
    });

    test('an empty history is not stability', () {
      final engine = engineAtOrigin();
      final ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 1000),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.isPoseSteady, isFalse);
      expect(ev.stabilityRejection, 'window');
      expect(ev.windowHeldMs, 0);
      expect(ev.isAligned, isTrue, reason: 'aligned and steady are separate');
      expect(ev.canTriggerShutter, isFalse);
    });

    test('a hold shorter than the window is refused', () {
      final engine = engineAtOrigin();
      // 10 samples at 33 ms = 297 ms of history.
      final ev = feed(engine, index: 0, rotation: lookingAt(0, 0), count: 10);
      expect(ev.windowHeldMs, lessThan(600));
      expect(ev.isPoseSteady, isFalse);
      expect(ev.stabilityRejection, 'window');
    });

    test(
      'a continuous 600 ms hold at 30 Hz is steady and arms the shutter',
      () {
        final engine = engineAtOrigin();
        final ev = feed(engine, index: 0, rotation: lookingAt(0, 0), count: 25);
        expect(ev.windowHeldMs, greaterThanOrEqualTo(600));
        expect(ev.stabilityRejection, isNull);
        expect(ev.isPoseSteady, isTrue);
        expect(ev.isAligned, isTrue);
        expect(ev.isWithinStation, isTrue);
        expect(ev.canTriggerShutter, isTrue);
        expect(ev.angularVelocityDegPerSec, closeTo(0, 1e-9));
      },
    );

    test('a hold at 60 Hz is steady too', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 5,
        rotation: lookingAt(150, 0),
        count: 50,
        stepMs: 16,
      );
      expect(ev.windowHeldMs, greaterThanOrEqualTo(600));
      expect(ev.isPoseSteady, isTrue);
    });

    test('two samples spanning the window are not a hold', () {
      final engine = engineAtOrigin();
      engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      final ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 600),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.isPoseSteady, isFalse);
      expect(ev.stabilityRejection, 'sample-gap');
      expect(ev.windowHeldMs, 0);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('an excessive gap inside a hold resets it', () {
      final engine = engineAtOrigin();
      // A good hold, then a 400 ms hole, then a good hold: the hole must break
      // the run rather than be interpolated over.
      feed(engine, index: 0, rotation: lookingAt(0, 0), count: 20);
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 5,
        startMs: 640 + 400,
      );
      expect(ev.isPoseSteady, isFalse);
      expect(ev.windowHeldMs, lessThan(600));
      expect(ev.canTriggerShutter, isFalse);
    });

    test('non-monotonic timestamps are refused', () {
      final engine = engineAtOrigin();
      feed(engine, index: 0, rotation: lookingAt(0, 0), count: 20);

      // Same timestamp as the last accepted sample.
      var ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 627),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.stabilityRejection, 'timestamp');
      expect(ev.isPoseSteady, isFalse);

      // The offending sample is not admitted as a baseline: the next sample
      // starts a fresh hold rather than inheriting a hold over a broken clock.
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 660),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.stabilityRejection, 'window');
      expect(ev.windowHeldMs, 0);

      // A timestamp that goes backwards.
      feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 20,
        startMs: 1000,
      );
      ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 1500),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.stabilityRejection, 'timestamp');
      expect(ev.isPoseSteady, isFalse);
    });

    test('a stale pose (250 ms old) is refused even mid-hold', () {
      final engine = engineAtOrigin();
      final steady = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
      );
      expect(steady.isPoseSteady, isTrue);

      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 1,
        startMs: 792 + 33,
        nowOffsetMs: 300,
      );
      expect(ev.stabilityRejection, 'measurement-age');
      expect(ev.isPoseSteady, isFalse);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('a pose exactly at the age limit is still fresh', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        nowOffsetMs: 250,
      );
      expect(ev.isPoseSteady, isTrue);
    });

    test('orientation drift beyond 1.5° across the window fails', () {
      final engine = engineAtOrigin();
      // 0.1° per 33 ms sample: 3.03 °/s, under the 5 °/s gate, but 1.8° of
      // cumulative drift over the window — over the window's 1.5° budget.
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        rotationFor: (i) => lookingAt(0, 0, rollDeg: i * 0.1),
      );
      expect(ev.stabilityRejection, 'orientation-drift');
      expect(ev.isPoseSteady, isFalse);
      expect(ev.angularVelocityDegPerSec, lessThanOrEqualTo(5.0));
      expect(ev.alignmentErrorDeg, lessThan(3.0), reason: 'still aimed at it');
    });

    test('orientation drift inside 1.5° stays steady', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        rotationFor: (i) => lookingAt(0, 0, rollDeg: i * 0.02),
      );
      expect(ev.isPoseSteady, isTrue);
      expect(ev.stabilityRejection, isNull);
    });

    test('position drift beyond 0.03 m across the window fails', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        positionFor: (i) => Vec3(0.04 * (i / 24.0), 0, 0),
      );
      expect(ev.stabilityRejection, 'position-drift');
      expect(ev.isPoseSteady, isFalse);
      expect(ev.isWithinStation, isTrue, reason: '4 cm is not station drift');
    });

    test('position drift inside 0.03 m stays steady', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        positionFor: (i) => Vec3(0.02 * (i / 24.0), 0, 0),
      );
      expect(ev.isPoseSteady, isTrue);
    });

    test('an angular rate above 5 °/s fails', () {
      final engine = engineAtOrigin();
      // 0.3° per 33 ms = 9.1 °/s.
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        rotationFor: (i) => lookingAt(0, 0, rollDeg: i * 0.3),
      );
      expect(ev.stabilityRejection, 'angular-rate');
      expect(ev.isPoseSteady, isFalse);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('a rate under 5 °/s does not fail', () {
      final engine = engineAtOrigin();
      // 0.05° per 33 ms sample: 1.5 °/s, and 0.95° of drift across the window —
      // under both the rate and the drift budgets.
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        rotationFor: (i) => lookingAt(0, 0, rollDeg: i * 0.05),
      );
      expect(ev.isPoseSteady, isTrue);
      expect(ev.angularVelocityDegPerSec, lessThanOrEqualTo(5.0));
    });

    test('losing tracking resets the hold', () {
      final engine = engineAtOrigin();
      final steady = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
      );
      expect(steady.isPoseSteady, isTrue);

      final lost = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 1,
        startMs: 792 + 33,
        trackingValid: false,
      );
      expect(lost.isPoseSteady, isFalse);
      expect(lost.stabilityRejection, 'invalid-pose');
      expect(lost.canTriggerShutter, isFalse);

      // And the window restarts: five samples after the gap are not enough.
      final resumed = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 5,
        startMs: 792 + 66,
      );
      expect(resumed.isPoseSteady, isFalse);
      expect(resumed.stabilityRejection, 'window');
    });

    test('resetStability clears the hold', () {
      final engine = engineAtOrigin();
      feed(engine, index: 0, rotation: lookingAt(0, 0), count: 25);
      expect(engine.heldMs, greaterThanOrEqualTo(600));
      engine.resetStability();
      expect(engine.historyLength, 0);
      expect(engine.heldMs, 0);
      final ev = feed(engine, index: 0, rotation: lookingAt(0, 0), count: 1);
      expect(ev.isPoseSteady, isFalse);
    });
  });

  // ══ F. the shutter expression ═════════════════════════════════════════

  group('F. shutter', () {
    test('every condition must hold together', () {
      final engine = engineAtOrigin();
      final ev = feed(engine, index: 0, rotation: lookingAt(0, 0), count: 25);
      expect(ev.isAligned, isTrue);
      expect(ev.isWithinStation, isTrue);
      expect(ev.isPoseSteady, isTrue);
      expect(ev.canTriggerShutter, isTrue);
    });

    test('invalid tracking alone blocks it', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        trackingValid: false,
      );
      expect(ev.canTriggerShutter, isFalse);
    });

    test('a target already captured alone blocks it', () {
      final engine = engineAtOrigin();
      feed(engine, index: 0, rotation: lookingAt(0, 0), count: 25);
      engine.markCaptured(0);
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 1,
        startMs: 792 + 33,
      );
      expect(ev.isAligned, isTrue);
      expect(ev.isPoseSteady, isTrue);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('a save in progress alone blocks it', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        isSavingInProgress: true,
      );
      expect(ev.isPoseSteady, isTrue);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('a latched failure alone blocks it', () {
      final engine = engineAtOrigin();
      final ev = feed(
        engine,
        index: 0,
        rotation: lookingAt(0, 0),
        count: 25,
        isFailureLatched: true,
      );
      expect(ev.canTriggerShutter, isFalse);
    });

    test(
      'leaving the station alone blocks it, while still aimed and steady',
      () {
        final engine = engineAtOrigin();
        // Stepping 0.5 m along the target's own direction: 0.5 m of drift, and
        // the target is still dead ahead.
        final ev = feed(
          engine,
          index: 0,
          rotation: lookingAt(0, 0),
          count: 25,
          position: const Vec3(0, 0, -0.5),
        );
        expect(ev.isAligned, isTrue);
        expect(ev.isPoseSteady, isTrue);
        expect(ev.isWithinStation, isFalse);
        expect(ev.canTriggerShutter, isFalse);
      },
    );

    test('misalignment alone blocks it, while still steady', () {
      final engine = engineAtOrigin();
      // Perfectly still, aimed 45° away from target 1 (yaw 45° in this layout).
      final ev = feed(engine, index: 1, rotation: lookingAt(0, 0), count: 25);
      expect(ev.isPoseSteady, isTrue);
      expect(ev.isAligned, isFalse);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('an unfinished hold alone blocks it, while still aimed', () {
      final engine = engineAtOrigin();
      final ev = feed(engine, index: 0, rotation: lookingAt(0, 0), count: 5);
      expect(ev.isAligned, isTrue);
      expect(ev.isPoseSteady, isFalse);
      expect(ev.canTriggerShutter, isFalse);
    });

    test('once captured, no later frame re-arms the shutter', () {
      // The engine-level half of the single-flight guard: the controller's
      // in-flight latch is the other half, and this is what stops the shutter
      // re-firing on every frame of a hold after a capture.
      final engine = engineAtOrigin();
      var triggers = 0;
      for (var i = 0; i < 40; i++) {
        final ev = feed(
          engine,
          index: 0,
          rotation: lookingAt(0, 0),
          count: 1,
          startMs: i * 33,
        );
        if (ev.canTriggerShutter) {
          triggers++;
          // The capture lifecycle: the target is only marked captured after
          // the photograph is validated and saved, which is why the loop marks
          // it here rather than upstream.
          engine.markCaptured(0);
        }
      }
      expect(triggers, 1);
    });

    test('the shutter needs a target that exists', () {
      final engine = VioSphereEngine();
      final ev = engine.evaluateFrame(
        pose: sample(Vec3.zero, lookingAt(0, 0), 0),
        trackingValid: true,
        activeTargetIndex: 0,
      );
      expect(ev.canTriggerShutter, isFalse);
      expect(ev.stabilityRejection, 'no-station');
    });
  });
}
