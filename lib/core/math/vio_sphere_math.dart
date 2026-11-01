/// VIO sphere mathematics for the 360° capture module.
///
/// **Pure Dart, no Flutter, no platform channels, no clock.** Everything the
/// capture runtime asks about the geometry — where the 26 world-locked targets
/// are, how far the phone may wander from the station, how large the alignment
/// error is, whether the pose has been steady for the whole window — is decided
/// here, and is therefore assertable by `flutter test` without a device.
///
/// ## The two phases, and why they are separated
///
/// **Phase A — calibration, once per session.** [VioSphereEngine.calibrate]
/// freezes the station origin and the gravitational-horizontal station yaw, then
/// generates the 26 target world positions exactly once. Nothing below can move
/// them afterwards: [stationOrigin], [stationYawRad] and [nodes] are written
/// once and read-only after that.
///
/// **Phase B — a live frame evaluation per pose update.** [evaluateFrame] reads
/// the frozen reference and the incoming pose and returns a
/// [FrameGateEvaluation]. It never writes to the station or to a target, so
/// "targets drift because they are recomputed from the live camera pose" is not
/// a bug that can be reintroduced here without deleting a method.
///
/// ## Coordinate convention (ARCore / OpenGL, right-handed)
///
/// * `+X` right, `+Y` up (gravity-aligned), `-Z` forward.
/// * A camera quaternion rotates camera-local axes into that world, so the
///   optical axis is `q.rotate(0, 0, -1)` ([Quat.cameraForward]).
/// * Azimuth is `atan2(f.x, -f.z)`: forward is 0°, right is +90°, rear is 180°,
///   left is -90°.
///
/// The components are `(x, y, z, w)`, which is the order ARCore's
/// `Pose.getRotationQuaternion` fills and the order this app's Kotlin host
/// forwards to Dart (`ArCoreSessionController` — `qx, qy, qz, qw`). The native
/// renderer consumes the same right-handed world with no Y/Z negation, so the
/// matrices built here are handed to Kotlin unchanged.
///
/// **This engine does not use the older `arCameraPoseToCaptureWorld` mirror.**
/// That conversion exists because the rest of the app's `Quat` convention puts
/// the optical axis on `+Z`; ARCore already puts it on `-Z`, which is the
/// convention this specification asks for, so the raw ARCore quaternion is used
/// directly and no extra rotation is applied.
library;

import 'dart:math' as math;
import 'dart:typed_data';

/// The constants of the specification.
///
/// Every threshold lives here once. The UI, the controller and the tests read
/// the same numbers, so a screen that says "≤ 0.20 m" and a gate that refuses at
/// 0.21 m cannot disagree.
abstract final class VioSphereConstants {
  /// How far in front of the station every target sits, in metres.
  static const double targetRadiusM = 1.0;

  /// Maximum allowed station-centre drift, in metres. **Inclusive**: exactly
  /// 0.20 m is inside, 0.2000001 m is not.
  ///
  /// Documentation recommendation (not active): 0.10 m for tighter station
  /// discipline on high-end devices; active value kept at 0.20 m.
  static const double maxStationDriftM = 0.20;

  /// Maximum allowed 3D alignment error, in degrees. **Inclusive**.
  ///
  /// The shutter's strict, instantaneous alignment limit. Hysteresis belongs
  /// to the capture hold in the controller; it must never widen this final
  /// acceptance test.
  ///
  /// Documentation recommendation (not active): 5.0° for looser aiming on
  /// small screens; active value kept at 3.0°.
  static const double maxAlignmentErrorDeg = 3.0;

  /// How long the pose must hold still before the shutter may fire.
  ///
  /// A full 600 ms of valid, stable aiming is required before capture.
  ///
  /// Documentation recommendation (not active): 500 ms for snappier capture on
  /// steady hands; active value kept at 600 ms.
  static const Duration stabilityWindow = Duration(milliseconds: 600);

  /// The oldest pose that may still be acted on, in milliseconds.
  static const Duration maxPoseAge = Duration(milliseconds: 250);

  /// How far the orientation may wander across the window, in degrees.
  ///
  static const double maxWindowOrientationDriftDeg = 1.5;

  /// How far the position may wander across the window, in metres.
  ///
  /// Documentation recommendation (not active): 0.05 m; active value kept at
  /// 0.03 m.
  static const double maxWindowPositionDriftM = 0.03;

  /// The fastest rotation that still counts as "held still", in degrees/second.
  ///
  /// Measured over the 100 ms rate baseline rather than adjacent frames, so
  /// small per-frame VIO noise does not masquerade as a deliberate turn.
  static const double maxAngularVelocityDegPerSec = 5.0;

  /// The largest gap between two consecutive poses that may still count as a
  /// continuous hold, in milliseconds.
  ///
  /// ARCore's pose stream runs at roughly 30–60 Hz, i.e. a 16.7–33.3 ms
  /// interval. A gap of 150 ms is therefore four to nine missed frames: long
  /// enough to tolerate a scheduling stall, short enough that a dropped stretch
  /// of samples cannot silently be interpolated over and then claimed as a
  /// 600 ms hold. Without this gate, two samples 649 ms apart would "span the
  /// window" while containing almost no evidence.
  static const Duration maxSampleGap = Duration(milliseconds: 150);

  /// The baseline the angular rate is measured over, in milliseconds.
  ///
  /// **Not the frame interval.** Differencing two consecutive 30 Hz poses turns
  /// ±0.35° of tracker noise into 21 °/s — four times the gate's budget — so a
  /// perfectly still phone would be refused forever. Over 100 ms the same noise
  /// averages to a few °/s, while a genuine turn is still caught within 100 ms.
  static const Duration rateBaseline = Duration(milliseconds: 100);

  /// How much horizontal forward component calibration needs.
  ///
  /// `sin(10°)`: beyond 10° from vertical the projection of the optical axis
  /// onto the horizontal plane is too short for its direction to mean anything,
  /// and the yaw of the whole 26-target arrangement would be decided by hand
  /// shake. Calibration is refused instead of silently defaulting to zero.
  static const double minCalibrationHorizontalComponent = 0.17364817766693041;

  /// Horizontal ring: 8 targets at 0°, every 45°.
  static const int horizontalCount = 8;
  static const double horizontalAzimuthStepDeg = 45.0;

  /// Upper and lower rings: 8 targets each at ±45°, every 45°.
  static const int upperCount = 8;
  static const int lowerCount = 8;
  static const double upperElevationDeg = 45.0;
  static const double lowerElevationDeg = -45.0;
  static const double ringAzimuthStepDeg = 45.0;

  /// The two exact poles.
  ///
  /// ±90° puts the zenith and nadir where `directionFromYawPitch` yields the
  /// pure ±Y axis. Yaw is mathematically undefined there, which is not a
  /// problem anywhere it is consumed: the direction vector is exact,
  /// `atan2(0, 0)` returns 0 rather than NaN, and the guidance path recognises
  /// a pole by pitch (`CaptureTarget.isPole` at ≥ 89.999°) and offers
  /// vertical-only wording instead of an azimuth that would mean nothing.
  static const double zenithElevationDeg = 90.0;
  static const double nadirElevationDeg = -90.0;

  /// 8 + 8 + 8 + 1 + 1.
  static const int nodeCount = 26;

  /// Index boundaries:
  /// horizontal 0–7, upper 8–15, lower 16–23, zenith 24, nadir 25.
  static const int upperFirstIndex = horizontalCount;
  static const int lowerFirstIndex = upperFirstIndex + upperCount;
  static const int zenithIndex = lowerFirstIndex + lowerCount;
  static const int nadirIndex = zenithIndex + 1;

  static const String horizontalRingName = 'Horizontal';
  static const String upperRingName = 'Upper';
  static const String lowerRingName = 'Lower';
  static const String zenithRingName = 'Zenith';
  static const String nadirRingName = 'Nadir';
}

/// An immutable 3D vector in AR world metres.
///
/// Every operation is total: [normalized] returns a zero vector rather than
/// dividing by zero, and [isUsable] reports whether a direction means anything.
/// Nothing in this file ever produces `NaN` or an infinity — the caller checks
/// [isFinite] on anything that came from outside.
class Vec3 {
  final double x;
  final double y;
  final double z;

  const Vec3(this.x, this.y, this.z);

  static const Vec3 zero = Vec3(0, 0, 0);
  static const Vec3 unitX = Vec3(1, 0, 0);
  static const Vec3 unitY = Vec3(0, 1, 0);
  static const Vec3 unitZ = Vec3(0, 0, 1);

  Vec3 operator +(Vec3 o) => Vec3(x + o.x, y + o.y, z + o.z);
  Vec3 operator -(Vec3 o) => Vec3(x - o.x, y - o.y, z - o.z);
  Vec3 operator -() => Vec3(-x, -y, -z);
  Vec3 operator *(double s) => Vec3(x * s, y * s, z * s);

  double dot(Vec3 o) => x * o.x + y * o.y + z * o.z;

  Vec3 cross(Vec3 o) =>
      Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);

  double get length => math.sqrt(x * x + y * y + z * z);

  double get lengthSquared => x * x + y * y + z * z;

  /// The unit vector, or [zero] when the length is numerically zero.
  Vec3 get normalized {
    final l = length;
    if (l < 1e-12) return zero;
    return Vec3(x / l, y / l, z / l);
  }

  /// The horizontal (XZ) length, i.e. the magnitude of the projection onto the
  /// plane perpendicular to gravity.
  double get horizontalLength => math.sqrt(x * x + z * z);

  bool get isFinite => x.isFinite && y.isFinite && z.isFinite;

  /// A direction that can actually be used: finite and not (numerically) zero.
  bool get isUsable => isFinite && length > 1e-9;

  List<double> toList() => <double>[x, y, z];

  static Vec3 fromList(List<double> v) =>
      Vec3(v[0].toDouble(), v[1].toDouble(), v[2].toDouble());

  /// The distance between two points, i.e. `|a - b|`.
  static double distance(Vec3 a, Vec3 b) => (a - b).length;

  @override
  String toString() =>
      '(${x.toStringAsFixed(4)}, ${y.toStringAsFixed(4)}, ${z.toStringAsFixed(4)})';

  @override
  bool operator ==(Object other) =>
      other is Vec3 && other.x == x && other.y == y && other.z == z;

  @override
  int get hashCode => Object.hash(x, y, z);
}

/// A unit quaternion with components in `(x, y, z, w)` order.
///
/// The order is not a detail: ARCore fills `getRotationQuaternion` as
/// `[x, y, z, w]` and the Kotlin host forwards `qx, qy, qz, qw` in that order,
/// so reading them as `(w, x, y, z)` would silently produce a valid-looking but
/// wrong rotation.
class Quat {
  final double x;
  final double y;
  final double z;
  final double w;

  const Quat(this.x, this.y, this.z, this.w);

  static const Quat identity = Quat(0, 0, 0, 1);

  /// The smallest squared length that still earns a normalisation, shared with
  /// the platform payload parser.
  ///
  /// The parser refuses a quaternion at or below this cut *before* it
  /// normalises (dividing by a near-zero length turns rounding dust into an
  /// arbitrary direction), and this class accepts one only above the same cut,
  /// so "usable" means one thing across both layers. Squared length is compared
  /// instead of length: no sqrt per check, and overflow stays detectable — a
  /// sum of squares becomes `Infinity` when the components' exact length is
  /// perfectly fine but double cannot hold its square.
  static const double degenerateLengthSquared = 1e-18;

  double get lengthSquared => x * x + y * y + z * z + w * w;

  bool get isFinite => x.isFinite && y.isFinite && z.isFinite && w.isFinite;

  /// A quaternion that can be used as a rotation: finite, not numerically
  /// zero, and not overflowing when squared.
  ///
  /// The product is checked separately from the components because finite
  /// components can still sum to `Infinity` when squared (`1e200 * 1e200`), and
  /// an unguarded `> [degenerateLengthSquared]` evaluates true for `Infinity`,
  /// which would pass a quaternion whose normalisation yields nothing.
  bool get isUsable {
    final ls = lengthSquared;
    return isFinite && ls.isFinite && ls > degenerateLengthSquared;
  }

  /// The unit quaternion, or [identity] when the input is unusable.
  ///
  /// Normalising a near-zero quaternion amplifies noise into an arbitrary
  /// rotation, so the degenerate case is answered with the identity rather than
  /// with a division by ~0. Callers that must reject rather than substitute
  /// check [isUsable] first.
  Quat get normalized {
    if (!isFinite) return identity;
    final l = math.sqrt(lengthSquared);
    if (l < 1e-9) return identity;
    return Quat(x / l, y / l, z / l, w / l);
  }

  Quat get conjugate => Quat(-x, -y, -z, w);

  /// `this * o`: apply [o] first, then `this`.
  Quat operator *(Quat o) => Quat(
    w * o.x + x * o.w + y * o.z - z * o.y,
    w * o.y - x * o.z + y * o.w + z * o.x,
    w * o.z + x * o.y - y * o.x + z * o.w,
    w * o.w - x * o.x - y * o.y - z * o.z,
  );

  /// A rotation of [degrees] about [axis], right-handed.
  factory Quat.axisAngle(double degrees, Vec3 axis) {
    final a = axis.normalized;
    if (!a.isUsable) return identity;
    final half = degrees * math.pi / 360;
    final s = math.sin(half);
    return Quat(a.x * s, a.y * s, a.z * s, math.cos(half));
  }

  /// Rotates [v] by this quaternion.
  ///
  /// The explicit matrix form of `q v q*`, which is also the form
  /// [cameraForward] is cross-checked against.
  Vec3 rotate(Vec3 v) {
    final q = normalized;
    final qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    final m00 = 1 - 2 * (qy * qy + qz * qz);
    final m01 = 2 * (qx * qy - qw * qz);
    final m02 = 2 * (qx * qz + qw * qy);
    final m10 = 2 * (qx * qy + qw * qz);
    final m11 = 1 - 2 * (qx * qx + qz * qz);
    final m12 = 2 * (qy * qz - qw * qx);
    final m20 = 2 * (qx * qz - qw * qy);
    final m21 = 2 * (qy * qz + qw * qx);
    final m22 = 1 - 2 * (qx * qx + qy * qy);
    return Vec3(
      m00 * v.x + m01 * v.y + m02 * v.z,
      m10 * v.x + m11 * v.y + m12 * v.z,
      m20 * v.x + m21 * v.y + m22 * v.z,
    );
  }

  /// The camera's optical axis in world space: `q.rotate(0, 0, -1)`.
  ///
  /// Written as the specification's closed form, which is the same expression
  /// [rotate] reduces to for `(0, 0, -1)` — the equality is asserted in
  /// `test/vio_sphere_math_test.dart` rather than assumed, because a sign slip
  /// here inverts every yaw downstream.
  Vec3 get cameraForward {
    final q = normalized;
    if (!q.isUsable) return Vec3.zero;
    return Vec3(
      -2 * (q.x * q.z + q.w * q.y),
      -2 * (q.y * q.z - q.w * q.x),
      -1 + 2 * (q.x * q.x + q.y * q.y),
    ).normalized;
  }

  /// The yaw of a direction *vector*, in degrees, in this file's convention:
  /// forward (`-Z`) is 0, right (`+X`) is +90, rear (`+Z`) is 180, left is -90.
  static double yawOf(Vec3 direction) =>
      math.atan2(direction.x, -direction.z) * 180 / math.pi;

  /// The pitch of a direction vector, in degrees: +90 is straight up.
  static double pitchOf(Vec3 direction) {
    final n = direction.normalized;
    if (!n.isUsable) return 0;
    return math.asin(clampDouble(n.y, -1, 1)) * 180 / math.pi;
  }

  /// This quaternion's yaw, i.e. the yaw of its optical axis, in degrees.
  double get cameraYawDeg => yawOf(cameraForward);

  /// The angle between two rotations, in degrees.
  ///
  /// `q` and `-q` are the same rotation, so the absolute value of the dot
  /// product is used: without it, a sign flip in the pose stream would read as a
  /// 360° turn and destroy the stability window forever.
  static double angularDistanceDeg(Quat a, Quat b) {
    final na = a.normalized, nb = b.normalized;
    if (!na.isUsable || !nb.isUsable) return double.nan;
    final dot = (na.x * nb.x + na.y * nb.y + na.z * nb.z + na.w * nb.w).abs();
    // atan2 form: stable for angles near 0 and near 180.
    final sinHalf = math.sqrt(math.max(0, 1 - dot * dot));
    return 2 * math.atan2(sinHalf, dot.clamp(0.0, 1.0)) * 180 / math.pi;
  }

  List<double> toList() => <double>[x, y, z, w];

  static Quat fromList(List<double> v) =>
      Quat(v[0].toDouble(), v[1].toDouble(), v[2].toDouble(), v[3].toDouble());

  @override
  String toString() =>
      '(${x.toStringAsFixed(4)}, ${y.toStringAsFixed(4)}, '
      '${z.toStringAsFixed(4)}, ${w.toStringAsFixed(4)})';
}

/// One ARCore pose as the engine consumes it.
///
/// [timestampNanoseconds] is ARCore's own `Frame.timestamp`, which is
/// **nanoseconds**, monotonic within a session, and the only clock the stability
/// window may use: a wall clock in the engine would make the gate untestable.
class VioPoseSample {
  final Vec3 position;
  final Quat rotation;
  final int timestampNanoseconds;

  const VioPoseSample({
    required this.position,
    required this.rotation,
    required this.timestampNanoseconds,
  });

  Duration get timestamp =>
      Duration(microseconds: timestampNanoseconds ~/ 1000);

  bool get isFinite => position.isFinite && rotation.isFinite;

  @override
  String toString() =>
      'VioPoseSample($position, $rotation, ${timestamp.inMilliseconds}ms)';
}

/// One of the 26 world-locked capture targets.
///
/// [worldPosition] and [modelMatrix] are computed **once**, at calibration, from
/// the frozen station reference. They are not recomputed from a live pose, and
/// nothing in this file ever writes them again — which is what "world-locked"
/// means here.
class TourTargetNode {
  /// 0-based, fixed by the layout:
  /// horizontal 0–7, upper 8–15, lower 16–23, zenith 24, nadir 25.
  final int index;

  /// `Horizontal`, `Upper`, `Lower`, `Zenith` or `Nadir`.
  final String ringName;

  /// Elevation above the horizontal plane, in degrees.
  final double elevationDeg;

  /// Station-relative azimuth, in degrees: 0 is the calibrated forward
  /// direction, positive turns right.
  final double azimuthDeg;

  /// Absolute azimuth in the world, i.e. `stationYaw + azimuth`, in degrees.
  final double worldAzimuthDeg;

  /// Unit direction from the station, in world coordinates.
  final Vec3 direction;

  /// AR world metres. Exactly `radiusM` from the station origin.
  final Vec3 worldPosition;

  /// Column-major 4×4 model matrix for the GLB, with the model's `+Z` face
  /// pointing back at the station.
  final Float64List modelMatrix;

  /// Whether a photograph for this target has been validated and saved.
  final bool captured;

  const TourTargetNode({
    required this.index,
    required this.ringName,
    required this.elevationDeg,
    required this.azimuthDeg,
    required this.worldAzimuthDeg,
    required this.direction,
    required this.worldPosition,
    required this.modelMatrix,
    this.captured = false,
  });

  bool get isZenith => index == VioSphereConstants.zenithIndex;
  bool get isNadir => index == VioSphereConstants.nadirIndex;
  bool get isPole => isZenith || isNadir;

  /// The 1-based target id the capture module stores and the UI shows.
  int get targetId => index + 1;

  TourTargetNode copyWith({bool? captured}) => TourTargetNode(
    index: index,
    ringName: ringName,
    elevationDeg: elevationDeg,
    azimuthDeg: azimuthDeg,
    worldAzimuthDeg: worldAzimuthDeg,
    direction: direction,
    worldPosition: worldPosition,
    modelMatrix: modelMatrix,
    captured: captured ?? this.captured,
  );

  @override
  String toString() =>
      'TourTargetNode($index, $ringName, ${azimuthDeg.toStringAsFixed(0)}° / '
      '${elevationDeg.toStringAsFixed(0)}°, $worldPosition)';
}

/// What one live frame evaluation found.
///
/// Every field is derived from the frozen station reference and the pose that
/// was passed in; none of them is stored back into the engine.
class FrameGateEvaluation {
  /// The 3D distance from the live camera to the frozen station origin.
  final double stationDriftM;

  /// [VioSphereConstants.maxStationDriftM] applied inclusively.
  final bool isWithinStation;

  /// 0-based index of the target this evaluation is about.
  final int activeTargetIndex;

  /// Distance from the camera to the active target, in metres.
  final double targetDistanceM;

  /// The 3D angle between the optical axis and the direction to the target.
  final double alignmentErrorDeg;

  /// [VioSphereConstants.maxAlignmentErrorDeg] applied inclusively.
  final bool isAligned;

  /// Signed yaw guidance in `(-180, 180]`: positive means "turn right".
  final double yawGuidanceDeg;

  /// Signed pitch guidance: positive means "look up".
  final double pitchGuidanceDeg;

  /// Whether the 600 ms window is satisfied.
  final bool isPoseSteady;

  /// Why it is not, or `null` when it is: one of `window`, `sample-gap`,
  /// `timestamp`, `invalid-pose`, `measurement-age`, `angular-rate`,
  /// `orientation-drift`, `position-drift`, `alignment`.
  final String? stabilityRejection;

  /// How long the hold has lasted, in milliseconds.
  final int windowHeldMs;

  /// The angular rate over [VioSphereConstants.rateBaseline], if measurable.
  final double? angularVelocityDegPerSec;

  /// The camera's optical axis, in world coordinates.
  final Vec3 cameraForward;

  /// The unit direction from the camera to the target.
  final Vec3 targetDirection;

  /// The specification's shutter expression, evaluated in one place:
  ///
  /// ```
  /// canTriggerShutter = isArTracking && !activeTarget.isCaptured &&
  ///   !isSavingInProgress && !isFailureLatched &&
  ///   (!enforceStationDrift || isWithinStationDriftGate) &&
  ///   isAligned && isPoseSteady
  /// ```
  final bool canTriggerShutter;

  const FrameGateEvaluation({
    required this.stationDriftM,
    required this.isWithinStation,
    required this.activeTargetIndex,
    required this.targetDistanceM,
    required this.alignmentErrorDeg,
    required this.isAligned,
    required this.yawGuidanceDeg,
    required this.pitchGuidanceDeg,
    required this.isPoseSteady,
    required this.stabilityRejection,
    required this.windowHeldMs,
    required this.angularVelocityDegPerSec,
    required this.cameraForward,
    required this.targetDirection,
    required this.canTriggerShutter,
  });

  /// "Turn right 20°" / "Turn left 8°" / "Tilt up 12°" / "Hold still".
  String get instruction {
    if (isAligned) {
      return isPoseSteady ? 'Hold still' : 'Hold still…';
    }
    final parts = <String>[];
    final yaw = yawGuidanceDeg;
    final pitch = pitchGuidanceDeg;
    if (yaw.abs() > 90) {
      parts.add(
        'Turn around ${yaw > 0 ? 'to the right' : 'to the left'} '
        '${(180 - yaw.abs()).abs().round()}°',
      );
    } else if (yaw > 1) {
      parts.add('Turn right ${yaw.abs().round()}°');
    } else if (yaw < -1) {
      parts.add('Turn left ${yaw.abs().round()}°');
    }
    if (pitch > 1) {
      parts.add('tilt up ${pitch.abs().round()}°');
    } else if (pitch < -1) {
      parts.add('tilt down ${pitch.abs().round()}°');
    }
    if (parts.isEmpty) return 'Hold still…';
    final s = parts.join(' and ');
    return '${s[0].toUpperCase()}${s.substring(1)}';
  }

  @override
  String toString() =>
      'FrameGateEvaluation(target $activeTargetIndex, drift '
      '${stationDriftM.toStringAsFixed(3)}m, error '
      '${alignmentErrorDeg.toStringAsFixed(2)}°, steady=$isPoseSteady, '
      'shutter=$canTriggerShutter)';
}

/// The result of a calibration attempt.
class CalibrationResult {
  /// Whether the station reference was established.
  final bool ok;

  /// Why not, in words for the user.
  final String? reason;

  /// The frozen station origin, when [ok].
  final Vec3? stationOrigin;

  /// The frozen gravitational-horizontal station yaw in radians, when [ok].
  final double? stationYawRad;

  /// The 26 world-locked targets, when [ok].
  final List<TourTargetNode> nodes;

  const CalibrationResult({
    required this.ok,
    this.reason,
    this.stationOrigin,
    this.stationYawRad,
    this.nodes = const <TourTargetNode>[],
  });
}

/// The station reference and the 26 targets, plus the live gate.
///
/// One instance per capture session. [calibrate] is the only writer.
class VioSphereEngine {
  VioSphereEngine({
    this.radiusM = VioSphereConstants.targetRadiusM,
    this.window = VioSphereConstants.stabilityWindow,
    this.maxPoseAge = VioSphereConstants.maxPoseAge,
    this.maxStationDriftM = VioSphereConstants.maxStationDriftM,
    this.maxAlignmentErrorDeg = VioSphereConstants.maxAlignmentErrorDeg,
    this.maxWindowOrientationDriftDeg =
        VioSphereConstants.maxWindowOrientationDriftDeg,
    this.maxWindowPositionDriftM = VioSphereConstants.maxWindowPositionDriftM,
    this.maxAngularVelocityDegPerSec =
        VioSphereConstants.maxAngularVelocityDegPerSec,
    this.maxSampleGap = VioSphereConstants.maxSampleGap,
    this.rateBaseline = VioSphereConstants.rateBaseline,
  });

  final double radiusM;
  final Duration window;
  final Duration maxPoseAge;
  final double maxStationDriftM;
  final double maxAlignmentErrorDeg;
  final double maxWindowOrientationDriftDeg;
  final double maxWindowPositionDriftM;
  final double maxAngularVelocityDegPerSec;
  final Duration maxSampleGap;
  final Duration rateBaseline;

  // ── the frozen station reference ──────────────────────────────────────

  Vec3? _stationOrigin;
  double? _stationYawRad;
  List<TourTargetNode> _nodes = const <TourTargetNode>[];

  /// The frozen station origin, or `null` before calibration.
  Vec3? get stationOrigin => _stationOrigin;

  /// The frozen station yaw in radians, or `null` before calibration.
  double? get stationYawRad => _stationYawRad;

  /// The frozen station yaw in degrees, or `null` before calibration.
  double? get stationYawDeg =>
      _stationYawRad == null ? null : _stationYawRad! * 180 / math.pi;

  /// The 26 world-locked targets. Empty before calibration.
  List<TourTargetNode> get nodes => _nodes;

  bool get isCalibrated => _stationOrigin != null && _stationYawRad != null;

  // ── Phase A: calibration ──────────────────────────────────────────────

  /// Extracts the gravitational-horizontal station yaw from a calibrated
  /// camera quaternion.
  ///
  /// Only the **yaw** is taken. The camera's pitch and roll are deliberately
  /// discarded: freezing the raw pose would tilt the entire sphere by whatever
  /// angle the phone happened to be held at, so "calibrate while looking 20°
  /// up" would put the horizontal ring at +20° and "level" would stop meaning
  /// level. The rings are defined against gravity, not against the phone.
  ///
  /// Returns `ok: false` with a stated reason when the optical axis points too
  /// close to straight up or down: there the horizontal projection is shorter
  /// than `sin(10°)` and its direction is noise, so the yaw of the whole
  /// arrangement would be decided by hand shake. The alternative — silently
  /// treating yaw as zero and calibrating anyway — is refused on purpose.
  ({bool ok, String? reason, double yawRad}) extractCalibratedStationYawRad({
    required Quat cameraOrientation,
  }) {
    if (!cameraOrientation.isUsable) {
      return (
        ok: false,
        reason: 'The camera pose is not usable yet. Wait for AR tracking.',
        yawRad: 0,
      );
    }
    final f = cameraOrientation.cameraForward;
    if (!f.isUsable) {
      return (
        ok: false,
        reason: 'The camera has no usable forward direction yet.',
        yawRad: 0,
      );
    }
    final h = f.horizontalLength;
    if (!(h > VioSphereConstants.minCalibrationHorizontalComponent)) {
      return (
        ok: false,
        reason:
            'Hold the phone closer to horizontal before calibrating — the '
            'front direction cannot be measured while the camera points within '
            '10° of straight up or down.',
        yawRad: 0,
      );
    }
    return (ok: true, reason: null, yawRad: math.atan2(f.x, -f.z));
  }

  /// Phase A, end to end: freezes the station origin and yaw from [pose] and
  /// generates the 26 world-locked targets.
  ///
  /// Call **once** per capture session, from the "Calibrate station" action. A
  /// second call replaces the reference — which is the deliberate "new
  /// calibration session" path, and the only way the station reference changes.
  CalibrationResult calibrate({required VioPoseSample pose}) {
    if (!pose.isFinite) {
      return const CalibrationResult(
        ok: false,
        reason: 'The camera pose is not finite; calibration is refused.',
      );
    }
    final yaw = extractCalibratedStationYawRad(
      cameraOrientation: pose.rotation,
    );
    if (!yaw.ok) {
      return CalibrationResult(ok: false, reason: yaw.reason);
    }
    return calibrateWithReference(
      stationOrigin: pose.position,
      stationYawRad: yaw.yawRad,
    );
  }

  /// Freezes an already-chosen reference and builds the sphere. Split out so a
  /// resumed station can re-use its stored origin and yaw without a live pose.
  CalibrationResult calibrateWithReference({
    required Vec3 stationOrigin,
    required double stationYawRad,
  }) {
    if (!stationOrigin.isFinite || !stationYawRad.isFinite) {
      return const CalibrationResult(
        ok: false,
        reason: 'The station reference is not finite; calibration is refused.',
      );
    }
    _stationOrigin = stationOrigin;
    _stationYawRad = stationYawRad;
    _nodes = buildNodeSphere(
      stationOriginWorld: stationOrigin,
      stationYawRad: stationYawRad,
      radiusM: radiusM,
    );
    resetStability();
    return CalibrationResult(
      ok: true,
      stationOrigin: stationOrigin,
      stationYawRad: stationYawRad,
      nodes: _nodes,
    );
  }

  /// Forgets the station reference and the targets. Used when the AR session is
  /// replaced, because a new session has a new world origin and the old
  /// coordinates no longer mean anything.
  void clearStation() {
    _stationOrigin = null;
    _stationYawRad = null;
    _nodes = const <TourTargetNode>[];
    resetStability();
  }

  /// Generates the 26 world-locked target positions.
  ///
  /// ```
  /// d_i = [ cosθ·sin(ψ0 + α),  sinθ,  -cosθ·cos(ψ0 + α) ]
  /// P_i = P_station + r · d_i
  /// ```
  ///
  /// With `r = 1.0 m`. The layout is fixed: 8 horizontal at 0° every 45°, 8
  /// upper at +45° every 45°, 8 lower at −45° every 45°, one zenith at +90°, one
  /// nadir at −90°, in that index order. Nothing here adapts to the input.
  List<TourTargetNode> buildNodeSphere({
    required Vec3 stationOriginWorld,
    required double stationYawRad,
    double? radiusM,
  }) {
    final r = radiusM ?? this.radiusM;
    final out = <TourTargetNode>[];

    void add({
      required String ring,
      required double elevationDeg,
      required double azimuthDeg,
    }) {
      final index = out.length;
      final theta = elevationDeg * math.pi / 180;
      final alpha = azimuthDeg * math.pi / 180;
      final absolute = stationYawRad + alpha;
      // Defensive normalisation: the closed form above is already unit length
      // for any ψ0, but the matrix and the alignment path both consume this as a
      // unit vector and a future edit should not be able to break that quietly.
      final direction = Vec3(
        math.cos(theta) * math.sin(absolute),
        math.sin(theta),
        -math.cos(theta) * math.cos(absolute),
      ).normalized;
      final worldPosition = stationOriginWorld + direction * r;
      out.add(
        TourTargetNode(
          index: index,
          ringName: ring,
          elevationDeg: elevationDeg,
          azimuthDeg: azimuthDeg,
          worldAzimuthDeg: (absolute * 180 / math.pi + 540) % 360 - 180,
          direction: direction,
          worldPosition: worldPosition,
          modelMatrix: buildModelMatrix(
            targetWorldPosition: worldPosition,
            stationOriginWorld: stationOriginWorld,
          ),
        ),
      );
    }

    for (var i = 0; i < VioSphereConstants.horizontalCount; i++) {
      add(
        ring: VioSphereConstants.horizontalRingName,
        elevationDeg: 0,
        azimuthDeg: i * VioSphereConstants.horizontalAzimuthStepDeg,
      );
    }
    for (var i = 0; i < VioSphereConstants.upperCount; i++) {
      add(
        ring: VioSphereConstants.upperRingName,
        elevationDeg: VioSphereConstants.upperElevationDeg,
        azimuthDeg: i * VioSphereConstants.ringAzimuthStepDeg,
      );
    }
    for (var i = 0; i < VioSphereConstants.lowerCount; i++) {
      add(
        ring: VioSphereConstants.lowerRingName,
        elevationDeg: VioSphereConstants.lowerElevationDeg,
        azimuthDeg: i * VioSphereConstants.ringAzimuthStepDeg,
      );
    }
    add(
      ring: VioSphereConstants.zenithRingName,
      elevationDeg: VioSphereConstants.zenithElevationDeg,
      azimuthDeg: 0,
    );
    add(
      ring: VioSphereConstants.nadirRingName,
      elevationDeg: VioSphereConstants.nadirElevationDeg,
      azimuthDeg: 0,
    );

    assert(out.length == VioSphereConstants.nodeCount);
    return List<TourTargetNode>.unmodifiable(out);
  }

  /// Marks one target captured. The only mutation the target list allows, and
  /// it never touches a position.
  void markCaptured(int index, {bool captured = true}) {
    if (index < 0 || index >= _nodes.length) return;
    final updated = List<TourTargetNode>.of(_nodes);
    updated[index] = updated[index].copyWith(captured: captured);
    _nodes = List<TourTargetNode>.unmodifiable(updated);
  }

  // ── the GLB model matrix ──────────────────────────────────────────────

  /// A column-major 4×4 model matrix placing the target GLB at
  /// [targetWorldPosition] with its `+Z` face turned towards the station.
  ///
  /// **Verified against the shipped asset, not assumed.**
  /// `assets/models/ar_capture_target.glb` is a disc in its local `XY` plane
  /// with its thickness along `Z`, its front face at `+0.006`; the Dart glTF
  /// parser bakes the node transforms into positions and the asset is authored
  /// 1 unit = 1 m, so no scale or Y-up→Z-up conversion belongs here.
  ///
  /// The basis is `[X | Y | Z]` with `Z` pointing from the target back at the
  /// station. Near the poles `Z` approaches ±world-up, where `up × Z` collapses,
  /// so the reference up flips to `+X` there — the cross product stays
  /// well-conditioned and the plate keeps a defined orientation.
  ///
  /// The renderer's multiplication order is `mvp = projection · view · model`
  /// (`Matrix.multiplyMM(out, 0, proj, 0, view, 0)` then `… , mvp, model`), and
  /// the 16 floats are uploaded with `transpose = false`, i.e. column-major, so
  /// the array below is passed through unchanged.
  static Float64List buildModelMatrix({
    required Vec3 targetWorldPosition,
    required Vec3 stationOriginWorld,
  }) {
    final m = Float64List(16);
    if (!targetWorldPosition.isFinite || !stationOriginWorld.isFinite) {
      m[0] = 1;
      m[5] = 1;
      m[10] = 1;
      m[15] = 1;
      return m;
    }
    // The inward-facing axis: the model's +Z points back at the station, which
    // is who aims at it from wherever they stand.
    var zAxis = (stationOriginWorld - targetWorldPosition).normalized;
    if (!zAxis.isUsable) {
      // Degenerate only if the target sits exactly on the station, which the
      // radius makes impossible; keep a valid basis rather than emitting NaNs.
      zAxis = Vec3.unitZ;
    }
    final referenceUp = zAxis.y.abs() > 0.999 ? Vec3.unitX : Vec3.unitY;
    var xAxis = (referenceUp.cross(zAxis)).normalized;
    var yAxis = zAxis.cross(xAxis).normalized;
    if (!xAxis.isUsable || !yAxis.isUsable) {
      // Only reachable for a direction that is not unit length or not finite in
      // a way the checks above missed. A valid basis is still preferred to
      // NaNs, but the **translation is kept either way**: dropping it would
      // silently draw the marker at the world origin, which reads to the user
      // as "the target vanished" rather than as a bad frame.
      xAxis = Vec3.unitX;
      yAxis = Vec3.unitY;
      zAxis = Vec3.unitZ;
    }
    // Column-major: element (row, col) lives at col * 4 + row.
    m[0] = xAxis.x;
    m[1] = xAxis.y;
    m[2] = xAxis.z;
    m[3] = 0;
    m[4] = yAxis.x;
    m[5] = yAxis.y;
    m[6] = yAxis.z;
    m[7] = 0;
    m[8] = zAxis.x;
    m[9] = zAxis.y;
    m[10] = zAxis.z;
    m[11] = 0;
    m[12] = targetWorldPosition.x;
    m[13] = targetWorldPosition.y;
    m[14] = targetWorldPosition.z;
    m[15] = 1;
    return m;
  }

  /// [buildModelMatrix] for one already-built node.
  Float64List modelMatrixFor(TourTargetNode node) => node.modelMatrix;

  // ── Phase B: the 600 ms stability window ──────────────────────────────

  final List<VioPoseSample> _history = <VioPoseSample>[];

  /// Clears the hold. Called on a tracking gap, a target switch, a station
  /// change and a capture attempt, so a hold never spans a discontinuity.
  void resetStability() {
    _history.clear();
  }

  int get historyLength => _history.length;

  /// The longest hold currently recorded, in milliseconds.
  int get heldMs {
    if (_history.length < 2) return 0;
    return (_history.last.timestampNanoseconds -
            _history.first.timestampNanoseconds) ~/
        1000000;
  }

  /// Evaluates one live frame against the frozen station reference.
  ///
  /// This is Phase B and it is **read-only with respect to the station**: the
  /// origin, the yaw and the 26 world positions are never recomputed here. The
  /// only state it advances is the stability history.
  ///
  /// [isSavingInProgress] and [isFailureLatched] are controller concerns rather
  /// than geometry, but they are part of the specification's shutter expression,
  /// so they are folded into [FrameGateEvaluation.canTriggerShutter] here —
  /// one expression, one place, impossible to re-derive differently elsewhere.
  /// [nowNanoseconds] is the caller's own clock, when it has one.
  ///
  /// The engine has no clock on purpose — a wall clock inside the maths would
  /// make the 250 ms age gate untestable — so the age of the newest pose is
  /// measured against this. Omitted, the pose is its own reference and the age
  /// gate is satisfied by construction; the controller passes
  /// `DateTime.now()`'s nanosecond value so a stream that stalls between two
  /// deliveries is caught by the gate that names staleness, rather than only by
  /// the sample-gap gate that names discontinuity.
  FrameGateEvaluation evaluateFrame({
    required VioPoseSample pose,
    required bool trackingValid,
    required int activeTargetIndex,
    bool isSavingInProgress = false,
    bool isFailureLatched = false,
    /// Allows an explicit, temporary diagnostic bypass of the station-distance
    /// gate without weakening pose freshness, alignment or stability. The raw
    /// distance remains in [FrameGateEvaluation] for diagnostics.
    bool enforceStationDrift = true,
    /// Whether this frame belongs to the continuous, hysteretic alignment
    /// window. The strict [isAligned] calculation below still controls the
    /// instant a shutter can fire.
    bool alignmentWindowValid = true,
    int? nowNanoseconds,
  }) {
    final origin = _stationOrigin;
    if (origin == null ||
        activeTargetIndex < 0 ||
        activeTargetIndex >= _nodes.length) {
      return _emptyEvaluation(
        activeTargetIndex: activeTargetIndex,
        rejection: 'no-station',
      );
    }
    final target = _nodes[activeTargetIndex];

    // 1. Validate the pose before any of it is used.
    if (!trackingValid || !pose.isFinite || !pose.rotation.isUsable) {
      resetStability();
      return _emptyEvaluation(
        activeTargetIndex: activeTargetIndex,
        rejection: 'invalid-pose',
        pose: pose,
        origin: origin,
      );
    }

    // 2. Station drift: a full 3D distance, not a horizontal-only one, because
    //    walking away vertically (a step, a chair) moves the composition too.
    final stationDriftM = Vec3.distance(pose.position, origin);
    final isWithinStation = stationDriftM <= maxStationDriftM;

    // A drift excursion invalidates the stability window: capture may not be
    // armed from a hold that spans a walk-away, and a fresh 600 ms hold is
    // required after returning inside the threshold.
    if (enforceStationDrift && !isWithinStation) {
      resetStability();
    }

    // 3. The live optical axis and the direction to the active target.
    final forward = pose.rotation.cameraForward;
    final delta = target.worldPosition - pose.position;
    final targetDistanceM = delta.length;
    final targetDirection = delta.normalized;

    double alignmentErrorDeg;
    double? yawGuidanceDeg;
    double? pitchGuidanceDeg;
    final aligned = targetDirection.isUsable && forward.isUsable;
    if (!aligned) {
      // A zero-length direction to the target, or an unusable camera axis,
      // cannot be aligned — it is not "aligned by default". The error is 180°
      // so the UI reads it as unreachable rather than as a good frame.
      alignmentErrorDeg = 180;
    } else {
      alignmentErrorDeg =
          math.acos(clampDouble(forward.dot(targetDirection), -1, 1)) *
          180 /
          math.pi;
      final targetYaw = Quat.yawOf(targetDirection);
      final cameraYaw = Quat.yawOf(forward);
      yawGuidanceDeg = wrapToMinus180To180(targetYaw - cameraYaw);
      final targetPitch = math.asin(clampDouble(targetDirection.y, -1, 1));
      final cameraPitch = math.asin(clampDouble(forward.y, -1, 1));
      pitchGuidanceDeg = (targetPitch - cameraPitch) * 180 / math.pi;
    }
    final isAligned = aligned && alignmentErrorDeg <= maxAlignmentErrorDeg;

    // 4. The stability window. A pose that has left the alignment hold by more
    // than its display hysteresis must start a new hold before it can arm a
    // capture. Without this, a phone held still while aimed elsewhere can fire
    // immediately on entering the target cone.
    if (!alignmentWindowValid) {
      resetStability();
      return FrameGateEvaluation(
        stationDriftM: stationDriftM,
        isWithinStation: isWithinStation,
        activeTargetIndex: activeTargetIndex,
        targetDistanceM: targetDistanceM,
        alignmentErrorDeg: alignmentErrorDeg,
        isAligned: isAligned,
        yawGuidanceDeg: yawGuidanceDeg ?? 0,
        pitchGuidanceDeg: pitchGuidanceDeg ?? 0,
        isPoseSteady: false,
        stabilityRejection: 'alignment',
        windowHeldMs: 0,
        angularVelocityDegPerSec: null,
        cameraForward: forward,
        targetDirection: targetDirection,
        canTriggerShutter: false,
      );
    }

    // 5. The stability window.
    final stability = _updateStability(
      pose,
      nowNanoseconds: nowNanoseconds ?? pose.timestampNanoseconds,
    );

    final canTrigger =
        trackingValid &&
        !target.captured &&
        !isSavingInProgress &&
        !isFailureLatched &&
        (!enforceStationDrift || isWithinStation) &&
        isAligned &&
        stability.steady;

    return FrameGateEvaluation(
      stationDriftM: stationDriftM,
      isWithinStation: isWithinStation,
      activeTargetIndex: activeTargetIndex,
      targetDistanceM: targetDistanceM,
      alignmentErrorDeg: alignmentErrorDeg,
      isAligned: isAligned,
      yawGuidanceDeg: yawGuidanceDeg ?? 0,
      pitchGuidanceDeg: pitchGuidanceDeg ?? 0,
      isPoseSteady: stability.steady,
      stabilityRejection: stability.rejection,
      windowHeldMs: stability.heldMs,
      angularVelocityDegPerSec: stability.angularVelocityDegPerSec,
      cameraForward: forward,
      targetDirection: targetDirection,
      canTriggerShutter: canTrigger,
    );
  }

  FrameGateEvaluation _emptyEvaluation({
    required int activeTargetIndex,
    required String rejection,
    VioPoseSample? pose,
    Vec3? origin,
  }) {
    final drift = pose != null && origin != null
        ? Vec3.distance(pose.position, origin)
        : 0.0;
    return FrameGateEvaluation(
      stationDriftM: drift,
      isWithinStation: drift <= maxStationDriftM,
      activeTargetIndex: activeTargetIndex,
      targetDistanceM: 0,
      alignmentErrorDeg: 180,
      isAligned: false,
      yawGuidanceDeg: 0,
      pitchGuidanceDeg: 0,
      isPoseSteady: false,
      stabilityRejection: rejection,
      windowHeldMs: 0,
      angularVelocityDegPerSec: null,
      cameraForward: Vec3.zero,
      targetDirection: Vec3.zero,
      canTriggerShutter: false,
    );
  }

  /// Feeds one sample into the hold and answers whether the whole 600 ms window
  /// is satisfied.
  ///
  /// Eight conditions, each closing a different hole:
  ///
  /// | condition | what it stops |
  /// | --- | --- |
  /// | age ≤ [maxPoseAge] | acting on a reading that is already 250 ms gone |
  /// | monotonically increasing timestamps | a re-ordered or wrapped stream |
  /// | gap ≤ [maxSampleGap] | a dropped stretch being interpolated over |
  /// | rate ≤ [maxAngularVelocityDegPerSec] | a phone carried through the target |
  /// | history spans the whole [window] | a "hold" made of two recent samples |
  /// | orientation drift ≤ 1.5° | a slow turn that never leaves the cone |
  /// | position drift ≤ 0.03 m | the user walking away mid-window |
  /// | finite values throughout | NaN making every comparison false |
  ///
  /// The age gate and the gap gate are **not** the same check and neither
  /// subsumes the other, which is why both exist:
  ///
  /// * the gap gate reads only the stream's own timestamps, so it sees a
  ///   discontinuity (a stall, then a jump) but is blind to a stream that keeps
  ///   advancing its timestamps while falling further behind real time — a
  ///   processing backlog, where every frame is stale yet consecutive;
  /// * the age gate reads [nowNanoseconds] against the newest timestamp, so it
  ///   sees exactly that backlog.
  ///
  /// [nowNanoseconds] must therefore be **monotonic and in the same base as the
  /// sample timestamps**, or equal to [VioPoseSample.timestampNanoseconds], in
  /// which case the age is zero by construction and the gate is satisfied. A
  /// caller that has a wall clock in another base (epoch milliseconds against a
  /// device's uptime nanos, say) must anchor the two once and advance the clock
  /// from the same reference, or every frame will be refused for staleness.
  ({
    bool steady,
    String? rejection,
    int heldMs,
    double? angularVelocityDegPerSec,
  })
  _updateStability(VioPoseSample pose, {required int nowNanoseconds}) {
    // Freshness first: a reading that describes a moment already past the age
    // limit cannot anchor a 600 ms hold, whatever its own timestamps claim.
    final ageMs = (nowNanoseconds - pose.timestampNanoseconds) ~/ 1000000;
    if (ageMs > maxPoseAge.inMilliseconds) {
      resetStability();
      _history.add(pose);
      return (
        steady: false,
        rejection: 'measurement-age',
        heldMs: 0,
        angularVelocityDegPerSec: null,
      );
    }

    final previous = _history.isEmpty ? null : _history.last;

    if (previous != null) {
      final gapNs = pose.timestampNanoseconds - previous.timestampNanoseconds;
      if (gapNs <= 0) {
        // Unlike the gap and age branches below, the offending sample is **not**
        // admitted: those two describe a discontinuous *delivery* of otherwise
        // good poses, so the new hold should start from the sample in hand,
        // whereas a repeated or backwards timestamp means the clock or the
        // ordering is broken and the pose may be a duplicate or an out-of-order
        // arrival. The hold restarts from whatever comes next.
        resetStability();
        return (
          steady: false,
          rejection: 'timestamp',
          heldMs: 0,
          angularVelocityDegPerSec: null,
        );
      }
      final gapMs = gapNs ~/ 1000000;
      if (gapMs > maxSampleGap.inMilliseconds) {
        // A gap this long is not evidence of holding: the hold restarts from
        // here rather than pretending the missing samples were still.
        resetStability();
        _history.add(pose);
        return (
          steady: false,
          rejection: 'sample-gap',
          heldMs: 0,
          angularVelocityDegPerSec: null,
        );
      }
    }

    _history.add(pose);
    // Keep the newest sample at or before the window cutoff, so "held for at
    // least the window" is decidable. A plain `removeWhere(older than cutoff)`
    // deletes exactly the sample that proves the window elapsed, and the hold
    // can then never reach 600 ms with 33 ms samples.
    final cutoffNs = pose.timestampNanoseconds - window.inMicroseconds * 1000;
    while (_history.length > 1 &&
        _history[1].timestampNanoseconds <= cutoffNs) {
      _history.removeAt(0);
    }

    final oldest = _history.first;
    final heldMs =
        (pose.timestampNanoseconds - oldest.timestampNanoseconds) ~/ 1000000;

    // The rate is measured against the newest sample at least [rateBaseline]
    // old, not against the previous frame: see [VioSphereConstants.rateBaseline].
    double? rateDegPerSec;
    VioPoseSample? anchor;
    for (final s in _history) {
      final deltaNs = pose.timestampNanoseconds - s.timestampNanoseconds;
      if (deltaNs >= rateBaseline.inMicroseconds * 1000) anchor = s;
    }
    anchor ??= _history.length >= 2 ? _history[_history.length - 2] : null;
    if (anchor != null) {
      final dtSeconds =
          (pose.timestampNanoseconds - anchor.timestampNanoseconds) / 1e9;
      if (dtSeconds >= 0.02) {
        final deg = Quat.angularDistanceDeg(anchor.rotation, pose.rotation);
        if (deg.isFinite) rateDegPerSec = deg / dtSeconds;
      }
    }
    if (rateDegPerSec != null && rateDegPerSec > maxAngularVelocityDegPerSec) {
      resetStability();
      _history.add(pose);
      return (
        steady: false,
        rejection: 'angular-rate',
        heldMs: 0,
        angularVelocityDegPerSec: rateDegPerSec,
      );
    }

    if (_history.length < 2 || heldMs < window.inMilliseconds) {
      return (
        steady: false,
        rejection: 'window',
        heldMs: heldMs,
        angularVelocityDegPerSec: rateDegPerSec,
      );
    }

    final orientationDriftDeg = Quat.angularDistanceDeg(
      oldest.rotation,
      pose.rotation,
    );
    if (orientationDriftDeg.isFinite &&
        orientationDriftDeg > maxWindowOrientationDriftDeg) {
      // This frame proves the preceding history was not a continuous hold.
      // Start the next candidate window here rather than leaving a 100%-full
      // progress ring above a shutter that remains blocked on orientation.
      resetStability();
      _history.add(pose);
      return (
        steady: false,
        rejection: 'orientation-drift',
        heldMs: 0,
        angularVelocityDegPerSec: rateDegPerSec,
      );
    }
    final positionDriftM = Vec3.distance(oldest.position, pose.position);
    if (!positionDriftM.isFinite || positionDriftM > maxWindowPositionDriftM) {
      // Same rule as orientation: a position excursion ends this hold. Keeping
      // the old history made a full green ring persist while the auto gate
      // correctly refused the unstable frame.
      resetStability();
      _history.add(pose);
      return (
        steady: false,
        rejection: 'position-drift',
        heldMs: 0,
        angularVelocityDegPerSec: rateDegPerSec,
      );
    }

    return (
      steady: true,
      rejection: null,
      heldMs: heldMs,
      angularVelocityDegPerSec: rateDegPerSec,
    );
  }
}

/// `v` clamped into `[lo, hi]`, and `lo` when `v` is NaN.
///
/// NaN is the value that matters: every comparison against NaN is false, so an
/// unguarded `acos(NaN)` would produce NaN and an unguarded `> limit` check
/// would pass. NaN is mapped to `lo`, which makes the product zero and the
/// alignment check fail rather than succeed.
double clampDouble(double v, double lo, double hi) {
  if (v.isNaN) return lo;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/// Wraps [degrees] into `(-180, 180]`.
///
/// `179° → −179°` is **not** a 358° turn: it is `+2°`, and the guidance says
/// "turn right 2°". Non-finite input is answered with 0 rather than NaN so a
/// broken pose cannot make the instruction nonsense.
double wrapToMinus180To180(double degrees) {
  if (!degrees.isFinite) return 0;
  // Dart's `%` takes the sign of the divisor, so `v % 360` is already in
  // [0, 360) for a positive divisor.
  final d = degrees % 360;
  return d > 180 ? d - 360 : d;
}
