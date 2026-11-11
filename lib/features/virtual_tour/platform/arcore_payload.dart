import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart'
    as vio;
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The platform payload → type conversions.
///
/// **Pure Dart, in its own file on purpose.** These are the most likely place
/// for the bridge to break silently — a platform payload that changes shape
/// produces a `null` rather than an exception — so they have to be testable
/// without a Flutter binding, and the channel plumbing in `arcore_ar.dart` must
/// not be dragged into a plain `dart` run along with them.
///
/// Every function returns `null` for an incomplete payload. The callers treat
/// that as "no pose" / "no hit", never as a default value: an identity pose
/// invented from a truncated map would aim the marker at nothing and look like a
/// tracking problem.

/// Reads a number, or `null` when it is absent, not a number, or not finite.
///
/// The finiteness check is the whole point. A platform channel can hand us
/// `double.nan` or `double.infinity` — a sensor value that never converged, a
/// native division by a zero denominator — and `v is num` alone accepts them.
/// Every number read here is later **compared**: `driftM > limit`, `h <= 0`,
/// `distanceM` used in arithmetic. Every comparison against NaN evaluates to
/// `false`, which makes a NaN-bearing payload sail through readiness checks.
/// Returning `null` routes the reading into the same
/// "incomplete payload" path the callers already handle, which is exactly the
/// contract this file documents for a missing key: no invented pose, no
/// invented default.
double? _d(Object? v) {
  if (v is! num) return null;
  final d = v.toDouble();
  return d.isFinite ? d : null;
}

/// `"TRACKING" | "PAUSED" | "STOPPED" | "UNKNOWN"` → [TrackingState].
TrackingState parseTracking(String? name) => switch (name) {
  'TRACKING' => TrackingState.tracking,
  'PAUSED' => TrackingState.paused,
  'STOPPED' => TrackingState.stopped,
  // No frame yet. "Initialising", not "unavailable": reporting unavailable here
  // would make a healthy cold start look like an unsupported device, and the
  // tier ladder would then refuse the guided flow on a perfectly good phone.
  _ => TrackingState.initialising,
};

/// Reads a `{px, py, pz}` triple.
Vec3? parseVec3(Map<Object?, Object?> m) {
  final x = _d(m['px']);
  final y = _d(m['py']);
  final z = _d(m['pz']);
  if (x == null || y == null || z == null) return null;
  return Vec3(x, y, z);
}

/// Reads a pose event. `null` when the payload is incomplete **or when the
/// orientation is one the engine would refuse**: zero-length, below the shared
/// degenerate cut, or overflowing when squared. A quaternion that cannot be
/// used as a rotation is refused here rather than normalised into an arbitrary
/// direction — the callers treat `null` as "no pose", which is the honest
/// reading of a payload that cannot stand behind itself.
ArPose? parsePose(Map<Object?, Object?> m) {
  final p = parseVec3(m);
  final qx = _d(m['qx']);
  final qy = _d(m['qy']);
  final qz = _d(m['qz']);
  final qw = _d(m['qw']);
  if (p == null || qx == null || qy == null || qz == null || qw == null) {
    return null;
  }
  // Same cut as the engine's `Quat.isUsable`, checked before the
  // normalisation below: `Infinity` passes every plain `>` threshold, and a
  // near-zero length scales rounding noise into a full unit rotation.
  final quatLengthSquared = qx * qx + qy * qy + qz * qz + qw * qw;
  if (!quatLengthSquared.isFinite ||
      quatLengthSquared <= vio.Quat.degenerateLengthSquared) {
    return null;
  }
  return ArPose(
    position: p,
    orientation: Quat(qx, qy, qz, qw).normalized,
    tracking: parseTracking(m['tracking'] as String?),
    angularRateDegPerSec: _d(m['angularRateDegPerSec']),
    timestampNs: (m['timestampNs'] as num?)?.toInt(),
  );
}

/// Reads the sensor intrinsics.
///
/// `null` when the platform did not report them, so the caller can say
/// "provisional default" instead of inventing a value.
ArCameraIntrinsics? parseIntrinsics(Map<Object?, Object?> m) {
  final fx = _d(m['fx']);
  final fy = _d(m['fy']);
  final cx = _d(m['cx']);
  final cy = _d(m['cy']);
  final w = (m['imageWidth'] as num?)?.toInt();
  final h = (m['imageHeight'] as num?)?.toInt();
  if (fx == null ||
      fy == null ||
      cx == null ||
      cy == null ||
      w == null ||
      h == null ||
      w <= 0 ||
      h <= 0) {
    return null;
  }
  return ArCameraIntrinsics(
    focalLengthXPx: fx,
    focalLengthYPx: fy,
    principalPointXPx: cx,
    principalPointYPx: cy,
    imageWidth: w,
    imageHeight: h,
    rotationDeg: (m['displayRotation'] as num?)?.toInt() ?? 90,
  );
}

/// Reads the camera quality report.
///
/// `null` when the platform did not report it — the host sends these keys once,
/// on the first pose event after the session started. An absent report is "not
/// known yet", never a default value: a session is not described by inventing
/// a 0×0 camera, and the UI's rule is to show the line only when the numbers
/// are real.
ArCameraQuality? parseCameraQuality(Map<Object?, Object?> m) {
  final stillW = (m['stillWidth'] as num?)?.toInt();
  final stillH = (m['stillHeight'] as num?)?.toInt();
  final previewW = (m['previewWidth'] as num?)?.toInt();
  final previewH = (m['previewHeight'] as num?)?.toInt();
  final focus = m['focusMode'] as String?;
  if (stillW == null ||
      stillH == null ||
      previewW == null ||
      previewH == null ||
      focus == null ||
      stillW <= 0 ||
      stillH <= 0) {
    return null;
  }
  return ArCameraQuality(
    stillWidth: stillW,
    stillHeight: stillH,
    previewWidth: previewW,
    previewHeight: previewH,
    focusMode: focus,
  );
}

/// Reads the display field of view, when the platform derived it from its own
/// projection matrix.
({double hfovDeg, double vfovDeg})? parseDisplayFov(Map<Object?, Object?> m) {
  final h = _d(m['displayHfovDeg']);
  final v = _d(m['displayVfovDeg']);
  if (h == null || v == null || h <= 0 || v <= 0) return null;
  return (hfovDeg: h, vfovDeg: v);
}

/// Reads a hit-test result.
///
/// The plane normal is computed here from the plane's *orientation*, rather than
/// being sent pre-rotated from the platform: the rotation maths already lives in
/// the domain and is unit-tested, and a second implementation of it in Kotlin
/// would be a second thing to get wrong.
PlaneHit? parsePlaneHit(Map<Object?, Object?> m) {
  final p = parseVec3(m);
  final distance = _d(m['distanceM']);
  final type = m['planeType'] as String?;
  if (p == null || distance == null || type == null) return null;

  final qx = _d(m['planeQx']);
  final qy = _d(m['planeQy']);
  final qz = _d(m['planeQz']);
  final qw = _d(m['planeQw']);
  final normal = (qx != null && qy != null && qz != null && qw != null)
      ? Quat(qx, qy, qz, qw).normalized.rotate(Vec3.unitY)
      : Vec3.unitY;

  final ex = _d(m['extentX']);
  final ez = _d(m['extentZ']);

  return PlaneHit(
    worldPosition: p,
    planeNormal: normal,
    distanceM: distance,
    planeType: type,
    extent: (ex != null && ez != null) ? (x: ex, z: ez) : null,
  );
}

/// Reads an anchor payload.
ArAnchorHandle? parseAnchor(Map<Object?, Object?> m) {
  final id = m['id'] as String?;
  final p = parseVec3(m);
  if (id == null || p == null) return null;
  return ArAnchorHandle(
    id: id,
    worldPosition: p,
    tracking: parseTracking(m['tracking'] as String?),
  );
}
