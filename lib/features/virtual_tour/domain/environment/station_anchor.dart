import 'dart:math' as math;
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

/// Station-origin displacement measurement.
///
/// The ARCore anchor is used for exactly one thing: measuring how far the
/// camera has moved from the station where the panorama group started. This is
/// diagnostic geometry only: shutter eligibility is controlled by tracking,
/// target alignment and the short-window quality checks.

/// A world-space station origin.
class StationOrigin {
  final Vec3 worldPosition;

  /// Device yaw at lock time, for the floor-plan heading lock.
  final double lockYawDeg;

  /// Camera height at lock time, in metres.
  final double lockCameraHeightM;

  /// Whether the origin came from a plane hit-test or was estimated.
  final StationConfidence confidence;

  const StationOrigin({
    required this.worldPosition,
    required this.lockYawDeg,
    required this.lockCameraHeightM,
    required this.confidence,
  });

  @override
  String toString() =>
      'StationOrigin(${worldPosition.x.toStringAsFixed(2)}, '
      '${worldPosition.y.toStringAsFixed(2)}, '
      '${worldPosition.z.toStringAsFixed(2)}, '
      'yaw ${lockYawDeg.toStringAsFixed(1)}°, ${confidence.name})';
}

/// How the station origin was obtained.
enum StationConfidence {
  /// A hit-test landed on a tracked plane.
  plane,

  /// AR was available but no plane was found in time.
  estimated,

  /// No AR support; the origin came from a manual entry or is absent.
  manual,
}

/// Result of evaluating the current camera position against the station origin.
class StationDrift {
  /// Horizontal distance from the origin, in metres.
  final double horizontalM;

  /// Full 3D distance, in metres.
  final double totalM;

  const StationDrift({
    required this.horizontalM,
    required this.totalM,
  });

  @override
  String toString() => '${horizontalM.toStringAsFixed(3)} m';
}

/// Evaluates the camera's displacement from the station origin.
StationDrift evaluateStationDrift({
  required StationOrigin origin,
  required Vec3 cameraPosition,
}) {
  final dx = cameraPosition.x - origin.worldPosition.x;
  final dz = cameraPosition.z - origin.worldPosition.z;
  final dy = cameraPosition.y - origin.worldPosition.y;

  final horizontal = math.sqrt(dx * dx + dz * dz);
  final total = math.sqrt(dx * dx + dy * dy + dz * dz);

  return StationDrift(
    horizontalM: horizontal,
    totalM: total,
  );
}
