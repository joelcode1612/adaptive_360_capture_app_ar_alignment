import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart'
    show VioSphereConstants;
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// One capture station: the physical spot a set of 26 photographs is taken from.
///
/// A station is **not** an anchor. It is a recorded pose: the AR camera's
/// position and orientation at the moment the user pressed "Calibrate station".
/// Everything the station owns — the 26 target directions, the rays that look
/// for surfaces, the place the GLB marker is aimed from — is derived from it.
///
/// ## Units and frames
///
/// * [referencePosition] — ARCore world metres. **Valid for this AR session
///   only.** ARCore's world origin is arbitrary and is re-chosen the next time a
///   session starts, so a station recorded yesterday cannot be walked back to
///   today unless persistent world mapping (a Cloud Anchor or an ARCore
///   `Geospatial` pose) is implemented, and it has not been.
/// * [referenceOrientation] — a unit quaternion in this project's convention
///   (`q·ẑ` is the optical axis the user was looking along, `q·ŷ` up), i.e. the
///   output of `arCameraPoseToCaptureWorld`, never a raw ARCore quaternion.
class CaptureStation {
  final String stationId;

  /// Station origin, in the AR session's world metres.
  final Vec3 referencePosition;

  /// Station reference orientation, this project's convention.
  final Quat referenceOrientation;

  final DateTime createdAt;

  /// What AR was reporting when the station was calibrated.
  final TrackingState trackingStatus;

  /// How many of [total] targets have a saved photograph.
  final int capturedCount;

  /// The number of targets this station was built with — the specification's
  /// 26, the value every real station is created with (`_specs.length`).
  final int total;

  const CaptureStation({
    required this.stationId,
    required this.referencePosition,
    required this.referenceOrientation,
    required this.createdAt,
    required this.trackingStatus,
    this.capturedCount = 0,
    this.total = VioSphereConstants.nodeCount,
  });

  bool get isCalibrated =>
      referencePosition.length.isFinite &&
      trackingStatus == TrackingState.tracking;

  double get progress => total == 0 ? 0 : capturedCount / total;

  CaptureStation copyWith({
    TrackingState? trackingStatus,
    int? capturedCount,
    int? total,
  }) => CaptureStation(
    stationId: stationId,
    referencePosition: referencePosition,
    referenceOrientation: referenceOrientation,
    createdAt: createdAt,
    trackingStatus: trackingStatus ?? this.trackingStatus,
    capturedCount: capturedCount ?? this.capturedCount,
    total: total ?? this.total,
  );

  Map<String, dynamic> toJson() => <String, dynamic>{
    'stationId': stationId,
    'referencePosition': referencePosition.toList(),
    'referenceOrientation': referenceOrientation.toList(),
    'createdAt': createdAt.toIso8601String(),
    'trackingStatus': trackingStatus.name,
    'capturedCount': capturedCount,
    'total': total,
  };

  static CaptureStation fromJson(Map<String, dynamic> json) => CaptureStation(
    stationId: json['stationId'] as String,
    referencePosition: Vec3.fromList(
      (json['referencePosition'] as List<dynamic>).cast<double>(),
    ),
    referenceOrientation: Quat.fromList(
      (json['referenceOrientation'] as List<dynamic>).cast<double>(),
    ),
    createdAt: DateTime.parse(json['createdAt'] as String),
    trackingStatus: TrackingState.values.firstWhere(
      (TrackingState s) => s.name == json['trackingStatus'],
      orElse: () => TrackingState.paused,
    ),
    capturedCount: (json['capturedCount'] as num?)?.toInt() ?? 0,
    total: (json['total'] as num?)?.toInt() ?? VioSphereConstants.nodeCount,
  );
}
