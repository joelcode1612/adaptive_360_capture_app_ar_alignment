import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

/// Where a target is in its own capture workflow.
enum CaptureTargetStatus {
  /// Never attempted.
  pending,

  /// The target the user is being guided towards right now.
  active,

  /// Aimed at and steady, but the shutter has not fired.
  aligned,

  /// A shutter request for this target is in flight.
  ///
  /// The automatic-capture machine moves the active target to this state
  /// **before** it awaits the photograph, which is what makes a second pose
  /// event unable to start a second capture for the same target: the guard
  /// reads this persisted transition, not a flag that is one frame behind.
  ///
  /// Not a resting state. A process death mid-capture leaves it on disk, so
  /// [CaptureTarget.fromJson] reads it back as [pending] — an interrupted
  /// photograph is a retry, not a captured target.
  capturing,

  /// A photograph exists on disk for this target.
  captured,

  /// The last attempt failed. Re-takeable without touching other targets.
  failed;

  bool get isDone => this == CaptureTargetStatus.captured;

  /// Whether a shutter request for this target is in flight.
  bool get isCapturing => this == CaptureTargetStatus.capturing;
}

/// One of the 26 spherical capture directions of a station.
///
/// The angles are the **station-local** ones from Section 1 §7: yaw is measured
/// in the station's own frame where `+X` is right, `+Y` up and `+Z` forward, so
/// a positive yaw really is "to your right" and the poles have no yaw at all.
///
/// [stationLocalDirection] is that (yaw, pitch) pair as a unit vector, computed
/// once by `target_sphere.dart`. The **world** position is not stored as a
/// direction but as an *anchor*: [anchorWorldPosition] is ARCore's own world
/// position for the marker, and it is the only position used for alignment.
/// Since the plane-dependent placement was removed the anchor is created
/// directly at the frozen station geometry, so a target always has a world
/// position as soon as the station is calibrated.
class CaptureTarget {
  /// 1-based: 1–8 Horizontal, 9–16 Upper, 17–24 Lower, 25 Zenith, 26 Nadir.
  final int targetId;

  final String stationId;

  final double yawDegrees;
  final double pitchDegrees;

  /// Unit direction in station-local coordinates.
  final Vec3 stationLocalDirection;

  /// ARCore's anchor id, when the marker has an anchor.
  final String? anchorIdentifier;

  /// Where the anchor actually is, in AR session world metres. `null` only
  /// before the station is calibrated.
  final Vec3? anchorWorldPosition;

  /// `PLANE_VERTICAL`, `PLANE_HORIZONTAL_UPWARD_FACING`, … Platform-reported
  /// surface type of the anchor, for diagnostics only — alignment never
  /// depends on it.
  final String? anchorSurfaceType;

  final CaptureTargetStatus status;

  /// Absolute path of the saved photograph. Only ever set by a successful save.
  final String? imagePath;

  /// Why the last attempt failed, or why no anchor exists.
  final String? message;

  const CaptureTarget({
    required this.targetId,
    required this.stationId,
    required this.yawDegrees,
    required this.pitchDegrees,
    required this.stationLocalDirection,
    this.anchorIdentifier,
    this.anchorWorldPosition,
    this.anchorSurfaceType,
    this.status = CaptureTargetStatus.pending,
    this.imagePath,
    this.message,
  });

  bool get isPole => pitchDegrees.abs() >= 89.999;
  bool get isZenith => pitchDegrees >= 89.999;
  bool get isNadir => pitchDegrees <= -89.999;

  /// Whether this target has a real, tracked position to aim at.
  bool get isAnchored =>
      anchorIdentifier != null && anchorWorldPosition != null;

  CaptureTarget copyWith({
    String? anchorIdentifier,
    Vec3? anchorWorldPosition,
    String? anchorSurfaceType,
    CaptureTargetStatus? status,
    String? imagePath,
    String? message,
    bool clearAnchor = false,
    bool clearImage = false,
    bool clearMessage = false,
  }) => CaptureTarget(
    targetId: targetId,
    stationId: stationId,
    yawDegrees: yawDegrees,
    pitchDegrees: pitchDegrees,
    stationLocalDirection: stationLocalDirection,
    anchorIdentifier: clearAnchor
        ? null
        : (anchorIdentifier ?? this.anchorIdentifier),
    anchorWorldPosition: clearAnchor
        ? null
        : (anchorWorldPosition ?? this.anchorWorldPosition),
    anchorSurfaceType: clearAnchor
        ? null
        : (anchorSurfaceType ?? this.anchorSurfaceType),
    status: status ?? this.status,
    imagePath: clearImage ? null : (imagePath ?? this.imagePath),
    message: clearMessage ? null : (message ?? this.message),
  );

  Map<String, dynamic> toJson() => <String, dynamic>{
    'targetId': targetId,
    'stationId': stationId,
    'yawDegrees': yawDegrees,
    'pitchDegrees': pitchDegrees,
    'stationLocalDirection': stationLocalDirection.toList(),
    'anchorIdentifier': anchorIdentifier,
    'anchorWorldPosition': anchorWorldPosition?.toList(),
    'anchorSurfaceType': anchorSurfaceType,
    'status': status.name,
    'imagePath': imagePath,
    'message': message,
  };

  static CaptureTarget fromJson(Map<String, dynamic> json) => CaptureTarget(
    targetId: (json['targetId'] as num).toInt(),
    stationId: json['stationId'] as String,
    yawDegrees: (json['yawDegrees'] as num).toDouble(),
    pitchDegrees: (json['pitchDegrees'] as num).toDouble(),
    stationLocalDirection: Vec3.fromList(
      (json['stationLocalDirection'] as List<dynamic>).cast<double>(),
    ),
    anchorIdentifier: json['anchorIdentifier'] as String?,
    anchorWorldPosition: json['anchorWorldPosition'] == null
        ? null
        : Vec3.fromList(
            (json['anchorWorldPosition'] as List<dynamic>).cast<double>(),
          ),
    anchorSurfaceType: json['anchorSurfaceType'] as String?,
    status: _statusFromJson(json['status'] as String?),
    imagePath: json['imagePath'] as String?,
    message: json['message'] as String?,
  );

  /// Parses a persisted status, treating an interrupted capture as retryable.
  ///
  /// `capturing` is a live, in-memory state: if the process died between the
  /// transition and the JPEG, the row on disk still says so, and reading it
  /// back verbatim would produce a target that is neither done nor capturable.
  static CaptureTargetStatus _statusFromJson(String? name) {
    if (name == CaptureTargetStatus.capturing.name) {
      return CaptureTargetStatus.pending;
    }
    return CaptureTargetStatus.values.firstWhere(
      (CaptureTargetStatus s) => s.name == name,
      orElse: () => CaptureTargetStatus.pending,
    );
  }
}
