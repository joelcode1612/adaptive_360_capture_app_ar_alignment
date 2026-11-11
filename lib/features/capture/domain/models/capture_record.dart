import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

/// The evidence for one saved photograph.
///
/// Written **after** the JPEG is on disk and verified readable, and only then.
/// A record therefore never describes a photograph that does not exist — which
/// is what lets the review screen list frames without re-scanning the directory.
///
/// The camera pose is stored in the AR session's world metres, with the same
/// caveat as the station that owns it: it is meaningful within the session that
/// produced it and not across sessions.
class CaptureRecord {
  final String stationId;
  final int targetId;

  /// Absolute path of the JPEG that was written and read back.
  final String imagePath;

  /// Byte length and SHA-256 of the accepted JPEG. They are recorded together
  /// with the capture-state manifest so a restart can distinguish an accepted
  /// frame from a truncated, replaced or orphaned file.
  ///
  /// Both are nullable only for legacy records written before durable manifests
  /// were introduced. Every newly committed record supplies both values.
  final int? imageByteLength;
  final String? imageSha256;

  /// When the image was saved, from the device clock.
  final DateTime capturedAt;

  final double targetYawDegrees;
  final double targetPitchDegrees;

  /// The camera pose at the instant of capture.
  final Vec3 cameraPosition;
  final Quat cameraOrientation;

  /// The 3-D angular error at the instant of capture, in degrees.
  final double alignmentErrorDeg;

  /// Whether the target was anchored to a real surface, and to what.
  final bool anchored;
  final String? anchorIdentifier;
  final String? anchorSurfaceType;

  final int imageWidth;
  final int imageHeight;

  final int sourceImageWidth;
  final int sourceImageHeight;
  final int? captureTimestampNs;
  final double? focalLengthXPx;
  final double? focalLengthYPx;
  final double? principalPointXPx;
  final double? principalPointYPx;

  /// `jpeg` — the only format the AR still path produces.
  final String imageFormat;

  /// How far the raw sensor image was rotated before encoding.
  final int rotationAppliedDeg;

  /// Alignment tolerance in force when the shutter fired.
  final double toleranceDeg;

  const CaptureRecord({
    required this.stationId,
    required this.targetId,
    required this.imagePath,
    this.imageByteLength,
    this.imageSha256,
    required this.capturedAt,
    required this.targetYawDegrees,
    required this.targetPitchDegrees,
    required this.cameraPosition,
    required this.cameraOrientation,
    required this.alignmentErrorDeg,
    required this.anchored,
    this.anchorIdentifier,
    this.anchorSurfaceType,
    required this.imageWidth,
    required this.imageHeight,
    this.sourceImageWidth = 0,
    this.sourceImageHeight = 0,
    this.captureTimestampNs,
    this.focalLengthXPx,
    this.focalLengthYPx,
    this.principalPointXPx,
    this.principalPointYPx,
    this.imageFormat = 'jpeg',
    this.rotationAppliedDeg = 0,
    this.toleranceDeg = 3,
  });

  Map<String, dynamic> toJson() => <String, dynamic>{
    'stationId': stationId,
    'targetId': targetId,
    'imagePath': imagePath,
    if (imageByteLength != null) 'imageByteLength': imageByteLength,
    if (imageSha256 != null) 'imageSha256': imageSha256,
    'capturedAt': capturedAt.toIso8601String(),
    'targetYawDegrees': targetYawDegrees,
    'targetPitchDegrees': targetPitchDegrees,
    'cameraPosition': cameraPosition.toList(),
    'cameraOrientation': cameraOrientation.toList(),
    'alignmentErrorDeg': alignmentErrorDeg,
    'anchored': anchored,
    'anchorIdentifier': anchorIdentifier,
    'anchorSurfaceType': anchorSurfaceType,
    'imageWidth': imageWidth,
    'imageHeight': imageHeight,
    'sourceImageWidth': sourceImageWidth,
    'sourceImageHeight': sourceImageHeight,
    if (captureTimestampNs != null) 'captureTimestampNs': captureTimestampNs,
    if (focalLengthXPx != null) 'focalLengthXPx': focalLengthXPx,
    if (focalLengthYPx != null) 'focalLengthYPx': focalLengthYPx,
    if (principalPointXPx != null) 'principalPointXPx': principalPointXPx,
    if (principalPointYPx != null) 'principalPointYPx': principalPointYPx,
    'imageFormat': imageFormat,
    'rotationAppliedDeg': rotationAppliedDeg,
    'toleranceDeg': toleranceDeg,
  };

  static CaptureRecord fromJson(Map<String, dynamic> json) => CaptureRecord(
    stationId: json['stationId'] as String,
    targetId: (json['targetId'] as num).toInt(),
    imagePath: json['imagePath'] as String,
    imageByteLength: (json['imageByteLength'] as num?)?.toInt(),
    imageSha256: json['imageSha256'] as String?,
    capturedAt: DateTime.parse(json['capturedAt'] as String),
    targetYawDegrees: (json['targetYawDegrees'] as num).toDouble(),
    targetPitchDegrees: (json['targetPitchDegrees'] as num).toDouble(),
    cameraPosition: Vec3.fromList(
      (json['cameraPosition'] as List<dynamic>).cast<double>(),
    ),
    cameraOrientation: Quat.fromList(
      (json['cameraOrientation'] as List<dynamic>).cast<double>(),
    ),
    alignmentErrorDeg: (json['alignmentErrorDeg'] as num).toDouble(),
    anchored: json['anchored'] as bool? ?? false,
    anchorIdentifier: json['anchorIdentifier'] as String?,
    anchorSurfaceType: json['anchorSurfaceType'] as String?,
    imageWidth: (json['imageWidth'] as num?)?.toInt() ?? 0,
    imageHeight: (json['imageHeight'] as num?)?.toInt() ?? 0,
    sourceImageWidth: (json['sourceImageWidth'] as num?)?.toInt() ?? 0,
    sourceImageHeight: (json['sourceImageHeight'] as num?)?.toInt() ?? 0,
    captureTimestampNs: (json['captureTimestampNs'] as num?)?.toInt(),
    focalLengthXPx: (json['focalLengthXPx'] as num?)?.toDouble(),
    focalLengthYPx: (json['focalLengthYPx'] as num?)?.toDouble(),
    principalPointXPx: (json['principalPointXPx'] as num?)?.toDouble(),
    principalPointYPx: (json['principalPointYPx'] as num?)?.toDouble(),
    imageFormat: json['imageFormat'] as String? ?? 'jpeg',
    rotationAppliedDeg: (json['rotationAppliedDeg'] as num?)?.toInt() ?? 0,
    toleranceDeg: (json['toleranceDeg'] as num?)?.toDouble() ?? 3,
  );

  CaptureRecord copyWith({
    String? imagePath,
    int? imageByteLength,
    String? imageSha256,
  }) => CaptureRecord(
    stationId: stationId,
    targetId: targetId,
    imagePath: imagePath ?? this.imagePath,
    imageByteLength: imageByteLength ?? this.imageByteLength,
    imageSha256: imageSha256 ?? this.imageSha256,
    capturedAt: capturedAt,
    targetYawDegrees: targetYawDegrees,
    targetPitchDegrees: targetPitchDegrees,
    cameraPosition: cameraPosition,
    cameraOrientation: cameraOrientation,
    alignmentErrorDeg: alignmentErrorDeg,
    anchored: anchored,
    anchorIdentifier: anchorIdentifier,
    anchorSurfaceType: anchorSurfaceType,
    imageWidth: imageWidth,
    imageHeight: imageHeight,
    sourceImageWidth: sourceImageWidth,
    sourceImageHeight: sourceImageHeight,
    captureTimestampNs: captureTimestampNs,
    focalLengthXPx: focalLengthXPx,
    focalLengthYPx: focalLengthYPx,
    principalPointXPx: principalPointXPx,
    principalPointYPx: principalPointYPx,
    imageFormat: imageFormat,
    rotationAppliedDeg: rotationAppliedDeg,
    toleranceDeg: toleranceDeg,
  );
}
