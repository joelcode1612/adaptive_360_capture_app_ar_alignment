/// The virtual-tour ingestion contract: the photographs the AR screen produced,
/// the answers the SiteSense API gives, and the rows the local tables hold.
///
/// Pure Dart. Nothing here imports Flutter, `dart:io` or a plugin, so the data
/// layer, the Bloc and the tests all describe the same records without a device
/// or a binding.
///
/// Field names split on purpose. [TourCapture] and [VirtualTourRecord] use
/// camelCase because they are the local JSON tables; [VirtualTourTicket],
/// [VirtualTourCaptureAck] and [CapturePose] use snake_case because they mirror
/// the Pydantic schemas in `backend/src/sitesense/api/v1/virtual_tours.py`,
/// which is the authoritative contract.
library;

/// Where a failed request failed, so the UI can say more than "something went
/// wrong" and a retry can explain what it will retry.
enum VirtualTourStage {
  /// The backend is being asked for a tour id to upload into.
  creating,

  /// The JPEGs are being sent, one request per capture.
  uploading,

  /// Every capture is stored and the backend is validating the count and
  /// writing the stitch manifest.
  finalising,

  /// Reading the server's answer for a tour it already knows about.
  status,

  /// The request never left the app: bad input, a missing file, no session.
  validation,
}

extension VirtualTourStageLabel on VirtualTourStage {
  String get label => switch (this) {
    VirtualTourStage.creating => 'Creating the tour',
    VirtualTourStage.uploading => 'Uploading photographs',
    VirtualTourStage.finalising => 'Finalising the tour',
    VirtualTourStage.status => 'Reading the tour status',
    VirtualTourStage.validation => 'Cannot continue',
  };
}

/// The status vocabulary the backend stores in `capture_status`
/// (`backend/src/sitesense/db/models/virtual_tour.py`).
///
/// Listing it here is what lets the app tell `stitch_pending` — the manifest is
/// written and no worker has run yet — apart from `stitched`, which this phase
/// must never claim for itself: there is no panorama to show.
abstract final class VirtualTourStatus {
  static const String uploading = 'uploading';
  static const String stitchPending = 'stitch_pending';
  static const String stitching = 'stitching';
  static const String stitched = 'stitched';
  static const String failed = 'failed';

  static const Set<String> known = <String>{
    uploading,
    stitchPending,
    stitching,
    stitched,
    failed,
  };
}

/// A failure that is safe to show to the user as-is.
///
/// [retryable] is not cosmetic: a missing photograph must not offer a retry that
/// will fail identically, while a timeout must. The Bloc copies both fields into
/// its error state and the screen enables its Retry button from it.
class VirtualTourFailure implements Exception {
  final String message;
  final bool retryable;

  /// The HTTP status that produced this failure, when one did.
  ///
  /// The caller needs it for one decision no message can carry: a `409` from
  /// finalise means either "you are finished" or "you are not ready", and only
  /// asking the server again tells them apart.
  final int? statusCode;

  const VirtualTourFailure(
    this.message, {
    this.retryable = true,
    this.statusCode,
  });

  @override
  String toString() => 'VirtualTourFailure($message)';
}

/// The camera pose at the instant a frame was accepted, in the shape the
/// backend's `pose_json` form field expects.
///
/// Every field is optional server-side, so an unmeasured value stays absent
/// instead of being invented as zero — [toJson] omits the key, which reproduces
/// the server's own `exclude_none`. Units match the schema: metres for position
/// and station drift, a unit quaternion for orientation, degrees for alignment
/// error.
class CapturePose {
  final double? positionX;
  final double? positionY;
  final double? positionZ;
  final double? quaternionX;
  final double? quaternionY;
  final double? quaternionZ;
  final double? quaternionW;
  final double? alignmentErrorDeg;
  final double? stationDriftM;

  const CapturePose({
    this.positionX,
    this.positionY,
    this.positionZ,
    this.quaternionX,
    this.quaternionY,
    this.quaternionZ,
    this.quaternionW,
    this.alignmentErrorDeg,
    this.stationDriftM,
  });

  Map<String, Object?> toJson() => <String, Object?>{
    if (positionX != null) 'position_x': positionX,
    if (positionY != null) 'position_y': positionY,
    if (positionZ != null) 'position_z': positionZ,
    if (quaternionX != null) 'quaternion_x': quaternionX,
    if (quaternionY != null) 'quaternion_y': quaternionY,
    if (quaternionZ != null) 'quaternion_z': quaternionZ,
    if (quaternionW != null) 'quaternion_w': quaternionW,
    if (alignmentErrorDeg != null) 'alignment_error_deg': alignmentErrorDeg,
    if (stationDriftM != null) 'station_drift_m': stationDriftM,
  };

  static CapturePose fromJson(Map<String, Object?> m) => CapturePose(
    positionX: (m['position_x'] as num?)?.toDouble(),
    positionY: (m['position_y'] as num?)?.toDouble(),
    positionZ: (m['position_z'] as num?)?.toDouble(),
    quaternionX: (m['quaternion_x'] as num?)?.toDouble(),
    quaternionY: (m['quaternion_y'] as num?)?.toDouble(),
    quaternionZ: (m['quaternion_z'] as num?)?.toDouble(),
    quaternionW: (m['quaternion_w'] as num?)?.toDouble(),
    alignmentErrorDeg: (m['alignment_error_deg'] as num?)?.toDouble(),
    stationDriftM: (m['station_drift_m'] as num?)?.toDouble(),
  );
}

/// One photograph captured at one world-locked node, ready to be uploaded.
///
/// [targetIndex] is the number the backend keys a capture by: it is half of the
/// server-side unique constraint `(tour_id, target_index)` and the slot the
/// stitch manifest orders by, so it is carried explicitly rather than re-derived
/// from list position at the wire. It is 1-based, matching
/// `target_index: int = Form(..., ge=1)`.
class TourCapture {
  final int targetIndex;
  final String nodeId;
  final String filePath;
  final double yawDeg;
  final double pitchDeg;
  final DateTime capturedAt;
  final CapturePose? pose;

  const TourCapture({
    required this.targetIndex,
    required this.nodeId,
    required this.filePath,
    required this.yawDeg,
    required this.pitchDeg,
    required this.capturedAt,
    this.pose,
  });

  Map<String, Object?> toJson() => <String, Object?>{
    'targetIndex': targetIndex,
    'nodeId': nodeId,
    'filePath': filePath,
    'yawDeg': yawDeg,
    'pitchDeg': pitchDeg,
    'capturedAt': capturedAt.toIso8601String(),
    'pose': pose?.toJson(),
  };

  static TourCapture fromJson(Map<String, Object?> m) => TourCapture(
    targetIndex: (m['targetIndex']! as num).toInt(),
    nodeId: m['nodeId']! as String,
    filePath: m['filePath']! as String,
    yawDeg: (m['yawDeg']! as num).toDouble(),
    pitchDeg: (m['pitchDeg']! as num).toDouble(),
    capturedAt: DateTime.parse(m['capturedAt']! as String),
    pose: m['pose'] == null
        ? null
        : CapturePose.fromJson((m['pose']! as Map).cast<String, Object?>()),
  );

  @override
  String toString() => 'TourCapture($targetIndex, $nodeId, $filePath)';
}

/// The backend's `TourResponse`: the state of one tour on the server.
///
/// [tourId] is always the id the backend generated — the app never invents one,
/// because every later request is addressed by it. An unrecognised [status] is
/// kept verbatim rather than mapped onto a known value, so a server that learns a
/// new state shows up as itself instead of as a guessed equivalent.
class VirtualTourTicket {
  final String tourId;
  final String propertyId;
  final String status;
  final int expectedCaptureCount;
  final int uploadedCaptureCount;

  /// Set by the backend once `finalise` has written the manifest. This is a
  /// pointer to a manifest, not an image: no panorama exists at this point.
  final String? stitchManifestKey;

  /// True only for the declared simulation, never for a server answer.
  final bool simulated;

  const VirtualTourTicket({
    required this.tourId,
    required this.propertyId,
    required this.status,
    required this.expectedCaptureCount,
    required this.uploadedCaptureCount,
    this.stitchManifestKey,
    this.simulated = false,
  });

  bool get isFullyUploaded =>
      expectedCaptureCount > 0 && uploadedCaptureCount >= expectedCaptureCount;

  /// Finalised: the manifest is stored and a stitching worker has not run.
  bool get awaitingStitching => status == VirtualTourStatus.stitchPending;

  bool get isFailed => status == VirtualTourStatus.failed;

  Map<String, Object?> toJson() => <String, Object?>{
    'id': tourId,
    'property_id': propertyId,
    'status': status,
    'expected_capture_count': expectedCaptureCount,
    'uploaded_capture_count': uploadedCaptureCount,
    'stitch_manifest_key': stitchManifestKey,
    'simulated': simulated,
  };

  static VirtualTourTicket fromJson(Map<String, Object?> m) =>
      VirtualTourTicket(
        tourId: m['id']! as String,
        propertyId: m['property_id']! as String,
        status: m['status']! as String,
        expectedCaptureCount: (m['expected_capture_count']! as num).toInt(),
        uploadedCaptureCount: (m['uploaded_capture_count']! as num).toInt(),
        stitchManifestKey: m['stitch_manifest_key'] as String?,
        simulated: m['simulated'] == true,
      );

  @override
  String toString() =>
      'VirtualTourTicket($tourId, $status, '
      '$uploadedCaptureCount/$expectedCaptureCount, '
      'manifest=${stitchManifestKey ?? '—'})';
}

/// The backend's `CaptureResponse`: one accepted photograph.
///
/// [duplicate] is the server saying these exact bytes were already stored for
/// this target index — the retry case that must count as success rather than as a
/// second photograph.
class VirtualTourCaptureAck {
  final int targetIndex;
  final String storageKey;
  final String sha256;
  final int width;
  final int height;
  final bool duplicate;

  const VirtualTourCaptureAck({
    required this.targetIndex,
    required this.storageKey,
    required this.sha256,
    required this.width,
    required this.height,
    required this.duplicate,
  });

  Map<String, Object?> toJson() => <String, Object?>{
    'target_index': targetIndex,
    'storage_key': storageKey,
    'sha256': sha256,
    'width': width,
    'height': height,
    'duplicate': duplicate,
  };

  static VirtualTourCaptureAck fromJson(Map<String, Object?> m) =>
      VirtualTourCaptureAck(
        targetIndex: (m['target_index']! as num).toInt(),
        storageKey: m['storage_key']! as String,
        sha256: m['sha256']! as String,
        width: (m['width']! as num).toInt(),
        height: (m['height']! as num).toInt(),
        duplicate: m['duplicate'] == true,
      );

  @override
  String toString() =>
      'VirtualTourCaptureAck($targetIndex, $sha256, '
      '${width}x$height${duplicate ? ', duplicate' : ''})';
}

/// One row of the local `VIRTUAL_TOUR` table.
///
/// **Why JSON files and not SQL.** The server owns its schema; this table is the
/// device's record of what it captured and how far the upload got, so that a
/// walk-away or a crash does not lose the backend tour id. [backendTourId] is
/// that link: the local [id] is chosen by the app, the server's is not, and
/// keeping both is what stops a retry from creating a second tour.
///
/// [status] is the local view; [backendStatus] is the last value the server
/// actually reported. They are stored apart on purpose — a local row must never
/// upgrade itself into a state the backend did not confirm.
class VirtualTourRecord {
  final String id;
  final String propertyId;
  final String stationId;
  final String status;
  final String? backendTourId;
  final String? backendStatus;
  final String? stitchManifestKey;
  final int expectedCaptureCount;
  final int uploadedCaptureCount;
  final int photoCount;
  final DateTime createdAt;
  final DateTime updatedAt;

  /// True only when nothing was sent to a real server.
  final bool simulated;

  const VirtualTourRecord({
    required this.id,
    required this.propertyId,
    required this.stationId,
    required this.status,
    required this.photoCount,
    required this.createdAt,
    required this.updatedAt,
    this.backendTourId,
    this.backendStatus,
    this.stitchManifestKey,
    this.expectedCaptureCount = 0,
    this.uploadedCaptureCount = 0,
    this.simulated = false,
  });

  VirtualTourRecord copyWith({
    String? status,
    String? backendTourId,
    String? backendStatus,
    String? stitchManifestKey,
    int? expectedCaptureCount,
    int? uploadedCaptureCount,
    int? photoCount,
    DateTime? updatedAt,
    bool? simulated,
  }) => VirtualTourRecord(
    id: id,
    propertyId: propertyId,
    stationId: stationId,
    status: status ?? this.status,
    backendTourId: backendTourId ?? this.backendTourId,
    backendStatus: backendStatus ?? this.backendStatus,
    stitchManifestKey: stitchManifestKey ?? this.stitchManifestKey,
    expectedCaptureCount: expectedCaptureCount ?? this.expectedCaptureCount,
    uploadedCaptureCount: uploadedCaptureCount ?? this.uploadedCaptureCount,
    photoCount: photoCount ?? this.photoCount,
    createdAt: createdAt,
    updatedAt: updatedAt ?? this.updatedAt,
    simulated: simulated ?? this.simulated,
  );

  Map<String, Object?> toJson() => <String, Object?>{
    'id': id,
    'propertyId': propertyId,
    'stationId': stationId,
    'status': status,
    'backendTourId': backendTourId,
    'backendStatus': backendStatus,
    'stitchManifestKey': stitchManifestKey,
    'expectedCaptureCount': expectedCaptureCount,
    'uploadedCaptureCount': uploadedCaptureCount,
    'photoCount': photoCount,
    'createdAt': createdAt.toIso8601String(),
    'updatedAt': updatedAt.toIso8601String(),
    'simulated': simulated,
  };

  static VirtualTourRecord fromJson(Map<String, Object?> m) =>
      VirtualTourRecord(
        id: m['id']! as String,
        propertyId: m['propertyId']! as String,
        stationId: m['stationId']! as String,
        status: m['status']! as String,
        backendTourId: m['backendTourId'] as String?,
        backendStatus: m['backendStatus'] as String?,
        stitchManifestKey: m['stitchManifestKey'] as String?,
        expectedCaptureCount:
            (m['expectedCaptureCount'] as num?)?.toInt() ?? 0,
        uploadedCaptureCount:
            (m['uploadedCaptureCount'] as num?)?.toInt() ?? 0,
        photoCount: (m['photoCount'] as num?)?.toInt() ?? 0,
        createdAt: DateTime.parse(m['createdAt']! as String),
        updatedAt: DateTime.parse(m['updatedAt']! as String),
        simulated: m['simulated'] == true,
      );
}

/// One row of the local `PROPERTY_PHOTO` table.
///
/// Written once per captured node. [storageKey] and [sha256] are what the server
/// answered with, so a resumed session can tell "uploaded and accepted" apart
/// from "captured on this device and never sent" without asking the network.
class PropertyPhotoRecord {
  final String id;
  final String virtualTourId;
  final String propertyId;
  final String nodeId;
  final String kind;
  final String location;
  final int targetIndex;
  final double yawDeg;
  final double pitchDeg;
  final int width;
  final int height;
  final String? storageKey;
  final String? sha256;
  final DateTime capturedAt;
  final DateTime createdAt;

  const PropertyPhotoRecord({
    required this.id,
    required this.virtualTourId,
    required this.propertyId,
    required this.nodeId,
    required this.kind,
    required this.location,
    required this.capturedAt,
    required this.createdAt,
    this.targetIndex = 0,
    this.yawDeg = 0,
    this.pitchDeg = 0,
    this.width = 0,
    this.height = 0,
    this.storageKey,
    this.sha256,
  });

  static const String kindCapture = 'capture';

  /// True once the backend has acknowledged this capture with a real digest.
  ///
  /// The digest has to be non-empty: the mock data source stores bytes on this
  /// device and no server hashed them, so an empty [sha256] is exactly the
  /// difference between "saved here" and "accepted by the tour service".
  bool get isUploaded =>
      storageKey != null && sha256 != null && (sha256 ?? '').isNotEmpty;

  Map<String, Object?> toJson() => <String, Object?>{
    'id': id,
    'virtualTourId': virtualTourId,
    'propertyId': propertyId,
    'nodeId': nodeId,
    'kind': kind,
    'location': location,
    'targetIndex': targetIndex,
    'yawDeg': yawDeg,
    'pitchDeg': pitchDeg,
    'width': width,
    'height': height,
    'storageKey': storageKey,
    'sha256': sha256,
    'capturedAt': capturedAt.toIso8601String(),
    'createdAt': createdAt.toIso8601String(),
  };

  static PropertyPhotoRecord fromJson(Map<String, Object?> m) =>
      PropertyPhotoRecord(
        id: m['id']! as String,
        virtualTourId: m['virtualTourId']! as String,
        propertyId: m['propertyId']! as String,
        nodeId: m['nodeId']! as String,
        kind: m['kind']! as String,
        location: m['location']! as String,
        targetIndex: (m['targetIndex'] as num?)?.toInt() ?? 0,
        yawDeg: (m['yawDeg'] as num?)?.toDouble() ?? 0,
        pitchDeg: (m['pitchDeg'] as num?)?.toDouble() ?? 0,
        width: (m['width'] as num?)?.toInt() ?? 0,
        height: (m['height'] as num?)?.toInt() ?? 0,
        storageKey: m['storageKey'] as String?,
        sha256: m['sha256'] as String?,
        capturedAt: DateTime.parse(m['capturedAt']! as String),
        createdAt: DateTime.parse(m['createdAt']! as String),
      );
}
