import 'dart:io';

import 'package:crypto/crypto.dart';
import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';

/// The locally verified AR capture plan that is eligible for remote ingestion.
///
/// The value is intentionally explicit rather than inferred from a list length.
/// The current backend protocol carries the fixed expected count but has no
/// persisted plan-version field yet; adding that server field needs a reviewed
/// migration. This adapter therefore refuses to present any other local capture
/// shape as the production AR upload set.
abstract final class ArCaptureUploadPlan {
  static const String id = 'arcore-station-26-v1';
  static const int targetCount = kCaptureTargetCount;
}

/// Translates one verified local AR station into the existing tour-upload wire
/// model.
///
/// No file is uploaded here. [HttpVirtualTourRemoteDataSource] performs the
/// multipart request. [verifiedForUpload] provides the corresponding disk
/// recheck before a caller starts an outbox/network operation. This seam makes
/// the important precondition testable without opening a camera or a socket:
/// only an exact, integrity-attested 26-target manifest is eligible.
abstract final class ArCaptureUploadAdapter {
  static List<TourCapture> fromPersistedState(CapturePersistedState state) {
    final station = state.station;
    if (station.total != ArCaptureUploadPlan.targetCount ||
        station.capturedCount != ArCaptureUploadPlan.targetCount ||
        state.targets.length != ArCaptureUploadPlan.targetCount ||
        state.records.length != ArCaptureUploadPlan.targetCount) {
      throw const VirtualTourFailure(
        'The AR station is not a complete 26-photograph capture plan. Finish '
        'or recover the station before uploading.',
        retryable: false,
      );
    }

    final targetsById = <int, CaptureTarget>{
      for (final target in state.targets) target.targetId: target,
    };
    final recordsById = <int, CaptureRecord>{
      for (final record in state.records) record.targetId: record,
    };
    for (
      var targetId = 1;
      targetId <= ArCaptureUploadPlan.targetCount;
      targetId++
    ) {
      final target = targetsById[targetId];
      final record = recordsById[targetId];
      if (target == null ||
          record == null ||
          !target.status.isDone ||
          target.stationId != station.stationId ||
          target.imagePath != record.imagePath ||
          record.stationId != station.stationId ||
          record.imagePath.isEmpty ||
          record.imageByteLength == null ||
          record.imageByteLength! <= 0 ||
          !_isSha256(record.imageSha256) ||
          record.imageFormat.toLowerCase() != 'jpeg') {
        throw VirtualTourFailure(
          'Target $targetId is not a complete, integrity-verified JPEG in '
          'the AR capture manifest. Reopen or recapture it before uploading.',
          retryable: false,
        );
      }
    }
    if (targetsById.length != ArCaptureUploadPlan.targetCount ||
        recordsById.length != ArCaptureUploadPlan.targetCount) {
      throw const VirtualTourFailure(
        'The AR capture manifest has duplicate or out-of-range target ids.',
        retryable: false,
      );
    }

    return <TourCapture>[
      for (
        var targetId = 1;
        targetId <= ArCaptureUploadPlan.targetCount;
        targetId++
      )
        _tourCapture(station.stationId, recordsById[targetId]!),
    ];
  }

  /// Rechecks each accepted file against the immutable-manifest digest just
  /// before it is handed to an upload outbox.
  ///
  /// A process can retain a loaded [CapturePersistedState] while external
  /// storage is damaged or a file is removed. Do not turn that stale memory
  /// into a remote session: the upload must start only if these exact bytes
  /// still exist. The HTTP data source repeats its own JPEG-decode check at the
  /// request boundary.
  static Future<List<TourCapture>> verifiedForUpload(
    CapturePersistedState state,
  ) async {
    final captures = fromPersistedState(state);
    final recordsById = <int, CaptureRecord>{
      for (final record in state.records) record.targetId: record,
    };
    for (final capture in captures) {
      final record = recordsById[capture.targetIndex]!;
      final file = File(record.imagePath);
      if (!await file.exists()) {
        throw VirtualTourFailure(
          'Target ${record.targetId} is missing from local storage. Recover '
          'or recapture it before uploading.',
          retryable: false,
        );
      }
      try {
        final bytes = await file.readAsBytes();
        if (bytes.length != record.imageByteLength ||
            sha256.convert(bytes).toString() != record.imageSha256) {
          throw VirtualTourFailure(
            'Target ${record.targetId} no longer matches its accepted JPEG '
            'digest. Recover or recapture it before uploading.',
            retryable: false,
          );
        }
      } on FileSystemException {
        throw VirtualTourFailure(
          'Target ${record.targetId} could not be read from local storage. '
          'Recover or recapture it before uploading.',
          retryable: false,
        );
      }
    }
    return captures;
  }

  static TourCapture _tourCapture(String stationId, CaptureRecord record) =>
      TourCapture(
        targetIndex: record.targetId,
        nodeId: 'ar-$stationId-t${record.targetId.toString().padLeft(2, '0')}',
        filePath: record.imagePath,
        yawDeg: record.targetYawDegrees,
        pitchDeg: record.targetPitchDegrees,
        capturedAt: record.capturedAt,
        pose: CapturePose(
          positionX: record.cameraPosition.x,
          positionY: record.cameraPosition.y,
          positionZ: record.cameraPosition.z,
          quaternionX: record.cameraOrientation.x,
          quaternionY: record.cameraOrientation.y,
          quaternionZ: record.cameraOrientation.z,
          quaternionW: record.cameraOrientation.w,
          alignmentErrorDeg: record.alignmentErrorDeg,
        ),
      );

  static bool _isSha256(String? value) =>
      value != null && RegExp(r'^[a-fA-F0-9]{64}$').hasMatch(value);
}
