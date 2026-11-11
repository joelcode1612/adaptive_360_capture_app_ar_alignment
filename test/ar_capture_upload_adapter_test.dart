import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/application/ar_capture_upload_adapter.dart';
import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

void main() {
  const stationId = 'station-26';

  CaptureStation station({int capturedCount = kCaptureTargetCount}) =>
      CaptureStation(
        stationId: stationId,
        referencePosition: Vec3.zero,
        referenceOrientation: Quat.identity,
        createdAt: DateTime.utc(2026, 11, 11),
        trackingStatus: TrackingState.tracking,
        capturedCount: capturedCount,
        total: kCaptureTargetCount,
      );

  CaptureTarget target(int id) {
    final spec = buildCaptureSphere().firstWhere(
      (CaptureTargetSpec item) => item.targetId == id,
    );
    return CaptureTarget(
      targetId: id,
      stationId: stationId,
      yawDegrees: spec.yawDegrees,
      pitchDegrees: spec.pitchDegrees,
      stationLocalDirection: spec.direction,
      status: CaptureTargetStatus.captured,
      imagePath: 'C:/captures/t$id.jpg',
    );
  }

  CaptureRecord record(int id, {String? sha256}) => CaptureRecord(
    stationId: stationId,
    targetId: id,
    imagePath: 'C:/captures/t$id.jpg',
    imageByteLength: 4096,
    imageSha256: sha256 ?? List<String>.filled(64, 'a').join(),
    capturedAt: DateTime.utc(2026, 11, 11, 12, 0, id),
    targetYawDegrees: id.toDouble(),
    targetPitchDegrees: -id.toDouble(),
    cameraPosition: Vec3(id.toDouble(), 1, 2),
    cameraOrientation: Quat.identity,
    alignmentErrorDeg: 0.5,
    anchored: true,
    imageWidth: 640,
    imageHeight: 480,
  );

  CapturePersistedState completeState() => CapturePersistedState(
    station: station(),
    targets: <CaptureTarget>[
      for (var id = 1; id <= kCaptureTargetCount; id++) target(id),
    ],
    records: <CaptureRecord>[
      for (var id = 1; id <= kCaptureTargetCount; id++) record(id),
    ],
  );

  test('maps the verified AR plan to exactly 26 ordered upload captures', () {
    final uploads = ArCaptureUploadAdapter.fromPersistedState(completeState());

    expect(ArCaptureUploadPlan.id, 'arcore-station-26-v1');
    expect(uploads, hasLength(kCaptureTargetCount));
    expect(
      uploads.map((TourCapture capture) => capture.targetIndex).toList(),
      List<int>.generate(kCaptureTargetCount, (int index) => index + 1),
    );
    expect(uploads.first.nodeId, 'ar-$stationId-t01');
    expect(uploads.last.nodeId, 'ar-$stationId-t26');
    expect(uploads[7].yawDeg, 8);
    expect(uploads[7].pitchDeg, -8);
    expect(uploads[7].pose!.positionX, 8);
    expect(uploads[7].pose!.alignmentErrorDeg, 0.5);
  });

  test('refuses a partial station rather than creating a smaller tour', () {
    final state = CapturePersistedState(
      station: station(capturedCount: kCaptureTargetCount - 1),
      targets: completeState().targets.take(kCaptureTargetCount - 1).toList(),
      records: completeState().records.take(kCaptureTargetCount - 1).toList(),
    );

    expect(
      () => ArCaptureUploadAdapter.fromPersistedState(state),
      throwsA(isA<VirtualTourFailure>()),
    );
  });

  test('refuses legacy or malformed integrity metadata', () {
    final records = List<CaptureRecord>.of(completeState().records);
    records[5] = record(6, sha256: 'not-a-sha');
    final state = CapturePersistedState(
      station: station(),
      targets: completeState().targets,
      records: records,
    );

    expect(
      () => ArCaptureUploadAdapter.fromPersistedState(state),
      throwsA(
        isA<VirtualTourFailure>().having(
          (VirtualTourFailure error) => error.retryable,
          'retryable',
          isFalse,
        ),
      ),
    );
  });

  test(
    'rechecks a missing accepted frame before an upload can start',
    () async {
      await expectLater(
        ArCaptureUploadAdapter.verifiedForUpload(completeState()),
        throwsA(
          isA<VirtualTourFailure>().having(
            (VirtualTourFailure error) => error.message,
            'message',
            contains('missing from local storage'),
          ),
        ),
      );
    },
  );
}
