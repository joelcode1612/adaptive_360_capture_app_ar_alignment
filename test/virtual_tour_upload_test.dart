import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:path/path.dart' as p;

import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_local_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_remote_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/repositories/virtual_tour_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/services/virtual_tour_photo_store.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/repositories/virtual_tour_repository.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/bloc/virtual_tour_bloc.dart';

const String propertyUuid = '22222222-2222-2222-2222-222222222222';
const String backendTourUuid = '33333333-3333-3333-3333-333333333333';
const String manifestKey =
    'tours/$backendTourUuid/stitch-manifest-0123456789abcdef.json';

final DateTime capturedAt = DateTime.utc(2026, 10, 25, 9, 30);

/// A structurally valid JPEG: SOI, an SOF0 carrying [width]×[height], SOS,
/// padding and EOI — everything `VirtualTourPhotoStore.inspectJpeg` reads.
Uint8List jpegFixture({int width = 1, int height = 1, int padding = 32}) =>
    Uint8List.fromList(<int>[
      0xFF, 0xD8, // SOI
      0xFF, 0xC0, 0x00, 0x11, 0x08, // SOF0, length 17, 8-bit samples
      (height >> 8) & 0xFF, height & 0xFF,
      (width >> 8) & 0xFF, width & 0xFF,
      0x03, 0x01, 0x11, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01,
      0xFF, 0xDA, // SOS
      ...List<int>.filled(padding, 0),
      0xFF, 0xD9, // EOI
    ]);

/// `TourResponse` as `backend/src/sitesense/api/v1/virtual_tours.py` answers it.
Map<String, Object?> tourJson({
  String id = backendTourUuid,
  String status = VirtualTourStatus.uploading,
  int expected = 3,
  int uploaded = 0,
  String? manifest,
}) => <String, Object?>{
  'id': id,
  'property_id': propertyUuid,
  'status': status,
  'expected_capture_count': expected,
  'uploaded_capture_count': uploaded,
  'stitch_manifest_key': manifest,
};

Map<String, Object?> captureAckJson({
  int targetIndex = 1,
  String storageKey =
      'tours/$backendTourUuid/captures/001-0123456789abcdef.jpg',
  String sha256 =
      '0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef',
  int width = 4000,
  int height = 3000,
  bool duplicate = false,
}) => <String, Object?>{
  'target_index': targetIndex,
  'storage_key': storageKey,
  'sha256': sha256,
  'width': width,
  'height': height,
  'duplicate': duplicate,
};

http.Response jsonResponse(Object? body, {int status = 200}) => http.Response(
  jsonEncode(body),
  status,
  headers: <String, String>{'content-type': 'application/json'},
);

VirtualTourTicket ticketFor(
  int count, {
  String status = VirtualTourStatus.uploading,
  String? manifest,
  int? uploaded,
  bool simulated = false,
}) => VirtualTourTicket(
  tourId: backendTourUuid,
  propertyId: propertyUuid,
  status: status,
  expectedCaptureCount: count,
  uploadedCaptureCount: uploaded ?? count,
  stitchManifestKey: manifest,
  simulated: simulated,
);

List<TourCapture> capturesFor(
  int count, {
  String directory = '/tmp/tour',
  int firstIndex = 1,
}) => <TourCapture>[
  for (var i = 0; i < count; i++)
    TourCapture(
      targetIndex: firstIndex + i,
      nodeId: 'n${(firstIndex + i).toString().padLeft(2, '0')}',
      filePath: '$directory/${firstIndex + i}.jpg',
      yawDeg: (firstIndex + i - 1) * 12,
      pitchDeg: (firstIndex + i).isEven ? 0.0 : 35.0,
      capturedAt: capturedAt.add(Duration(seconds: i)),
      pose: CapturePose(
        positionX: 1.0 + i,
        positionY: 0.5,
        positionZ: -2.0,
        quaternionW: 1.0,
        alignmentErrorDeg: 1.5 + i,
      ),
    ),
];

/// The backend, in memory, with the counters and the failures the tests ask for.
class FakeRemote implements VirtualTourRemoteDataSource {
  FakeRemote({this.expectedCaptureCount = 3});

  int createCalls = 0;
  int uploadCalls = 0;
  int finaliseCalls = 0;
  int statusCalls = 0;

  String? createdPropertyId;
  int? createdExpectedCount;
  final List<String> uploadTourIds = <String>[];
  final List<TourCapture> uploads = <TourCapture>[];
  final List<String> finaliseTourIds = <String>[];
  final List<String> statusTourIds = <String>[];

  int createFailuresRemaining = 0;
  int uploadFailuresRemaining = 0;
  int finaliseFailuresRemaining = 0;
  int statusFailuresRemaining = 0;
  VirtualTourFailure failure = const VirtualTourFailure(
    'The backend is unavailable.',
  );

  Completer<VirtualTourCaptureAck>? uploadGate;
  Completer<VirtualTourTicket>? finaliseGate;

  String status = VirtualTourStatus.uploading;
  int expectedCaptureCount;
  String? manifestKey;
  final Set<int> accepted = <int>{};

  @override
  Future<VirtualTourTicket> createTour({
    required String propertyId,
    required int expectedCaptureCount,
  }) async {
    createCalls++;
    createdPropertyId = propertyId;
    createdExpectedCount = expectedCaptureCount;
    if (createFailuresRemaining > 0) {
      createFailuresRemaining--;
      throw failure;
    }
    this.expectedCaptureCount = expectedCaptureCount;
    return ticket();
  }

  @override
  Future<VirtualTourCaptureAck> uploadCapture({
    required String tourId,
    required TourCapture capture,
  }) async {
    uploadCalls++;
    uploadTourIds.add(tourId);
    uploads.add(capture);
    if (uploadFailuresRemaining > 0) {
      uploadFailuresRemaining--;
      throw failure;
    }
    final gate = uploadGate;
    if (gate != null) await gate.future;
    final duplicate = accepted.contains(capture.targetIndex);
    accepted.add(capture.targetIndex);
    return VirtualTourCaptureAck(
      targetIndex: capture.targetIndex,
      storageKey: 'tours/$tourId/captures/${capture.targetIndex}.jpg',
      sha256: 'digest-${capture.targetIndex}',
      width: 4000,
      height: 3000,
      duplicate: duplicate,
    );
  }

  @override
  Future<VirtualTourTicket> finaliseTour({required String tourId}) async {
    finaliseCalls++;
    finaliseTourIds.add(tourId);
    if (finaliseFailuresRemaining > 0) {
      finaliseFailuresRemaining--;
      throw failure;
    }
    final gate = finaliseGate;
    if (gate != null) return gate.future;
    if (accepted.length < expectedCaptureCount) {
      throw VirtualTourFailure(
        'Only ${accepted.length} of $expectedCaptureCount captures exist.',
        retryable: false,
        statusCode: 409,
      );
    }
    status = VirtualTourStatus.stitchPending;
    manifestKey = manifestKey ?? 'tours/$tourId/stitch-manifest-abc.json';
    return ticket();
  }

  @override
  Future<VirtualTourTicket> tourStatus({required String tourId}) async {
    statusCalls++;
    statusTourIds.add(tourId);
    if (statusFailuresRemaining > 0) {
      statusFailuresRemaining--;
      throw failure;
    }
    return ticket();
  }

  VirtualTourTicket ticket() => VirtualTourTicket(
    tourId: backendTourUuid,
    propertyId: createdPropertyId ?? propertyUuid,
    status: status,
    expectedCaptureCount: expectedCaptureCount,
    uploadedCaptureCount: accepted.length,
    stitchManifestKey: manifestKey,
  );

  @override
  Future<void> dispose() async {}
}

/// The repository seam the Bloc is written against.
class FakeRepository implements VirtualTourRepository {
  int uploadCalls = 0;
  int finaliseCalls = 0;
  int refreshCalls = 0;

  final List<List<TourCapture>> receivedCaptures = <List<TourCapture>>[];
  final List<String> finaliseTourIds = <String>[];

  int uploadFailuresRemaining = 0;
  int finaliseFailuresRemaining = 0;
  int refreshFailuresRemaining = 0;
  VirtualTourFailure failure = const VirtualTourFailure(
    'The backend is unavailable.',
  );

  /// Accept this many captures, reporting progress, then fail.
  int? failAfterProgress;

  Completer<VirtualTourTicket>? uploadGate;
  Completer<VirtualTourTicket>? finaliseGate;

  VirtualTourTicket finaliseResult = ticketFor(
    3,
    status: VirtualTourStatus.stitchPending,
    manifest: manifestKey,
  );
  VirtualTourTicket refreshResult = ticketFor(
    3,
    status: VirtualTourStatus.stitching,
    manifest: manifestKey,
  );

  @override
  Future<VirtualTourTicket> uploadCaptures({
    required String tourId,
    required String propertyId,
    required String stationId,
    required List<TourCapture> captures,
    VirtualTourUploadProgress? onProgress,
  }) async {
    uploadCalls++;
    receivedCaptures.add(captures);
    if (uploadFailuresRemaining > 0) {
      uploadFailuresRemaining--;
      final stop = failAfterProgress;
      if (stop != null) {
        for (var i = 0; i < stop; i++) {
          onProgress?.call(i + 1, captures.length);
        }
      }
      throw failure;
    }
    final gate = uploadGate;
    if (gate != null) return gate.future;
    for (var i = 0; i < captures.length; i++) {
      onProgress?.call(i + 1, captures.length);
    }
    return ticketFor(captures.length);
  }

  @override
  Future<VirtualTourTicket> finaliseTour({required String tourId}) async {
    finaliseCalls++;
    finaliseTourIds.add(tourId);
    if (finaliseFailuresRemaining > 0) {
      finaliseFailuresRemaining--;
      throw failure;
    }
    final gate = finaliseGate;
    if (gate != null) return gate.future;
    return finaliseResult;
  }

  @override
  Future<VirtualTourTicket> refreshTourStatus({required String tourId}) async {
    refreshCalls++;
    if (refreshFailuresRemaining > 0) {
      refreshFailuresRemaining--;
      throw failure;
    }
    return refreshResult;
  }

  @override
  Future<VirtualTourRecord?> tourRecord(String tourId) async => null;
}

Future<void> settle() async {
  for (var i = 0; i < 24; i++) {
    await Future<void>.delayed(Duration.zero);
  }
}

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = await Directory.systemTemp.createTemp('virtual_tour_test_');
  });

  tearDown(() async {
    if (await tempDir.exists()) await tempDir.delete(recursive: true);
  });

  group('contract models', () {
    test('reads the server TourResponse with its snake_case keys', () {
      final ticket = VirtualTourTicket.fromJson(
        tourJson(
          status: 'stitch_pending',
          expected: 30,
          uploaded: 30,
          manifest: manifestKey,
        ),
      );
      expect(ticket.tourId, backendTourUuid);
      expect(ticket.propertyId, propertyUuid);
      expect(ticket.status, 'stitch_pending');
      expect(ticket.expectedCaptureCount, 30);
      expect(ticket.uploadedCaptureCount, 30);
      expect(ticket.stitchManifestKey, manifestKey);
      expect(ticket.isFullyUploaded, isTrue);
      expect(ticket.awaitingStitching, isTrue);
      expect(ticket.isFailed, isFalse);
      // A TourResponse carries no panorama, and nothing here invents one.
      expect(ticket.simulated, isFalse);
    });

    test('keeps an unknown server status verbatim', () {
      final ticket = VirtualTourTicket.fromJson(tourJson(status: 'quarantined'));
      expect(ticket.status, 'quarantined');
      expect(VirtualTourStatus.known.contains(ticket.status), isFalse);
    });

    test('reads CaptureResponse including duplicate', () {
      final ack = VirtualTourCaptureAck.fromJson(
        captureAckJson(targetIndex: 7, duplicate: true, width: 640, height: 480),
      );
      expect(ack.targetIndex, 7);
      expect(ack.duplicate, isTrue);
      expect(ack.width, 640);
      expect(ack.height, 480);
      expect(ack.storageKey, contains('captures/'));
    });

    test('CapturePose omits what was not measured, like exclude_none', () {
      expect(const CapturePose(positionX: 1.25).toJson(), <String, Object?>{
        'position_x': 1.25,
      });
      expect(const CapturePose().toJson(), isEmpty);
      final full = CapturePose.fromJson(<String, Object?>{
        'position_x': 1.0,
        'quaternion_w': 1.0,
        'alignment_error_deg': 2.5,
        'station_drift_m': 0.013,
      });
      expect(full.positionX, 1.0);
      expect(full.positionY, isNull);
      expect(full.alignmentErrorDeg, 2.5);
      expect(full.stationDriftM, 0.013);
    });
  });

  group('repository', () {
    late FakeRemote remote;
    late VirtualTourLocalDataSource local;
    late VirtualTourRepositoryImpl repository;

    setUp(() {
      remote = FakeRemote();
      local = VirtualTourLocalDataSource(root: tempDir);
      repository = VirtualTourRepositoryImpl(remote: remote, local: local);
    });

    test(
      'creates one backend session, uploads every capture, then records',
      () async {
        final progress = <String>[];
        final ticket = await repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: capturesFor(3),
          onProgress: (uploaded, total) => progress.add('$uploaded/$total'),
        );

        expect(remote.createCalls, 1);
        expect(remote.createdPropertyId, propertyUuid);
        expect(remote.createdExpectedCount, 3);
        expect(remote.uploadCalls, 3);
        // The server's id, never the app's, addresses every later call.
        expect(remote.uploadTourIds, <String>[
          backendTourUuid,
          backendTourUuid,
          backendTourUuid,
        ]);
        expect(progress, <String>['1/3', '2/3', '3/3']);
        expect(ticket.status, VirtualTourStatus.uploading);
        expect(ticket.uploadedCaptureCount, 3);
        expect(ticket.stitchManifestKey, isNull);
        expect(ticket.simulated, isFalse);

        final photos = await local.propertyPhotos('tour-1');
        expect(photos, hasLength(3));
        expect(
          photos.every((r) => r.kind == PropertyPhotoRecord.kindCapture),
          isTrue,
        );
        expect(photos.map((r) => r.targetIndex), <int>[1, 2, 3]);
        expect(photos.every((r) => r.isUploaded), isTrue);
        expect(photos.first.storageKey, contains('captures/'));
        expect(photos.first.sha256, 'digest-1');
        expect(photos.first.location, '/tmp/tour/1.jpg');
        expect(photos.first.yawDeg, 0);
        expect(photos.first.pitchDeg, 35);

        final tour = await local.virtualTour('tour-1');
        expect(tour, isNotNull);
        expect(tour!.backendTourId, backendTourUuid);
        expect(tour.status, VirtualTourStatus.uploading);
        expect(tour.backendStatus, VirtualTourStatus.uploading);
        expect(tour.expectedCaptureCount, 3);
        expect(tour.uploadedCaptureCount, 3);
        expect(tour.stitchManifestKey, isNull);
        expect(tour.simulated, isFalse);
      },
    );

    test('the backend session id is stored before any JPEG moves', () async {
      remote.uploadFailuresRemaining = 1;
      await expectLater(
        repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: capturesFor(2),
        ),
        throwsA(isA<VirtualTourFailure>()),
      );

      // The first capture failed, yet the session survives on disk — which is
      // the only reason a retry can resume instead of filing a second tour.
      final tour = await local.virtualTour('tour-1');
      expect(tour, isNotNull);
      expect(tour!.backendTourId, backendTourUuid);
      expect(tour.uploadedCaptureCount, 0);

      final resumed = await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: capturesFor(2),
      );
      expect(remote.createCalls, 1);
      expect(remote.statusCalls, 1);
      expect(resumed.uploadedCaptureCount, 2);
      expect(await local.listVirtualTours(), hasLength(1));
    });

    test('bad target indices fail before anything is sent', () async {
      await expectLater(
        repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: capturesFor(1, firstIndex: 0),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having(
                (f) => f.message,
                'message',
                contains('numbers captures from 1'),
              ),
        ),
      );
      await expectLater(
        repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: <TourCapture>[...capturesFor(1), ...capturesFor(1)],
        ),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('claim target index 1'),
          ),
        ),
      );
      expect(remote.createCalls, 0);
      expect(remote.uploadCalls, 0);
    });

    test('an empty capture list never reaches the backend', () async {
      await expectLater(
        repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: const <TourCapture>[],
        ),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.retryable,
            'retryable',
            isFalse,
          ),
        ),
      );
      expect(remote.createCalls, 0);
    });

    test('a session the server already moved on is not uploaded into', () async {
      await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: capturesFor(3),
      );
      remote.status = VirtualTourStatus.stitchPending;
      final before = remote.uploadCalls;

      await expectLater(
        repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: capturesFor(3),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('stitch_pending')),
        ),
      );
      expect(remote.uploadCalls, before);
      expect(remote.createCalls, 1);
    });

    test('a capture count that no longer matches is refused', () async {
      await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: capturesFor(3),
      );

      await expectLater(
        repository.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: capturesFor(4),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having(
                (f) => f.message,
                'message',
                contains('fixed when the tour is created'),
              ),
        ),
      );
      expect(remote.uploadCalls, 3);
    });

    test('re-sending accepted bytes counts each target once', () async {
      final captures = capturesFor(3);
      await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: captures,
      );
      final second = await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: captures,
      );

      expect(remote.uploadCalls, 6);
      // Three targets, six acknowledgements: the counter follows the backend's
      // (tour_id, target_index) key, not the number of requests.
      expect(second.uploadedCaptureCount, 3);
      expect(await local.propertyPhotos('tour-1'), hasLength(3));
      expect(await local.listVirtualTours(), hasLength(1));
    });

    test('finalising stores the server status and the manifest key', () async {
      await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: capturesFor(3),
      );

      final ticket = await repository.finaliseTour(tourId: 'tour-1');
      expect(remote.finaliseTourIds, <String>[backendTourUuid]);
      expect(ticket.status, VirtualTourStatus.stitchPending);
      expect(ticket.stitchManifestKey, isNotNull);
      expect(ticket.isFullyUploaded, isTrue);
      expect(ticket.awaitingStitching, isTrue);

      final tour = await local.virtualTour('tour-1');
      expect(tour!.status, VirtualTourStatus.stitchPending);
      expect(tour.stitchManifestKey, ticket.stitchManifestKey);
      // The table holds captures: there is no panorama row to claim.
      final photos = await local.propertyPhotos('tour-1');
      expect(
        photos.every((r) => r.kind == PropertyPhotoRecord.kindCapture),
        isTrue,
      );
    });

    test('finalising a tour that never reached the backend fails locally', () async {
      await expectLater(
        repository.finaliseTour(tourId: 'tour-9'),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having(
                (f) => f.message,
                'message',
                contains('Upload its photographs first'),
              ),
        ),
      );
      expect(remote.finaliseCalls, 0);
      await expectLater(
        repository.refreshTourStatus(tourId: 'tour-9'),
        throwsA(isA<VirtualTourFailure>()),
      );
      expect(remote.statusCalls, 0);
    });

    test('a refresh writes the backend word into the local row', () async {
      await repository.uploadCaptures(
        tourId: 'tour-1',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: capturesFor(3),
      );
      remote.status = 'quarantined';

      final ticket = await repository.refreshTourStatus(tourId: 'tour-1');
      expect(ticket.status, 'quarantined');
      final tour = await local.virtualTour('tour-1');
      expect(tour!.status, 'quarantined');
      expect(tour.backendStatus, 'quarantined');
      expect(ticket.uploadedCaptureCount, 3);
    });

    test('a local write failure says the backend has the data', () async {
      // A file where the table directory belongs: the session is created and
      // then cannot be recorded, which is a different failure from "the upload
      // broke" and has to be reported as such.
      final blocked = File(p.join(tempDir.path, 'blocked'))
        ..writeAsBytesSync(<int>[1, 2, 3]);
      final repo = VirtualTourRepositoryImpl(
        remote: FakeRemote(),
        local: VirtualTourLocalDataSource(root: Directory(blocked.path)),
      );

      await expectLater(
        repo.uploadCaptures(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: capturesFor(1),
        ),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('could not save'),
          ),
        ),
      );
    });

    test('a declared simulation stays labelled as one', () async {
      final captures = <TourCapture>[];
      for (var i = 1; i <= 2; i++) {
        final file = File(p.join(tempDir.path, 'sim_$i.jpg'))
          ..writeAsBytesSync(jpegFixture(width: 640, height: 480));
        captures.add(
          TourCapture(
            targetIndex: i,
            nodeId: 'n0$i',
            filePath: file.path,
            yawDeg: 0,
            pitchDeg: 0,
            capturedAt: capturedAt,
          ),
        );
      }
      final simRepo = VirtualTourRepositoryImpl(
        remote: LocalVirtualTourRemoteDataSource(),
        local: local,
      );

      final created = await simRepo.uploadCaptures(
        tourId: 'tour-sim',
        propertyId: propertyUuid,
        stationId: 'station-1',
        captures: captures,
      );
      expect(created.simulated, isTrue);
      expect(created.stitchManifestKey, isNull);

      final finalised = await simRepo.finaliseTour(tourId: 'tour-sim');
      expect(finalised.status, VirtualTourStatus.stitchPending);
      // No manifest was written anywhere, so none is claimed.
      expect(finalised.stitchManifestKey, isNull);

      final tour = await local.virtualTour('tour-sim');
      expect(tour!.simulated, isTrue);
      final photos = await local.propertyPhotos('tour-sim');
      expect(photos, hasLength(2));
      // Nothing here can read as "accepted by a tour service".
      expect(photos.every((r) => r.isUploaded), isFalse);
    });
  });

  group('HTTP remote', () {
    late File first;
    late File second;
    late VirtualTourApiConfig config;

    setUp(() async {
      first = File(p.join(tempDir.path, 'first.jpg'))
        ..writeAsBytesSync(jpegFixture(width: 640, height: 480));
      second = File(p.join(tempDir.path, 'second.jpg'))
        ..writeAsBytesSync(jpegFixture(width: 641, height: 481));
      config = VirtualTourApiConfig(
        baseUri: Uri.parse('https://api.example.test'),
        userId: VirtualTourApiConfig.defaultDevelopmentUserId,
      );
    });

    TourCapture captureOf(int targetIndex, {CapturePose? pose}) => TourCapture(
      targetIndex: targetIndex,
      nodeId: 'h${targetIndex.toString().padLeft(2, '0')}',
      filePath: targetIndex == 1 ? first.path : second.path,
      yawDeg: 12.0,
      pitchDeg: 0.0,
      capturedAt: capturedAt,
      pose: pose,
    );

    test('create posts the server schema and reads the id it generated', () async {
      final requests = <http.Request>[];
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          requests.add(request);
          return jsonResponse(tourJson(expected: 3), status: 201);
        }),
      );

      final ticket = await remote.createTour(
        propertyId: propertyUuid,
        expectedCaptureCount: 3,
      );

      expect(requests, hasLength(1));
      expect(requests.single.method, 'POST');
      expect(
        requests.single.url.toString(),
        'https://api.example.test/api/v1/virtual-tours',
      );
      expect(
        requests.single.headers['content-type'],
        contains('application/json'),
      );
      expect(
        requests.single.headers['X-User-Id'],
        VirtualTourApiConfig.defaultDevelopmentUserId,
      );
      expect(jsonDecode(requests.single.body), <String, Object?>{
        'property_id': propertyUuid,
        'expected_capture_count': 3,
      });
      expect(ticket.tourId, backendTourUuid);
      expect(ticket.status, VirtualTourStatus.uploading);
      expect(ticket.simulated, isFalse);
      await remote.dispose();
    });

    test('a property id that is not a UUID never leaves the app', () async {
      var calls = 0;
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          calls++;
          return jsonResponse(tourJson(), status: 201);
        }),
      );

      await expectLater(
        remote.createTour(
          propertyId: 'property_local',
          expectedCaptureCount: 3,
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('not a UUID')),
        ),
      );
      expect(calls, 0);
      await remote.dispose();
    });

    test('a capture is one multipart request with the server field names', () async {
      late http.Request seen;
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          seen = request;
          return jsonResponse(captureAckJson(), status: 201);
        }),
      );

      final ack = await remote.uploadCapture(
        tourId: backendTourUuid,
        capture: captureOf(
          1,
          pose: const CapturePose(positionX: 1.5, positionY: 0.25),
        ),
      );

      expect(seen.method, 'POST');
      expect(
        seen.url.toString(),
        'https://api.example.test/api/v1/virtual-tours/$backendTourUuid/captures',
      );
      expect(seen.headers['content-type'], contains('multipart/form-data'));
      expect(seen.headers['X-User-Id'], config.userId);
      expect(seen.headers['Idempotency-Key'], '$backendTourUuid/1');

      final body = latin1.decode(seen.bodyBytes);
      expect(body, contains('name="target_index"'));
      expect(body, contains('name="captured_at"'));
      expect(body, contains('2026-10-25T09:30:00.000Z'));
      expect(body, contains('name="pose_json"'));
      expect(body, contains('"position_x":1.5'));
      expect(body, isNot(contains('station_drift_m')));
      expect(body, contains('name="image"; filename="t001.jpg"'));
      // The client writes part headers lower-cased; the server does not care.
      expect(body.toLowerCase(), contains('content-type: image/jpeg'));
      // The route owns the tour id, and the schema has no manifest field.
      expect(body, isNot(contains('name="tour_id"')));
      expect(body, isNot(contains('name="manifest"')));
      // The frame itself is on the wire, not a placeholder: the exact bytes of
      // the file the capture points at appear in the request body, which also
      // carries the 640×480 SOF the server reads back.
      expect(
        latin1.decode(seen.bodyBytes),
        contains(latin1.decode(jpegFixture(width: 640, height: 480))),
      );

      expect(ack.targetIndex, 1);
      expect(ack.sha256, hasLength(64));
      expect(ack.width, 4000);
      expect(ack.duplicate, isFalse);
      await remote.dispose();
    });

    test('an unmeasured pose still satisfies the required form field', () async {
      late http.Request seen;
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          seen = request;
          return jsonResponse(captureAckJson(), status: 201);
        }),
      );

      await remote.uploadCapture(tourId: backendTourUuid, capture: captureOf(1));
      expect(latin1.decode(seen.bodyBytes), contains('{}'));
      await remote.dispose();
    });

    test('an acknowledgement for another target is refused', () async {
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async =>
              jsonResponse(captureAckJson(targetIndex: 2), status: 201),
        ),
      );

      await expectLater(
        remote.uploadCapture(tourId: backendTourUuid, capture: captureOf(1)),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('sent as target 1'),
          ),
        ),
      );
      await remote.dispose();
    });

    test('a missing or unreadable frame fails before any request', () async {
      var calls = 0;
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          calls++;
          return jsonResponse(captureAckJson(), status: 201);
        }),
      );

      await expectLater(
        remote.uploadCapture(
          tourId: backendTourUuid,
          capture: TourCapture(
            targetIndex: 1,
            nodeId: 'h01',
            filePath: p.join(tempDir.path, 'absent.jpg'),
            yawDeg: 0,
            pitchDeg: 0,
            capturedAt: capturedAt,
          ),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('missing')),
        ),
      );

      final junk = File(p.join(tempDir.path, 'junk.jpg'))
        ..writeAsBytesSync(Uint8List.fromList(<int>[0xff, 0xd8, 1, 2, 3]));
      await expectLater(
        remote.uploadCapture(
          tourId: backendTourUuid,
          capture: TourCapture(
            targetIndex: 2,
            nodeId: 'h02',
            filePath: junk.path,
            yawDeg: 0,
            pitchDeg: 0,
            capturedAt: capturedAt,
          ),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('truncated')),
        ),
      );
      expect(calls, 0);
      await remote.dispose();
    });

    test('the server message is kept for 401, 404 and 409', () async {
      final unauthorized = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async => jsonResponse(
            <String, Object?>{'detail': 'No authenticated user.'},
            status: 401,
          ),
        ),
      );
      await expectLater(
        unauthorized.createTour(
          propertyId: propertyUuid,
          expectedCaptureCount: 3,
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.statusCode, 'statusCode', 401)
              .having(
                (f) => f.message,
                'message',
                contains('ALLOW_DEVELOPMENT_IDENTITY'),
              ),
        ),
      );

      final gone = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async => jsonResponse(
            <String, Object?>{
              'error': <String, Object?>{
                'code': 'not_found',
                'message': 'Virtual tour not found.',
              },
            },
            status: 404,
          ),
        ),
      );
      await expectLater(
        gone.tourStatus(tourId: backendTourUuid),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('not found')),
        ),
      );

      final conflict = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async => jsonResponse(
            <String, Object?>{
              'error': <String, Object?>{
                'code': 'conflict',
                'message':
                    'A different image already exists for target_index 3.',
              },
            },
            status: 409,
          ),
        ),
      );
      await expectLater(
        conflict.uploadCapture(
          tourId: backendTourUuid,
          capture: captureOf(3),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having(
                (f) => f.message,
                'message',
                contains('different image already exists'),
              ),
        ),
      );
      await unauthorized.dispose();
      await gone.dispose();
      await conflict.dispose();
    });

    test('a 503 is retryable and a 422 is not', () async {
      var code = 503;
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async =>
              http.Response('database is locked', code),
        ),
      );

      await expectLater(
        remote.tourStatus(tourId: backendTourUuid),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isTrue)
              .having((f) => f.message, 'message', contains('503'))
              .having(
                (f) => f.message,
                'message',
                contains('database is locked'),
              ),
        ),
      );

      code = 422;
      await expectLater(
        remote.tourStatus(tourId: backendTourUuid),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.retryable,
            'retryable',
            isFalse,
          ),
        ),
      );
      await remote.dispose();
    });

    test('finalise posts to the server route with no body', () async {
      final requests = <http.Request>[];
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          requests.add(request);
          return jsonResponse(
            tourJson(
              status: 'stitch_pending',
              expected: 3,
              uploaded: 3,
              manifest: manifestKey,
            ),
          );
        }),
      );

      final ticket = await remote.finaliseTour(tourId: backendTourUuid);
      expect(requests.single.method, 'POST');
      expect(
        requests.single.url.toString(),
        'https://api.example.test/api/v1/virtual-tours/$backendTourUuid/finalise',
      );
      expect(requests.single.body, isEmpty);
      expect(ticket.status, 'stitch_pending');
      expect(ticket.stitchManifestKey, manifestKey);
      await remote.dispose();
    });

    test('a finalise 409 after a lost response resolves through status', () async {
      final paths = <String>[];
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          paths.add(request.url.path);
          if (request.url.path.endsWith('/finalise')) {
            return jsonResponse(
              <String, Object?>{
                'error': <String, Object?>{
                  'code': 'conflict',
                  'message': 'Tour is not in uploading state.',
                },
              },
              status: 409,
            );
          }
          return jsonResponse(
            tourJson(
              status: 'stitch_pending',
              uploaded: 3,
              manifest: manifestKey,
            ),
          );
        }),
      );

      final ticket = await remote.finaliseTour(tourId: backendTourUuid);
      expect(ticket.status, VirtualTourStatus.stitchPending);
      expect(ticket.stitchManifestKey, manifestKey);
      expect(paths, hasLength(2), reason: 'the conflict was settled by asking');
      await remote.dispose();
    });

    test('a finalise 409 for missing captures still fails', () async {
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          if (request.url.path.endsWith('/finalise')) {
            return jsonResponse(
              <String, Object?>{
                'error': <String, Object?>{
                  'code': 'conflict',
                  'message': 'Expected 3 captures, found 2.',
                },
              },
              status: 409,
            );
          }
          return jsonResponse(tourJson(uploaded: 2));
        }),
      );

      await expectLater(
        remote.finaliseTour(tourId: backendTourUuid),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.statusCode, 'statusCode', 409)
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('found 2')),
        ),
      );
      await remote.dispose();
    });

    test('status reads with a URL-encoded tour id', () async {
      late http.Request seen;
      final remote = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient((http.Request request) async {
          seen = request;
          return jsonResponse(tourJson(id: 'a b/c'));
        }),
      );

      final ticket = await remote.tourStatus(tourId: 'a b/c');
      expect(seen.method, 'GET');
      expect(
        seen.url.toString(),
        'https://api.example.test/api/v1/virtual-tours/a%20b%2Fc',
      );
      expect(ticket.tourId, 'a b/c');
      await remote.dispose();
    });

    test('an empty or non-object body is reported, not guessed', () async {
      final empty = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async => http.Response('', 200),
        ),
      );
      await expectLater(
        empty.tourStatus(tourId: backendTourUuid),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('empty response'),
          ),
        ),
      );

      final list = HttpVirtualTourRemoteDataSource(
        config: config,
        client: MockClient(
          (http.Request request) async => jsonResponse(<int>[1, 2], status: 200),
        ),
      );
      await expectLater(
        list.tourStatus(tourId: backendTourUuid),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('not an object'),
          ),
        ),
      );
      await empty.dispose();
      await list.dispose();
    });
  });

  group('mock remote', () {
    late LocalVirtualTourRemoteDataSource remote;
    late File frame;

    setUp(() async {
      remote = LocalVirtualTourRemoteDataSource();
      frame = File(p.join(tempDir.path, 'mock.jpg'))
        ..writeAsBytesSync(jpegFixture(width: 640, height: 480));
    });

    tearDown(() async {
      await remote.dispose();
    });

    TourCapture mockCapture(int index, {String? path}) => TourCapture(
      targetIndex: index,
      nodeId: 'n0$index',
      filePath: path ?? frame.path,
      yawDeg: 0,
      pitchDeg: 0,
      capturedAt: capturedAt,
    );

    test('creates a session it owns, marked as simulated', () async {
      final ticket = await remote.createTour(
        propertyId: propertyUuid,
        expectedCaptureCount: 2,
      );
      expect(ticket.tourId, isNotEmpty);
      expect(ticket.status, VirtualTourStatus.uploading);
      expect(ticket.simulated, isTrue);
      expect(ticket.stitchManifestKey, isNull);
    });

    test('a tour it never created is reported, not invented', () async {
      await expectLater(
        remote.uploadCapture(tourId: 'never-created', capture: mockCapture(1)),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having(
                (f) => f.message,
                'message',
                contains('no local session'),
              ),
        ),
      );
    });

    test('finalise refuses until every target exists', () async {
      final created = await remote.createTour(
        propertyId: propertyUuid,
        expectedCaptureCount: 3,
      );
      await remote.uploadCapture(
        tourId: created.tourId,
        capture: mockCapture(1),
      );

      await expectLater(
        remote.finaliseTour(tourId: created.tourId),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.message, 'message', contains('1 of 3')),
        ),
      );

      await remote.uploadCapture(
        tourId: created.tourId,
        capture: mockCapture(2),
      );
      final third = await remote.uploadCapture(
        tourId: created.tourId,
        capture: mockCapture(3),
      );
      expect(third.duplicate, isFalse);
      // The same target re-sent is reported as a duplicate, like the server.
      final again = await remote.uploadCapture(
        tourId: created.tourId,
        capture: mockCapture(3),
      );
      expect(again.duplicate, isTrue);

      final ticket = await remote.finaliseTour(tourId: created.tourId);
      expect(ticket.status, VirtualTourStatus.stitchPending);
      expect(ticket.uploadedCaptureCount, 3);
      expect(ticket.stitchManifestKey, isNull);
      expect(ticket.simulated, isTrue);
      // No worker exists, so nothing past stitch_pending is ever claimed.
      expect(ticket.status, isNot(VirtualTourStatus.stitched));
    });

    test('finalise is idempotent and images stop being accepted after it', () async {
      final created = await remote.createTour(
        propertyId: propertyUuid,
        expectedCaptureCount: 1,
      );
      await remote.uploadCapture(
        tourId: created.tourId,
        capture: mockCapture(1),
      );
      final first = await remote.finaliseTour(tourId: created.tourId);
      final second = await remote.finaliseTour(tourId: created.tourId);
      expect(second.status, first.status);

      await expectLater(
        remote.uploadCapture(tourId: created.tourId, capture: mockCapture(1)),
        throwsA(isA<VirtualTourFailure>()),
      );
    });

    test('rejects a zero target index and a non-JPEG file', () async {
      final created = await remote.createTour(
        propertyId: propertyUuid,
        expectedCaptureCount: 1,
      );
      await expectLater(
        remote.uploadCapture(tourId: created.tourId, capture: mockCapture(0)),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('numbers captures from 1'),
          ),
        ),
      );

      final notJpeg = File(p.join(tempDir.path, 'not_a_jpeg.jpg'))
        ..writeAsBytesSync(Uint8List.fromList(<int>[1, 2, 3, 4, 5]));
      await expectLater(
        remote.uploadCapture(
          tourId: created.tourId,
          capture: mockCapture(1, path: notJpeg.path),
        ),
        throwsA(
          isA<VirtualTourFailure>().having(
            (f) => f.message,
            'message',
            contains('not a JPEG'),
          ),
        ),
      );
    });
  });

  group('mode selection', () {
    test('an undeclared build is mock, and says so in its type', () {
      expect(VirtualTourMode.fromEnvironment(), VirtualTourMode.mock);
      expect(
        createVirtualTourRemoteDataSource(),
        isA<LocalVirtualTourRemoteDataSource>(),
      );
    });

    test('backend mode with a config talks HTTP', () {
      expect(
        createVirtualTourRemoteDataSource(
          mode: VirtualTourMode.backend,
          config: VirtualTourApiConfig(
            baseUri: Uri.parse('http://10.0.2.2:8000'),
            userId: VirtualTourApiConfig.defaultDevelopmentUserId,
          ),
        ),
        isA<HttpVirtualTourRemoteDataSource>(),
      );
    });

    test('backend mode without an endpoint is a hard failure', () {
      // This is the bug the old "is a URL defined?" fallback hid: a capture
      // session that looked uploaded because the app quietly simulated.
      expect(
        () => createVirtualTourRemoteDataSource(mode: VirtualTourMode.backend),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having(
                (f) => f.message,
                'message',
                contains('VIRTUAL_TOUR_API_BASE_URL'),
              ),
        ),
      );
      expect(VirtualTourApiConfig.fromEnvironment(), isNull);
    });

    test('the development identities are valid UUIDs', () {
      expect(
        VirtualTourApiConfig.isUuid(
          VirtualTourApiConfig.defaultDevelopmentUserId,
        ),
        isTrue,
      );
      expect(
        VirtualTourApiConfig.isUuid(
          VirtualTourApiConfig.defaultDevelopmentPropertyId,
        ),
        isTrue,
      );
      expect(VirtualTourApiConfig.isUuid('property_local'), isFalse);
      expect(
        VirtualTourApiConfig.propertyIdFromEnvironment(),
        VirtualTourApiConfig.defaultDevelopmentPropertyId,
      );
      expect(VirtualTourApiConfig.fileNameFor(1), 't001.jpg');
      expect(VirtualTourApiConfig.fileNameFor(26), 't026.jpg');
    });
  });

  group('photo store', () {
    test('writes NN_nodeId.jpg atomically and verifies the bytes', () async {
      final store = VirtualTourPhotoStore(root: tempDir);
      final bytes = jpegFixture(width: 1280, height: 720, padding: 32);
      final saved = await store.saveJpeg(
        tourId: 'tour-1',
        index: 0,
        nodeId: 'h01',
        bytes: bytes,
      );

      expect(p.basename(saved.file.path), '01_h01.jpg');
      expect(p.join(tempDir.path, 'tour-1', '01_h01.jpg'), saved.file.path);
      expect(saved.file.existsSync(), isTrue);
      expect(saved.byteCount, bytes.length);
      expect(await saved.file.length(), bytes.length);
      expect(await saved.file.readAsBytes(), bytes);
      expect(File('${saved.file.path}.part').existsSync(), isFalse);
      expect(saved.width, 1280);
      expect(saved.height, 720);

      final smaller = jpegFixture();
      await store.saveJpeg(
        tourId: 'tour-1',
        index: 0,
        nodeId: 'h01',
        bytes: smaller,
      );
      expect(await saved.file.length(), smaller.length);
    });

    test('rejects an empty frame instead of writing a 0-byte JPEG', () async {
      final store = VirtualTourPhotoStore(root: tempDir);
      await expectLater(
        store.saveJpeg(
          tourId: 'tour-1',
          index: 4,
          nodeId: 'h05',
          bytes: Uint8List(0),
        ),
        throwsA(isA<VirtualTourFailure>()),
      );
      expect(
        File(p.join(tempDir.path, 'tour-1', '05_h05.jpg')).existsSync(),
        isFalse,
      );
    });

    test('inspection reads the frame size and rejects broken markers', () {
      final info = VirtualTourPhotoStore.inspectJpeg(
        jpegFixture(width: 4000, height: 3000),
      );
      expect(info.width, 4000);
      expect(info.height, 3000);

      expect(
        () => VirtualTourPhotoStore.inspectJpeg(
          Uint8List.fromList(<int>[0x50, 0x4b, 0x03, 0x04]),
        ),
        throwsA(isA<VirtualTourFailure>()),
      );
      expect(
        () => VirtualTourPhotoStore.inspectJpeg(
          Uint8List.fromList(<int>[0xFF, 0xD8, 0xFF, 0xC0, 0x00]),
        ),
        throwsA(isA<VirtualTourFailure>()),
      );
    });
  });

  group('bloc', () {
    late FakeRepository repository;
    late VirtualTourBloc bloc;

    setUp(() {
      repository = FakeRepository();
      bloc = VirtualTourBloc(repository: repository);
    });

    tearDown(() async {
      await bloc.close();
    });

    VirtualTourUploadRequested uploadEvent({List<TourCapture>? captures}) =>
        VirtualTourUploadRequested(
          tourId: 'tour-1',
          propertyId: propertyUuid,
          stationId: 'station-1',
          captures: captures ?? capturesFor(3),
        );

    test(
      'an upload emits creating, one state per capture, then uploaded',
      () async {
        final states = <VirtualTourState>[];
        final sub = bloc.stream.listen(states.add);
        addTearDown(sub.cancel);

        bloc.add(uploadEvent());
        await settle();

        expect(states, hasLength(5));
        final opening = states[0] as VirtualTourProcessing;
        expect(opening.stage, VirtualTourStage.creating);
        expect(opening.total, 3);
        expect(opening.uploaded, 0);
        for (var i = 0; i < 3; i++) {
          final progress = states[1 + i] as VirtualTourProcessing;
          expect(progress.stage, VirtualTourStage.uploading);
          expect(progress.uploaded, i + 1);
          expect(progress.total, 3);
          expect(progress.message, contains('${i + 1} of 3'));
        }
        final done = states.last as VirtualTourUploaded;
        expect(done.ticket.status, VirtualTourStatus.uploading);
        expect(done.ticket.uploadedCaptureCount, 3);
        expect(done.ticket.stitchManifestKey, isNull);
      },
    );

    test('a second request while one is running is ignored', () async {
      final gate = Completer<VirtualTourTicket>();
      repository.uploadGate = gate;
      bloc.add(uploadEvent());
      await settle();
      expect(repository.uploadCalls, 1);

      bloc.add(uploadEvent());
      await settle();
      expect(repository.uploadCalls, 1);

      gate.complete(ticketFor(3));
      await settle();
      expect(bloc.state, isA<VirtualTourUploaded>());
      expect(repository.finaliseCalls, 0);
    });

    test('finalising lands on awaiting stitching with the manifest key', () async {
      final states = <VirtualTourState>[];
      final sub = bloc.stream.listen(states.add);
      addTearDown(sub.cancel);

      bloc.add(uploadEvent());
      await settle();
      bloc.add(const VirtualTourFinaliseRequested(tourId: 'tour-1'));
      await settle();

      expect(repository.finaliseTourIds, <String>['tour-1']);
      final working = states.whereType<VirtualTourProcessing>().last;
      expect(working.stage, VirtualTourStage.finalising);
      final awaiting = states.last as VirtualTourAwaitingStitching;
      expect(awaiting.ticket.status, VirtualTourStatus.stitchPending);
      expect(awaiting.ticket.stitchManifestKey, manifestKey);
      // The manifest is the furthest thing this machine can report.
      expect(states.whereType<VirtualTourAwaitingStitching>(), hasLength(1));
    });

    test('a retryable upload failure is retried with the same captures', () async {
      repository.uploadFailuresRemaining = 1;
      final states = <VirtualTourState>[];
      final sub = bloc.stream.listen(states.add);
      addTearDown(sub.cancel);

      bloc.add(uploadEvent());
      await settle();

      final error = states.whereType<VirtualTourError>().single;
      expect(error.retryable, isTrue);
      expect(error.stage, VirtualTourStage.creating);
      expect(error.ticket, isNull);

      bloc.add(const VirtualTourRetryRequested());
      await settle();

      expect(repository.uploadCalls, 2);
      expect(
        repository.receivedCaptures[0].map((c) => c.targetIndex),
        repository.receivedCaptures[1].map((c) => c.targetIndex),
      );
      expect(bloc.state, isA<VirtualTourUploaded>());

      bloc.add(const VirtualTourRetryRequested());
      await settle();
      expect(repository.uploadCalls, 2);
    });

    test('an upload failure keeps the counts the backend accepted', () async {
      repository.uploadFailuresRemaining = 1;
      repository.failAfterProgress = 1;
      repository.failure = const VirtualTourFailure(
        'The backend rejected this request (HTTP 409).',
        retryable: false,
        statusCode: 409,
      );

      bloc.add(uploadEvent());
      await settle();

      final error = bloc.state as VirtualTourError;
      expect(error.stage, VirtualTourStage.uploading);
      expect(error.retryable, isFalse);
      // The operation never completed, so there is no confirmed server state
      // to carry — only the acknowledgement that arrived before the 409.
      expect(error.ticket, isNull);
      expect(error.uploaded, 1);
      expect(error.total, 3);

      // Nothing is queued to retry, so "Try again" cannot re-send a 409.
      bloc.add(const VirtualTourRetryRequested());
      await settle();
      expect(bloc.state, isA<VirtualTourError>());
      expect(repository.uploadCalls, 1);
    });

    test('a finalise failure keeps the uploaded counts', () async {
      bloc.add(uploadEvent());
      await settle();
      repository.finaliseFailuresRemaining = 1;

      bloc.add(const VirtualTourFinaliseRequested(tourId: 'tour-1'));
      await settle();

      final error = bloc.state as VirtualTourError;
      expect(error.stage, VirtualTourStage.finalising);
      expect(error.ticket!.uploadedCaptureCount, 3);

      bloc.add(const VirtualTourRetryRequested());
      await settle();
      expect(bloc.state, isA<VirtualTourAwaitingStitching>());
      expect(repository.finaliseCalls, 2);
    });

    test('an empty request is a non-retryable validation error', () async {
      bloc.add(uploadEvent(captures: const <TourCapture>[]));
      await settle();

      final error = bloc.state as VirtualTourError;
      expect(error.stage, VirtualTourStage.validation);
      expect(error.retryable, isFalse);
      expect(repository.uploadCalls, 0);
    });

    test('reset makes a late response stale instead of emitting success', () async {
      final gate = Completer<VirtualTourTicket>();
      repository.uploadGate = gate;
      final states = <VirtualTourState>[];
      final sub = bloc.stream.listen(states.add);
      addTearDown(sub.cancel);

      bloc.add(uploadEvent());
      await settle();
      expect(bloc.state, isA<VirtualTourProcessing>());

      bloc.add(const VirtualTourResetRequested());
      await settle();
      expect(bloc.state, isA<VirtualTourInitial>());

      gate.complete(ticketFor(3));
      await settle();

      expect(states.last, isA<VirtualTourInitial>());
      expect(states.whereType<VirtualTourUploaded>(), isEmpty);
      expect(states.whereType<VirtualTourAwaitingStitching>(), isEmpty);
    });

    test('refresh reports whatever the server says', () async {
      bloc.add(const VirtualTourRefreshRequested(tourId: 'tour-1'));
      await settle();
      expect(bloc.state, isA<VirtualTourAwaitingStitching>());

      repository.refreshResult = ticketFor(
        3,
        status: 'quarantined',
        manifest: manifestKey,
      );
      bloc.add(const VirtualTourRefreshRequested(tourId: 'tour-1'));
      await settle();
      final awaiting = bloc.state as VirtualTourAwaitingStitching;
      expect(awaiting.ticket.status, 'quarantined');

      repository.refreshResult = ticketFor(3);
      bloc.add(const VirtualTourRefreshRequested(tourId: 'tour-1'));
      await settle();
      expect(bloc.state, isA<VirtualTourUploaded>());
    });

    test('a refresh failure is retryable and names its stage', () async {
      repository.refreshFailuresRemaining = 1;
      bloc.add(const VirtualTourRefreshRequested(tourId: 'tour-1'));
      await settle();

      final error = bloc.state as VirtualTourError;
      expect(error.stage, VirtualTourStage.status);
      expect(error.retryable, isTrue);

      bloc.add(const VirtualTourRetryRequested());
      await settle();
      expect(bloc.state, isA<VirtualTourAwaitingStitching>());
      expect(repository.refreshCalls, 2);
    });

    test('a status still uploading reads as uploaded, not as done', () async {
      repository.finaliseResult = ticketFor(3, uploaded: 2);
      bloc.add(const VirtualTourFinaliseRequested(tourId: 'tour-1'));
      await settle();
      expect(bloc.state, isA<VirtualTourUploaded>());
    });
  });
}
