import 'dart:io';
import 'dart:math';
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:path/path.dart' as p;

import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_local_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_remote_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/repositories/virtual_tour_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/services/virtual_tour_photo_store.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';

/// The same four routes the app calls, driven against a **running** FastAPI
/// process — real sockets, real JPEG bytes, the server's own SQLite database
/// and its own object storage.
///
/// Nothing here is mocked, which is the point: `virtual_tour_upload_test.dart`
/// pins the client against recorded answers, and only a live server can prove
/// those answers were the right ones (that the multipart field names parse,
/// that Pillow accepts what the camera wrote, that `sha256` means the same
/// thing on both sides, that finalise really leaves a manifest behind).
///
/// **Run it with the backend up**, from `backend/`:
///
/// ```
/// uv run alembic upgrade head
/// SITESENSE_ALLOW_DEVELOPMENT_IDENTITY=true uv run uvicorn sitesense.main:app --host 127.0.0.1 --port 8000
/// ```
///
/// and then `VIRTUAL_TOUR_TEST_API_URL=http://127.0.0.1:8000 flutter test
/// test/virtual_tour_backend_integration_test.dart`. With no endpoint in the
/// environment every test below is reported as skipped rather than passed — a
/// green run that never talked to a server would be exactly the false evidence
/// this file exists to prevent. The one exception is the fixture group, which
/// checks the photograph itself and always runs.

/// Where the server under test lives, or `null` when none was declared.
final Uri? endpoint = _endpointFromEnvironment();

/// Every test that needs the network.
final String? offline = endpoint == null
    ? 'No local backend to talk to. Start uvicorn on 127.0.0.1:8000 and set '
          'VIRTUAL_TOUR_TEST_API_URL (see the header of this file and '
          'docs/SECTION4_5_2_BACKEND_UPLOAD_RUNBOOK.md).'
    : null;

const String fixturePath = 'test/fixtures/virtual_tour_frame.jpg';

Uri? _endpointFromEnvironment() {
  final raw = (Platform.environment['VIRTUAL_TOUR_TEST_API_URL'] ?? '').trim();
  if (raw.isEmpty) return null;
  final uri = Uri.tryParse(raw);
  if (uri == null || !uri.hasScheme || uri.host.isEmpty) {
    throw StateError('VIRTUAL_TOUR_TEST_API_URL is not an absolute URL: "$raw".');
  }
  return uri;
}

final VirtualTourApiConfig config = VirtualTourApiConfig(
  baseUri: endpoint ?? Uri.parse('http://127.0.0.1:1'),
  userId: _environment(
    'VIRTUAL_TOUR_TEST_USER_ID',
    VirtualTourApiConfig.defaultDevelopmentUserId,
  ),
  timeout: const Duration(seconds: 30),
);

String _environment(String name, String fallback) {
  final value = (Platform.environment[name] ?? '').trim();
  return value.isEmpty ? fallback : value;
}

/// The property a live run files its tours under.
///
/// `virtual_tours.property_id` has no foreign key, so this needs no seeded
/// owner or property row — it only has to be a UUID the server can parse.
final String propertyId = _environment(
  'VIRTUAL_TOUR_TEST_PROPERTY_ID',
  VirtualTourApiConfig.defaultDevelopmentPropertyId,
);

final DateTime capturedAt = DateTime.utc(2026, 11, 6, 8, 15);

String randomUuid() {
  final random = Random.secure();
  final bytes = List<int>.generate(16, (_) => random.nextInt(256));
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  final hex = bytes
      .map((int b) => b.toRadixString(16).padLeft(2, '0'))
      .join();
  return '${hex.substring(0, 8)}-${hex.substring(8, 12)}'
      '-${hex.substring(12, 16)}-${hex.substring(16, 20)}-${hex.substring(20)}';
}

/// One station: its own frames directory, its own local tables, its own id.
///
/// Each test builds a fresh one, so a resumed upload is proven against state
/// this test wrote rather than a row a previous run left behind.
class LiveStation {
  LiveStation(Directory work)
    : store = VirtualTourPhotoStore(root: Directory(p.join(work.path, 'frames'))),
      local = VirtualTourLocalDataSource(
        root: Directory(p.join(work.path, 'state')),
      ),
      tourId = 'live-${DateTime.now().microsecondsSinceEpoch}';

  final VirtualTourPhotoStore store;
  final VirtualTourLocalDataSource local;
  final String tourId;

  final List<TourCapture> captures = <TourCapture>[];

  /// Saves [byteLists] as targets 1..n and returns what goes up the wire.
  Future<List<TourCapture>> record(List<Uint8List> byteLists) async {
    for (var i = 0; i < byteLists.length; i++) {
      final nodeId = 'h${(i + 1).toString().padLeft(2, '0')}';
      final saved = await store.saveJpeg(
        tourId: tourId,
        index: i,
        nodeId: nodeId,
        bytes: byteLists[i],
        expectedWidth: 640,
        expectedHeight: 480,
      );
      captures.add(
        TourCapture(
          targetIndex: i + 1,
          nodeId: nodeId,
          filePath: saved.file.path,
          yawDeg: i * 120.0,
          pitchDeg: 0.0,
          capturedAt: capturedAt.add(Duration(seconds: i)),
          pose: CapturePose(
            positionX: 0.1 * (i + 1),
            positionY: 1.45,
            positionZ: 0.0,
            quaternionW: 1.0,
            alignmentErrorDeg: 1.5 + i,
          ),
        ),
      );
    }
    return captures;
  }

  Future<void> dispose() async {
    await store.clearTour(tourId);
  }
}

Directory createWorkDirectory(String name) {
  final directory = Directory.systemTemp.createTempSync('vt_live_$name');
  addTearDown(() {
    if (directory.existsSync()) directory.deleteSync(recursive: true);
  });
  return directory;
}

HttpVirtualTourRemoteDataSource remoteDataSource({String? asUserId}) {
  final remote = HttpVirtualTourRemoteDataSource(
    config: asUserId == null
        ? config
        : VirtualTourApiConfig(
            baseUri: config.baseUri,
            userId: asUserId,
            timeout: config.timeout,
          ),
  );
  addTearDown(remote.dispose);
  return remote;
}

VirtualTourRepositoryImpl repositoryFor(LiveStation station) {
  final remote = remoteDataSource();
  return VirtualTourRepositoryImpl(remote: remote, local: station.local);
}

void main() {
  group('the frame the live tests upload', () {
    late Uint8List bytes;

    setUpAll(() {
      final file = File(fixturePath);
      expect(
        file.existsSync(),
        isTrue,
        reason: '$fixturePath is missing; the live tests have nothing to send.',
      );
      bytes = file.readAsBytesSync();
    });

    test('is a real JPEG the app accepts', () {
      final info = VirtualTourPhotoStore.inspectJpeg(bytes);
      expect(info.width, 640);
      expect(info.height, 480);
    });

    test('is big enough for the server to consider stitching it', () {
      // The backend refuses anything under 640×480, so a fixture that quietly
      // shrank would make every live test fail for the wrong reason.
      expect(bytes.lengthInBytes, greaterThan(1024));
      final info = VirtualTourPhotoStore.inspectJpeg(bytes);
      expect(info.width, 640);
      expect(info.height, 480);
    });
  });

  group('live backend', () {
    late Directory work;

    setUp(() {
      work = createWorkDirectory('suite');
    });

    test('the declared endpoint answers the two tour routes', () async {
      final remote = remoteDataSource();
      final created = await remote.createTour(
        propertyId: propertyId,
        expectedCaptureCount: 1,
      );
      expect(VirtualTourApiConfig.isUuid(created.tourId), isTrue);
      expect(created.simulated, isFalse);
      expect(created.status, VirtualTourStatus.uploading);
      expect(created.uploadedCaptureCount, 0);
      expect(created.stitchManifestKey, isNull);
      final status = await remote.tourStatus(tourId: created.tourId);
      expect(status.tourId, created.tourId);
      expect(status.propertyId, propertyId);
    }, skip: offline);

    test('a property id that is not a UUID is refused, by both sides',
        () async {
      // The client refuses first: `_requireUuid` turns it into a message with
      // a fix in it rather than a 422 round trip. So the server's own rule is
      // checked by sending the request the client will not make.
      await expectLater(
        remoteDataSource().createTour(
          propertyId: 'property_local',
          expectedCaptureCount: 3,
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.statusCode, 'statusCode', isNull)
              .having(
                (f) => f.message,
                'message',
                contains('is not a UUID'),
              ),
        ),
      );

      final raw = await http.post(
        config.toursUri,
        headers: <String, String>{
          'Content-Type': 'application/json',
          'X-User-Id': config.userId,
        },
        body: '{"property_id":"property_local","expected_capture_count":3}',
      );
      expect(raw.statusCode, 422);
    }, skip: offline);

    test(
      'an upload of the whole station is accepted, one JPEG per target',
      () async {
        final station = LiveStation(work);
        addTearDown(station.dispose);
        final frame = File(fixturePath).readAsBytesSync();
        final captures = await station.record(<Uint8List>[frame, frame, frame]);
        expect(captures.map((c) => c.targetIndex), <int>[1, 2, 3]);

        final repository = repositoryFor(station);
        final progress = <int>[];
        final ticket = await repository.uploadCaptures(
          tourId: station.tourId,
          propertyId: propertyId,
          stationId: 'station_live',
          captures: captures,
          onProgress: (uploaded, total) => progress.add(uploaded),
        );

        expect(progress, <int>[1, 2, 3]);
        expect(ticket.status, VirtualTourStatus.uploading);
        expect(ticket.expectedCaptureCount, 3);
        expect(ticket.uploadedCaptureCount, 3);
        expect(ticket.isFullyUploaded, isTrue);
        expect(ticket.simulated, isFalse);
        expect(ticket.stitchManifestKey, isNull);

        // The device's rows were written *from* the server's answers.
        final photos = await station.local.propertyPhotos(station.tourId);
        expect(photos, hasLength(3));
        for (final row in photos) {
          expect(row.isUploaded, isTrue);
          expect(row.width, 640);
          expect(row.height, 480);
          expect(row.storageKey, startsWith('tours/${ticket.tourId}/captures/'));
          expect(row.sha256, hasLength(64));
        }
        expect(
          photos.map((r) => r.storageKey).toSet(),
          hasLength(3),
          reason: 'three targets are three objects, even with identical bytes',
        );

        final record = await station.local.virtualTour(station.tourId);
        expect(record!.backendTourId, ticket.tourId);
        expect(record.backendStatus, VirtualTourStatus.uploading);
        expect(record.uploadedCaptureCount, 3);
        expect(record.simulated, isFalse);
      },
      skip: offline,
    );

    test(
      'a second upload resumes the same backend tour instead of filing '
      'another one',
      () async {
        final station = LiveStation(work);
        addTearDown(station.dispose);
        final frame = File(fixturePath).readAsBytesSync();
        final captures = await station.record(<Uint8List>[frame, frame]);
        final repository = repositoryFor(station);

        final first = await repository.uploadCaptures(
          tourId: station.tourId,
          propertyId: propertyId,
          stationId: 'station_live',
          captures: captures,
        );
        final second = await repository.uploadCaptures(
          tourId: station.tourId,
          propertyId: propertyId,
          stationId: 'station_live',
          captures: captures,
        );

        expect(second.tourId, first.tourId);
        expect(second.uploadedCaptureCount, 2);
        expect(await station.local.propertyPhotos(station.tourId), hasLength(2));

        // The server's dedup is what makes this safe; confirm it answered that
        // way rather than storing the frame twice.
        final ack = await remoteDataSource().uploadCapture(
          tourId: first.tourId,
          capture: captures.first,
        );
        expect(ack.duplicate, isTrue);
        expect(ack.storageKey, contains('/captures/001-'));
      },
      skip: offline,
    );

    test(
      'finalise validates the count and leaves a stitch manifest',
      () async {
        final station = LiveStation(work);
        addTearDown(station.dispose);
        final frame = File(fixturePath).readAsBytesSync();
        final captures = await station.record(<Uint8List>[frame, frame, frame]);
        final repository = repositoryFor(station);
        final remote = remoteDataSource();
        final uploaded = await repository.uploadCaptures(
          tourId: station.tourId,
          propertyId: propertyId,
          stationId: 'station_live',
          captures: captures,
        );

        // A tour the server expects three frames for, holding two, is refused.
        // This half goes straight to the remote on purpose: the repository
        // sizes the session from the captures it is handed, so it cannot
        // produce a deliberately half-filled tour.
        final short = await remote.createTour(
          propertyId: propertyId,
          expectedCaptureCount: 3,
        );
        await remote.uploadCapture(tourId: short.tourId, capture: captures[0]);
        await remote.uploadCapture(tourId: short.tourId, capture: captures[1]);
        await expectLater(
          remote.finaliseTour(tourId: short.tourId),
          throwsA(
            isA<VirtualTourFailure>()
                .having((f) => f.retryable, 'retryable', isFalse)
                .having((f) => f.statusCode, 'statusCode', 409)
                .having(
                  (f) => f.message,
                  'message',
                  contains('all 3 captures are required'),
                ),
          ),
        );
        expect(
          (await remote.tourStatus(tourId: short.tourId)).status,
          VirtualTourStatus.uploading,
          reason: 'a refused finalise leaves the tour open, not half-closed',
        );

        final ticket = await repository.finaliseTour(tourId: station.tourId);
        expect(ticket.status, VirtualTourStatus.stitchPending);
        expect(ticket.uploadedCaptureCount, 3);
        expect(ticket.expectedCaptureCount, uploaded.expectedCaptureCount);
        expect(
          ticket.stitchManifestKey,
          startsWith('tours/${uploaded.tourId}/stitch-manifest-'),
        );
        expect(
          ticket.stitchManifestKey,
          endsWith('.json'),
          reason: 'finalise writes a manifest, never a panorama',
        );

        final record = await station.local.virtualTour(station.tourId);
        expect(record!.backendStatus, VirtualTourStatus.stitchPending);
        expect(record.stitchManifestKey, ticket.stitchManifestKey);

        // Re-asking is settled by reading the status, not by a second manifest.
        final replay = await repository.finaliseTour(tourId: station.tourId);
        expect(replay.status, VirtualTourStatus.stitchPending);
        expect(replay.stitchManifestKey, ticket.stitchManifestKey);

        // And a closed tour will not take more frames.
        await expectLater(
          repository.uploadCaptures(
            tourId: station.tourId,
            propertyId: propertyId,
            stationId: 'station_live',
            captures: captures,
          ),
          throwsA(
            isA<VirtualTourFailure>()
                .having((f) => f.retryable, 'retryable', isFalse)
                .having((f) => f.message, 'message', contains('stitch_pending')),
          ),
        );
      },
      skip: offline,
    );

    test('a frame outside the capture plan comes back as the server explained',
        () async {
      final remote = remoteDataSource();
      final created = await remote.createTour(
        propertyId: propertyId,
        expectedCaptureCount: 1,
      );
      final frame = File(fixturePath).readAsBytesSync();
      final station = LiveStation(work);
      addTearDown(station.dispose);
      final captures = await station.record(<Uint8List>[frame]);

      await expectLater(
        remote.uploadCapture(
          tourId: created.tourId,
          capture: TourCapture(
            targetIndex: 4,
            nodeId: captures.first.nodeId,
            filePath: captures.first.filePath,
            yawDeg: 0,
            pitchDeg: 0,
            capturedAt: capturedAt,
          ),
        ),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isFalse)
              .having((f) => f.statusCode, 'statusCode', 409)
              .having(
                (f) => f.message,
                'message',
                contains("tour's capture plan"),
              ),
        ),
      );
    }, skip: offline);

    test('another identity cannot see the tour', () async {
      final remote = remoteDataSource();
      final created = await remote.createTour(
        propertyId: propertyId,
        expectedCaptureCount: 1,
      );

      final intruder = remoteDataSource(asUserId: randomUuid());
      await expectLater(
        intruder.tourStatus(tourId: created.tourId),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.statusCode, 'statusCode', 404)
              .having((f) => f.retryable, 'retryable', isFalse),
        ),
      );
      await expectLater(
        intruder.finaliseTour(tourId: created.tourId),
        throwsA(isA<VirtualTourFailure>()),
      );
    }, skip: offline);

    test('an unreachable endpoint is reported as unreachable, not as accepted',
        () async {
      // A port the OS just handed out and then took back. Windows can drop a
      // connection to a privileged port such as 1 instead of refusing it,
      // which would arrive as a timeout; a closed ephemeral port refuses
      // promptly on both platforms.
      final probe = await ServerSocket.bind(InternetAddress.loopbackIPv4, 0);
      final deadPort = probe.port;
      await probe.close();
      final dead = HttpVirtualTourRemoteDataSource(
        config: VirtualTourApiConfig(
          baseUri: Uri.parse('http://127.0.0.1:$deadPort'),
          userId: config.userId,
          timeout: const Duration(seconds: 5),
        ),
      );
      addTearDown(dead.dispose);
      await expectLater(
        dead.createTour(propertyId: propertyId, expectedCaptureCount: 1),
        throwsA(
          isA<VirtualTourFailure>()
              .having((f) => f.retryable, 'retryable', isTrue)
              .having((f) => f.message, 'message', contains('could not be reached')),
        ),
      );
    }, skip: offline);

    test('the server still owns the session after the app forgets it', () async {
      // What a reinstall looks like to the backend: the local row is gone, so
      // the app opens a new tour rather than guessing at the old id. The old
      // one stays `uploading` on the server — nothing here pretends to clean it.
      final remote = remoteDataSource();
      final station = LiveStation(work);
      addTearDown(station.dispose);
      final stationCaptures = await station.record(<Uint8List>[
        File(fixturePath).readAsBytesSync(),
      ]);
      final first = await repositoryFor(station).uploadCaptures(
        tourId: station.tourId,
        propertyId: propertyId,
        stationId: 'station_live',
        captures: stationCaptures,
      );

      final rebuilt = LiveStation(createWorkDirectory('reinstall'));
      addTearDown(rebuilt.dispose);
      final rebuiltCaptures = await rebuilt.record(<Uint8List>[
        File(fixturePath).readAsBytesSync(),
      ]);
      final second = await repositoryFor(rebuilt).uploadCaptures(
        tourId: rebuilt.tourId,
        propertyId: propertyId,
        stationId: 'station_live',
        captures: rebuiltCaptures,
      );

      expect(second.tourId, isNot(first.tourId));
      final orphan = await remote.tourStatus(tourId: first.tourId);
      expect(orphan.tourId, first.tourId);
      expect(orphan.status, VirtualTourStatus.uploading);
      expect(orphan.uploadedCaptureCount, 1);
    }, skip: offline);
  });
}
