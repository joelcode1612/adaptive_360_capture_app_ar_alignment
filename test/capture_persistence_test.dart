import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as p;
import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The real persistence layer's schema contract.
///
/// There is no SQL table in this project: the station store is
/// `<appDocs>/ar_capture/<stationId>/`, where the station id is the directory
/// name **and** the primary key, and `targets.json` / `records.json` carry it
/// as the foreign key while `targetId` is the frame index. These tests pin
/// that contract against the real [FileCaptureRepository] - including the two
/// properties a resume depends on: a retake replaces its record instead of
/// appending a duplicate, and one station's frames never appear under
/// another's.
void main() {
  late Directory root;
  late FileCaptureRepository repository;

  setUp(() async {
    root = await Directory.systemTemp.createTemp('ar_capture_persistence');
    repository = FileCaptureRepository(root.path);
  });

  tearDown(() async {
    if (await root.exists()) await root.delete(recursive: true);
  });

  CaptureStation station(String id, {int capturedCount = 0}) => CaptureStation(
    stationId: id,
    referencePosition: const Vec3(1, 1.6, -2),
    referenceOrientation: Quat.identity,
    createdAt: DateTime.utc(2026, 10, 20),
    trackingStatus: TrackingState.tracking,
    capturedCount: capturedCount,
    total: kCaptureTargetCount,
  );

  CaptureRecord record(String stationId, int targetId) => CaptureRecord(
    stationId: stationId,
    targetId: targetId,
    imagePath: '/tmp/$stationId-t$targetId.jpg',
    capturedAt: DateTime.utc(2026, 10, 20, 12, targetId),
    targetYawDegrees: 0,
    targetPitchDegrees: 45,
    cameraPosition: Vec3.zero,
    cameraOrientation: Quat.identity,
    alignmentErrorDeg: 0.4,
    anchored: true,
    anchorIdentifier: 'anchor-1',
    anchorSurfaceType: 'WORLD',
    imageWidth: 1920,
    imageHeight: 1080,
  );

  CaptureTarget target(String stationId, int targetId) {
    final spec = buildCaptureSphere().firstWhere(
      (candidate) => candidate.targetId == targetId,
    );
    return CaptureTarget(
      targetId: targetId,
      stationId: stationId,
      yawDegrees: spec.yawDegrees,
      pitchDegrees: spec.pitchDegrees,
      stationLocalDirection: spec.direction,
    );
  }

  Future<Uint8List> fixtureJpeg() => File(
    p.join(
      Directory.current.path,
      'test',
      'fixtures',
      'virtual_tour_frame.jpg',
    ),
  ).readAsBytes();

  CaptureRecord captureDraft(String stationId, int targetId) => CaptureRecord(
    stationId: stationId,
    targetId: targetId,
    imagePath: '',
    capturedAt: DateTime.utc(2026, 10, 20, 12, targetId),
    targetYawDegrees: 0,
    targetPitchDegrees: 45,
    cameraPosition: Vec3.zero,
    cameraOrientation: Quat.identity,
    alignmentErrorDeg: 0.4,
    anchored: true,
    anchorIdentifier: 'anchor-1',
    anchorSurfaceType: 'WORLD',
    imageWidth: 640,
    imageHeight: 480,
  );

  Future<void> initialiseManifest(
    String stationId, {
    List<CaptureTarget>? targets,
  }) => repository.saveCaptureState(
    station: station(stationId),
    targets: targets ?? <CaptureTarget>[target(stationId, 1)],
    records: const <CaptureRecord>[],
  );

  test('a station is its own record, keyed by its own id', () async {
    const id = 'station-1760000000000';
    await repository.saveStation(station(id));

    final file = File(p.join(root.path, id, 'station.json'));
    expect(await file.exists(), isTrue);

    final loaded = await repository.loadStation(id);
    expect(loaded, isNotNull);
    expect(loaded!.stationId, id);
    expect(loaded.referencePosition.x, closeTo(1, 1e-9));
    expect(loaded.referencePosition.y, closeTo(1.6, 1e-9));
    expect(loaded.referenceOrientation.length, closeTo(1, 1e-9));
    expect(loaded.trackingStatus, TrackingState.tracking);
    expect(loaded.total, kCaptureTargetCount);
    expect(await repository.loadStation('station-does-not-exist'), isNull);
  });

  test('all 26 targets are filed under the station that owns them', () async {
    const id = 'station-a';
    await repository.saveStation(station(id));
    await repository.saveTargets(<CaptureTarget>[
      for (final spec in buildCaptureSphere())
        CaptureTarget(
          targetId: spec.targetId,
          stationId: id,
          yawDegrees: spec.yawDegrees,
          pitchDegrees: spec.pitchDegrees,
          stationLocalDirection: spec.direction,
        ),
    ]);

    final loaded = await repository.loadTargets(id);
    expect(loaded.length, kCaptureTargetCount);
    expect(
      loaded.every((CaptureTarget t) => t.stationId == id),
      isTrue,
      reason: 'the station id is the foreign key on every target',
    );
    expect(
      loaded.map((CaptureTarget t) => t.targetId).toSet().length,
      kCaptureTargetCount,
      reason: '26 distinct frame indices',
    );
    expect(await repository.loadTargets('station-b'), isEmpty);
  });

  test(
    'a retake replaces its record instead of appending a duplicate',
    () async {
      const id = 'station-a';
      await repository.saveStation(station(id));
      await repository.saveRecord(record(id, 3));
      await repository.saveRecord(record(id, 7));
      // The same frame taken again: same station, same frame index.
      await repository.saveRecord(record(id, 3));

      final records = await repository.loadRecords(id);
      expect(records.length, 2, reason: 'one record per frame index');
      expect(records.map((CaptureRecord r) => r.targetId).toList(), <int>[
        3,
        7,
      ]);
    },
  );

  test('a station with a different id never sees other frames', () async {
    await repository.saveStation(station('station-a'));
    await repository.saveStation(station('station-b'));
    await repository.saveRecord(record('station-a', 1));
    await repository.saveRecord(record('station-b', 2));

    expect((await repository.loadRecords('station-a')).length, 1);
    expect((await repository.loadRecords('station-b')).single.targetId, 2);
    expect(
      await repository.stationIds(),
      containsAll(<String>['station-a', 'station-b']),
    );
  });

  test('frame paths are per station and indexed by frame', () async {
    final first = await repository.framePath('station-a', 7);
    final second = await repository.framePath('station-a', 7);
    expect(first, contains('station-a'));
    expect(first, contains('frames'));
    expect(p.basename(p.dirname(first)), 't07');
    expect(p.extension(first), '.jpg');
    expect(second, isNot(first), reason: 'retakes get immutable file versions');
  });

  test(
    'a verified JPEG and its complete state publish in one manifest',
    () async {
      const id = 'station-manifest';
      await initialiseManifest(id);

      final committed = await repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: await fixtureJpeg(),
      );

      expect(File(committed.record.imagePath).existsSync(), isTrue);
      expect(committed.record.imageByteLength, greaterThan(0));
      expect(committed.record.imageSha256, hasLength(64));
      expect(p.basename(p.dirname(committed.record.imagePath)), 't01');
      expect(
        (await repository.loadRecords(id)).single.imagePath,
        committed.record.imagePath,
      );
      expect((await repository.loadStation(id))!.capturedCount, 1);
      expect(
        (await repository.loadTargets(id)).single.status,
        CaptureTargetStatus.captured,
      );
    },
  );

  test('concurrent captures serialise without losing either target', () async {
    const id = 'station-concurrent';
    await initialiseManifest(
      id,
      targets: <CaptureTarget>[target(id, 1), target(id, 2)],
    );
    final jpeg = await fixtureJpeg();

    await Future.wait<CaptureCommitResult>(<Future<CaptureCommitResult>>[
      repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: jpeg,
      ),
      repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 2),
        jpegBytes: jpeg,
      ),
    ]);

    final committed = await repository.loadCaptureState(id);
    expect(committed, isNotNull);
    expect(committed!.station.capturedCount, 2);
    expect(
      committed.records.map((CaptureRecord r) => r.targetId).toList(),
      <int>[1, 2],
    );
    expect(
      committed.targets.every(
        (CaptureTarget target) => target.status == CaptureTargetStatus.captured,
      ),
      isTrue,
    );
  });

  test(
    'a successful retake preserves the prior JPEG and switches the record',
    () async {
      const id = 'station-retake';
      await initialiseManifest(id);
      final jpeg = await fixtureJpeg();
      final first = await repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: jpeg,
      );
      final second = await repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: jpeg,
      );

      expect(second.record.imagePath, isNot(first.record.imagePath));
      expect(File(first.record.imagePath).existsSync(), isTrue);
      expect(File(second.record.imagePath).existsSync(), isTrue);
      expect(
        (await repository.loadRecords(id)).single.imagePath,
        second.record.imagePath,
      );
      expect((await repository.loadStation(id))!.capturedCount, 1);
    },
  );

  test(
    'restart falls back to the prior manifest when a retake image is corrupt',
    () async {
      const id = 'station-reconcile';
      await initialiseManifest(id);
      final jpeg = await fixtureJpeg();
      final first = await repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: jpeg,
      );
      final second = await repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: jpeg,
      );
      await File(second.record.imagePath).writeAsBytes(
        Uint8List.fromList(<int>[0xff, 0xd8, 0xff, 0xd9]),
        flush: true,
      );

      final restarted = FileCaptureRepository(root.path);
      expect(
        (await restarted.loadRecords(id)).single.imagePath,
        first.record.imagePath,
      );
      expect((await restarted.loadStation(id))!.capturedCount, 1);
    },
  );

  test(
    'restart rejects a manifest with mismatched target and record rows',
    () async {
      const id = 'station-inconsistent-manifest';
      await initialiseManifest(id);
      await repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: await fixtureJpeg(),
      );
      final manifests = <File>[];
      await for (final entity in Directory(
        p.join(root.path, id, 'manifests'),
      ).list()) {
        if (entity is File && entity.path.endsWith('.json')) {
          manifests.add(entity);
        }
      }
      manifests.sort((File a, File b) => a.path.compareTo(b.path));
      final latest = manifests.last;
      final malformed =
          jsonDecode(await latest.readAsString()) as Map<String, dynamic>;
      malformed['records'] = <Object>[];
      await latest.writeAsString(jsonEncode(malformed), flush: true);

      final restarted = FileCaptureRepository(root.path);
      final recovered = await restarted.loadCaptureState(id);
      expect(recovered, isNotNull);
      expect(recovered!.records, isEmpty);
      expect(recovered.station.capturedCount, 0);
      expect(recovered.targets.single.status, CaptureTargetStatus.pending);
    },
  );

  test('orphan and partial images never become accepted records', () async {
    const id = 'station-orphan';
    await initialiseManifest(id);
    final path = await repository.framePath(id, 1);
    final part = File('$path.part');
    await part.parent.create(recursive: true);
    await part.writeAsBytes(await fixtureJpeg(), flush: true);
    await File(path).writeAsBytes(await fixtureJpeg(), flush: true);

    final restarted = FileCaptureRepository(root.path);
    expect(await restarted.loadRecords(id), isEmpty);
    expect((await restarted.loadStation(id))!.capturedCount, 0);
  });

  test('a marker-only JPEG is rejected before a manifest accepts it', () async {
    const id = 'station-invalid-jpeg';
    await initialiseManifest(id);
    final invalid = Uint8List.fromList(<int>[
      0xff,
      0xd8,
      ...List<int>.filled(4092, 0),
      0xff,
      0xd9,
    ]);

    await expectLater(
      repository.commitCapture(
        station: station(id),
        record: captureDraft(id, 1),
        jpegBytes: invalid,
      ),
      throwsA(isA<StorageFailure>()),
    );
    expect(await repository.loadRecords(id), isEmpty);
  });

  test(
    'a legacy record is verified and promoted into its first manifest',
    () async {
      const id = 'station-legacy-upgrade';
      final imagePath = p.join(root.path, id, 'frames', 't01', 'legacy.jpg');
      final image = File(imagePath);
      await image.parent.create(recursive: true);
      await image.writeAsBytes(await fixtureJpeg(), flush: true);
      final acceptedTarget = target(
        id,
        1,
      ).copyWith(status: CaptureTargetStatus.captured, imagePath: imagePath);
      final legacyRecord = captureDraft(id, 1).copyWith(imagePath: imagePath);
      await File(
        p.join(root.path, id, 'station.json'),
      ).writeAsString(jsonEncode(station(id, capturedCount: 1).toJson()));
      await File(p.join(root.path, id, 'targets.json')).writeAsString(
        jsonEncode(<Map<String, dynamic>>[acceptedTarget.toJson()]),
      );
      await File(p.join(root.path, id, 'records.json')).writeAsString(
        jsonEncode(<Map<String, dynamic>>[legacyRecord.toJson()]),
      );

      final legacy = await repository.loadCaptureState(id);
      expect(legacy, isNotNull);
      expect(legacy!.records.single.imageSha256, isNull);

      await repository.saveCaptureState(
        station: legacy.station,
        targets: legacy.targets,
        records: legacy.records,
      );

      final promoted = await repository.loadCaptureState(id);
      expect(promoted, isNotNull);
      expect(promoted!.records.single.imageByteLength, greaterThan(0));
      expect(promoted.records.single.imageSha256, hasLength(64));
      expect(
        await Directory(p.join(root.path, id, 'manifests')).exists(),
        isTrue,
      );
    },
  );

  test('an invalid manifest never revives older split-file state', () async {
    const id = 'station-no-legacy-resurrection';
    await repository.saveStation(station(id));
    final manifest = File(p.join(root.path, id, 'manifests', 'broken.json'));
    await manifest.parent.create(recursive: true);
    await manifest.writeAsString('{not-json', flush: true);

    final restarted = FileCaptureRepository(root.path);
    expect(await restarted.loadCaptureState(id), isNull);
    expect(await restarted.loadStation(id), isNull);
    expect(await restarted.loadRecords(id), isEmpty);
  });
}
