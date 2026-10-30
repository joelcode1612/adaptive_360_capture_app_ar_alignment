import 'dart:async';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/photo_capture_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

import 'capture_flow_test.dart' show fakeJpeg;
import 'capture_test_fakes.dart';

/// The capture pipeline itself: camera → file → verify → record.
///
/// The regression this pins is a device recording that sat on "Saving…" at
/// 0 of 26 forever. The camera request had been queued, nothing consumed it,
/// and the Dart future had no deadline — so the UI could not tell a hang from
/// a slow encoder. Every stage now has a timeout and a timing log, and these
/// tests hold both.
void main() {
  late FakeArGateway gateway;
  late FakeCaptureRepository repository;
  late CaptureTarget target;

  final alignment = const AlignmentCalculator().compute(
    camera: RigidPose(position: Vec3.zero, orientation: Quat.identity),
    targetPosition: const Vec3(0, 0, 1.5),
    toleranceDeg: 3,
  );

  setUp(() {
    gateway = FakeArGateway()..stillBytes = fakeJpeg;
    repository = FakeCaptureRepository();
    target = CaptureTarget(
      targetId: 1,
      stationId: 's1',
      yawDegrees: 0,
      pitchDegrees: 0,
      stationLocalDirection: buildCaptureSphere().first.direction,
    );
  });

  PhotoCaptureService service({Duration still = const Duration(seconds: 6)}) =>
      PhotoCaptureService(
        host: gateway,
        repository: repository,
        stillTimeout: still,
      );

  Future<CaptureRecord> run(PhotoCaptureService s) => s.capture(
    stationId: 's1',
    target: target,
    camera: const RigidPose(position: Vec3.zero, orientation: Quat.identity),
    alignment: alignment,
    anchored: true,
    anchorIdentifier: 'anchor-1',
    anchorSurfaceType: 'WORLD',
    toleranceDeg: 3,
  );

  test('a real frame is written, verified and recorded once', () async {
    final record = await run(service());

    expect(File(record.imagePath).existsSync(), isTrue);
    expect(File(record.imagePath).lengthSync(), fakeJpeg.length);
    expect(record.imageWidth, 1920);
    expect(record.rotationAppliedDeg, 90);
    expect((await repository.loadRecords('s1')).length, 1);
    expect((await repository.loadRecords('s1')).single.targetId, 1);
  });

  test('an image below the sanity floor is rejected, nothing saved', () async {
    gateway.stillBytes = Uint8List.fromList(<int>[1, 2, 3]);
    await expectLater(run(service()), throwsA(isA<StorageFailure>()));
    expect(await repository.loadRecords('s1'), isEmpty);
  });

  test('a record-write failure leaves the target retryable', () async {
    repository.recordError = const StorageFailure('Fake: disk full.');
    await expectLater(run(service()), throwsA(isA<StorageFailure>()));
    expect(service().busy, isFalse);

    // The retry succeeds once the store recovers.
    repository.recordError = null;
    final record = await run(service());
    expect(
      (await repository.loadRecords('s1')).single.imagePath,
      record.imagePath,
    );
  });

  test(
    'a camera that never answers fails after the deadline, not forever',
    () async {
      // The exact device symptom: a still request that is queued and never
      // consumed.
      gateway.stillGate = Completer<void>().future;

      final started = DateTime.now();
      await expectLater(
        run(service(still: const Duration(milliseconds: 120))),
        throwsA(
          isA<StorageFailure>().having(
            (StorageFailure f) => f.message,
            'message',
            contains('did not return an image'),
          ),
        ),
      );
      final elapsed = DateTime.now().difference(started);
      expect(elapsed, lessThan(const Duration(seconds: 3)));
      // The guard is released, so the next press is not refused as "already
      // saving".
      final s = service();
      gateway.stillGate = null;
      expect(await run(s), isNotNull);
    },
  );

  test(
    'a second shutter while one is in flight is refused, not doubled',
    () async {
      final gate = Completer<void>();
      gateway.stillGate = gate.future;
      final s = service();

      final first = run(s);
      await expectLater(run(s), throwsA(isA<StorageFailure>()));
      expect(gateway.stillRequests, 1);

      gate.complete();
      await first;
      expect((await repository.loadRecords('s1')).length, 1);
    },
  );

  test('every stage is logged with its elapsed time', () async {
    final lines = <String>[];
    final original = debugPrint;
    debugPrint = (String? message, {int? wrapWidth}) {
      if (message != null) lines.add(message);
    };
    try {
      await run(service());
    } finally {
      debugPrint = original;
    }
    expect(
      lines.where((String l) => l.contains('stage=takeStill')),
      isNotEmpty,
    );
    expect(lines.where((String l) => l.contains('stage=write')), isNotEmpty);
    expect(lines.where((String l) => l.contains('stage=verify')), isNotEmpty);
    expect(lines.where((String l) => l.contains('stage=record')), isNotEmpty);
    expect(lines.first, contains('[capture] +'));
  });
}
