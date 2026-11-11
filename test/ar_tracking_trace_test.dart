import 'dart:convert';
import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as p;
import 'package:adaptive_360_capture_app/features/capture/data/services/ar_tracking_trace.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

import 'capture_flow_test.dart' show Harness, bootHarness;

/// The tracking trace exists because every field it carries was already being
/// printed to logcat and then thrown away: a session that lost VIO tracking an
/// hour earlier left no record of *which* `TrackingFailureReason` fired, at
/// which continuity epoch, or what the station distance read at that instant.
/// These tests pin the two properties that make it usable evidence — one line
/// per event, and a broken file never breaks a capture.
void main() {
  late Directory root;

  setUp(() async {
    root = await Directory.systemTemp.createTemp('ar_tracking_trace');
  });

  tearDown(() async {
    if (await root.exists()) await root.delete(recursive: true);
  });

  String tracePath() => p.join(root.path, 'trace.jsonl');

  List<Map<String, dynamic>> readLines(String path) => File(path)
      .readAsLinesSync()
      .where((l) => l.trim().isNotEmpty)
      .map((l) => jsonDecode(l) as Map<String, dynamic>)
      .toList();

  group('the file itself', () {
    test('appends one JSON object per event, keeping every field', () async {
      final trace = ArTrackingTrace.atFilePath(tracePath());
      trace.record('boundary', {
        'wallMs': 1000,
        'cameraFrom': 'tracking',
        'cameraTo': 'paused',
        'failureReason': 'INSUFFICIENT_LIGHT',
        'epoch': 3,
        'gyroHz': null,
      });
      trace.record('sessionEnd', {'wallMs': 2000, 'epoch': 3});
      await trace.flush();

      final lines = readLines(tracePath());
      expect(lines, hasLength(2));
      expect(lines[0]['event'], 'boundary');
      expect(lines[0]['failureReason'], 'INSUFFICIENT_LIGHT');
      expect(lines[0]['epoch'], 3);
      // An absent measurement stays absent rather than becoming a number.
      expect(lines[0].containsKey('gyroHz'), isTrue);
      expect(lines[0]['gyroHz'], isNull);
      expect(lines[1]['event'], 'sessionEnd');
    });

    test('a disabled trace records nothing and never throws', () async {
      final trace = ArTrackingTrace.disabled();
      expect(trace.enabled, isFalse);
      expect(trace.path, isNull);
      trace.record('boundary', {'failureReason': 'BAD_STATE'});
      await trace.flush();
      expect(File(tracePath()).existsSync(), isFalse);
    });

    test('one failed write turns the trace off instead of failing each event',
        () async {
      // A path whose parent does not exist: the append itself cannot succeed,
      // which is what a full or read-only documents directory looks like.
      final trace = ArTrackingTrace.atFilePath(
        p.join(root.path, 'missing', 'trace.jsonl'),
      );
      trace.record('boundary', {'epoch': 1});
      await trace.flush();
      expect(trace.enabled, isFalse);
      final before = File(p.join(root.path, 'missing', 'trace.jsonl'))
          .existsSync();
      trace.record('boundary', {'epoch': 2});
      await trace.flush();
      expect(before, isFalse);
      expect(trace.enabled, isFalse);
    });
  });

  group('through the controller', () {
    test('a camera tracking boundary is recorded with its failure reason',
        () async {
      final trace = ArTrackingTrace.atFilePath(tracePath());
      final harness = await bootHarness(trackingTrace: trace);
      await Harness.flush();

      harness.gateway.emitPose(
        position: const Vec3(0, 1.6, 0),
        worldOrientation: Quat.identity,
        tracking: TrackingState.paused,
        failureReason: 'INSUFFICIENT_LIGHT',
        trackingEpoch: 1,
        trackingTransition: true,
      );
      await Harness.flush();
      await trace.flush();

      final boundaries = readLines(tracePath())
          .where((l) => l['event'] == 'boundary')
          .toList();
      expect(boundaries, hasLength(1));
      expect(boundaries.single['cameraTo'], 'paused');
      expect(boundaries.single['failureReason'], 'INSUFFICIENT_LIGHT');
      expect(boundaries.single['epoch'], 1);
      expect(boundaries.single['nativeTransition'], isTrue);
    });

    test('a healthy stream records the acquisition and nothing else', () async {
      final trace = ArTrackingTrace.atFilePath(tracePath());
      final harness = await bootHarness(trackingTrace: trace);
      await Harness.flush();

      for (var i = 0; i < 5; i++) {
        harness.gateway.emitPose(
          position: Vec3(0.01 * i, 1.6, 0),
          worldOrientation: Quat.identity,
        );
        await Harness.flush();
      }
      await trace.flush();

      final boundaries = readLines(tracePath())
          .where((l) => l['event'] == 'boundary')
          .toList();
      // The first pose after boot moves the camera out of `initialising`, so a
      // healthy session does emit one line — the acquisition stamp that gives
      // every later loss an interval to measure against. Silence beyond it is
      // what keeps the file small enough to read on the phone.
      expect(boundaries, hasLength(1));
      expect(boundaries.single['cameraFrom'], 'initialising');
      expect(boundaries.single['cameraTo'], 'tracking');
      expect(boundaries.single['failureReason'], isNull);
    });
  });
}
