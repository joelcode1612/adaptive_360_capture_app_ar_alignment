import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/services/virtual_tour_photo_store.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/repositories/virtual_tour_repository.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/screens/virtual_tour_camera_screen.dart';

import 'capture_test_fakes.dart';

/// The 30-node camera screen's wiring, tested through the widget tree.
///
/// Everything that made these paths impossible to test without a phone now has
/// a seam: [FakeArGateway] supplies the session and its event stream, the screen
/// takes a monotonic clock so a 600 ms hold runs instantly, and permission is
/// injectable so boot never reaches the platform plugin.
///
/// The rule each test below pins is a behaviour, not a rendering detail: a
/// stalled stream must not fire the shutter, a fatal error must stop capture
/// until an explicit recalibration, backgrounding must release the camera
/// without losing the station, and the acquisition deadline must be separate
/// from the save deadline.

/// A real 64×48 baseline JPEG: decoding it in the test proves the screen's
/// image check runs against an actual image rather than a byte count.
final Uint8List testJpeg = base64Decode(
  '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGBcU'
  'FhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/2wBDAQcHBwoIChMKChMoGhYaKCgoKCgo'
  'KCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCj/wAARCAAwAEADASIA'
  'AhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQA'
  'AAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3'
  'ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWm'
  'p6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEA'
  'AwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSEx'
  'BhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElK'
  'U1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3'
  'uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwAooooA'
  'KKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigD/2Q==',
);

/// A raw ARCore quaternion whose [ArPose.captureWorld] is the identity, i.e.
/// the pose that armed the shutter for `h01` (looking down +Z at the station
/// origin).
ArPose stillPoseAtStation() => ArPose(
  position: Vec3.zero,
  orientation: Quat.axisAngle(180, Vec3.unitY),
  tracking: TrackingState.tracking,
);

/// A repository that fails loudly rather than quietly if a test reaches it.
///
/// Every path below stops at "the frame is on disk"; the create → upload →
/// finalise lifecycle belongs to `virtual_tour_upload_test.dart`. A screen test
/// that ever called one of these would be asserting an upload the fake cannot
/// honestly answer, so the `StateError` is the point.
class _NoUploadRepository implements VirtualTourRepository {
  @override
  Future<VirtualTourTicket> uploadCaptures({
    required String tourId,
    required String propertyId,
    required String stationId,
    required List<TourCapture> captures,
    VirtualTourUploadProgress? onProgress,
  }) async {
    throw StateError('These tests never reach the upload.');
  }

  @override
  Future<VirtualTourTicket> finaliseTour({required String tourId}) async {
    throw StateError('These tests never reach finalisation.');
  }

  @override
  Future<VirtualTourTicket> refreshTourStatus({required String tourId}) async {
    throw StateError('These tests never read a server status.');
  }

  @override
  Future<VirtualTourRecord?> tourRecord(String tourId) async => null;
}

/// A store whose write can be held open, so the save deadline can be tested
/// without a slow disk.
class GatedPhotoStore extends VirtualTourPhotoStore {
  GatedPhotoStore({required super.root});

  final Completer<void> saveGate = Completer<void>();

  /// How many saves reached the store, i.e. the photograph was acquired and
  /// decoded and the write is now held open.
  int saveCalls = 0;

  /// How many times the screen removed a frame. The save-timeout path attaches
  /// this cleanup to the store's late answer, so it is the observable that the
  /// timed-out frame was discarded rather than left on disk.
  int deleteCalls = 0;

  @override
  Future<void> deletePhoto({
    required String tourId,
    required int index,
    required String nodeId,
    int? generation,
  }) async {
    await super.deletePhoto(
      tourId: tourId,
      index: index,
      nodeId: nodeId,
      generation: generation,
    );
    deleteCalls++;
  }

  @override
  Future<VirtualTourSavedPhoto> saveJpeg({
    required String tourId,
    required int index,
    required String nodeId,
    required Uint8List bytes,
    int? expectedWidth,
    int? expectedHeight,
    int? generation,
  }) async {
    saveCalls++;
    await saveGate.future;
    return super.saveJpeg(
      tourId: tourId,
      index: index,
      nodeId: nodeId,
      bytes: bytes,
      expectedWidth: expectedWidth,
      expectedHeight: expectedHeight,
      generation: generation,
    );
  }
}

class ScreenHarness {
  ScreenHarness(this.gateway);

  final FakeArGateway gateway;
  final VirtualTourRepository repository = _NoUploadRepository();
  final String tourId = 'tour_screen_test';

  late Directory root;
  late VirtualTourPhotoStore store;

  /// The screen's monotonic clock, advanced by the test.
  Duration now = Duration.zero;

  /// Feeds a pose whose optical axis is the station's +Z, i.e. perfectly aimed
  /// at `h01`.
  void emitAligned({
    TrackingState tracking = TrackingState.tracking,
    Duration? at,
  }) {
    if (at != null) now = at;
    gateway.emitPose(
      position: Vec3.zero,
      worldOrientation: Quat.identity,
      tracking: tracking,
    );
  }
}

/// Boots the screen and lets every awaited boot step finish.
///
/// The async steps are pure Dart plus the fake gateway, so a handful of frames
/// is enough; no real work is deferred beyond them.
Future<ScreenHarness> pumpScreen(
  WidgetTester tester, {
  bool supported = true,
  Duration stillTimeout = const Duration(seconds: 8),
  Duration saveTimeout = const Duration(seconds: 10),
  int minimumJpegBytes = 2048,
  Future<void> Function()? ensurePermissions,
  GatedPhotoStore? gatedStore,
}) async {
  final harness = ScreenHarness(FakeArGateway(supported: supported));
  harness.root =
      gatedStore?.root ??
      Directory.systemTemp.createTempSync('virtual_tour_screen');
  harness.store = gatedStore ?? VirtualTourPhotoStore(root: harness.root);

  addTearDown(() async {
    await tester.pumpWidget(const SizedBox.shrink());
    await tester.pump();
    harness.gateway.close();
    if (harness.root.existsSync()) {
      harness.root.deleteSync(recursive: true);
    }
  });

  await tester.pumpWidget(
    MaterialApp(
      home: VirtualTourCameraScreen(
        host: harness.gateway,
        repository: harness.repository,
        photoStore: harness.store,
        tourId: harness.tourId,
        clock: () => harness.now,
        ensurePermissions: ensurePermissions ?? () async {},
        stillTimeout: stillTimeout,
        saveTimeout: saveTimeout,
        minimumJpegBytes: minimumJpegBytes,
      ),
    ),
  );
  for (var i = 0; i < 8; i++) {
    await tester.pump(const Duration(milliseconds: 10));
  }
  return harness;
}

/// Waits for real asynchronous work (image decode, file IO) to finish.
///
/// `pump` alone runs the fake clock and microtasks; the codec and the file
/// system answer on the real event loop, which needs [WidgetTester.runAsync].
Future<void> waitForReal(
  WidgetTester tester,
  bool Function() done, {
  String reason = 'the asynchronous work did not finish',
}) async {
  final deadline = DateTime.now().add(const Duration(seconds: 5));
  while (!done()) {
    if (DateTime.now().isAfter(deadline)) fail(reason);
    await tester.runAsync(
      () => Future<void>.delayed(const Duration(milliseconds: 10)),
    );
    await tester.pump();
  }
}

File photoFile(ScreenHarness harness, {String nodeId = 'h01', int index = 0}) =>
    File(
      harness.store.pathFor(
        tourId: harness.tourId,
        index: index,
        nodeId: nodeId,
      ),
    );

/// Holds the phone still at `h01` across the full 600 ms window.
///
/// Poses arrive every 50 ms, like a real 30 Hz stream. A 300 ms jump would be
/// a pose gap, which the screen correctly treats as a stalled stream and uses
/// to drop the hold — so a coarser fake would test the opposite behaviour.
Future<void> holdSteady(WidgetTester tester, ScreenHarness harness) async {
  for (var ms = 0; ms <= 600; ms += 50) {
    harness.emitAligned(at: Duration(milliseconds: ms));
    await tester.pump();
  }
}

void main() {
  testWidgets('boot starts one session and shows the 30-node HUD', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester);

    expect(harness.gateway.startCalls, 1);
    expect(harness.gateway.stopCalls, 0);
    expect(find.byType(Texture), findsOneWidget);
    expect(find.text('0 / 30 captured'), findsOneWidget);
    expect(find.text('DIAGNOSTICS · 30-NODE MODE'), findsOneWidget);
  });

  testWidgets('an unsupported device reports the boot failure, not a camera', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester, supported: false);

    expect(find.text('The virtual tour could not start'), findsOneWidget);
    expect(find.textContaining('Fake: unsupported device.'), findsOneWidget);
    expect(harness.gateway.startCalls, 0);
    expect(find.byType(Texture), findsNothing);
  });

  testWidgets('a denied permission is reported with its own message', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(
      tester,
      ensurePermissions: () async => throw const PermissionFailure(
        'Camera permission permanently denied. Open app settings to enable it.',
        permanentlyDenied: true,
      ),
    );

    expect(
      find.textContaining('Camera permission permanently denied'),
      findsOneWidget,
    );
    expect(find.text('Open settings'), findsOneWidget);
    expect(harness.gateway.startCalls, 0);
  });

  testWidgets('a stalled pose stream cannot fill the hold window', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester);

    harness.emitAligned();
    await tester.pump();
    harness.emitAligned(at: const Duration(milliseconds: 100));
    await tester.pump();
    harness.emitAligned(at: const Duration(milliseconds: 200));
    await tester.pump();
    expect(harness.gateway.stillRequests, 0);

    // The stream stops while the last sample still says "tracking". The
    // watchdog may only drop the hold; re-feeding that sample would have let
    // the 200 ms of true samples plus one stale stamp complete the window.
    harness.now = const Duration(milliseconds: 1300);
    await tester.pump(const Duration(milliseconds: 100));
    await tester.pump(const Duration(milliseconds: 100));

    expect(harness.gateway.stillRequests, 0);
    expect(find.text('SETTING UP'), findsOneWidget);
  });

  testWidgets('a full hold captures h01 through the session and stores it', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester, minimumJpegBytes: 512);
    harness.gateway
      ..stillBytes = testJpeg
      ..stillWidth = 64
      ..stillHeight = 48
      ..stillPose = stillPoseAtStation();

    await holdSteady(tester, harness);
    expect(harness.gateway.stillRequests, 1);

    await waitForReal(
      tester,
      () => find.text('1 / 30 captured').evaluate().isNotEmpty,
      reason: 'the photograph was never marked captured',
    );
    await tester.pump();

    expect(find.text('1 / 30 captured'), findsOneWidget);
    expect(photoFile(harness).existsSync(), isTrue);
    expect(photoFile(harness).lengthSync(), testJpeg.length);
  });

  testWidgets('a fatal render error blocks the shutter until recalibration', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester);

    harness.gateway.emitFault(
      'render loop failed',
      frame: 'ArCoreSessionController.kt:101',
    );
    await tester.pump();
    await tester.pump();

    expect(find.textContaining('render loop failed'), findsWidgets);
    expect(find.text('Recalibrate'), findsOneWidget);

    await holdSteady(tester, harness);
    expect(
      harness.gateway.stillRequests,
      0,
      reason: 'a fatal render error is a black camera, not a capture cue',
    );

    await tester.tap(find.text('Recalibrate'));
    await tester.pump();
    await waitForReal(
      tester,
      () => harness.gateway.startCalls == 2,
      reason: 'the recalibration never started a new session',
    );
    await tester.pump();

    expect(harness.gateway.stopCalls, 1);
    expect(find.text('Recalibrate'), findsNothing);
    expect(find.byType(Texture), findsOneWidget);
    expect(find.text('0 / 30 captured'), findsOneWidget);
  });

  testWidgets('a stopped camera requires recalibration and cannot capture', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester);

    harness.emitAligned();
    await tester.pump();
    harness.emitAligned(tracking: TrackingState.stopped);
    await tester.pump();
    await tester.pump();

    expect(find.textContaining('ARCore stopped the camera'), findsWidgets);
    expect(find.text('Recalibrate'), findsOneWidget);

    await holdSteady(tester, harness);
    expect(harness.gateway.stillRequests, 0);
    expect(find.text('Recalibrate'), findsOneWidget);
  });

  testWidgets(
    'backgrounding pauses the camera and resuming keeps the station',
    (WidgetTester tester) async {
      final harness = await pumpScreen(tester);

      tester.binding.handleAppLifecycleStateChanged(AppLifecycleState.paused);
      await tester.pump();
      await tester.pump();

      expect(harness.gateway.pauseCalls, 1);
      expect(harness.gateway.startCalls, 1);

      tester.binding.handleAppLifecycleStateChanged(AppLifecycleState.resumed);
      await tester.pump();
      await tester.pump();

      expect(harness.gateway.resumeCalls, 1);
      expect(
        harness.gateway.startCalls,
        1,
        reason: 'a resume must reopen the same session, not build a new world',
      );
      expect(find.text('Recalibrate'), findsNothing);
      expect(find.byType(Texture), findsOneWidget);
    },
  );

  testWidgets('a camera that cannot reopen requires recalibration', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester);

    tester.binding.handleAppLifecycleStateChanged(AppLifecycleState.paused);
    await tester.pump();
    await tester.pump();

    harness.gateway.resumeError = const ArCoreFailure('camera is busy');
    tester.binding.handleAppLifecycleStateChanged(AppLifecycleState.resumed);
    await tester.pump();
    await tester.pump();

    expect(find.textContaining('could not be reopened'), findsOneWidget);
    expect(find.text('Recalibrate'), findsOneWidget);

    await tester.tap(find.text('Recalibrate'));
    await tester.pump();
    await waitForReal(
      tester,
      () => harness.gateway.startCalls == 2,
      reason: 'the recalibration never started a new session',
    );
    await tester.pump();

    expect(find.text('Recalibrate'), findsNothing);
    expect(find.byType(Texture), findsOneWidget);
  });

  testWidgets('a still that never arrives fails on its own acquisition clock', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(
      tester,
      stillTimeout: const Duration(seconds: 1),
    );
    final held = Completer<void>();
    harness.gateway.stillGate = held.future;

    await holdSteady(tester, harness);
    expect(harness.gateway.stillRequests, 1);

    await tester.pump(const Duration(milliseconds: 1100));

    expect(
      find.textContaining('did not return an image within 1s'),
      findsWidgets,
    );
    expect(find.text('0 / 30 captured'), findsOneWidget);
    expect(photoFile(harness).existsSync(), isFalse);

    held.complete();
    await tester.pump();
  });

  testWidgets('a save that never finishes fails after the acquisition', (
    WidgetTester tester,
  ) async {
    final root = Directory.systemTemp.createTempSync('virtual_tour_gated');
    final gated = GatedPhotoStore(root: root);
    final harness = await pumpScreen(
      tester,
      minimumJpegBytes: 512,
      saveTimeout: const Duration(seconds: 1),
      gatedStore: gated,
    );
    harness.gateway
      ..stillBytes = testJpeg
      ..stillWidth = 64
      ..stillHeight = 48
      ..stillPose = stillPoseAtStation();

    await holdSteady(tester, harness);
    await waitForReal(
      tester,
      () => gated.saveCalls == 1,
      reason: 'the photograph never reached the store',
    );
    await tester.pump(const Duration(milliseconds: 1100));

    expect(
      find.textContaining('Checking and saving the photograph took longer'),
      findsWidgets,
    );
    expect(find.text('0 / 30 captured'), findsOneWidget);

    // The late write must not become a captured frame. The screen deletes it
    // when the store finally answers, so the count and the disk agree.
    gated.saveGate.complete();
    await waitForReal(
      tester,
      () => gated.deleteCalls == 1,
      reason: 'the timed-out frame was never deleted',
    );
    expect(photoFile(harness).existsSync(), isFalse);
    expect(find.text('0 / 30 captured'), findsOneWidget);
  });

  testWidgets('an attempt cancelled mid-still cannot commit its frame', (
    WidgetTester tester,
  ) async {
    final harness = await pumpScreen(tester, minimumJpegBytes: 512);
    final held = Completer<void>();
    harness.gateway
      ..stillBytes = testJpeg
      ..stillWidth = 64
      ..stillHeight = 48
      ..stillPose = stillPoseAtStation()
      ..stillGate = held.future;

    await holdSteady(tester, harness);
    expect(harness.gateway.stillRequests, 1);

    // Tracking is lost while the native still is queued. The attempt is
    // cancelled, so the photograph that arrives afterwards belongs to nothing.
    harness.emitAligned(tracking: TrackingState.paused);
    await tester.pump();

    held.complete();
    await waitForReal(
      tester,
      () => harness.gateway.stillReturns == 1,
      reason: 'the still never answered',
    );
    await tester.pump();

    expect(photoFile(harness).existsSync(), isFalse);
    expect(find.text('0 / 30 captured'), findsOneWidget);
  });
}
