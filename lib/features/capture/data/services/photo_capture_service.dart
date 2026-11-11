// The constructor parameters are deliberately named without the leading
// underscore so the public API reads `session:` rather than `_session:`, and
// a private named parameter cannot be used outside this library. That is the
// one case `prefer_initializing_formals` cannot express, so it is suppressed
// for this file rather than worked around with a worse API.
// ignore_for_file: prefer_initializing_formals
import 'dart:async';
import 'dart:io';

import 'package:flutter/foundation.dart';

import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/alignment_calculator.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// Takes the photograph and puts it on disk.
///
/// ## Where the pixels come from
///
/// ARCore holds the camera device exclusively: while a session is live, CameraX
/// and `camera2` cannot open the sensor at all — they fail with "camera in use",
/// not with a slower stream. The only real photograph available without tearing
/// the session down (which would throw away the pose the guidance depends on) is
/// `Frame.acquireCameraImage()`, a `YUV_420_888` `Image` ARCore hands over at
/// the CPU image resolution of the session's camera config.
///
/// The Kotlin host converts those raw planes straight to JPEG, rotating them by
/// the quarter turn it measured from ARCore's own image→display transform. That
/// means:
///
/// * **the photograph is a real camera frame of the room**;
/// * **it contains none of the app** — no GLB marker, no alignment overlay, no
///   Flutter widget — because none of that exists in the camera image;
/// * **its resolution is the AR session's CPU image size**, reported on every
///   record as [CaptureRecord.imageWidth]/[imageHeight]. On current devices that
///   is typically 640×480 to 1920×1080 depending on the camera config ARCore
///   picked. This is **not** a full-sensor still, and it is not claimed to be:
///   the numbers are recorded per photograph rather than asserted once in a
///   comment.
///
/// If ARCore ever refuses the image (some camera configs do not expose a CPU
/// image), [capture] throws and the target stays pending — it is never marked
/// captured on the strength of a request alone.
class PhotoCaptureService {
  PhotoCaptureService({
    required ArCoreGateway host,
    required CaptureRepository repository,
    this.jpegQuality = 95,
    this.minimumBytes = 2048,
    this.stillTimeout = const Duration(seconds: 6),
    this.saveTimeout = const Duration(seconds: 10),
  }) : _host = host,
       _repository = repository;

  /// How long the camera may take to hand over a frame before the attempt is
  /// failed instead of left hanging.
  ///
  /// A device recording showed the UI stuck on "Saving…" indefinitely: the
  /// native still request had been queued, the render loop stopped consuming
  /// it, and nothing ever answered the Dart future. A timeout turns that into
  /// a retryable failure with a stated reason — the one thing a hang can never
  /// do — and the stage timings below then name the stage that stalled.
  final Duration stillTimeout;

  /// When a durable commit has taken this long, emit an operational warning.
  ///
  /// This is deliberately not a cancellation deadline. Dart's [Future.timeout]
  /// cannot cancel filesystem work; rejecting its caller while the repository
  /// continues would let a late manifest commit contradict a reported failure.
  /// The in-flight latch remains held until the transaction really settles.
  final Duration saveTimeout;

  final ArCoreGateway _host;
  final CaptureRepository _repository;

  /// JPEG quality passed to the platform encoder.
  final int jpegQuality;

  /// A sanity floor: a JPEG smaller than this is not a photograph of a room.
  /// It catches an encoder that returned a truncated buffer, which would
  /// otherwise be saved and counted.
  final int minimumBytes;

  /// Whether a capture is in flight. Guards against a second shutter for the
  /// same target while the first JPEG is still being written.
  bool get busy => _busy;
  bool _busy = false;

  /// Captures one photograph and records it.
  ///
  /// Order matters and is enforced here: **stage → verify → manifest commit**.
  /// The repository owns the immutable JPEG version and the complete metadata
  /// snapshot, so a failed retake leaves the prior accepted photograph untouched.
  Future<CaptureCommitResult> capture({
    required CaptureStation station,
    required CaptureTarget target,
    required RigidPose camera,
    required AlignmentResult alignment,
    required bool anchored,
    String? anchorIdentifier,
    String? anchorSurfaceType,
    required double toleranceDeg,
    ArCameraIntrinsics? intrinsics,
    int? shutterPoseTimestampNs,
    Duration maxStillFrameLag = const Duration(milliseconds: 250),
  }) async {
    if (_busy) {
      throw const StorageFailure('A photograph is already being saved.');
    }
    _busy = true;
    final started = DateTime.now();
    var stage = 'takeStill';
    try {
      final ArCoreStill still;
      try {
        still = await _host
            .takeStill(quality: jpegQuality)
            .timeout(
              stillTimeout,
              onTimeout: () => throw StorageFailure(
                'The camera did not return an image within '
                '${stillTimeout.inSeconds}s. Nothing was saved.',
              ),
            );
      } catch (e) {
        _logCapture(started, 'takeStill', 'FAILED $e');
        rethrow;
      }
      _logCapture(
        started,
        'takeStill',
        '${still.jpeg.length} bytes ${still.width}x${still.height}',
      );
      if (still.jpeg.length < minimumBytes) {
        throw StorageFailure(
          'The camera returned ${still.jpeg.length} bytes, which is not a '
          'photograph. Nothing was saved.',
        );
      }
      if (!_isCompleteJpeg(still.jpeg)) {
        throw const StorageFailure(
          'The camera returned bytes that are not a complete JPEG '
          'photograph. Nothing was saved.',
        );
      }
      if (still.width <= 0 || still.height <= 0) {
        throw const StorageFailure(
          'The camera returned an image without valid encoded dimensions.',
        );
      }
      if (still.capturePose != null &&
          still.capturePose!.tracking != TrackingState.tracking) {
        throw const StorageFailure(
          'AR tracking was not valid at the instant the photograph was taken.',
        );
      }
      // A shutter request is consumed by the *next* render frame, not by the
      // EventChannel sample that armed it. Both timestamps come from ARCore's
      // monotonic frame clock, so this comparison never mixes wall time with
      // sensor time. Reject a queued/stalled request rather than saving a
      // photograph taken after its alignment and stability evidence expired.
      if (shutterPoseTimestampNs != null) {
        final captureTimestampNs = still.capturePose?.timestampNs;
        if (captureTimestampNs == null) {
          throw const StorageFailure(
            'The camera image had no ARCore timestamp, so its alignment could '
            'not be verified.',
          );
        }
        final lagNs = captureTimestampNs - shutterPoseTimestampNs;
        final maximumLagNs = maxStillFrameLag.inMicroseconds * 1000;
        if (lagNs < 0 || lagNs > maximumLagNs) {
          throw StorageFailure(
            'The camera image arrived ${lagNs < 0 ? 'before' : 'after'} the '
            'armed AR frame by ${(lagNs.abs() / 1000000).toStringAsFixed(0)} '
            'ms, outside the ${maxStillFrameLag.inMilliseconds} ms limit. '
            'Nothing was saved.',
          );
        }
      }

      final capturePose = still.capturePose;
      final capturedCamera = capturePose == null
          ? camera
          : RigidPose(
              position: capturePose.position,
              orientation: capturePose.captureWorld,
            );
      final model = _rotatedIntrinsics(still, intrinsics);
      final record = CaptureRecord(
        // `imagePath` is unpublished here. `commitCapture` assigns its unique
        // immutable final path only after the staged JPEG read-back, checksum
        // and decoder verification all succeed.
        stationId: station.stationId,
        targetId: target.targetId,
        imagePath: '',
        capturedAt: DateTime.now(),
        targetYawDegrees: target.yawDegrees,
        targetPitchDegrees: target.pitchDegrees,
        cameraPosition: capturedCamera.position,
        cameraOrientation: capturedCamera.orientation,
        alignmentErrorDeg: alignment.angularErrorDeg,
        anchored: anchored,
        anchorIdentifier: anchorIdentifier,
        anchorSurfaceType: anchorSurfaceType,
        imageWidth: still.width,
        imageHeight: still.height,
        sourceImageWidth: still.sourceWidth,
        sourceImageHeight: still.sourceHeight,
        captureTimestampNs: capturePose?.timestampNs,
        focalLengthXPx: model?.fx,
        focalLengthYPx: model?.fy,
        principalPointXPx: model?.cx,
        principalPointYPx: model?.cy,
        imageFormat: 'jpeg',
        rotationAppliedDeg: still.rotationAppliedDeg,
        toleranceDeg: toleranceDeg,
      );
      stage = 'commit';
      final slowCommitNotice = Timer(saveTimeout, () {
        _logCapture(
          started,
          'commit',
          'still waiting for the atomic image and manifest commit',
        );
      });
      final CaptureCommitResult committed;
      try {
        committed = await _repository.commitCapture(
          station: station,
          record: record,
          jpegBytes: still.jpeg,
        );
      } finally {
        slowCommitNotice.cancel();
      }
      _logCapture(started, 'write', committed.record.imagePath);
      _logCapture(
        started,
        'verify',
        '${committed.record.imageByteLength} bytes with SHA-256 '
            '${committed.record.imageSha256}',
      );
      _logCapture(started, 'record', 'target ${target.targetId} accepted');
      return committed;
    } catch (e) {
      _logCapture(started, stage, 'FAILED $e');
      rethrow;
    } finally {
      _busy = false;
    }
  }

  /// Moves source-image intrinsics into the coordinates of the rotated JPEG.
  /// A dimension mismatch means the two measurements are not provably from the
  /// same camera stream, so metadata is omitted rather than guessed.
  ({double fx, double fy, double cx, double cy})? _rotatedIntrinsics(
    ArCoreStill still,
    ArCameraIntrinsics? intrinsics,
  ) {
    if (intrinsics == null ||
        !intrinsics.isUsable ||
        intrinsics.imageWidth != still.sourceWidth ||
        intrinsics.imageHeight != still.sourceHeight) {
      return null;
    }
    final width = still.sourceWidth.toDouble();
    final height = still.sourceHeight.toDouble();
    final fx = intrinsics.focalLengthXPx;
    final fy = intrinsics.focalLengthYPx;
    final cx = intrinsics.principalPointXPx;
    final cy = intrinsics.principalPointYPx;
    return switch (still.rotationAppliedDeg % 360) {
      0 => (fx: fx, fy: fy, cx: cx, cy: cy),
      90 => (fx: fy, fy: fx, cx: height - 1 - cy, cy: cx),
      180 => (fx: fx, fy: fy, cx: width - 1 - cx, cy: height - 1 - cy),
      270 => (fx: fy, fy: fx, cx: cy, cy: width - 1 - cx),
      _ => null,
    };
  }

  /// One line per stage, with the elapsed time since the shutter.
  ///
  /// Deliberately unconditional: a capture happens 26 times per station, so
  /// this is 26 short lines in `logcat`, and the one thing a hang report needs
  /// is the stage it stopped at.
  void _logCapture(DateTime started, String stage, String detail) {
    final ms = DateTime.now().difference(started).inMilliseconds;
    debugPrint('[capture] +${ms}ms stage=$stage $detail');
  }
}

/// JPEG's SOI/EOI markers are a cheap, deterministic early rejection. The
/// repository performs the stronger post-write decoder/checksum verification
/// before it publishes an accepted manifest.
bool _isCompleteJpeg(List<int> bytes) =>
    bytes.length >= 4 &&
    bytes[0] == 0xff &&
    bytes[1] == 0xd8 &&
    bytes[bytes.length - 2] == 0xff &&
    bytes.last == 0xd9;

/// Reads a saved photograph back, for the review thumbnail.
Future<Uint8List?> readCaptureImage(String path) async {
  try {
    final file = File(path);
    if (!await file.exists()) return null;
    return await file.readAsBytes();
  } catch (_) {
    return null;
  }
}
