import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';

import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/core/permissions/permission_service.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_local_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_remote_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/repositories/virtual_tour_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/services/virtual_tour_photo_store.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour_capture_engine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/repositories/virtual_tour_repository.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_target_model.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/glb/glb_renderer.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/bloc/virtual_tour_bloc.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/overlays/glb_target_overlay.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/overlays/virtual_tour_overlay.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/overlays/virtual_tour_scene.dart';

/// The 30-node virtual-tour station: world-locked GLB markers, automatic
/// capture, then upload and stitch.
///
/// **This is a separate mode from the 30-node AR capture screen.** Its sphere
/// is 30 directions at 1.0 m (12 horizon / 8 upper / 8 lower / zenith / nadir),
/// it never uses surface planes or anchors, and it fires the shutter itself once
/// the camera is inside the 3.0° cone and has been still for 600 ms. The older
/// flow is untouched.
///
/// The camera image is the ARCore session's own texture. Every marker is drawn
/// by the pure-Dart GLB projector in [VirtualTourScene] from its world position,
/// so moving the phone moves markers exactly as it moves the room. The bezel
/// (`white_frame_bezel.glb`) is re-planted on the optical axis each frame and
/// therefore stays at screen centre as the reticle.
class VirtualTourCameraScreen extends StatefulWidget {
  const VirtualTourCameraScreen({
    super.key,
    this.host,
    this.repository,
    this.photoStore,
    this.apiConfig,
    this.tourId,
    this.propertyId,
    this.stationId = 'station_01',
    this.stillTimeout = const Duration(seconds: 8),
    this.saveTimeout = const Duration(seconds: 10),
    this.minimumJpegBytes = 2048,
    this.diagnosticsEnabled = kDebugMode,
    this.clock,
    this.ensurePermissions,
  });

  /// The AR bridge, injectable for tests. A real run uses [ArCoreHost].
  final ArCoreGateway? host;

  /// The upload repository, when a caller already built one.
  final VirtualTourRepository? repository;

  /// Where the 30 JPEGs are written.
  final VirtualTourPhotoStore? photoStore;

  /// The SiteSense endpoint set. `null` means "read `--dart-define`", and the
  /// mode comes from `VIRTUAL_TOUR_MODE`: declaring `backend` without a URL is
  /// a boot failure, not a quiet downgrade to simulation.
  final VirtualTourApiConfig? apiConfig;

  final String? tourId;

  /// The property this station is filed under, as a UUID.
  ///
  /// `null` reads `VIRTUAL_TOUR_PROPERTY_ID`, because the backend's
  /// `property_id` is typed `uuid.UUID` and a placeholder string is a 422.
  final String? propertyId;
  final String stationId;

  /// How long ARCore may take to hand over a still before the attempt fails.
  final Duration stillTimeout;

  /// How long validation and the disk write may take before the attempt fails.
  final Duration saveTimeout;

  /// A JPEG smaller than this is not a photograph of a room.
  final int minimumJpegBytes;

  /// Shows the development diagnostics panel. Defaults to debug builds only.
  final bool diagnosticsEnabled;

  /// A monotonic clock, injectable so a widget test can walk through the
  /// 600 ms hold and the staleness budget without sleeping. A real run leaves
  /// this null and uses the screen's own [Stopwatch]; it is never the wall
  /// clock, because a clock adjustment must not open the shutter.
  final Duration Function()? clock;

  /// Requests camera permission. Injectable so widget tests can boot without
  /// the platform permission plugin; a real run uses [PermissionService].
  final Future<void> Function()? ensurePermissions;

  @override
  State<VirtualTourCameraScreen> createState() =>
      _VirtualTourCameraScreenState();
}

class _VirtualTourCameraScreenState extends State<VirtualTourCameraScreen>
    with WidgetsBindingObserver {
  /// About 15 marker frames per second. Poses arrive at ~30 Hz and the capture
  /// gate reads every one of them; only the projection is throttled, because
  /// each frame can carry several thousand translucent triangles.
  static const Duration _uiInterval = Duration(milliseconds: 66);

  ArCoreGateway? _host;
  VirtualTourRemoteDataSource? _ownedRemote;
  VirtualTourBloc? _bloc;
  VirtualTourCaptureEngine? _engine;
  VirtualTourScene? _scene;
  VirtualTourPhotoStore? _photoStore;
  StreamSubscription<ArCoreEvent>? _events;

  GlbCaptureModels? _models;
  bool _modelsReady = false;
  String? _modelFailure;

  late final String _tourId;
  String? _bootError;
  bool _permissionDenied = false;
  String? _arFault;
  bool _booted = false;
  bool _bootScheduled = false;
  bool _disposed = false;
  int? _textureId;

  final Map<String, String> _photoPaths = <String, String>{};
  final Map<String, DateTime> _capturedAt = <String, DateTime>{};

  /// The pose ARCore reported for each saved frame, in the shape the backend's
  /// `pose_json` field takes.
  ///
  /// Held beside the path rather than recomputed at upload time: the upload
  /// happens after the user has walked away from that spot, and the camera
  /// pose of a photograph is a fact about the moment it was taken.
  final Map<String, CapturePose> _capturePoses = <String, CapturePose>{};
  VirtualTourCaptureAttempt? _activeAttempt;
  bool _uploadRequested = false;
  String? _captureError;

  /// Allocated once per capture attempt and threaded through every file
  /// operation for it, so a late cleanup from an older attempt cannot delete a
  /// newer attempt's photograph.
  int _saveGeneration = 0;

  VirtualTourGuidance? _guidance;
  ArPoseSample? _lastPose;
  TrackingState _tracking = TrackingState.initialising;
  List<GlbScreenTriangle> _triangles = const <GlbScreenTriangle>[];
  int _revision = 0;
  Size _viewport = Size.zero;
  ArCameraIntrinsics? _intrinsics;

  /// The sensor image's +x direction in view coordinates, measured by the
  /// native host. Null until the first `imageOrientation` event.
  ({double x, double y})? _imageAxis;
  double _hfovDeg = 53.669;
  double _vfovDeg = 68;

  /// Monotonic. Every capture gate and every timeout is measured against this;
  /// `DateTime.now()` is used only for the upload's capture timestamps.
  final Stopwatch _clock = Stopwatch()..start();

  /// The single reading of the monotonic clock.
  Duration _now() => widget.clock?.call() ?? _clock.elapsed;

  Duration _lastPoseAt = Duration.zero;
  Duration _lastUiAt = Duration.zero;
  Timer? _watchdog;

  /// The app is in the background and ARCore released the camera. Poses that
  /// were already queued when that happened are ignored.
  bool _pausedForLifecycle = false;

  /// A fatal AR/render error or a stopped camera: no capture may start until
  /// the station is rebuilt with `_recalibrate`.
  bool _recoveryRequired = false;
  bool _recalibrating = false;

  /// Where the last failure happened, for the diagnostics panel. The category
  /// names the phase (native, lifecycle, capture · savingImage, …) and the
  /// frame is the native one when the host reported it.
  String? _lastErrorCategory;
  String? _lastErrorFrame;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _tourId =
        widget.tourId ??
        'tour_${DateTime.now().toUtc().millisecondsSinceEpoch}';
    final fallback = ArCameraIntrinsics.fallbackDisplayFov;
    _hfovDeg = fallback.hfovDeg;
    _vfovDeg = fallback.vfovDeg;
    _watchdog = Timer.periodic(
      const Duration(milliseconds: 50),
      (_) => _onWatchdog(),
    );
    unawaited(_loadModels());
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    super.didChangeAppLifecycleState(state);
    if (_disposed) return;
    switch (state) {
      case AppLifecycleState.paused:
      case AppLifecycleState.hidden:
        unawaited(_pauseForLifecycle());
      case AppLifecycleState.resumed:
        unawaited(_resumeFromLifecycle());
      case AppLifecycleState.inactive:
      case AppLifecycleState.detached:
        // `inactive` also covers a notification shade or a permission dialog,
        // where releasing and reopening the camera would interrupt capture for
        // no reason. Only a real background releases it.
        break;
    }
  }

  Future<void> _pauseForLifecycle() async {
    if (_pausedForLifecycle || _disposed) return;
    final host = _host;
    if (host == null) return;
    _pausedForLifecycle = true;

    final attempt = _activeAttempt;
    if (attempt != null) {
      _engine?.cancelCapture(attempt);
      _activeAttempt = null;
      _captureError =
          'The app went to the background while the photograph was being '
          'taken. The attempt was cancelled and nothing was marked captured.';
      _lastErrorCategory = 'lifecycle · capturing';
    }
    _engine?.invalidateStability();
    _guidance = null;
    // A pose from before the background is not a measurement now. Clearing it
    // also makes the first pose after the resume count as a fresh sample
    // rather than one that happens to follow a very long gap.
    _lastPose = null;
    _tracking = TrackingState.paused;
    if (mounted) setState(() {});

    if (_textureId == null) {
      // The session was still starting. It will be paused by the resume path's
      // no-op if it never ran; there is nothing to release yet.
      return;
    }
    try {
      await host.pause();
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _arFault = 'The AR camera could not be released: $e';
        _lastErrorCategory = 'lifecycle · pause';
      });
    }
  }

  Future<void> _resumeFromLifecycle() async {
    if (!_pausedForLifecycle || _disposed) return;
    _pausedForLifecycle = false;
    final host = _host;
    if (host == null) return;
    if (_textureId == null) {
      // The session had not finished starting when the app was backgrounded,
      // so the pause never reached it. Whatever is still booting is already the
      // fresh session a resume would ask for.
      return;
    }
    try {
      await host.resume();
      if (!mounted) return;
      _engine?.invalidateStability();
      _guidance = null;
      _lastPose = null;
      _lastPoseAt = _now();
      setState(() {});
    } catch (e) {
      if (!mounted) return;
      _requireRecovery(
        'The AR camera could not be reopened: $e. Recalibrate to restart this '
        'station.',
        category: 'lifecycle · resume',
      );
    }
  }

  /// Halts capture and asks for a recalibration.
  ///
  /// A fatal render error, a stopped camera or a failed resume all mean the
  /// current world no longer has a dependable camera pose. Guessing that the
  /// next frame will be fine is what lets a black viewport keep "capturing"
  /// photographs taken from a stale pose, so nothing may restart automatically.
  void _requireRecovery(String message, {String? frame, String? category}) {
    if (_recalibrating) return;
    final attempt = _activeAttempt;
    if (attempt != null) {
      _engine?.cancelCapture(attempt);
      _activeAttempt = null;
    }
    _engine?.invalidateStability();
    _guidance = null;
    _recoveryRequired = true;
    _arFault = message;
    _lastErrorFrame = frame ?? _lastErrorFrame;
    _lastErrorCategory = category ?? 'native';
    if (mounted) setState(() {});
  }

  Future<void> _loadModels() async {
    try {
      final models = await GlbCaptureModels.load();
      if (!mounted) return;
      if (!models.matchesSpec) {
        setState(() {
          _modelFailure =
              'The marker assets no longer match the recorded dimensions; '
              'the aiming reticle is standing in for them.';
        });
        return;
      }
      setState(() {
        _models = models;
        _modelsReady = true;
        _modelFailure = null;
      });
      _ensureScene();
    } catch (e) {
      if (!mounted) return;
      setState(() => _modelFailure = '$e');
    }
  }

  Future<void> _boot(Size viewport) async {
    if (_booted) return;
    _booted = true;
    try {
      final ensurePermissions =
          widget.ensurePermissions ??
          PermissionService().ensureCapturePermissions;
      await ensurePermissions();
      final host = widget.host ?? ArCoreHost();
      _host = host;

      final availability = await host.checkAvailability();
      if (!availability.supported) {
        throw ArCoreFailure(
          availability.reason ?? 'This device cannot run ARCore.',
        );
      }
      if (!availability.serviceInstalled) {
        final install = await host.requestInstall();
        if (!install.installed) {
          throw ArCoreFailure(
            'ARCore is not installed on this device '
            '(${install.status ?? 'no install confirmation'}).',
          );
        }
      }

      // Storage first: a pose can arrive on the frame the session starts, and
      // the first capture must have somewhere to go.
      _photoStore =
          widget.photoStore ?? await VirtualTourPhotoStore.createDefault();
      final repository = widget.repository ?? await _createRepository();
      _bloc = VirtualTourBloc(repository: repository);

      // No planes and no anchors: the 30 nodes are computed from the first
      // tracked pose, not discovered in the room.
      await _startSession(viewport);
    } on PermissionFailure catch (failure) {
      if (!mounted) return;
      setState(() {
        _bootError = failure.message;
        _permissionDenied = failure.permanentlyDenied;
      });
    } catch (e) {
      if (!mounted) return;
      setState(() => _bootError = '$e');
    }
  }

  /// Starts the AR session and attaches the event stream.
  ///
  /// Shared by the first boot and by a recalibration, so both wire the errors
  /// the same way: a stream that dies is a reason to stop capturing, not a
  /// banner over a screen that keeps firing the shutter.
  Future<void> _startSession(Size viewport) async {
    if (!mounted) return;
    final host = _host!;
    final ratio = MediaQuery.of(context).devicePixelRatio;
    final session = await host.start(
      width: (viewport.width * ratio).round(),
      height: (viewport.height * ratio).round(),
      planes: false,
    );
    if (session.textureId < 0) {
      throw const ArCoreFailure(
        'The AR session started without a camera texture.',
      );
    }
    _events = host.events.listen(
      _onArEvent,
      onError: (Object error, StackTrace _) {
        _requireRecovery(
          'The AR event stream failed: $error. Recalibrate to restart this '
          'station.',
          category: 'event stream',
        );
      },
    );
    if (!mounted) {
      await host.stop();
      return;
    }
    setState(() => _textureId = session.textureId);
  }

  Future<VirtualTourRepository> _createRepository() async {
    final local = await VirtualTourLocalDataSource.createDefault();
    // Declared mode, never inferred from "did a URL happen to be defined":
    // a backend build with no endpoint fails here, and the boot handler shows
    // the reason, instead of the app simulating a whole station unnoticed.
    final remote = createVirtualTourRemoteDataSource(
      mode: widget.apiConfig == null ? null : VirtualTourMode.backend,
      config: widget.apiConfig,
    );
    if (remote is HttpVirtualTourRemoteDataSource) _ownedRemote = remote;
    return VirtualTourRepositoryImpl(remote: remote, local: local);
  }

  void _ensureScene() {
    final models = _models;
    final engine = _engine;
    if (models == null || engine == null || _scene != null) return;
    _scene = VirtualTourScene(
      targetModel: models.target,
      bezelModel: models.bezel,
      nodes: engine.nodes,
      startPosition: engine.startPosition,
      startOrientation: engine.startOrientation,
    );
    if (mounted) setState(() {});
  }

  void _onArEvent(ArCoreEvent event) {
    if (!mounted) return;
    switch (event.type) {
      case ArCoreEventType.pose:
        final sample = event.pose;
        if (sample != null) _onPose(sample);
      case ArCoreEventType.error:
        if (_recalibrating) break;
        final message = event.message ?? 'The AR host reported an error.';
        _lastErrorFrame = event.frame;
        if (event.fatal) {
          // A fatal render error is a black camera: frames may keep arriving,
          // and their poses may even say "tracking", but what the user sees is
          // not the room. Capturing from it is not a recoverable guess.
          _requireRecovery(message, frame: event.frame, category: 'native');
        } else {
          setState(() {
            _arFault = message;
            _lastErrorCategory = 'native · transient';
          });
        }
      case ArCoreEventType.imageOrientation:
        final axis = event.imageAxis;
        if (axis != null) {
          _imageAxis = axis;
          _applyDisplayFov();
        }
      case ArCoreEventType.unknown:
        break;
    }
  }

  /// Derives the display field of view from the sensor intrinsics and the
  /// measured image axis.
  ///
  /// The axis is the authority. The native host measures it with ARCore's own
  /// image→view transform, while `displayRotation` is the surface rotation —
  /// 0 for this portrait-locked app even though the landscape sensor's x axis
  /// runs down the screen. Until the axis arrives, portrait is assumed so the
  /// first intrinsics cannot flip the projection by 90°.
  void _applyDisplayFov() {
    final intrinsics = _intrinsics;
    if (intrinsics == null || !intrinsics.isUsable) return;
    final axis = _imageAxis;
    final fov = intrinsics.displayFovForImageAxis(
      axisX: axis?.x ?? 0,
      axisY: axis?.y ?? 1,
    );
    _hfovDeg = fov.hfovDeg;
    _vfovDeg = fov.vfovDeg;
  }

  void _onPose(ArPoseSample sample) {
    if (_pausedForLifecycle || _recalibrating) return;
    final now = _now();
    final gap = _lastPose == null ? Duration.zero : now - _lastPoseAt;
    _tracking = sample.tracking;
    _lastPose = sample;
    _lastPoseAt = now;

    if (sample.tracking == TrackingState.stopped && !_recoveryRequired) {
      // ARCore only reports STOPPED when the session itself can no longer
      // deliver a camera pose. A new pose will not fix it; a rebuilt station
      // will, and that is a decision for the user.
      final reason = sample.failureReason;
      _requireRecovery(
        'ARCore stopped the camera${reason == null ? '' : ' ($reason)'}. '
        'Recalibrate to restart this station.',
        category: 'tracking · stopped',
      );
      _updateFrame(now);
      return;
    }

    final intrinsics = sample.intrinsics;
    if (intrinsics != null && intrinsics.isUsable) {
      _intrinsics = intrinsics;
      _applyDisplayFov();
    }

    var engine = _engine;
    if (engine == null && sample.tracking == TrackingState.tracking) {
      // The first tracked pose is the station: every node is built once from
      // it and never rebuilt, which is what keeps the markers still.
      engine = VirtualTourCaptureEngine.atStart(
        position: sample.position,
        orientation: sample.orientation,
      );
      _engine = engine;
      _ensureScene();
    }

    if (_activeAttempt != null && sample.tracking != TrackingState.tracking) {
      _cancelActiveAttempt(
        'AR tracking was lost while the photograph was being taken. '
        'The attempt was cancelled and nothing was marked captured.',
      );
    }

    // A pose gap longer than the staleness budget is not a continuous
    // measurement. Restarting the hold is what stops a 600 ms window from
    // silently spanning a period in which no frames were delivered.
    if (engine != null && gap > engine.maxPoseAge) {
      engine.invalidateStability();
      _guidance = null;
    }

    _evaluateAt(sample, now, Duration.zero);
    _updateFrame(now);
  }

  /// Catches a pose stream that stops while its last sample still says
  /// "tracking".
  ///
  /// This only updates the UI. It deliberately does **not** feed the last
  /// sample back into [VirtualTourCaptureEngine.evaluate]: that sample is not a
  /// new measurement, and counting it toward the 600 ms window let a stalled
  /// stream look stable. Once the age exceeds the engine's budget the hold is
  /// dropped entirely; no auto-capture can start without a fresh pose.
  void _onWatchdog() {
    if (!mounted || _pausedForLifecycle || _recalibrating) return;
    final engine = _engine;
    final sample = _lastPose;
    if (engine == null || sample == null) return;
    final now = _now();
    final age = now - _lastPoseAt;
    if (age < const Duration(milliseconds: 45)) return;
    if (age >= engine.maxPoseAge) {
      engine.invalidateStability();
      _guidance = null;
    }
    _updateFrame(now);
  }

  VirtualTourGuidance? _evaluateAt(
    ArPoseSample sample,
    Duration now,
    Duration measurementAge,
  ) {
    final engine = _engine;
    if (engine == null) {
      _guidance = null;
      return null;
    }
    final guidance = engine.evaluate(
      cameraPosition: sample.position,
      cameraOrientation: sample.orientation,
      tracking: sample.tracking,
      now: now,
      measurementAge: measurementAge,
      angularRateDegPerSec: sample.raw.angularRateDegPerSec,
    );
    _guidance = guidance;

    if (_activeAttempt != null && engine.attempt?.id != _activeAttempt!.id) {
      _activeAttempt = null;
    }

    if (guidance.readyToCapture &&
        measurementAge <= engine.maxPoseAge &&
        _activeAttempt == null &&
        !_uploadRequested &&
        !_recoveryRequired &&
        !_pausedForLifecycle) {
      final target = guidance.target;
      if (target != null) {
        final attempt = engine.beginCapture(node: target, now: now);
        if (attempt != null) {
          _activeAttempt = attempt;
          unawaited(_capture(attempt));
        }
      }
    }
    return guidance;
  }

  void _cancelActiveAttempt(String reason) {
    final attempt = _activeAttempt;
    if (attempt == null) return;
    _engine?.cancelCapture(attempt);
    _activeAttempt = null;
    _captureError = reason;
  }

  bool _isCurrentAttempt(VirtualTourCaptureAttempt attempt) =>
      _activeAttempt?.id == attempt.id && _engine?.attempt?.id == attempt.id;

  void _updateFrame(Duration now) {
    if (!mounted) return;
    if (now - _lastUiAt < _uiInterval) return;
    _lastUiAt = now;
    final scene = _scene;
    final sample = _lastPose;
    var triangles = const <GlbScreenTriangle>[];
    if (scene != null &&
        sample != null &&
        _viewport.width > 0 &&
        _viewport.height > 0) {
      final poseAge = now - _lastPoseAt;
      final maxPoseAge =
          _engine?.maxPoseAge ?? const Duration(milliseconds: 150);
      final trackingAvailable =
          _tracking == TrackingState.tracking && poseAge <= maxPoseAge;
      triangles = scene.triangles(
        camera: GlbCamera(
          pose: sample.orientation,
          position: sample.position,
          viewportW: _viewport.width,
          viewportH: _viewport.height,
          hfovDeg: _hfovDeg,
          vfovDeg: _vfovDeg,
        ),
        capturedIds: _engine?.capturedIds ?? const <String>{},
        hudState: _hudState(_guidance),
        trackingAvailable: trackingAvailable,
      );
    }
    setState(() {
      _triangles = triangles;
      _revision++;
    });
  }

  Future<void> _capture(VirtualTourCaptureAttempt attempt) async {
    final host = _host;
    final store = _photoStore;
    final engine = _engine;
    if (host == null || store == null || engine == null) {
      // Leaving the engine's attempt reserved would block every later shutter
      // for this node: `beginCapture` refuses while an attempt exists.
      engine?.cancelCapture(attempt);
      if (_activeAttempt?.id == attempt.id) {
        _activeAttempt = null;
        _captureError =
            'The capture session was released before the photograph could be '
            'taken. No target was marked captured.';
        _lastErrorCategory = 'capture · session released';
        if (mounted) setState(() {});
      }
      return;
    }
    final node = attempt.node;
    final generation = ++_saveGeneration;
    if (mounted) {
      setState(() {
        _captureError = null;
      });
    }

    var success = false;
    String? failureMessage;

    try {
      // Acquisition has its own deadline. The validation/save budget starts
      // only once the JPEG exists: previously a single 10 s clock included the
      // 8 s still timeout, leaving 2 s for decoding, checking and writing.
      final still = await host
          .takeStill(quality: 95)
          .timeout(
            widget.stillTimeout,
            onTimeout: () => throw VirtualTourFailure(
              'The camera did not return an image within '
              '${widget.stillTimeout.inSeconds}s. Nothing was saved.',
            ),
          );
      if (!_isCurrentAttempt(attempt)) return;
      final deadline = _now() + widget.saveTimeout;
      Duration remaining() {
        final left = deadline - _now();
        return left > Duration.zero ? left : Duration.zero;
      }

      engine.markAttemptState(attempt, VirtualTourCaptureState.validatingImage);

      if (still.jpeg.length < widget.minimumJpegBytes) {
        throw VirtualTourFailure(
          'The camera returned ${still.jpeg.length} bytes, which is not a '
          'photograph. Nothing was saved.',
        );
      }
      final stillPose = still.capturePose;
      if (stillPose == null) {
        throw const VirtualTourFailure(
          'The camera did not report the pose of this photograph, so it '
          'cannot be matched to its target. Nothing was saved.',
        );
      }
      if (stillPose.tracking != TrackingState.tracking) {
        throw const VirtualTourFailure(
          'AR was not tracking when the photograph was taken. Nothing was '
          'saved.',
        );
      }
      final poseError = engine.validateCapturePose(
        attempt: attempt,
        cameraPosition: stillPose.position,
        cameraOrientation: stillPose.captureWorld,
      );
      if (poseError != null) throw VirtualTourFailure(poseError);

      final decoded = await _decodeJpeg(still.jpeg).timeout(
        remaining(),
        onTimeout: () => throw const VirtualTourFailure(
          'Checking the photograph took too long. Nothing was saved.',
        ),
      );
      if (!_isCurrentAttempt(attempt)) {
        decoded.dispose();
        return;
      }
      final decodedWidth = decoded.width;
      final decodedHeight = decoded.height;
      decoded.dispose();
      if (still.width > 0 &&
          still.height > 0 &&
          !_dimensionsMatch(
            decodedWidth: decodedWidth,
            decodedHeight: decodedHeight,
            reportedWidth: still.width,
            reportedHeight: still.height,
          )) {
        throw VirtualTourFailure(
          'The photograph decoded to $decodedWidth×$decodedHeight, but the '
          'camera reported ${still.width}×${still.height}. Nothing was saved.',
        );
      }

      engine.markAttemptState(attempt, VirtualTourCaptureState.savingImage);
      final saveFuture = store.saveJpeg(
        tourId: _tourId,
        index: node.index,
        nodeId: node.id,
        bytes: still.jpeg,
        generation: generation,
        expectedWidth: still.width > 0 ? still.width : null,
        expectedHeight: still.height > 0 ? still.height : null,
      );
      final VirtualTourSavedPhoto photo;
      try {
        photo = await saveFuture.timeout(remaining());
      } on TimeoutException {
        unawaited(
          saveFuture
              .then(
                (_) => store.deletePhoto(
                  tourId: _tourId,
                  index: node.index,
                  nodeId: node.id,
                  generation: generation,
                ),
              )
              .catchError((Object _) {}),
        );
        throw VirtualTourFailure(
          'Checking and saving the photograph took longer than '
          '${widget.saveTimeout.inSeconds}s. Nothing was marked captured.',
        );
      }

      if (!_isCurrentAttempt(attempt)) {
        await store.deletePhoto(
          tourId: _tourId,
          index: node.index,
          nodeId: node.id,
          generation: generation,
        );
        return;
      }
      _photoPaths[node.id] = photo.file.path;
      _capturedAt[node.id] = DateTime.now().toUtc();
      _capturePoses[node.id] = _poseOfFrame(
        pose: stillPose,
        attempt: attempt,
      );
      success = true;
    } catch (e) {
      failureMessage = e is VirtualTourFailure
          ? e.message
          : 'The photograph could not be saved: $e';
      _captureError = failureMessage;
      _lastErrorCategory =
          'capture · ${engine.attempt?.state.name ?? 'capturing'}';
    } finally {
      if (_isCurrentAttempt(attempt)) {
        engine.completeCapture(
          attempt,
          success: success,
          now: _now(),
          error: failureMessage,
        );
        _activeAttempt = null;
        if (success) {
          _captureError = null;
          _lastErrorCategory = null;
          _lastErrorFrame = null;
        }
        if (mounted) setState(() {});
      } else if (_activeAttempt?.id == attempt.id) {
        _activeAttempt = null;
      }
    }

    if (success && engine.allCaptured && !_disposed) _requestUpload();
  }

  Future<ui.Image> _decodeJpeg(Uint8List bytes) async {
    final codec = await ui.instantiateImageCodec(bytes);
    try {
      final frame = await codec.getNextFrame();
      return frame.image;
    } finally {
      codec.dispose();
    }
  }

  static bool _dimensionsMatch({
    required int decodedWidth,
    required int decodedHeight,
    required int reportedWidth,
    required int reportedHeight,
  }) =>
      (decodedWidth == reportedWidth && decodedHeight == reportedHeight) ||
      (decodedWidth == reportedHeight && decodedHeight == reportedWidth);

  void _manualRetry() {
    if (_recoveryRequired || _pausedForLifecycle) return;
    final engine = _engine;
    final guidance = _guidance;
    final node = guidance?.target;
    if (engine == null || node == null || _activeAttempt != null) return;
    if (!engine.manualRetry(node: node)) return;
    _captureError = null;
    final sample = _lastPose;
    if (sample != null && mounted) {
      final now = _now();
      _evaluateAt(sample, now, now - _lastPoseAt);
      _updateFrame(now);
    }
  }

  /// Rebuilds the station from scratch: a new session, a new world origin, a
  /// new set of 30 targets and an empty photo folder.
  ///
  /// This is the only path back from [recoveryRequired]. It deletes the
  /// photographs of the old world on purpose: they were taken from targets
  /// expressed in a world origin that no longer exists, and the stitcher would
  /// otherwise mix two layouts into one panorama.
  Future<void> _recalibrate() async {
    if (_recalibrating) return;
    final host = _host;
    final store = _photoStore;
    if (host == null || store == null || _disposed) return;
    _recalibrating = true;
    _recoveryRequired = false;
    _arFault = null;
    _captureError = null;
    _lastErrorCategory = null;
    _lastErrorFrame = null;

    final attempt = _activeAttempt;
    if (attempt != null) {
      _engine?.cancelCapture(attempt);
      _activeAttempt = null;
    }

    // Drop the texture before the native entry behind it is released, or the
    // Texture widget would ask Flutter to draw a texture that no longer exists.
    setState(() {
      _textureId = null;
      _triangles = const <GlbScreenTriangle>[];
      _guidance = null;
      _tracking = TrackingState.initialising;
      _revision++;
    });

    try {
      await _events?.cancel();
      _events = null;
      await host.stop();
      await store.clearTour(_tourId);
      if (!mounted) return;
      final fallback = ArCameraIntrinsics.fallbackDisplayFov;
      setState(() {
        _engine = null;
        _scene = null;
        _lastPose = null;
        _lastPoseAt = _now();
        _intrinsics = null;
        _imageAxis = null;
        _hfovDeg = fallback.hfovDeg;
        _vfovDeg = fallback.vfovDeg;
        _photoPaths.clear();
        _capturedAt.clear();
        _capturePoses.clear();
        // `_saveGeneration` deliberately keeps counting across a station
        // reset: an attempt still in flight from before the recalibration has
        // an older number, so its late cleanup can never delete a frame the
        // new station just saved.
        _uploadRequested = false;
      });
      await _startSession(_viewport);
      if (mounted) setState(() => _recalibrating = false);
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _recalibrating = false;
        _recoveryRequired = true;
        _arFault = 'The station could not be recalibrated: $e';
        _lastErrorCategory = 'recalibration';
      });
    }
  }

  /// The pose recorded for one saved frame, in the backend's `CapturePose`
  /// shape.
  ///
  /// Position and orientation are ARCore's own values at the instant of
  /// exposure — the same ones the gate validated against the target — in the
  /// ARCore world frame. The two error figures are what the hold was measured
  /// to be when the shutter fired, not a restatement of the pose.
  CapturePose _poseOfFrame({
    required ArPose pose,
    required VirtualTourCaptureAttempt attempt,
  }) => CapturePose(
    positionX: pose.position.x,
    positionY: pose.position.y,
    positionZ: pose.position.z,
    quaternionX: pose.captureWorld.x,
    quaternionY: pose.captureWorld.y,
    quaternionZ: pose.captureWorld.z,
    quaternionW: pose.captureWorld.w,
    alignmentErrorDeg: attempt.alignmentDeg,
    stationDriftM: attempt.driftM,
  );

  void _requestUpload() {
    if (_uploadRequested) return;
    final engine = _engine;
    final bloc = _bloc;
    if (engine == null || bloc == null || _disposed) return;
    final captures = <TourCapture>[
      for (final node in engine.nodes)
        if (_photoPaths[node.id] case final path?)
          TourCapture(
            // 1-based: this is the backend's `target_index`, whose form field
            // is declared `ge=1` and which orders the stitch manifest.
            targetIndex: node.index + 1,
            nodeId: node.id,
            filePath: path,
            yawDeg: node.yawDeg,
            pitchDeg: node.pitchDeg,
            capturedAt: _capturedAt[node.id] ?? DateTime.now().toUtc(),
            pose: _capturePoses[node.id],
          ),
    ];
    if (captures.length != engine.totalCount) {
      setState(() {
        _captureError =
            'All targets are marked captured, but only ${captures.length} of '
            '${engine.totalCount} photographs are on disk.';
      });
      return;
    }
    final missingPose = captures.where((capture) => capture.pose == null);
    if (missingPose.isNotEmpty) {
      // Uploading without a pose would file a manifest entry the stitcher
      // cannot order, so it is said out loud rather than sent as `{}`.
      setState(() {
        _captureError =
            '${missingPose.length} photograph(s) have no recorded capture '
            'pose, so this station cannot be uploaded. Recalibrate and '
            'recapture them.';
      });
      return;
    }
    _uploadRequested = true;
    bloc.add(
      VirtualTourUploadRequested(
        tourId: _tourId,
        propertyId:
            widget.propertyId ?? VirtualTourApiConfig.propertyIdFromEnvironment(),
        stationId: widget.stationId,
        captures: List<TourCapture>.unmodifiable(captures),
      ),
    );
  }

  void _finalise() => _bloc?.add(
    VirtualTourFinaliseRequested(tourId: _tourId),
  );

  void _refreshStatus() => _bloc?.add(
    VirtualTourRefreshRequested(tourId: _tourId),
  );

  void _retry() => _bloc?.add(const VirtualTourRetryRequested());

  void _close() => Navigator.of(context).maybePop();

  ArTargetVisualState _hudState(VirtualTourGuidance? guidance) {
    if (guidance == null) return ArTargetVisualState.idle;
    if (guidance.complete) return ArTargetVisualState.done;
    if (!guidance.tracking) {
      return ArTargetVisualState.failed;
    }
    return switch (guidance.state) {
      VirtualTourCaptureState.captureFailed ||
      VirtualTourCaptureState.awaitingTrackingRecovery ||
      VirtualTourCaptureState.trackingUnavailable => ArTargetVisualState.failed,
      VirtualTourCaptureState.captureSucceeded ||
      VirtualTourCaptureState.completed => ArTargetVisualState.done,
      VirtualTourCaptureState.waitingForStability ||
      VirtualTourCaptureState.capturing ||
      VirtualTourCaptureState.validatingImage ||
      VirtualTourCaptureState.savingImage => ArTargetVisualState.capturing,
      VirtualTourCaptureState.readyForAlignment ||
      VirtualTourCaptureState.aligning => ArTargetVisualState.idle,
    };
  }

  Color _hudColor(VirtualTourGuidance? guidance) {
    if (guidance == null) return Colors.white;
    if (guidance.complete ||
        guidance.state == VirtualTourCaptureState.captureSucceeded) {
      return CaptureColors.ok;
    }
    if (!guidance.tracking ||
        guidance.state == VirtualTourCaptureState.captureFailed ||
        guidance.state == VirtualTourCaptureState.awaitingTrackingRecovery ||
        guidance.state == VirtualTourCaptureState.trackingUnavailable) {
      return CaptureColors.warn;
    }
    return guidance.aligned ? CaptureColors.ok : Colors.white;
  }

  @override
  void dispose() {
    _disposed = true;
    WidgetsBinding.instance.removeObserver(this);
    _watchdog?.cancel();
    _events?.cancel();
    unawaited(_host?.stop() ?? Future<void>.value());
    _bloc?.close();
    final remote = _ownedRemote;
    if (remote != null) unawaited(remote.dispose());
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_bootError != null) {
      return _BootFailure(
        message: _bootError!,
        permissionDenied: _permissionDenied,
        onClose: _close,
      );
    }
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        _viewport = Size(constraints.maxWidth, constraints.maxHeight);
        if (!_booted && !_bootScheduled) {
          _bootScheduled = true;
          final size = _viewport;
          WidgetsBinding.instance.addPostFrameCallback((_) => _boot(size));
        }
        if (_textureId == null || _bloc == null) {
          if (_recoveryRequired) {
            // Recalibration released the old texture and the new session could
            // not be started. A spinner here would never end, so the failure is
            // shown with the one action that can help.
            return _BootFailure(
              message: _arFault ?? 'The AR station could not be rebuilt.',
              permissionDenied: false,
              onClose: _close,
              onRetry: _recalibrating ? null : _recalibrate,
            );
          }
          return const Scaffold(
            backgroundColor: Colors.black,
            body: Center(child: CircularProgressIndicator()),
          );
        }
        return _buildCamera(context);
      },
    );
  }

  Widget _buildCamera(BuildContext context) {
    final guidance = _guidance;
    final hudColor = _hudColor(guidance);
    final topPadding = MediaQuery.paddingOf(context).top;
    final captured = _engine?.capturedCount ?? guidance?.capturedCount ?? 0;
    final total = _engine?.totalCount ?? VirtualTourLayout.nodeCount;

    return PopScope(
      canPop: true,
      child: Scaffold(
        backgroundColor: Colors.black,
        body: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            Texture(textureId: _textureId!, filterQuality: FilterQuality.none),
            VirtualTourOverlay(
              triangles: _triangles,
              revision: _revision,
              modelsReady: _modelsReady,
              failureReason: _modelFailure,
              hudColor: hudColor,
              holdProgress: guidance?.holdProgress ?? 0,
              topInset: topPadding + 64,
            ),
            Positioned(
              left: 12,
              right: 12,
              top: topPadding + 8,
              child: _TopBar(
                captured: captured,
                total: total,
                tracking: _tracking,
                onClose: _close,
              ),
            ),
            Positioned(
              left: 12,
              right: 12,
              top: topPadding + 74,
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  if (widget.diagnosticsEnabled)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 8),
                      child: _DiagnosticsPanel(
                        tracking: _tracking,
                        planeCount: _lastPose?.planeCount ?? 0,
                        failureReason: _lastPose?.failureReason,
                        guidance: guidance,
                        cameraPosition: _lastPose?.position,
                        stationPosition: _engine?.startPosition,
                        poseAge: _now() - _lastPoseAt,
                        savedCount: _photoPaths.length,
                        latestError:
                            _captureError ??
                            _engine?.lastCaptureError ??
                            _arFault,
                        errorCategory: _lastErrorCategory,
                        errorFrame: _lastErrorFrame,
                      ),
                    ),
                  if (_recoveryRequired)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 8),
                      child: _RecoveryBanner(
                        message:
                            _arFault ??
                            'AR must be restarted before capture can continue.',
                        onRecalibrate: _recalibrate,
                      ),
                    )
                  else if (_arFault != null)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 8),
                      child: _ArFaultBanner(message: _arFault!),
                    ),
                ],
              ),
            ),
            Align(
              alignment: Alignment.bottomCenter,
              child: _HudPanel(
                guidance: guidance,
                tracking: _tracking,
                captured: captured,
                total: total,
                capturing: _activeAttempt != null,
                captureError: _captureError,
                hudColor: hudColor,
                modelsFailed: _modelFailure != null,
                onManualRetry: _manualRetry,
              ),
            ),
            BlocBuilder<VirtualTourBloc, VirtualTourState>(
              bloc: _bloc!,
              builder: (BuildContext context, VirtualTourState state) =>
                  _UploadOverlay(
                    state: state,
                    onRetry: _retry,
                    onFinalise: _finalise,
                    onRefresh: _refreshStatus,
                    onDone: _close,
                  ),
            ),
          ],
        ),
      ),
    );
  }
}

/// What is left to do, at a glance: how many are saved and whether AR is
/// tracking.
class _TopBar extends StatelessWidget {
  const _TopBar({
    required this.captured,
    required this.total,
    required this.tracking,
    required this.onClose,
  });

  final int captured;
  final int total;
  final TrackingState tracking;
  final VoidCallback onClose;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: <Widget>[
        _RoundButton(icon: Icons.close, label: 'Close', onTap: onClose),
        const SizedBox(width: 10),
        Expanded(
          child: Container(
            padding: const EdgeInsets.fromLTRB(12, 8, 12, 8),
            decoration: BoxDecoration(
              color: CaptureColors.chromeOverlay(0.9),
              borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
              border: Border.all(color: CaptureColors.hairlineOnDark),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Text(
                  '$captured / $total captured',
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 13,
                    fontWeight: FontWeight.w600,
                  ),
                ),
                Text(
                  '30 world-locked nodes · ${_trackingLabel(tracking)}',
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    color: CaptureColors.mutedOnDark,
                    fontSize: 10,
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

class _RoundButton extends StatelessWidget {
  const _RoundButton({
    required this.icon,
    required this.label,
    required this.onTap,
  });

  final IconData icon;
  final String label;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) => Semantics(
    button: true,
    label: label,
    child: Material(
      color: CaptureColors.chromeOverlay(0.9),
      shape: const CircleBorder(),
      child: InkWell(
        customBorder: const CircleBorder(),
        onTap: onTap,
        child: SizedBox.square(
          dimension: CaptureMetrics.headerIconSize + 8,
          child: Icon(icon, color: Colors.white, size: 20),
        ),
      ),
    ),
  );
}

/// The instruction panel. It never hides *why* the shutter is blocked.
class _HudPanel extends StatelessWidget {
  const _HudPanel({
    required this.guidance,
    required this.tracking,
    required this.captured,
    required this.total,
    required this.capturing,
    required this.captureError,
    required this.hudColor,
    required this.modelsFailed,
    required this.onManualRetry,
  });

  final VirtualTourGuidance? guidance;
  final TrackingState tracking;
  final int captured;
  final int total;
  final bool capturing;
  final String? captureError;
  final Color hudColor;
  final bool modelsFailed;
  final VoidCallback onManualRetry;

  @override
  Widget build(BuildContext context) {
    final aligned = guidance?.aligned ?? false;
    final instruction =
        guidance?.instruction ??
        tracking.advice ??
        'Move the phone slowly so AR can find the room.';
    final hold = guidance?.readyToCapture == true
        ? 1.0
        : guidance?.holdProgress ?? 0.0;

    return Container(
      width: double.infinity,
      color: CaptureColors.cameraChrome,
      child: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(18, 12, 18, 14),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              ClipRRect(
                borderRadius: BorderRadius.circular(2),
                child: LinearProgressIndicator(
                  value: total == 0 ? 0 : captured / total,
                  minHeight: 3,
                  backgroundColor: CaptureColors.trackOnDark,
                  valueColor: const AlwaysStoppedAnimation<Color>(
                    CaptureColors.ok,
                  ),
                ),
              ),
              const SizedBox(height: 10),
              Row(
                children: <Widget>[
                  _PhaseChip(
                    label: _phaseLabel(guidance, capturing, tracking),
                    color: hudColor,
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      guidance?.target?.label ?? 'All targets captured',
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(
                        color: CaptureColors.mutedOnDark,
                        fontSize: 11,
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Text(
                instruction,
                maxLines: 3,
                overflow: TextOverflow.ellipsis,
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 17,
                  height: 1.25,
                  fontWeight: FontWeight.w600,
                ),
              ),
              const SizedBox(height: 10),
              ClipRRect(
                borderRadius: BorderRadius.circular(3),
                child: LinearProgressIndicator(
                  value: hold,
                  minHeight: 6,
                  backgroundColor: CaptureColors.trackOnDark,
                  valueColor: AlwaysStoppedAnimation<Color>(
                    aligned ? CaptureColors.ok : CaptureColors.mutedOnDark,
                  ),
                ),
              ),
              const SizedBox(height: 8),
              Row(
                children: <Widget>[
                  Expanded(
                    child: Text(
                      'Automatic capture inside 3.0° · hold 0.60 s · '
                      'still ≤ 3°/s',
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(
                        color: CaptureColors.mutedOnDark,
                        fontSize: 10,
                      ),
                    ),
                  ),
                ],
              ),
              if (modelsFailed) ...<Widget>[
                const SizedBox(height: 6),
                const Text(
                  'The GLB markers could not be drawn — the drawn reticle is '
                  'standing in for them. Capture is still possible.',
                  style: TextStyle(
                    color: CaptureColors.warn,
                    fontSize: 10,
                    height: 1.3,
                  ),
                ),
              ],
              if (captureError != null) ...<Widget>[
                const SizedBox(height: 6),
                Text(
                  captureError!,
                  maxLines: 3,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    color: CaptureColors.warn,
                    fontSize: 11,
                    height: 1.3,
                  ),
                ),
              ],
              if (guidance?.requiresManualRetry == true) ...<Widget>[
                const SizedBox(height: 8),
                Align(
                  alignment: Alignment.centerLeft,
                  child: OutlinedButton.icon(
                    onPressed: capturing ? null : onManualRetry,
                    icon: const Icon(Icons.refresh, size: 16),
                    label: const Text('Retry this target'),
                    style: OutlinedButton.styleFrom(
                      foregroundColor: Colors.white,
                      side: const BorderSide(
                        color: CaptureColors.hairlineOnDark,
                      ),
                    ),
                  ),
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }
}

class _DiagnosticsPanel extends StatelessWidget {
  const _DiagnosticsPanel({
    required this.tracking,
    required this.planeCount,
    required this.failureReason,
    required this.guidance,
    required this.cameraPosition,
    required this.stationPosition,
    required this.poseAge,
    required this.savedCount,
    required this.latestError,
    required this.errorCategory,
    required this.errorFrame,
  });

  final TrackingState tracking;
  final int planeCount;
  final String? failureReason;
  final VirtualTourGuidance? guidance;
  final Vec3? cameraPosition;

  /// The world point the station's 30 nodes were computed from. Every
  /// diagnostic angle and distance is relative to it.
  final Vec3? stationPosition;
  final Duration poseAge;
  final int savedCount;
  final String? latestError;
  final String? errorCategory;
  final String? errorFrame;

  static String _f(double value) => value.toStringAsFixed(2);
  static String _xyz(Vec3? v) =>
      v == null ? '—' : '${_f(v.x)}, ${_f(v.y)}, ${_f(v.z)}';

  @override
  Widget build(BuildContext context) {
    final g = guidance;
    final attempt = g?.attempt;
    final rate = g?.angularRateDegPerSec;
    final window = g;
    final windowRot = g?.windowOrientationDriftDeg;
    final windowPos = g?.windowPositionDriftM;
    return Container(
      padding: const EdgeInsets.fromLTRB(10, 8, 10, 8),
      decoration: BoxDecoration(
        color: CaptureColors.chromeOverlay(0.88),
        borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
        border: Border.all(color: CaptureColors.hairlineOnDark),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Row(
            children: <Widget>[
              const Text(
                'DIAGNOSTICS · 30-NODE MODE',
                style: TextStyle(
                  color: CaptureColors.mutedOnDark,
                  fontSize: 9,
                  letterSpacing: 0.6,
                ),
              ),
              const Spacer(),
              Text(
                g?.state.name ?? 'no guidance',
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 9,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
          const SizedBox(height: 5),
          Wrap(
            spacing: 12,
            runSpacing: 3,
            children: <Widget>[
              _fact(
                'tracking',
                '${tracking.name}${failureReason == null ? '' : ' · $failureReason'}',
              ),
              _fact('planes', '$planeCount (diagnostic only)'),
              _fact('target', g?.target?.id ?? '—'),
              _fact('target xyz', _xyz(g?.target?.worldPosition)),
              _fact('camera xyz', _xyz(cameraPosition)),
              _fact('station xyz', _xyz(stationPosition)),
              _fact('station offset', g == null ? '—' : '${_f(g.driftM)} m'),
              _fact(
                'align',
                g == null || !g.alignmentDeg.isFinite
                    ? '—'
                    : '${_f(g.alignmentDeg)}° / 3.0°',
              ),
              _fact('rate', rate == null ? '—' : '${_f(rate)} / 3.0 °/s'),
              _fact(
                'window',
                window == null
                    ? '—'
                    : '${(window.holdProgress * 100).round()}% · '
                          '${(window.stabilityHeld.inMilliseconds / 1000).toStringAsFixed(2)} / 0.60 s',
              ),
              _fact('pose age', '${poseAge.inMilliseconds} / 150 ms'),
              _fact(
                'window rot',
                windowRot == null ? '—' : '${_f(windowRot)}° / 1.0°',
              ),
              _fact(
                'window pos',
                windowPos == null
                    ? '—'
                    : '${(windowPos * 100).toStringAsFixed(1)} cm / 2.0 cm',
              ),
              _fact('saved', '$savedCount / ${g?.totalCount ?? 30}'),
              _fact(
                'attempt',
                attempt == null
                    ? '—'
                    : '#${attempt.id} · try ${attempt.attemptNumber} · '
                          '${attempt.state.name}',
              ),
              _fact(
                'auto retries',
                g == null ? '—' : '${g.automaticRetriesRemaining}',
              ),
              _fact('error', latestError ?? '—'),
              _fact('error cat', errorCategory ?? '—'),
            ],
          ),
          if (errorFrame != null) ...<Widget>[
            const SizedBox(height: 3),
            Text(
              'frame  $errorFrame',
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(
                color: CaptureColors.mutedOnDark,
                fontSize: 9,
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _fact(String label, String value) => RichText(
    text: TextSpan(
      children: <InlineSpan>[
        TextSpan(
          text: '$label  ',
          style: const TextStyle(color: CaptureColors.mutedOnDark, fontSize: 9),
        ),
        TextSpan(
          text: value,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 9,
            fontFeatures: <FontFeature>[FontFeature.tabularFigures()],
          ),
        ),
      ],
    ),
  );
}

class _PhaseChip extends StatelessWidget {
  const _PhaseChip({required this.label, required this.color});

  final String label;
  final Color color;

  @override
  Widget build(BuildContext context) => Container(
    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
    decoration: BoxDecoration(
      color: color.withValues(alpha: 0.16),
      borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
      border: Border.all(color: color.withValues(alpha: 0.55)),
    ),
    child: Text(
      label,
      style: TextStyle(
        color: color,
        fontSize: 10,
        fontWeight: FontWeight.w600,
        letterSpacing: 0.3,
      ),
    ),
  );
}

class _RecoveryBanner extends StatelessWidget {
  const _RecoveryBanner({required this.message, required this.onRecalibrate});

  final String message;
  final VoidCallback onRecalibrate;

  @override
  Widget build(BuildContext context) => Container(
    padding: const EdgeInsets.fromLTRB(12, 8, 8, 8),
    decoration: BoxDecoration(
      color: CaptureColors.chromeOverlay(0.94),
      borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
      border: Border.all(color: CaptureColors.danger),
    ),
    child: Row(
      children: <Widget>[
        const Icon(Icons.restart_alt, color: CaptureColors.danger, size: 18),
        const SizedBox(width: 8),
        Expanded(
          child: Text(
            message,
            maxLines: 3,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 11,
              height: 1.35,
            ),
          ),
        ),
        const SizedBox(width: 8),
        TextButton(
          onPressed: onRecalibrate,
          style: TextButton.styleFrom(
            foregroundColor: Colors.white,
            backgroundColor: CaptureColors.danger.withValues(alpha: 0.22),
          ),
          child: const Text('Recalibrate', style: TextStyle(fontSize: 11)),
        ),
      ],
    ),
  );
}

class _ArFaultBanner extends StatelessWidget {
  const _ArFaultBanner({required this.message});

  final String message;

  @override
  Widget build(BuildContext context) => Container(
    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
    decoration: BoxDecoration(
      color: CaptureColors.chromeOverlay(0.94),
      borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
      border: Border.all(color: CaptureColors.danger),
    ),
    child: Row(
      children: <Widget>[
        const Icon(Icons.error_outline, color: CaptureColors.danger, size: 18),
        const SizedBox(width: 8),
        Expanded(
          child: Text(
            message,
            maxLines: 3,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 11,
              height: 1.35,
            ),
          ),
        ),
      ],
    ),
  );
}

/// The upload / finalise / status overlay, driven only by the Bloc's state.
///
/// It deliberately has nothing to show for a finished panorama: the honest
/// furthest point is [VirtualTourAwaitingStitching], which reports the
/// backend's status word and the stitch manifest key it read.
class _UploadOverlay extends StatelessWidget {
  const _UploadOverlay({
    required this.state,
    required this.onRetry,
    required this.onFinalise,
    required this.onRefresh,
    required this.onDone,
  });

  final VirtualTourState state;
  final VoidCallback onRetry;
  final VoidCallback onFinalise;
  final VoidCallback onRefresh;
  final VoidCallback onDone;

  @override
  Widget build(BuildContext context) {
    switch (state) {
      case VirtualTourInitial():
        return const SizedBox.shrink();
      case final VirtualTourProcessing processing:
        return _Scrim(
          child: _Panel(
            title: processing.stage.label,
            message: processing.message,
            detail: processing.total > 0
                ? '${processing.uploaded} of ${processing.total} accepted by '
                      'the server'
                : null,
            leading: const SizedBox(
              width: 18,
              height: 18,
              child: CircularProgressIndicator(strokeWidth: 2),
            ),
            body: processing.total > 0
                ? ClipRRect(
                    borderRadius: BorderRadius.circular(2),
                    child: LinearProgressIndicator(
                      value: processing.uploaded / processing.total,
                      minHeight: 4,
                    ),
                  )
                : null,
            actions: const <Widget>[],
          ),
        );
      case final VirtualTourError error:
        final ticket = error.ticket;
        final String? detail;
        if (error.stage == VirtualTourStage.validation) {
          detail = null;
        } else if (error.uploaded > 0) {
          // The failure landed between two acknowledgements, so no ticket
          // exists to quote — these are the acknowledgements that did arrive.
          detail =
              '${error.uploaded} of ${error.total} photographs are already '
              'stored by the server. Sending again is safe: it keeps one '
              'capture per position.';
        } else if (ticket == null) {
          detail = 'The photographs are still on this device; nothing was '
              'deleted.';
        } else {
          detail =
              '${ticket.uploadedCaptureCount} of '
              '${ticket.expectedCaptureCount} captures are already stored by '
              'the server; a retry sends only what is missing.'
              '${ticket.stitchManifestKey == null ? '' : ' The stitch manifest has already been written.'}';
        }
        return _Scrim(
          child: _Panel(
            title: 'The tour is not complete yet',
            message: error.message,
            detail: detail,
            leading: const Icon(
              Icons.error_outline,
              color: CaptureColors.danger,
              size: 20,
            ),
            actions: <Widget>[
              if (error.retryable)
                FilledButton(
                  onPressed: onRetry,
                  child: const Text('Try again'),
                ),
              OutlinedButton(onPressed: onDone, child: const Text('Close')),
            ],
          ),
        );
      case final VirtualTourUploaded uploaded:
        final ticket = uploaded.ticket;
        return _Scrim(
          child: _Panel(
            title: 'Photographs stored',
            message:
                '${ticket.uploadedCaptureCount} of '
                '${ticket.expectedCaptureCount} captures are on the server '
                'for this station.',
            detail:
                'Finalising makes the server check the count and write the '
                'stitch manifest. No panorama exists until that manifest is '
                'processed.',
            leading: const Icon(
              Icons.cloud_done_outlined,
              color: CaptureColors.ok,
              size: 22,
            ),
            body: _ModeNote(simulated: ticket.simulated),
            actions: <Widget>[
              FilledButton(
                onPressed: onFinalise,
                child: const Text('Finalise tour'),
              ),
              OutlinedButton(onPressed: onDone, child: const Text('Later')),
            ],
          ),
        );
      case final VirtualTourAwaitingStitching awaiting:
        final ticket = awaiting.ticket;
        return _Scrim(
          child: _Panel(
            title: ticket.isFailed
                ? 'The backend failed this tour'
                : 'Awaiting stitching',
            message:
                'Status reported by the backend: ${ticket.status}. '
                '${ticket.isFailed ? 'The captures are stored, but the '
                      'manifest could not be processed.'
                    : ticket.stitchManifestKey == null
                    ? 'The captures are validated; the server has not '
                          'reported a stitch manifest yet.'
                    : 'The stitch manifest is written and waiting for a '
                          'stitching worker, which this build does not run.'}',
            leading: Icon(
              ticket.isFailed
                  ? Icons.error_outline
                  : Icons.hourglass_top_outlined,
              color: ticket.isFailed
                  ? CaptureColors.danger
                  : CaptureColors.warn,
              size: 22,
            ),
            body: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                if (ticket.stitchManifestKey != null) ...<Widget>[
                  const SizedBox(height: 10),
                  _MonoBlock(ticket.stitchManifestKey!),
                ],
                _ModeNote(simulated: ticket.simulated),
              ],
            ),
            actions: <Widget>[
              OutlinedButton(
                onPressed: onRefresh,
                child: const Text('Check status'),
              ),
              OutlinedButton(onPressed: onDone, child: const Text('Done')),
            ],
          ),
        );
    }
  }
}

/// Says plainly whether what is on screen came from the server or from the
/// declared local simulation.
class _ModeNote extends StatelessWidget {
  const _ModeNote({required this.simulated});

  final bool simulated;

  @override
  Widget build(BuildContext context) {
    if (!simulated) return const SizedBox.shrink();
    return const Padding(
      padding: EdgeInsets.only(top: 10),
      child: Text(
        'Simulation mode: this session was recorded by the app itself, not by '
        'the backend. Rebuild with --dart-define=VIRTUAL_TOUR_MODE=backend and '
        'a VIRTUAL_TOUR_API_BASE_URL to upload real captures.',
        style: TextStyle(color: CaptureColors.warn, fontSize: 11, height: 1.35),
      ),
    );
  }
}

/// A storage key, selectable so it can be copied into a terminal.
class _MonoBlock extends StatelessWidget {
  const _MonoBlock(this.text);

  final String text;

  @override
  Widget build(BuildContext context) => Container(
    width: double.infinity,
    padding: const EdgeInsets.all(10),
    decoration: BoxDecoration(
      color: CaptureColors.noteOnDark,
      borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
      border: Border.all(color: CaptureColors.hairlineOnDark),
    ),
    child: SelectableText(
      text,
      maxLines: 3,
      style: const TextStyle(
        color: CaptureColors.bulletOnDark,
        fontSize: 11,
        height: 1.35,
      ),
    ),
  );
}

class _Scrim extends StatelessWidget {
  const _Scrim({required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context) => Positioned.fill(
    child: ColoredBox(
      color: CaptureColors.scrim,
      child: SafeArea(
        child: Center(
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(20),
            child: child,
          ),
        ),
      ),
    ),
  );
}

class _Panel extends StatelessWidget {
  const _Panel({
    required this.title,
    required this.message,
    required this.leading,
    required this.actions,
    this.detail,
    this.body,
  });

  final String title;
  final String message;
  final String? detail;
  final Widget leading;
  final Widget? body;
  final List<Widget> actions;

  @override
  Widget build(BuildContext context) => ConstrainedBox(
    constraints: const BoxConstraints(maxWidth: 420),
    child: Container(
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(
        color: CaptureColors.sheet,
        borderRadius: BorderRadius.circular(CaptureMetrics.cornerRadius),
        border: Border.all(color: CaptureColors.hairlineOnDark),
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          Row(
            children: <Widget>[
              leading,
              const SizedBox(width: 10),
              Expanded(
                child: Text(
                  title,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 17,
                    fontWeight: FontWeight.w600,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 10),
          Text(
            message,
            style: const TextStyle(
              color: CaptureColors.bulletOnDark,
              fontSize: 12,
              height: 1.45,
            ),
          ),
          if (detail != null) ...<Widget>[
            const SizedBox(height: 6),
            Text(
              detail!,
              style: const TextStyle(
                color: CaptureColors.mutedOnDark,
                fontSize: 11,
                height: 1.35,
              ),
            ),
          ],
          ?body,
          if (actions.isNotEmpty) ...<Widget>[
            const SizedBox(height: 16),
            for (var i = 0; i < actions.length; i++) ...<Widget>[
              if (i > 0) const SizedBox(height: 8),
              SizedBox(width: double.infinity, child: actions[i]),
            ],
          ],
        ],
      ),
    ),
  );
}

class _BootFailure extends StatelessWidget {
  const _BootFailure({
    required this.message,
    required this.permissionDenied,
    required this.onClose,
    this.onRetry,
  });

  final String message;
  final bool permissionDenied;
  final VoidCallback onClose;

  /// Present when the failure is recoverable — a recalibration that did not
  /// start. Null while a retry is already running.
  final VoidCallback? onRetry;

  @override
  Widget build(BuildContext context) => Scaffold(
    backgroundColor: CaptureColors.cameraChrome,
    body: SafeArea(
      child: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(24),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              const Icon(Icons.threesixty, size: 48, color: Colors.white70),
              const SizedBox(height: 16),
              const Text(
                'The virtual tour could not start',
                textAlign: TextAlign.center,
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 20,
                  fontWeight: FontWeight.w600,
                ),
              ),
              const SizedBox(height: 8),
              Text(
                message,
                textAlign: TextAlign.center,
                style: const TextStyle(
                  color: CaptureColors.mutedOnDark,
                  height: 1.4,
                ),
              ),
              const SizedBox(height: 20),
              if (permissionDenied) ...<Widget>[
                FilledButton(
                  onPressed: () => PermissionService().openSettings(),
                  child: const Text('Open settings'),
                ),
                const SizedBox(height: 8),
              ],
              if (onRetry != null) ...<Widget>[
                FilledButton(
                  onPressed: onRetry,
                  child: const Text('Try again'),
                ),
                const SizedBox(height: 8),
              ],
              OutlinedButton(onPressed: onClose, child: const Text('Back')),
            ],
          ),
        ),
      ),
    ),
  );
}

String _trackingLabel(TrackingState tracking) => switch (tracking) {
  TrackingState.tracking => 'AR tracking OK',
  TrackingState.initialising => 'AR starting',
  TrackingState.paused => 'AR paused',
  TrackingState.stopped => 'AR stopped',
  TrackingState.unavailable => 'AR unavailable',
};

String _phaseLabel(
  VirtualTourGuidance? guidance,
  bool capturing,
  TrackingState tracking,
) {
  if (capturing) return 'CAPTURE';
  if (guidance == null) {
    return tracking == TrackingState.tracking ? 'SETTING UP' : 'STARTING';
  }
  return switch (guidance.state) {
    VirtualTourCaptureState.trackingUnavailable => 'STARTING',
    VirtualTourCaptureState.readyForAlignment => 'ALIGN',
    VirtualTourCaptureState.aligning => 'ALIGN',
    VirtualTourCaptureState.waitingForStability => 'HOLD',
    VirtualTourCaptureState.capturing => 'CAPTURE',
    VirtualTourCaptureState.validatingImage => 'CHECK',
    VirtualTourCaptureState.savingImage => 'SAVE',
    VirtualTourCaptureState.captureSucceeded => 'SAVED',
    VirtualTourCaptureState.captureFailed =>
      guidance.requiresManualRetry ? 'RETRY' : 'FAILED',
    VirtualTourCaptureState.awaitingTrackingRecovery => 'RECOVER',
    VirtualTourCaptureState.completed => 'DONE',
  };
}
