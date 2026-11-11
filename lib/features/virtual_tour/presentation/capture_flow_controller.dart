import 'dart:async';

// `widgets.dart` re-exports `foundation.dart`, so `ChangeNotifier` comes from
// here — one import rather than two.
import 'package:flutter/widgets.dart';
import 'package:wakelock_plus/wakelock_plus.dart';
import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/core/permissions/permission_service.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_bootstrap.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_camera.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/plugin_camera.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/app_capture_paths.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/capture_paths.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/session_repository.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/gyro_rate.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/capture_flow_engine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/flow_state.dart';

/// The one Flutter-aware piece of the capture flow.
///
/// Everything below it — the state machine, the plan, the coverage engine,
/// station-reference diagnostics, the flow engine — is pure Dart and is tested without a binding
/// (see `tool/verify_all.dart` for why that matters in this environment). This
/// class does the four things that genuinely need Flutter:
///
/// * it builds the engine from the platform's AR, camera, gyroscope and storage
///   implementations;
/// * it drives [CaptureFlowEngine.tick] from a timer while the capture screen is
///   up, so the state machine advances at a fixed 20 Hz regardless of the frame
///   rate;
/// * it turns the app-lifecycle callbacks into the flow's interruption and
///   resume, which is how a phone call or an app switch stops losing frames;
/// * it holds the display awake for exactly as long as the ticker runs, so a
///   station cannot be lost to the idle timeout (see [_syncScreenAwake]).
///
/// It is a [ChangeNotifier] rather than a state-management package because the
/// flow has exactly one consumer — the screen stack above it — and the engine
/// already owns all the state. Wrapping that state in a second copy is how the
/// two drift apart.
class CaptureFlowController extends ChangeNotifier {
  final CaptureFlowEngine engine;

  /// The tick interval. 50 ms, matching the reference prototype's
  /// `setInterval(tick, 50)` and the tick the ported tests use, so the timing
  /// constants (the 400 ms settle window, the 250 ms drift poll) mean the same
  /// thing in the app as they do in the suite.
  static const Duration tickInterval = Duration(milliseconds: 50);

  Timer? _ticker;

  /// Guards against overlapping ticks. [CaptureFlowEngine.tick] is async — it
  /// may write a JPEG — and a timer that fires while the previous tick is still
  /// running would advance the machine twice for one 50 ms of real time.
  bool _ticking = false;

  bool _disposed = false;

  /// The last state handed to the platform, so leaving a capture screen and
  /// moving between the static ones does not re-issue `disable` on every tap.
  bool _screenAwake = false;

  CaptureFlowController(this.engine) : _permissions = PermissionService();

  /// The permission gate, injectable so a test can drive the denied paths.
  final PermissionService _permissions;

  /// Builds the engine from the platform implementations and starts the flow.
  ///
  /// [resumeSessionId] resumes an existing session from disk instead of
  /// creating a new one, which is the "save and exit, come back later" path.
  ///
  /// The camera permission is asked for **before** the camera is opened. On
  /// Android the open itself fails when it has been denied, but the failure
  /// arrives as a `CameraException` whose text ("ConnectException"/"camera in
  /// use") does not tell the user they said no three screens ago. Asking first
  /// puts the accurate sentence on screen, and distinguishes "denied" from
  /// "permanently denied, you have to go to Settings".
  static Future<CaptureFlowController> create({
    CaptureFlowSettings settings = const CaptureFlowSettings(),
    String? resumeSessionId,
  }) async {
    final CapturePaths paths = await defaultCapturePaths();
    final repository = CaptureSessionRepository(paths);
    final stillCamera = PluginCaptureCamera();
    final grabber = createArFrameGrabber();

    final engine = CaptureFlowEngine(
      camera: stillCamera,
      // Present on Android, `null` everywhere else. When it is present the AR
      // session owns the camera for the whole station and the stills come
      // through it; when it is not, `beginCapture` closes the session and uses
      // `stillCamera`, warning that the guidance is off.
      arCamera: grabber == null ? null : ArCaptureCamera(grabber),
      poseSource: createArPoseSource(),
      anchorSource: createArStationAnchorSource(),
      gyro: GyroRateSource(),
      repository: repository,
      paths: paths,
      settings: settings,
    );

    final controller = CaptureFlowController(engine);
    final permissionMessage = await controller.askForCamera();
    await engine.start(resumeSessionId: resumeSessionId);
    // `start()` clears and re-decides `fatal`, and on a denied camera it reports
    // the generic open failure. The permission wording is the more useful of the
    // two, so it wins — and it is re-asserted after the start rather than
    // sneaked in before it.
    if (permissionMessage != null) engine.fatal = permissionMessage;
    return controller;
  }

  /// Whether the last permission ask was refused for good.
  bool get permissionPermanentlyDenied => _permissionPermanentlyDenied;
  bool _permissionPermanentlyDenied = false;

  /// Requests the camera permission and records the outcome on the engine.
  ///
  /// Returns the message to show, or `null` when the camera is usable. Never
  /// throws: a denial is a supported state that the intro screen renders, not an
  /// exception to crash the boot.
  Future<String?> askForCamera() async {
    try {
      await _permissions.ensureCapturePermissions();
      _permissionPermanentlyDenied = false;
      engine.fatal = null;
      return null;
    } on CaptureFailure catch (f) {
      _permissionPermanentlyDenied =
          f is PermissionFailure && f.permanentlyDenied;
      engine.fatal = f.message;
      return f.message;
    } catch (e) {
      // A plugin that is not registered at all (a desktop hot reload, an
      // unusual ROM) must not block the flow: `PluginCaptureCamera` reports the
      // real outcome when it opens the sensor.
      _permissionPermanentlyDenied = false;
      engine.fatal = 'The camera permission could not be checked: $e';
      return engine.fatal;
    }
  }

  /// Re-asks for the permission and re-opens the camera.
  ///
  /// The recovery path behind the intro screen's "Try again": granting the
  /// permission after the app started does nothing on its own, because the
  /// camera was already probed and failed.
  Future<String?> retryCamera() async {
    final denied = await askForCamera();
    if (denied != null) return denied;
    final caps = await engine.camera.initialize();
    engine.fatal = caps.ready
        ? null
        : (caps.limitation ?? 'The camera could not be opened.');
    notifyListeners();
    return engine.fatal;
  }

  /// Opens the camera again from inside the flow, without re-asking permission.
  ///
  /// The viewfinder's own recovery. Distinct from [retryCamera] because that one
  /// raises the permission dialog first, which is right on the intro screen and
  /// wrong here: a camera that was closed by "Save and exit" or by the end of a
  /// station needs opening, not a second interrogation of a user who already
  /// said yes.
  Future<String?> reopenCamera() async {
    final caps = await engine.activeCamera.initialize();
    engine.fatal = caps.ready
        ? null
        : (caps.limitation ?? 'The camera could not be opened.');
    refresh();
    return engine.fatal;
  }

  /// Hands the user to the system settings page for this app.
  Future<void> openPermissionSettings() => _permissions.openSettings();

  CaptureScreen get screen => engine.screen;

  // ── ticking ────────────────────────────────────────────────────────────

  /// Starts or stops the ticker to match the current screen.
  ///
  /// Called after every navigation. The ticker only runs on the capture screen:
  /// the other screens have no state machine to advance, and a timer that keeps
  /// running on a static screen is a battery cost with no purpose.
  void syncTicker() {
    if (_disposed) return;
    if (screen == CaptureScreen.arCapture) {
      _ticker ??= Timer.periodic(tickInterval, (_) => _tick());
      _syncScreenAwake(true);
    } else {
      _ticker?.cancel();
      _ticker = null;
      _syncScreenAwake(false);
    }
  }

  /// Holds the display awake only while a station is actually being shot.
  ///
  /// A station is minutes of holding the phone perfectly still without touching
  /// it, which is exactly the pattern the OS idle timeout exists for; a screen
  /// that sleeps takes the preview and the AR session with it, and the user is
  /// left holding a black phone over a session that stopped seconds ago.
  ///
  /// Best-effort by design: the call throws on a platform with no plugin
  /// registered, and losing the wakelock is a battery annoyance, not a reason to
  /// abort a capture. It is therefore fire-and-forget with the error dropped,
  /// and never awaited from a widget teardown.
  void _syncScreenAwake(bool capturing) {
    if (_disposed || _screenAwake == capturing) return;
    _screenAwake = capturing;
    final future = capturing ? WakelockPlus.enable() : WakelockPlus.disable();
    unawaited(future.catchError((Object _) {}));
  }

  Future<void> _tick() async {
    if (_disposed || _ticking) return;
    if (engine.screen != CaptureScreen.arCapture) return;
    _ticking = true;
    try {
      await engine.tick(tickInterval.inMilliseconds / 1000);
    } finally {
      _ticking = false;
    }
    if (_disposed) return;
    // The machine can move the flow on by itself: reaching COMPLETE switches to
    // the review screen, which has to stop the ticker.
    if (engine.screen != CaptureScreen.arCapture) syncTicker();
    notifyListeners();
  }

  // ── navigation ─────────────────────────────────────────────────────────

  void goTo(CaptureScreen s) {
    engine.goTo(s);
    syncTicker();
    notifyListeners();
  }

  void back() {
    engine.back();
    syncTicker();
    notifyListeners();
  }

  /// Rebuilds the listeners without changing the flow. Used after a call that
  /// returns a value the screen shows (a hit-test result, a station record).
  void refresh() {
    if (_disposed) return;
    notifyListeners();
  }

  // ── lifecycle ──────────────────────────────────────────────────────────

  /// The app is going to the background, or a call arrived.
  ///
  /// Every accepted frame is already on disk by the time the machine marks it
  /// done, so nothing is lost here — only the frame being captured at this
  /// instant is dropped, and the next target is offered again on resume.
  void onAppPaused() {
    if (engine.screen != CaptureScreen.arCapture) return;
    engine.onInterruption();
    _ticker?.cancel();
    _ticker = null;
    // Released with the ticker, and re-armed by `syncTicker` when the user
    // presses Resume: a backgrounded app has no business holding the screen on.
    _syncScreenAwake(false);
    notifyListeners();
  }

  /// The app came back to the foreground.
  ///
  /// The flow does **not** resume by itself. The user gets the "Capture paused"
  /// sheet with Resume and Save and exit, because silently restarting a camera
  /// and an AR session while the phone was in a pocket is how a session captures
  /// the inside of a bag.
  void onAppResumed() {
    notifyListeners();
  }

  Future<void> resumeAfterInterruption() async {
    await engine.resumeAfterInterruption();
    syncTicker();
    notifyListeners();
  }

  /// Saves and leaves the capture screen, keeping every frame on disk.
  Future<void> saveAndExit() async {
    engine.onInterruption();
    _ticker?.cancel();
    _ticker = null;
    await engine.endAr();
    await engine.camera.dispose();
    goTo(CaptureScreen.stationSetup);
  }

  @override
  void dispose() {
    // First, because `_syncScreenAwake` refuses to talk to the platform once the
    // controller is disposed — and leaving the screen awake after the flow is
    // gone is the exact leak this guard exists to avoid.
    _syncScreenAwake(false);
    _disposed = true;
    _ticker?.cancel();
    _ticker = null;
    // Best-effort: `dispose` cannot await, and a failed shutdown must not throw
    // out of a widget teardown.
    unawaited(engine.shutdown().catchError((Object _) {}));
    super.dispose();
  }
}

/// Ties the controller's ticker to the widget lifecycle.
///
/// A `StatefulWidget` that owns the controller and calls [syncTicker] on the
/// first frame. Kept as a mixin so both the host screen and any screen that can
/// start the flow use the same wiring.
mixin CaptureFlowTicker<T extends StatefulWidget> on State<T> {
  CaptureFlowController get controller;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted) controller.syncTicker();
    });
  }
}
