import 'dart:math' as math;
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_availability.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_sources.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_target_model.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/motion_rate.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/live_capture_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/station_drift_gate.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/capture_camera.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/capture_paths.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/frame_integrity.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/session_repository.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/station_group.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/environment/room_occlusion.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/environment/station_anchor.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/guidance/hold_check.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/plan/plan_generator.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/plan/commercial_plan.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/quality/decision_rules.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/session/capture_state_machine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/session/capture_guidance.dart';
// The only import that crosses a feature boundary. `stitch/` is the pre-upload
// gate on the Step 0 keep-list, and it reads the same `StationGroup` this engine
// writes, so wiring it here is what gives it a caller. Nothing under `stitch/`
// imports `flow/`, so this is not a cycle.
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/stitch/stitch_validator.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/capture_copy.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/flow_state.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/tour_viewpoint.dart';

/// Everything tunable about the flow, in one place.
///
/// Every field the brief calls provisional is here and is marked provisional,
/// so the on-site calibration work has one file to edit rather than a scatter of
/// literals.
class CaptureFlowSettings {
  final double overlapH;
  final double overlapV;

  /// Provisional quality thresholds, from the ported engine.
  final QualityThresholds thresholds;

  /// Provisional hold-check thresholds.
  final HoldThresholds hold;

  /// Used when the platform reports no camera intrinsics at all.
  ///
  /// **Provisional.** 68° on the sensor's long side, the value the previous
  /// implementation used and the prototype assumes. Every screen that shows a
  /// field of view also says where the number came from, so a provisional value
  /// is never presented as a measurement.
  final double fallbackLongSideFovDeg;

  /// How often the station-reference diagnostic polls the platform for the
  /// camera position and anchor tracking state.
  final Duration driftPollInterval;

  /// The room dimensions implied by the user's answer.
  ///
  /// **A user estimate, not a measurement** — the prototype says as much
  /// ("the app does not measure the room"), and it is only used to convert the
  /// floor plan's normalised coordinates into the metres that
  /// [StationGroup.position] stores.
  final RoomBox Function(RoomSize) roomBoxFor;

  const CaptureFlowSettings({
    this.overlapH = 0.25,
    this.overlapV = 0.25,
    this.thresholds = QualityThresholds.provisional,
    this.hold = HoldThresholds.provisional,
    this.fallbackLongSideFovDeg = 68,
    this.driftPollInterval = const Duration(milliseconds: 250),
    this.roomBoxFor = defaultRoomBoxFor,
  });

  /// The nominal room for each answer. Deliberately coarse.
  static RoomBox defaultRoomBoxFor(RoomSize size) => switch (size) {
    RoomSize.small => const RoomBox(lengthM: 3.0, widthM: 3.0),
    RoomSize.medium => const RoomBox(lengthM: 5.0, widthM: 4.0),
    RoomSize.large => const RoomBox(lengthM: 8.0, widthM: 6.0),
  };

  CaptureFlowSettings copyWith({
    double? overlapH,
    double? overlapV,
    QualityThresholds? thresholds,
    HoldThresholds? hold,
    double? fallbackLongSideFovDeg,
    Duration? driftPollInterval,
  }) => CaptureFlowSettings(
    overlapH: overlapH ?? this.overlapH,
    overlapV: overlapV ?? this.overlapV,
    thresholds: thresholds ?? this.thresholds,
    hold: hold ?? this.hold,
    fallbackLongSideFovDeg:
        fallbackLongSideFovDeg ?? this.fallbackLongSideFovDeg,
    driftPollInterval: driftPollInterval ?? this.driftPollInterval,
    roomBoxFor: roomBoxFor,
  );
}

/// Where the plan's field of view came from. Shown to the user, because a
/// provisional value must never look like a measurement.
enum FovSource {
  /// The platform reported its own projection matrix.
  platformProjection('the platform projection matrix'),

  /// Derived from the sensor intrinsics the platform reported.
  intrinsics('the camera intrinsics'),

  /// Neither was available; a provisional constant was used.
  provisional('a provisional default — the camera reported no intrinsics');

  const FovSource(this.label);
  final String label;
}

/// The outcome of "Set Station Here".
class StationSetupResult {
  final bool ok;
  final String message;

  /// Whether an anchor was actually created.
  final bool anchorPlaced;

  const StationSetupResult({
    required this.ok,
    required this.message,
    this.anchorPlaced = false,
  });
}

/// The capture flow.
///
/// Composes the four layers and holds the state the screens read. **Pure
/// Dart**: it takes a camera, a pose source, an anchor source and a repository,
/// and it never imports Flutter. That is what lets the whole live path —
/// station setup, the anchor diagnostic, the state machine, and the frames
/// reaching disk — be tested with fakes, in this repository, without a device.
class CaptureFlowEngine {
  final CaptureCamera camera;

  /// The camera the AR session provides, when there is one.
  ///
  /// ARCore holds the camera device exclusively, so while the session is up this
  /// is the only camera that can produce a frame. See `camera/ar_camera.dart`
  /// for why the session spans the whole station and what that costs.
  ///
  /// `null` on a device without AR, and in the simulated flows: there the
  /// session is not running and [camera] owns the sensor, exactly as before.
  final CaptureCamera? arCamera;

  final ArPoseSource poseSource;
  final ArStationAnchorSource anchorSource;
  final MotionRateSource gyro;
  final CaptureSessionRepository repository;
  final CapturePaths paths;
  CaptureFlowSettings settings;

  CaptureFlowEngine({
    required this.camera,
    required this.poseSource,
    required this.anchorSource,
    required this.gyro,
    required this.repository,
    required this.paths,
    this.arCamera,
    this.settings = const CaptureFlowSettings(),
  });

  // ── flow state ─────────────────────────────────────────────────────────

  CaptureScreen screen = CaptureScreen.intro;
  RoomSize roomSize = RoomSize.medium;
  Obstruction obstruction = Obstruction.open;

  final List<StationRecord> stations = <StationRecord>[];
  int currentStation = 0;
  int selectedCandidate = 0;

  /// The on-disk session manifest, once [start] has created it.
  SessionManifest? manifest;

  String get sessionId => manifest?.sessionId ?? '';

  /// The current station's on-disk group. Held so frames append to the same
  /// manifest the state machine is filling.
  StationGroup? currentGroup;

  /// AR tier, decided by the pure ladder in `ar/ar_availability.dart`.
  ArTierDecision arTier = const ArTierDecision(
    tier: ArTier.unavailable,
    headline: 'Checking this device…',
    detail: 'AR support has not been checked yet.',
    guidanceEnabled: false,
    sensorFallback: false,
  );

  ArCapabilities arCapabilities = ArCapabilities.none;

  FovSource fovSource = FovSource.provisional;
  double planHfovDeg = 68;
  double planVfovDeg = 53.669;

  String status = '';
  String? warning;
  String? fatal;

  // ── live state ─────────────────────────────────────────────────────────

  CaptureSession? session;

  /// The prototype's 6-state capture guidance (init/search/align/cap/done/fail).
  ///
  /// Drives the orb colour, the instruction line and the capture progress ring
  /// on the AR capture screen. It is a pure visual mirror of the live capture:
  /// the session still owns the real photo and its persistence.
  CaptureGuidance guidance = CaptureGuidance();

  /// Whether the current target is outside the view frustum, reported by the
  /// capture screen from its projection.
  bool _outOfView = false;

  /// When set, the next captured frame is marked as failed (the prototype's
  /// `ctl('fail')` simulation toggle). Cleared once consumed.
  bool _forceFail = false;

  /// Index of the frame that was forced to fail, so [retryFailed] re-shoots
  /// exactly that view rather than the next one.
  int? _lastFailIndex;

  /// The native ARCore glTF renderer, when the device provides one.
  ///
  /// Null in this repository — the renderer is the one native piece that is not
  /// here (see `ar/ar_target_model.dart`); when set, the flow drives it every
  /// tick with [arTargetRenderState] so it draws the prototype's target + bezel
  /// over the camera feed.
  ArTargetRenderer? arTargetRenderer;

  /// The visual state the target renderer should apply this frame, derived from
  /// the 6-state guidance and the out-of-view flag (the prototype's `glUpd`
  /// inputs: bezel colour, check visibility, 0.75 opacity when out of view).
  ArTargetRenderState get arTargetRenderState {
    final visual = switch (guidance.state) {
      CaptureGuidanceState.init ||
      CaptureGuidanceState.search => ArTargetVisualState.idle,
      CaptureGuidanceState.align => ArTargetVisualState.aligned,
      CaptureGuidanceState.cap => ArTargetVisualState.capturing,
      CaptureGuidanceState.done => ArTargetVisualState.done,
      CaptureGuidanceState.fail => ArTargetVisualState.failed,
    };
    return ArTargetRenderState(state: visual, outOfView: _outOfView);
  }

  /// Called by the capture screen each frame with the target's on-screen state.
  void setOutOfView(bool out) => _outOfView = out;

  /// Requests the next captured frame to fail (simulation toggle).
  void simulateFail() => _forceFail = true;

  /// Retries the frame that failed (the prototype's `retry()`).
  ///
  /// Re-runs the guidance and restarts capture from the failed frame, so the
  /// amber/skipped frame is replaced by a real one.
  void retryFailed() {
    if (!guidance.isFail) return;
    final idx = _lastFailIndex ?? session?.currentTargetIndex ?? 0;
    guidance.retry();
    beginCapture(fromTarget: idx);
  }

  /// The AR-independent motion rate.
  double? get angularRate => gyro.rateDegPerSec;

  /// The camera that owns the sensor right now.
  ///
  /// While the AR session is up it owns the camera — ARCore does not share the
  /// device — so stills have to come through it. Otherwise the still camera
  /// does. Every frame the flow writes goes through this getter, so there is one
  /// answer to "who can take a photo" rather than one per call site.
  CaptureCamera get activeCamera =>
      arRunning && arCamera != null ? arCamera! : camera;

  StationOrigin? stationOrigin;
  StationDriftGate driftGate = StationDriftGate.disabled();
  HoldAssessment? hold;

  PlaneHit? lastHit;
  bool planeFound = false;
  bool arRunning = false;

  /// Guidance is running on the motion sensors, with no AR session.
  ///
  /// Set by [beginStationSetup] when the pose source says it cannot do AR but
  /// the tier ladder still allows guidance. It changes three things, and only
  /// three: the still camera is **not** disposed (nothing else holds the
  /// sensor), the station is locked by heading instead of by a floor hit-test,
  /// and station displacement is unavailable. Everything else — `ALIGN`, `SETTLE`, the
  /// shutter, the frame validation — reads the same live pose either way.
  bool poseOnlyGuidance = false;
  bool interrupted = false;
  bool trackingLost = false;
  int persistedFrames = 0;

  Vec3? _lastCameraPosition;
  DateTime? _lastDriftPoll;
  String? _anchorId;
  TrackingState _anchorTracking = TrackingState.tracking;
  double? _lockYawDeg;

  /// The camera height above the floor at the moment the station was locked.
  double? lockedCameraHeightM;

  // ── lifecycle ──────────────────────────────────────────────────────────

  /// Probes AR, decides the tier, opens the camera and creates the session.
  ///
  /// Everything here is allowed to fail. A failure produces a tier and a
  /// message, never an exception, because a device without AR is a supported
  /// configuration rather than an error.
  Future<void> start({String? resumeSessionId}) async {
    fatal = null;
    warning = null;

    final caps = await poseSource.start();
    arCapabilities = caps;
    // The probe above opens a session on the ARCore path. Close it again: the
    // station-setup screen is where AR is actually needed, and holding the
    // camera from app start would block the capture camera later.
    await poseSource.stop();
    arRunning = false;

    final fov = resolveFov(caps, settings);
    planHfovDeg = fov.hfovDeg;
    planVfovDeg = fov.vfovDeg;
    fovSource = fov.source;

    final camCaps = await camera.initialize();
    if (!camCaps.ready) {
      fatal = camCaps.limitation ?? 'No usable camera on this device.';
    }
    gyro.start();

    // The gyroscope is the non-AR pose fallback. Whether it actually reports is
    // only known once it has had a chance to, so the tier is re-decided after
    // the first samples rather than guessed at here.
    arTier = decideArTier(ar: caps, poseSensorsAvailable: !gyro.hasFailed);

    if (resumeSessionId != null) {
      await resumeInto(resumeSessionId);
      // `resumeInto` just wrote "Resuming: N frames already saved", which is the
      // one sentence that tells the user a restart did not lose their work. The
      // screens read the tier headline off `arTier` themselves, so overwriting it
      // here only ever deleted the resume line.
    } else {
      await createNewSession();
      status = arTier.headline;
    }
  }

  Future<void> createNewSession() async {
    final id = 'session_${DateTime.now().millisecondsSinceEpoch}';
    manifest = await repository.create(
      sessionId: id,
      room: settings.roomBoxFor(roomSize),
      planParams: _planParams(),
    );
    stations.clear();
    currentStation = 0;
    currentGroup = null;
  }

  PlanParams _planParams() => PlanParams(
    hfovDeg: planHfovDeg,
    vfovDeg: planVfovDeg,
    overlapH: settings.overlapH,
    overlapV: settings.overlapV,
  );

  /// Stops everything. Called from `dispose`.
  Future<void> shutdown() async {
    await poseSource.stop();
    await gyro.stop();
    await arCamera?.dispose();
    await camera.dispose();
  }

  // ── navigation ─────────────────────────────────────────────────────────

  void goTo(CaptureScreen s) {
    screen = s;
    status = '';
  }

  void back() {
    screen = switch (screen) {
      CaptureScreen.intro => CaptureScreen.intro,
      CaptureScreen.roomDetails => CaptureScreen.intro,
      CaptureScreen.plan => CaptureScreen.roomDetails,
      CaptureScreen.stationSetup => CaptureScreen.plan,
      // The capture screen has no back: leaving it mid-station without saving
      // would discard the frames the machine has already validated.
      CaptureScreen.arCapture => CaptureScreen.arCapture,
      CaptureScreen.progressMap => CaptureScreen.arCapture,
      CaptureScreen.stationReview => CaptureScreen.addStation,
      CaptureScreen.addStation => CaptureScreen.stationReview,
      CaptureScreen.tourPreview => CaptureScreen.addStation,
    };
  }

  PlanRecommendation get recommendation => recommendPlan(roomSize, obstruction);

  List<StationCandidate> get candidates => stationCandidates(obstruction);

  ({double x, double y}) get firstPosition => firstStationPosition(obstruction);

  /// The field of view the plan is using, and where it came from.
  ///
  /// Order matters: the platform's own projection matrix already accounts for
  /// the display rotation and the viewport, so it wins; the sensor intrinsics
  /// are next best; the constant is last and is labelled as such everywhere it
  /// surfaces. The brief forbids reading the FOV from a constant, and this is
  /// how that is honoured — the constant is only reached when the platform
  /// reported nothing at all, and then the UI says so.
  static ({double hfovDeg, double vfovDeg, FovSource source}) resolveFov(
    ArCapabilities caps,
    CaptureFlowSettings settings,
  ) {
    final platform = caps.displayFov;
    if (platform != null && platform.hfovDeg > 1 && platform.vfovDeg > 1) {
      return (
        hfovDeg: platform.hfovDeg,
        vfovDeg: platform.vfovDeg,
        source: FovSource.platformProjection,
      );
    }
    final k = caps.intrinsics;
    if (k != null && k.isUsable) {
      final d = k.portraitDisplayFov();
      return (
        hfovDeg: d.hfovDeg,
        vfovDeg: d.vfovDeg,
        source: FovSource.intrinsics,
      );
    }
    // Provisional: a 4:3 sensor whose long side sees `fallbackLongSideFovDeg`,
    // held in portrait. The short side is derived rather than being a second
    // constant, so the two cannot disagree.
    final long = settings.fallbackLongSideFovDeg;
    final short =
        2 * math.atan(math.tan(long * math.pi / 360) * 3 / 4) * 180 / math.pi;
    return (hfovDeg: short, vfovDeg: long, source: FovSource.provisional);
  }

  // ── station setup ──────────────────────────────────────────────────────

  /// Starts the pose source for the station calibration.
  ///
  /// Returns true when *something* can guide the user, which is not the same as
  /// AR being up: with no ARCore but live motion sensors, this opens the sensor
  /// path, leaves the still camera holding the sensor, and [poseOnlyGuidance]
  /// says which of the two happened.
  ///
  /// Returns false only when neither can — no AR and no usable pose. The setup
  /// screen then stays on the plan view, and the tier's own wording explains why
  /// the station cannot be anchored.
  Future<bool> beginStationSetup() async {
    if (!arTier.tier.canCapture) {
      status = arTier.limitation ?? 'Guided capture is unavailable.';
      return false;
    }
    if (!arTier.guidanceEnabled) {
      status = 'AR is unavailable, so this station is not anchored.';
      return false;
    }

    final caps = await poseSource.start();
    arCapabilities = caps;
    lastHit = null;
    planeFound = false;

    if (!caps.supported) {
      // There is no AR session. The ladder then decides whether the motion
      // sensors are enough to guide by — and if they are, the station is locked
      // by heading and anchored to nothing, which is the labelled sensor path
      // rather than a claim of AR tracking.
      arRunning = false;
      final sensorsLive = poseSource.tracking != TrackingState.unavailable;
      arTier = decideArTier(
        ar: caps,
        poseSensorsAvailable: sensorsLive && !gyro.hasFailed,
      );
      warning = caps.reason;
      if (!arTier.guidanceEnabled) return false;
      // The still camera keeps the sensor. The hand-over below belongs to the AR
      // path only: disposing the camera here would blank the preview for a
      // session that was never opened.
      poseOnlyGuidance = true;
      // … but it still has to be *open*. This branch is the only route to the
      // calibration view that never touches [beginCapture], and the camera is
      // closed by "Save and exit" and at the end of every station, so a second
      // station would otherwise aim at a black box. Re-opening costs one
      // `initialize()`, which is exactly what the AR path's hand-back also does.
      if (!camera.isRunning) {
        final camCaps = await camera.initialize();
        if (!camCaps.ready) {
          fatal = camCaps.limitation ?? 'The camera could not be opened.';
        }
      }
      status =
          'No floor anchor: the station locks to the direction you are facing. '
          'Hold the phone still, then set it.';
      return true;
    }

    poseOnlyGuidance = false;

    // The camera handover, part one. ARCore and the still camera cannot hold the
    // camera at the same time, so the still camera is released *before* the AR
    // session opens. It is not reopened until the station finishes, because the
    // session now spans the station — see [setStationHere].
    if (camera.isRunning) await camera.dispose();

    // Part two: the AR session is up, so the stills now come through it. The
    // anchor scan is opened at the same time, because it is the same session.
    if (arCamera != null) await arCamera!.initialize();
    await anchorSource.beginStationScan();

    arRunning = true;
    planeFound = caps.planesTracked;
    status = planeFound
        ? 'Point the centre of the screen at the floor.'
        : 'Move the phone slowly so AR can find the floor.';
    return true;
  }

  /// Whether "Set Station Here" may be used right now.
  ///
  /// Two different gates, and the distinction is the whole point of the field:
  /// with AR it takes a hit-test that actually landed on a horizontal plane,
  /// because the prototype refuses the tap on a miss rather than guessing a
  /// point. Without AR there is no plane to miss, so the gate is the sensor
  /// having a direction to lock — and the screen labels the result as a
  /// heading, not an anchor.
  bool get stationLockReady {
    if (poseOnlyGuidance) return poseSource.latest != null;
    return arRunning && lastHit != null;
  }

  /// Polls the screen-centre hit-test. Call from the setup screen's timer.
  ///
  /// Returns the current hit, or `null`. The caller uses [stationLockReady] to
  /// enable the button: a miss keeps it disabled, which is what the brief asks
  /// for instead of guessing a point.
  Future<PlaneHit?> pollStationHit() async {
    if (poseOnlyGuidance) {
      final p = poseSource.latest;
      // Not `tracking.advice`: that text is written for ARCore ("so AR can find
      // the room"), and this is precisely the branch where there is no AR.
      status = p == null
          ? 'The motion sensors have not produced a direction yet. Move the '
                'phone slowly for a moment.'
          : 'Direction live'
                '${p.angularRateDegPerSec == null ? '' : ', moving '
                          '${p.angularRateDegPerSec!.toStringAsFixed(1)}°/s'}'
                '. Hold still, then set the station.';
      return null;
    }
    if (!arRunning) return null;
    if (!planeFound) {
      planeFound = poseSource.capabilities.planesTracked;
      if (!planeFound) {
        status = 'Move the phone slowly so AR can find the floor.';
        return null;
      }
    }
    final h = await anchorSource.refreshHitTest();
    lastHit = h;
    status = h == null
        ? 'Point the centre of the screen at the floor.'
        : 'Surface found ${h.distanceM.toStringAsFixed(2)} m away.';
    return h;
  }

  /// Places the station anchor.
  ///
  /// **The AR session stays up.** This is the correction to the first draft,
  /// which tore AR down here. ARCore owns the camera exclusively, so closing the
  /// session to free the camera for stills also destroys the pose — and `ALIGN`,
  /// the zenith and nadir checks, station-reference tracking and the tracking-loss recovery
  /// all need a live pose *during* capture. Closing it here left the machine
  /// with nothing to aim by, and it never captured a single frame.
  ///
  /// The session is therefore opened in [beginStationSetup] and closed in
  /// [finishStation], with the stills taken through the AR camera in between.
  Future<StationSetupResult> setStationHere() async {
    final record = _ensureStationRecord();

    final poseOnly = poseOnlyGuidance;
    final hit = lastHit;
    if (!poseOnly) {
      final validation = validateStationHit(hit);
      if (!validation.ok) {
        return StationSetupResult(
          ok: false,
          message:
              validation.reason ?? 'No surface found at the screen centre.',
        );
      }
    }

    final pose = poseSource.latest;
    _lockYawDeg = pose?.captureWorld.yawPitch.yawDeg;
    if (poseOnly && _lockYawDeg == null) {
      // No pose, so no heading to lock. Recording a station with an undefined
      // frame would make every later target direction meaningless.
      return StationSetupResult(
        ok: false,
        message:
            'The motion sensors are not reporting a direction yet. Move the '
            'phone slowly for a moment, then set the station again.',
      );
    }

    // A sensor-only build has no plane to anchor to, so it does not ask for one
    // and does not pretend the null anchor is a failure it worked around.
    final anchor = poseOnly ? null : await anchorSource.createAnchor(hit!);
    final cameraPosition = poseOnly
        ? null
        : await anchorSource.cameraPosition();
    _lastCameraPosition = cameraPosition;

    if (anchor != null && cameraPosition != null) {
      lockedCameraHeightM = cameraHeightAbovePlane(cameraPosition, hit!);
      stationOrigin = StationDriftGate.originFrom(
        anchor: anchor,
        lockYawDeg: _lockYawDeg ?? 0,
        cameraHeightM: lockedCameraHeightM!,
      );
      driftGate = StationDriftGate(origin: stationOrigin);
      _anchorId = anchor.id;
      _anchorTracking = anchor.tracking;
      record.anchorPlaced = true;
    } else {
      // Anchoring failed. The station still works; station displacement simply
      // has no anchor-backed diagnostic reference.
      stationOrigin = null;
      driftGate = StationDriftGate.disabled();
      _anchorId = null;
      // Not measured, so not kept from a previous station either.
      lockedCameraHeightM = null;
      record.anchorPlaced = false;
    }

    // Persist the origin into the station group, so a resume can rebuild the
    // gate without asking ARCore to re-find an anchor it has already released.
    final group = currentGroup;
    if (group != null) {
      group.origin = stationOrigin;
      await repository.frames.saveGroup(group);
    }

    return StationSetupResult(
      ok: true,
      anchorPlaced: record.anchorPlaced,
      message: record.anchorPlaced
          ? 'Station placed. Capture starts now.'
          : poseOnly
          ? 'Station locked to the direction you are facing. There is no floor '
                'anchor, so moving away cannot be detected.'
          : 'Station recorded without an anchor. Drift cannot be measured.',
    );
  }

  /// Ends the AR session. Safe to call when nothing is running.
  ///
  /// Called when a station finishes, when the setup screen is left, and when the
  /// capture is abandoned. Not called between the anchor and the capture, because
  /// the capture needs the pose.
  ///
  /// The AR camera goes with the session, and it is not a detail: ARCore owns the
  /// sensor for as long as that camera is open, so leaving it running hands a
  /// later station a "camera in use" failure and keeps the sensor warm for
  /// nothing. [beginStationSetup] re-probes it, so releasing here costs the next
  /// station an open, not a broken flow.
  Future<void> endAr() async {
    await anchorSource.endStationScan();
    await poseSource.stop();
    await arCamera?.dispose();
    arTargetRenderer?.setVisible(false);
    arRunning = false;
    poseOnlyGuidance = false;
    lastHit = null;
    _anchorId = null;
    _lastCameraPosition = null;
    _lastDriftPoll = null;
  }

  // ── stations ───────────────────────────────────────────────────────────

  /// The plan for a new station: the fixed 26-point commercial grid.
  ///
  /// The prototype specifies 26 views (8 upper / 8 middle / 8 lower / zenith /
  /// nadir) regardless of lens, so this is the plan the station uses and the
  /// one the progress map and review grid are drawn from.
  CapturePlan planForStation() =>
      plan26(hfovDeg: planHfovDeg, vfovDeg: planVfovDeg);

  /// Creates the station's record and its on-disk group.
  ///
  /// The group is what makes a station a separate panorama: one directory, one
  /// manifest, one set of frames. Frames are never merged across stations, which
  /// is the brief's requirement and also the reason the tour viewer links
  /// panoramas rather than pretending they are one sphere.
  Future<StationRecord> createStation({
    required double planX,
    required double planY,
  }) async {
    final m = manifest;
    if (m == null) {
      await createNewSession();
    }
    final activeManifest = manifest!;

    final index = stations.length;
    final plan = planForStation();
    final room = settings.roomBoxFor(roomSize);
    final position = (x: planX * room.lengthM, z: planY * room.widthM);

    final groupIndex = await repository.addStation(
      activeManifest,
      stationId: 'station_$index',
      position: position,
      plannedFrameCount: plan.frames.length,
    );
    currentGroup = await repository.frames.loadGroup(
      activeManifest.sessionId,
      groupIndex,
    );

    final record = StationRecord(
      index: groupIndex,
      roomSize: roomSize,
      obstruction: obstruction,
      plan: plan,
      planX: planX,
      planY: planY,
    );
    stations.add(record);
    currentStation = groupIndex;
    return record;
  }

  StationRecord _ensureStationRecord() {
    if (stations.length > currentStation) return stations[currentStation];
    // Should not happen: the setup screen creates the station before the button
    // is reachable. Kept so a logic error shows up as a wrong position rather
    // than a crash mid-capture.
    final plan = planForStation();
    final pos = firstPosition;
    final record = StationRecord(
      index: stations.length,
      roomSize: roomSize,
      obstruction: obstruction,
      plan: plan,
      planX: pos.x,
      planY: pos.y,
    );
    stations.add(record);
    currentStation = record.index;
    return record;
  }

  StationRecord get currentRecord => stations[currentStation];

  /// The pre-stitch gate for the station being reviewed.
  ///
  /// This answers a different question from [StationRecord.outcome]: `outcome`
  /// says whether the capture *finished*, this says whether the frames it
  /// finished with are plausibly stitchable. A station can be complete and still
  /// fail here — a frame that was never written, or two neighbours too far apart
  /// to share features.
  ///
  /// `null` when there is no group on disk for the station being reviewed. That
  /// is the honest answer rather than `ready`: "nothing is wrong" is a claim
  /// about frames this engine cannot see.
  ///
  /// The index guard matters. `currentGroup` is loaded by [createStation] and by
  /// [resumeInto], and if it ever drifts from [currentStation] the gate would
  /// report one station's problems against another's without any visible sign —
  /// so the mismatch is treated as "unknown" instead.
  StitchReadiness? get reviewPreStitch {
    final group = currentGroup;
    if (group == null || group.index != currentStation) return null;
    return const StitchValidator().checkStation(group);
  }

  /// Reads one frame's photograph and evidence back from disk.
  ///
  /// Used by the review preview, so what the user sees next to the picture is
  /// what the file actually contains rather than what the app remembered.
  Future<FrameEvidence> readFrameEvidence(
    StationRecord record,
    int frameIndex,
  ) async {
    final path = paths.frameImage(sessionId, record.index, frameIndex);
    return FrameEvidence(
      imagePath: path,
      integrity: await repository.frames.integrity.inspect(path),
      meta: await repository.frames.loadFrameMeta(
        sessionId,
        record.index,
        frameIndex,
      ),
    );
  }

  /// Reads every image of [record] back from disk, keyed by plan frame index.
  ///
  /// The review screen offers this as an explicit action because it is real I/O:
  /// a station's worth of header reads, not a pixel decode. A frame accepted
  /// before the phone was plugged into a charger and killed mid-write is found
  /// here, and the target is offered again instead of being reported as done.
  Future<Map<int, FrameIntegrity>> readBackStation(StationRecord record) =>
      repository.frames.inspectFrames(sessionId, record.index);

  /// Where each station's stitched panorama ended up, keyed by station index.
  ///
  /// Empty until the processing service returns something, and that is the
  /// normal state: the capture flow does not upload or stitch. This map is the
  /// seam the app fills in once it has, so the viewer can be opened without
  /// `flow/` ever having to model an upload it does not perform.
  final Map<int, PanoramaRef> panoramas = <int, PanoramaRef>{};

  /// The stops for the 360° viewer, one per accepted station that has a
  /// panorama. Empty until [panoramas] is filled in — check before navigating.
  List<TourViewpoint> get tourViewpoints => buildTourViewpoints(
    acceptedStations,
    panoramaFor: (s) => panoramas[s.index] ?? const PanoramaRef.empty(),
  );

  // ── capture ────────────────────────────────────────────────────────────

  /// Builds the live state machine for the current station and starts ticking.
  ///
  /// [fromTarget] fast-forwards the machine past targets whose frames are
  /// already on disk, so an interruption never makes the user repeat work. It
  /// is only ever called with indices that [FrameStore] confirms are present.
  Future<void> beginCapture({int? fromTarget}) async {
    _ensureStationRecord();

    // Who owns the sensor. With AR up, ARCore does, and the stills come through
    // the AR camera — opening the still camera here would fail with "camera in
    // use" on a real device. Without AR, this is the ordinary camera open.
    if (arRunning && arCamera == null) {
      // AR holds the camera and there is no way to take a still through it. The
      // only honest recovery is to hand the sensor back and lose the pose, and
      // to say so rather than pretend the guidance still works.
      warning =
          'This device cannot take photos while AR is running, so the AR '
          'session was closed. Aim by eye and use the manual button.';
      await endAr();
    }

    final cam = activeCamera;
    // Both flags, because they answer different questions and either alone has
    // been wrong on a device. `capabilities.ready` is a record of the last
    // successful `initialize()`; `isRunning` is what the sensor says right now.
    // Trusting only the record is how a camera closed by "Save and exit" or by
    // the previous station never reopened, and the capture screen then showed a
    // black viewfinder over a camera that could not take a picture.
    if (!cam.capabilities.ready || !cam.isRunning) {
      final caps = await cam.initialize();
      if (!caps.ready) {
        fatal = caps.limitation ?? 'The camera could not be opened.';
        return;
      }
    }
    if (!gyro.hasFailed) gyro.start();

    final source = LiveCaptureSource(
      pose: poseSource,
      // The sensor's current owner, not always the still camera: with AR up the
      // analysis frames come from the AR session's camera image.
      camera: activeCamera,
      stationLockYawDeg: () => _lockYawDeg,
      angularRateProvider: () => gyro.rateDegPerSec,
    );

    final s = CaptureSession(
      SessionConfig(
        plan: _planParams().copyWith(settleMs: 400),
        // The fixed 26-point commercial grid, so the session's targets match
        // the prototype exactly.
        fixedPlan: plan26(hfovDeg: planHfovDeg, vfovDeg: planVfovDeg),
        thresholds: settings.thresholds,
        live: true,
        sensorSource: source,
        // A live capture has no room model, so the simulator's fixtures must
        // not be turned into an occlusion percentage. See SessionConfig.
        roomIsMeasured: false,
        // Metres on the room model's axes, the same conversion
        // [createStation] writes into the group. The floor plan is normalised,
        // and feeding 0…1 where the room model expects metres would put every
        // station in the same corner of a phantom room.
        position: (
          x: currentRecord.planX * settings.roomBoxFor(roomSize).lengthM,
          z: currentRecord.planY * settings.roomBoxFor(roomSize).widthM,
        ),
      ),
    );

    session = s;
    guidance.reset();
    arTargetRenderer?.setVisible(true);
    persistedFrames = 0;
    trackingLost = false;
    interrupted = false;

    if (fromTarget != null && fromTarget > 0) {
      _markTargetsDone(s, fromTarget);
    }
    screen = CaptureScreen.arCapture;
  }

  void _markTargetsDone(CaptureSession s, int upTo) {
    for (var i = 0; i < upTo && i < s.targets.length; i++) {
      s.targets[i].status = TargetStatus.ok;
    }
    if (s.targets.isNotEmpty) {
      s.currentTargetIndex = upTo.clamp(0, s.targets.length - 1);
    }
  }

  /// Advances the machine. Called from the capture screen's ticker.
  ///
  /// This is where the brief's "a frame is never marked done unless it passed
  /// validation" meets the filesystem: a frame is written only after the machine
  /// has moved into ACCEPT, and the station record's status is set only after
  /// the bytes are on disk.
  Future<void> tick(double dt) async {
    final s = session;
    if (s == null) return;
    if (interrupted) return;

    _updateHold();
    await _updateDrift();

    if (driftGate.paused) {
      // The anchor itself is untracked, so its world reference is unreliable.
      // This is independent of physical distance, which remains diagnostic.
      s.pause();
      status = driftGate.banner ?? 'Waiting for the station marker to recover.';
      return;
    }
    if (s.paused) s.resume();

    final acceptedBefore = s.acceptedFrames.length;
    final prevTarget = s.currentTargetIndex;
    s.tick(dt);

    // Drive the prototype's 6-state guidance from the same target and head
    // pose the session is using, so the orb colour, the instruction and the
    // progress ring match the prototype. `targets` is empty for the warm-up
    // ticks (INITIALISE → SURVEY → PLAN), so nothing is read until PLAN has
    // actually built them — otherwise `current` throws on the first tick.
    if (s.targets.isNotEmpty) {
      final head = s.displayPose;
      final target = s.current.frame;
      guidance.update(
        dt,
        headYawDeg: head.yawDeg,
        headPitchDeg: head.pitchDeg,
        targetYawDeg: target.yawDeg,
        targetPitchDeg: target.pitchDeg,
        outOfView: _outOfView,
      );
      arTargetRenderer?.apply(arTargetRenderState);
      // A captured frame advances to the next target, so the guidance
      // re-searches — but a fail holds, so the user sees it and taps Retry
      // before capture continues.
      if (s.currentTargetIndex != prevTarget &&
          guidance.state != CaptureGuidanceState.fail) {
        guidance.reset();
        guidance.state = CaptureGuidanceState.search;
      }
    }

    final live = s.sensorSource;
    if (live is LiveCaptureSource) {
      trackingLost = !live.isGuiding;
    }

    // Persist *before* anything else. The machine's ACCEPT step runs on the tick
    // after the frame was validated, and that is the first moment the
    // `AcceptedFrame` exists — reading `acceptedFrames.last` on the tick the
    // frame was validated would persist the *previous* frame, which is exactly
    // the off-by-one this used to have.
    //
    // The still is therefore taken one tick (50 ms) after the machine accepted
    // the frame, while the phone is still held where the user aimed it: the
    // machine does not advance to the next target until the tick after that.
    if (s.acceptedFrames.length > acceptedBefore) {
      await _persistNew(s, acceptedBefore);
    }

    if (s.phase == CapturePhase.complete) {
      await _onStationComplete(s);
      return;
    }

    status = s.message;
  }

  void _updateHold() {
    final pose = poseSource.latest;
    if (pose == null) {
      hold = null;
      return;
    }
    final s = session;
    final targetIsPole = (s == null || s.targets.isEmpty)
        ? false
        : s.current.frame.isPole;
    hold = assessHold(
      pose.captureWorld,
      thresholds: settings.hold,
      targetIsPole: targetIsPole,
    );
  }

  Future<void> _updateDrift() async {
    if (!driftGate.enabled) return;
    final now = DateTime.now();
    final last = _lastDriftPoll;
    if (last != null && now.difference(last) < settings.driftPollInterval) {
      // Re-evaluate against the cached position so the hysteresis still runs
      // between platform polls.
      driftGate.update(
        cameraPosition: _lastCameraPosition,
        anchorTracking: _anchorTracking,
      );
      return;
    }
    _lastDriftPoll = now;

    final pos = await anchorSource.cameraPosition();
    if (pos != null) _lastCameraPosition = pos;

    final id = _anchorId;
    if (id != null) {
      final a = await anchorSource.refreshAnchor(id);
      _anchorTracking = a?.tracking ?? TrackingState.paused;
    }
    driftGate.update(
      cameraPosition: _lastCameraPosition,
      anchorTracking: _anchorTracking,
    );
  }

  /// Writes every frame accepted since [from] to disk, and marks the targets
  /// done.
  ///
  /// The order is the contract: bytes first, then the status. If the write
  /// fails, the target stays pending, so a resumed session retries it instead of
  /// believing it has a frame it does not have.
  Future<void> _persistNew(CaptureSession s, int from) async {
    for (var i = from; i < s.acceptedFrames.length; i++) {
      await _persistFrame(s, s.acceptedFrames[i]);
    }
  }

  Future<void> _persistFrame(CaptureSession s, AcceptedFrame frame) async {
    final group = currentGroup;
    if (group == null) return;

    // Through the sensor's current owner: the AR session while it is up, the
    // still camera otherwise. The bytes are the same kind of JPEG either way.
    final bytes = await activeCamera.takeStill();
    if (bytes == null) {
      warning =
          'The image for frame ${frame.index + 1} could not be written. It '
          'will be offered again.';
      return;
    }

    final written = await repository.frames.appendFrame(
      group: group,
      frameIndex: frame.index,
      jpeg: bytes,
      frame: frame,
    );
    if (!written.stored) {
      // The store verified the file it just wrote and it did not hold up — the
      // bytes stopped halfway, or they were not a JPEG. Nothing was recorded, so
      // the target is still pending and will be offered again. Saying "saved"
      // here is the one thing the flow must never do.
      warning =
          'Frame ${frame.index + 1} could not be read back after writing '
          '(${written.integrity.label}). It has not been accepted; take it '
          'again.';
      return;
    }

    final record = stations[currentStation];
    // The plan can grow during VERIFY, so a repair frame's index may be past the
    // end of the record's status list. Grow it rather than dropping the status,
    // which would leave a frame on disk that the review does not know about.
    if (frame.index >= record.statuses.length) {
      record.statuses.addAll(
        List<TargetStatus>.filled(
          frame.index - record.statuses.length + 1,
          TargetStatus.pending,
        ),
      );
    }
    record.statuses[frame.index] = TargetStatus.ok;
    if (_forceFail) {
      // The prototype's `ctl('fail')`: this frame is marked as failed (amber),
      // not done. Consumed so only the one frame is affected.
      record.statuses[frame.index] = TargetStatus.skipped;
      _lastFailIndex = frame.index;
      _forceFail = false;
    }
    record.savedFrames = record.capturedCount;
    persistedFrames++;
    warning = null;
  }

  Future<void> _onStationComplete(CaptureSession s) async {
    final record = stations[currentStation];
    final group = currentGroup;

    // The plan can grow during VERIFY: the coverage-gap repair pass appends
    // targets, and those frames are real and must be accounted for.
    final total = math.max(record.plan.frames.length, s.targets.length);
    if (record.statuses.length < total) {
      record.statuses.addAll(
        List<TargetStatus>.filled(
          total - record.statuses.length,
          TargetStatus.pending,
        ),
      );
    }
    for (var i = 0; i < s.targets.length && i < record.statuses.length; i++) {
      record.statuses[i] = s.targets[i].status;
    }

    record.coveragePct = s.viewpoint.stats?.coveragePct;
    record.warnings
      ..clear()
      ..addAll(s.viewpoint.warnings);
    record.unresolved
      ..clear()
      ..addAll(s.viewpoint.unresolved);

    // A percentage is the problem; the direction is the part the user can act
    // on. VERIFY clusters the uncovered samples into 15° bins, so naming the
    // largest ones is the difference between "0.8% is missing" and "the area
    // behind you and to your left is missing".
    final stats = s.viewpoint.stats;
    if (stats != null && stats.uncoveredPct > 0.001) {
      record.unresolved.addAll([
        for (final area in gapAreaNames(stats.uncovered))
          'Still missing: $area.',
      ]);
    }
    record.suggestedStation = s.viewpoint.recommendation;

    final needsReview =
        record.unresolved.isNotEmpty ||
        (record.coveragePct != null && record.coveragePct! < 99.9);
    record.outcome = needsReview
        ? StationOutcome.needsReview
        : StationOutcome.inProgress;

    if (group != null) {
      group.plannedFrameCount = total;
      for (var i = 0; i < record.statuses.length; i++) {
        group.statuses[i] = record.statuses[i];
      }
      group.coverageBefore = s.viewpoint.coverageBefore;
      group.coverageAfter = s.viewpoint.coverageAfter;
      group.notes
        ..clear()
        ..addAll(record.warnings)
        ..addAll(record.unresolved);
      group.status = needsReview
          ? StationStatus.needsReview
          : StationStatus.inProgress;
      await repository.frames.saveGroup(group);
    }
    if (manifest != null) await repository.saveManifest(manifest!);

    session = null;
    // The station is done, so the AR session and its camera can go — `endAr`
    // releases both. This is where the sensor is handed back, not at the anchor,
    // because the capture needs the pose.
    await endAr();
    // Release the still camera too, so the *next* station's AR session can take
    // it. The gyroscope stays up: it costs nothing and the next station's SETTLE
    // phase needs it immediately.
    await camera.dispose();
    screen = CaptureScreen.stationReview;
  }

  /// Accepts the current station's panorama.
  Future<void> acceptStation() async {
    final record = stations[currentStation];
    record.outcome = StationOutcome.accepted;
    final group = currentGroup;
    if (group != null) {
      group.status = StationStatus.accepted;
      await repository.frames.saveGroup(group);
    }
    if (manifest != null) await repository.saveManifest(manifest!);
    status = '${record.displayName} accepted.';
  }

  /// Marks the station for a retake.
  ///
  /// Only the frames that are not on disk are re-shot; everything already
  /// written is kept, which is what the brief requires.
  void retakeStation() {
    final record = stations[currentStation];
    for (var i = 0; i < record.statuses.length; i++) {
      if (record.statuses[i] != TargetStatus.ok) {
        record.statuses[i] = TargetStatus.pending;
      }
    }
    record.outcome = StationOutcome.inProgress;
    status = 'Retaking the frames that are not yet on disk.';
  }

  /// Finishes the room and moves to the tour preview.
  Future<void> finish() async {
    await gyro.stop();
    await arCamera?.dispose();
    await camera.dispose();
    screen = CaptureScreen.tourPreview;
  }

  /// Every station the user accepted, in order. The tour viewer links these.
  List<StationRecord> get acceptedStations =>
      stations.where((s) => s.outcome == StationOutcome.accepted).toList();

  // ── interruption ───────────────────────────────────────────────────────

  /// Pauses for an interruption: an incoming call, an app switch, a back
  /// gesture.
  ///
  /// Every accepted frame is already on disk, so nothing is lost. Only the frame
  /// being captured at the moment of the interruption is dropped.
  void onInterruption() {
    if (interrupted) return;
    interrupted = true;
    session?.pause();
    status = 'Capture paused. Your captured photos are saved.';
  }

  /// Resumes after an interruption, from the next target that still needs a
  /// frame.
  Future<void> resumeAfterInterruption() async {
    interrupted = false;
    final s = session;
    if (s == null) {
      // The session was torn down. Rebuild from the persisted state so no
      // accepted frame is repeated.
      final record = stations[currentStation];
      final next = record.nextPending();
      await beginCapture(fromTarget: next < 0 ? null : next);
      return;
    }
    s.resume();
    status = 'Resumed.';
  }

  /// Captures the current target by hand.
  ///
  /// This is the documented fallback for the two cases where guidance cannot
  /// work: tracking was lost, or the device has no pose source at all. The
  /// frame is recorded with `manual: true` so the review can say how many frames
  /// were captured without orientation data — an honest caveat rather than a
  /// silent one.
  ///
  /// It is a no-op unless the machine is actually waiting in ALIGN, so a stray
  /// tap cannot fire the shutter mid-settle.
  bool manualCapture() {
    final s = session;
    if (s == null) return false;
    if (s.phase != CapturePhase.align) return false;
    if (!trackingLost && s.sensorSource is LiveCaptureSource) {
      final live = s.sensorSource! as LiveCaptureSource;
      if (live.isGuiding) return false;
    }
    s.manual = true;
    s.hold = true;
    return true;
  }

  /// Whether the manual button should be offered.
  bool get manualCaptureOffered =>
      session != null &&
      session!.phase == CapturePhase.align &&
      (trackingLost || !arTier.guidanceEnabled);

  /// Whether the station anchor is currently tracked.
  bool get withinStation => !driftGate.paused;

  // ── resume from disk ───────────────────────────────────────────────────

  Future<void> resumeInto(String id) async {
    final loaded = await repository.loadManifest(id);
    if (loaded == null) {
      warning = 'That session could not be read. Starting a new one.';
      await createNewSession();
      return;
    }
    manifest = loaded;
    final groups = await repository.loadStations(loaded);
    stations.clear();
    for (final g in groups) {
      if (g == null) continue;
      stations.add(
        StationRecord(
          index: g.index,
          roomSize: roomSize,
          obstruction: obstruction,
          plan: planAll(g.planParams),
          planX: g.position.x,
          planY: g.position.z,
          statuses: List<TargetStatus>.generate(
            g.plannedFrameCount,
            (i) => g.statuses[i] ?? TargetStatus.pending,
          ),
          outcome: switch (g.status) {
            StationStatus.accepted => StationOutcome.accepted,
            StationStatus.needsReview => StationOutcome.needsReview,
            StationStatus.inProgress => StationOutcome.inProgress,
          },
          savedFrames: g.frames.length,
          anchorPlaced: g.origin != null,
        ),
      );
      // A resumed station's origin is restored for station diagnostics without
      // asking ARCore for an anchor it has already released.
      if (g.origin != null) stationOrigin = g.origin;
    }
    final point = await repository.resumePoint(loaded);
    currentStation = stations.isEmpty
        ? 0
        : point.stationIndex.clamp(0, stations.length - 1);
    currentGroup = stations.isEmpty
        ? null
        : await repository.frames.loadGroup(loaded.sessionId, currentStation);

    // A frame can go bad *after* it was accepted: the volume filled up, the file
    // was edited outside the app, a cleanup tool half-deleted it. The manifest
    // still says done, so the progress grid would show a green cell with no
    // photograph behind it. Take those frames back out of the record so the
    // station re-offers them instead of pretending.
    if (currentGroup != null && point.unreadable.isNotEmpty) {
      final record = stations[currentStation];
      for (final u in point.unreadable) {
        if (u.frameIndex < record.statuses.length) {
          record.statuses[u.frameIndex] = TargetStatus.pending;
        }
        await repository.frames.unacceptFrame(
          group: currentGroup!,
          frameIndex: u.frameIndex,
        );
      }
      record.savedFrames = record.capturedCount;
    }

    screen = CaptureScreen.stationSetup;
    status = point.isFresh
        ? 'New station.'
        : 'Resuming: ${point.alreadyAccepted.length} frames already saved'
              '${point.unreadable.isEmpty ? '.' : ', and '
                        '${point.unreadable.length} could not be read back from disk and '
                        'will be shot again.'}';
  }

  /// The resume point for the current session, for the "resume or exit" sheet.
  Future<ResumePoint?> currentResumePoint() async {
    final m = manifest;
    if (m == null) return null;
    return repository.resumePoint(m);
  }
}
