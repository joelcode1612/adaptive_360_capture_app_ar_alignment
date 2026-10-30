import 'dart:async';
import 'dart:io';
import 'dart:typed_data';

import 'package:path/path.dart' as p;

import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/glb_mesh_source.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/glb/glb_model.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// Fakes for the Section 1 capture tests.
///
/// These exist so the failure paths can be tested at all. A test that needs a
/// real ARCore session can only be run on a device, and the paths that matter
/// most — a raycast that misses, a JPEG that never lands, a pose that stops
/// tracking — are exactly the ones a device cannot be made to produce on demand.

/// A [ArCoreGateway] that answers whatever the test told it to answer.
class FakeArGateway implements ArCoreGateway {
  FakeArGateway({
    this.hostPresent = true,
    this.supported = true,
    this.serviceInstalled = true,
    this.hit,
    this.stillBytes,
    this.stillError,
    this.surfaceFound = true,
  });

  bool hostPresent;
  bool supported;
  bool serviceInstalled;

  /// Whether the raycast finds anything at all. `false` is the "scan this
  /// surface" case; [hit] is then irrelevant.
  bool surfaceFound;

  /// The raycast answer, when [surfaceFound].
  ArRaycastHit? hit;

  Uint8List? stillBytes;
  Object? stillError;

  /// The dimensions the fake camera reports for a still. A screen test encodes
  /// a real image at these dimensions so the decode check can accept it.
  int stillWidth = 1920;
  int stillHeight = 1080;

  /// The pose the fake still reports. `null` (default) mimics a healthy host:
  /// the photograph's pose is the last camera pose the session emitted, so the
  /// service's ≤250 ms frame-lag validation sees a frame-consistent session.
  /// A test that needs a specific still pose sets this explicitly; a test that
  /// wants to reproduce a host that did *not* report the photograph's pose sets
  /// [omitStillPose].
  ArPose? stillPose;

  /// When true, [takeStill] reports `capturePose: null` even if a camera pose
  /// was emitted — the "host did not report the photograph's pose" refusal path.
  bool omitStillPose = false;

  /// The most recent camera pose the session emitted, retained so a healthy
  /// host's still can carry it.
  ArPose? _lastCameraPose;

  /// How many anchors were created. Asserted by "a miss must not anchor".
  int anchorCreations = 0;

  /// How many raycasts were asked for. Placement must not make any — the
  /// world position is computed, not discovered. The probe still uses them.
  int raycastCalls = 0;

  /// When set, [createAnchor] refuses: the "AR would not hold the anchor"
  /// path.
  bool refuseAnchors = false;

  /// The tracking state [createAnchor] reports. ARCore commonly starts a world
  /// anchor paused and tracks it a moment later, so the default tracks
  /// immediately; a test sets `paused` to reproduce the invisible-marker case.
  TrackingState anchorTracking = TrackingState.tracking;

  /// Drops an anchor as ARCore does when tracking is lost: [refreshAnchor]
  /// then answers "unknown_anchor" (null), which is what makes the controller
  /// re-place.
  void dropAnchor(String id) => anchors.remove(id);

  /// When set, [createAnchor] waits on this future before creating the anchor.
  ///
  /// A placement is several awaits deep (`release` → `create` → `pushScene`),
  /// and a device delivers poses at 30–60 Hz *while those awaits are open*.
  /// This hook lets a test hold the placement open between the release of the
  /// old anchor and the creation of the new one — the exact window the race the
  /// transition tests guard against lives in.
  Future<void>? anchorGate;

  /// When set, [refreshAnchor] waits on this future before answering.
  ///
  /// Holds a refresh open across a placement, so the test can deliver an
  /// answer about an anchor that has since been replaced — the stale-answer
  /// case a real host can produce when its detach and its answer race.
  Future<void>? refreshGate;

  /// How many stills were requested.
  int stillRequests = 0;

  /// How many of those requests answered. A held [stillGate] makes this the
  /// observable for "the capture continuation has resumed".
  int stillReturns = 0;

  /// When set, [takeStill] waits on this before answering — the fake's version
  /// of a native still request that is never consumed.
  Future<void>? stillGate;

  /// How many anchor refreshes were asked for. A refresh re-reads an existing
  /// anchor; a creation invents one, and placement must not do the latter per
  /// frame.
  int refreshCalls = 0;

  /// The last scene that was pushed.
  List<ArSceneDraw> lastScene = const <ArSceneDraw>[];

  final Map<String, MeshBuffers> meshes = <String, MeshBuffers>{};

  final StreamController<ArCoreEvent> _events =
      StreamController<ArCoreEvent>.broadcast();

  int _anchorCounter = 0;

  /// The base the fake stamps its pose events with.
  ///
  /// The real host forwards `frame.timestamp` — monotonic nanoseconds whose
  /// origin is the device's boot, which is why a test cannot assume any
  /// particular base. The default keeps that shape while staying deterministic:
  /// a suite substitutes the same clock the controller is given, so the
  /// engine's 600 ms window is walked by the injected clock rather than by real
  /// elapsed time. Without it, a test that emits fourteen poses in seventy
  /// milliseconds of real time could never elapse a 600 ms window, and the
  /// shutter would stay shut for a reason no assertion names.
  DateTime Function() clock = DateTime.now;

  /// Pushes a pose onto the event stream, as the real host would.
  ///
  /// The real host reports the active anchor's live pose on every pose event
  /// while it holds one, so the fake does too: unless a test passes an explicit
  /// [activeAnchorPose], the current [activeAnchorId]'s stored position and
  /// tracking are attached. [includeActiveAnchorPose] set to `false` reproduces
  /// the event a host emits when it cannot report the anchor — the case the
  /// controller must not read as "still tracking".
  void emitPose({
    required Vec3 position,
    required Quat worldOrientation,
    TrackingState tracking = TrackingState.tracking,
    int planesTracked = 3,
    // The record shape mirrors `ArPoseSample.activeAnchorPose`: the native host
    // reports the anchor's full pose (position, orientation, tracking), not
    // just a position.
    ({Vec3 position, Quat orientation, TrackingState tracking})?
    activeAnchorPose,
    String? reportedActiveAnchorId,
    bool includeActiveAnchorPose = true,
  }) {
    var live = activeAnchorPose;
    final liveId = reportedActiveAnchorId ?? activeAnchorId;
    if (includeActiveAnchorPose && live == null) {
      final active = anchors[activeAnchorId];
      if (active != null) {
        // The fake's stored anchors carry only position and tracking; the
        // record still needs an orientation, so it is stamped `Quat.identity`.
        // Honest for current consumers: no reader of `activeAnchorPose`
        // consumes the orientation (the controller reads position and
        // tracking, capture_controller.dart ~2848), so an invented orientation
        // cannot silently steer a test. A test that needs a specific anchor
        // orientation passes [activeAnchorPose] explicitly.
        live = (
          orientation: Quat.identity,
          position: active.worldPosition,
          tracking: active.tracking,
        );
      }
    }
    final rawPose = ArPose(
      position: position,
      // `ArPose.captureWorld` is `arCameraPoseToCaptureWorld(orientation)`,
      // so the fake hands over the *converted* quaternion and the getter
      // must be given the pre-conversion one. Round-tripping through the
      // real inverse keeps the fake honest.
      orientation: worldOrientation * Quat.axisAngle(180, Vec3.unitY),
      tracking: tracking,
      timestampNs: clock().microsecondsSinceEpoch * 1000,
    );
    // Retain the camera pose so a healthy host's still can carry it.
    _lastCameraPose = rawPose;
    _events.add(
      ArCoreEvent.pose(
        ArPoseSample(
          raw: rawPose,
          planesTracked: planesTracked,
          planeCount: planesTracked,
          activeAnchorPose: live,
          activeAnchorId: live == null ? null : liveId,
        ),
      ),
    );
  }

  /// Pushes a native failure, as the host would on a broken render loop.
  void emitFault(String message, {String? frame, bool fatal = true}) {
    _events.add(ArCoreEvent.fault(message, frame: frame, fatal: fatal));
  }

  @override
  Future<bool> get isAvailable async => hostPresent;

  @override
  Future<ArCoreAvailability> checkAvailability() async => ArCoreAvailability(
    supported: supported,
    serviceInstalled: serviceInstalled,
    reason: supported ? null : 'Fake: unsupported device.',
  );

  @override
  Future<ArCoreInstallResult> requestInstall() async =>
      const ArCoreInstallResult(installed: true, status: 'INSTALLED');

  /// Whether `start()` reports that it reused the live session. A reused session
  /// keeps ARCore's world origin; a fresh one does not.
  bool reusedSession = false;

  /// How many times a session was started. A recalibration must start a new
  /// one, and a plain resume must not.
  int startCalls = 0;

  /// When set, `start()` counts the call and then throws this — the way a real
  /// broken session start surfaces to the controller: as a thrown failure, not
  /// as an absent session or a mocked-out method.
  Object? startError;

  @override
  Future<ArCoreSession> start({
    required int width,
    required int height,
    bool planes = true,
  }) async {
    startCalls++;
    final failure = startError;
    if (failure != null) throw failure;
    return ArCoreSession(
      textureId: 1,
      planes: true,
      anchors: true,
      cameraImage: true,
      depth: false,
      reused: reusedSession,
    );
  }

  @override
  Future<void> resize({required int width, required int height}) async {}

  @override
  Future<void> setDisplayRotation(int rotation) async {}

  @override
  Future<void> setPlaneDetection(bool enabled) async {}

  @override
  Future<void> setMesh({
    required String id,
    required Float32List positions,
    required Float32List colors,
  }) async {
    meshes[id] = MeshBuffers(
      positions: positions,
      colors: colors,
      vertexCount: positions.length ~/ 3,
    );
  }

  @override
  Future<void> setScene(List<ArSceneDraw> draws) async {
    lastScene = draws;
  }

  /// Per-direction probe results, keyed by direction index. Empty means every
  /// direction reports a miss.
  List<SurfaceProbe> probeResults = const <SurfaceProbe>[];

  int probeCalls = 0;

  @override
  Future<List<SurfaceProbe>> probeDirections({
    required Vec3 origin,
    required List<Vec3> directions,
    double maxDistanceM = 8,
  }) async {
    probeCalls++;
    if (probeResults.isEmpty) {
      return List<SurfaceProbe>.filled(
        directions.length,
        const SurfaceProbe.miss(),
      );
    }
    return <SurfaceProbe>[
      for (var i = 0; i < directions.length; i++)
        i < probeResults.length ? probeResults[i] : const SurfaceProbe.miss(),
    ];
  }

  @override
  Future<ArRaycastOutcome> raycast({
    required Vec3 origin,
    required Vec3 direction,
    double maxDistanceM = 8,
    String surface = 'plane',
  }) async {
    raycastCalls++;
    return surfaceFound
        ? ArRaycastOutcome.hit(hit ?? wallHitAhead())
        : const ArRaycastOutcome.miss(ArRaycastMiss.noCandidates);
  }

  @override
  Future<ArCoreAnchor?> createAnchor({
    required Vec3 position,
    required Quat orientation,
    String? trackableKey,
  }) async {
    final gate = anchorGate;
    if (gate != null) await gate;
    if (refuseAnchors) return null;
    anchorCreations++;
    final anchor = ArCoreAnchor(
      id: 'anchor-${++_anchorCounter}',
      worldPosition: position,
      tracking: anchorTracking,
    );
    anchors[anchor.id] = anchor;
    return anchor;
  }

  /// Anchors by id, so [refreshAnchor] can answer the way ARCore does: the
  /// anchor is still there, possibly with a refined pose.
  final Map<String, ArCoreAnchor> anchors = <String, ArCoreAnchor>{};

  /// A refined pose per anchor id, invented by a test as ARCore refining the
  /// anchor would. Applied by [refreshAnchor] and kept in [anchors], so the
  /// next pose event reports the same pose the refresh just returned.
  final Map<String, Vec3> refinedAnchorPositions = <String, Vec3>{};

  @override
  Future<ArCoreAnchor?> refreshAnchor(String id) async {
    refreshCalls++;
    final gate = refreshGate;
    if (gate != null) await gate;
    final existing = anchors[id];
    if (existing == null) return null;
    // ARCore's tracking state is dynamic over an anchor's life, so a refresh
    // re-reads it rather than replaying the creation-time value. A test flips
    // `anchorTracking` to simulate ARCore catching up (or losing) the anchor.
    final refreshed = ArCoreAnchor(
      id: existing.id,
      worldPosition:
          refinedAnchorPositions[existing.id] ?? existing.worldPosition,
      tracking: anchorTracking,
    );
    anchors[id] = refreshed;
    return refreshed;
  }

  @override
  Future<void> detachAnchor(String id) async {
    anchors.remove(id);
  }

  /// The anchor the controller last named as active, or `null`.
  ///
  /// [emitPose] attaches this anchor's stored pose to the event, the way the
  /// native host reports the active anchor on every frame.
  String? activeAnchorId;

  @override
  Future<void> setActiveAnchor(String? id) async {
    activeAnchorId = id;
  }

  /// How many times "Scan again" reached the host.
  int rescanCalls = 0;

  /// Lifecycle pause/resume, as the screen's [WidgetsBindingObserver] drives it.
  int pauseCalls = 0;
  int resumeCalls = 0;
  bool paused = false;

  /// When set, [pause] throws instead of releasing the camera.
  Object? pauseError;

  /// When set, [resume] throws — the "camera could not be reopened" path.
  Object? resumeError;

  @override
  Future<void> rescan() async {
    rescanCalls++;
    anchors.clear();
  }

  @override
  Future<void> pause() async {
    pauseCalls++;
    if (pauseError != null) throw pauseError!;
    paused = true;
  }

  @override
  Future<void> resume() async {
    resumeCalls++;
    if (resumeError != null) throw resumeError!;
    paused = false;
  }

  @override
  Future<ArCoreStill> takeStill({int quality = 95}) async {
    stillRequests++;
    // A still that never returns, as a stopped render loop produces: the
    // native request is queued and nothing consumes it.
    final gate = stillGate;
    if (gate != null) await gate;
    if (stillError != null) throw stillError!;
    final bytes = stillBytes;
    if (bytes == null) {
      throw const ArCoreFailure('Fake: the session produced no camera image.');
    }
    stillReturns++;
    // A healthy host reports the photograph's pose (with a timestamp from the
    // same monotonic clock as the frame that armed the shutter). The fake
    // defaults to the last emitted camera pose so the service's ≤250 ms
    // frame-lag validation sees a frame-consistent session. An explicitly set
    // [stillPose] wins; [omitStillPose] reproduces a host that did not report
    // the photograph's pose.
    final basePose = omitStillPose
        ? null
        : (stillPose ?? _lastCameraPose);
    final capturePose = basePose == null
        ? null
        : (basePose.timestampNs == null
              ? ArPose(
                  position: basePose.position,
                  orientation: basePose.orientation,
                  tracking: basePose.tracking,
                  timestampNs: clock().microsecondsSinceEpoch * 1000,
                )
              : basePose);
    return ArCoreStill(
      jpeg: bytes,
      width: stillWidth,
      height: stillHeight,
      // The fake stamps a 90° rotation, so the raw sensor frame is the
      // transpose of the encoded one: rotating (w, h) by 90° yields (h, w).
      // These source dimensions are the ones intrinsics refer to, which the
      // photo service validates against (`_rotatedIntrinsics`).
      sourceWidth: stillHeight,
      sourceHeight: stillWidth,
      rotationAppliedDeg: 90,
      capturePose: capturePose,
    );
  }

  /// How many times the session was stopped (and its texture released).
  int stopCalls = 0;

  @override
  Future<void> stop() async {
    stopCalls++;
    paused = false;
  }

  @override
  Stream<ArCoreEvent> get events => _events.stream;

  void close() {
    _events.close();
  }
}

/// A hit 3 m along +Z on a vertical wall whose normal faces back down −Z.
ArRaycastHit wallHitAhead([double distance = 3]) => ArRaycastHit(
  worldPosition: Vec3(0, 0, distance),
  planeOrientation: Quat.identity,
  distanceM: distance,
  planeType: 'PLANE_VERTICAL',
  trackableKey: 'plane-1',
  extent: (x: 4, z: 3),
);

/// An in-memory [CaptureRepository] that can be told to fail a save.
class FakeCaptureRepository implements CaptureRepository {
  final Map<String, CaptureStation> stations = <String, CaptureStation>{};
  final Map<String, List<CaptureTarget>> targets =
      <String, List<CaptureTarget>>{};
  final Map<String, List<CaptureRecord>> records =
      <String, List<CaptureRecord>>{};

  /// When set, [saveRecord] throws instead of writing.
  Object? recordError;

  /// When set, saving writes but the file "disappears" — the read-back check is
  /// what has to catch this.
  bool vanishAfterWrite = false;

  int saveRecordCalls = 0;

  @override
  Future<String> framePath(String stationId, int targetId) async => p.join(
    Directory.systemTemp.path,
    'ar_capture_test',
    stationId,
    't${targetId.toString().padLeft(2, '0')}.jpg',
  );

  @override
  Future<List<String>> stationIds() async =>
      stations.keys.toList(growable: false);

  @override
  Future<CaptureStation?> loadStation(String stationId) async =>
      stations[stationId];

  @override
  Future<List<CaptureTarget>> loadTargets(String stationId) async =>
      targets[stationId] ?? const <CaptureTarget>[];

  @override
  Future<List<CaptureRecord>> loadRecords(String stationId) async =>
      records[stationId] ?? const <CaptureRecord>[];

  @override
  Future<void> saveRecord(CaptureRecord record) async {
    saveRecordCalls++;
    if (recordError != null) throw recordError!;
    final list = records.putIfAbsent(record.stationId, () => <CaptureRecord>[]);
    list.removeWhere((CaptureRecord r) => r.targetId == record.targetId);
    list.add(record);
  }

  @override
  Future<void> saveStation(CaptureStation station) async {
    stations[station.stationId] = station;
  }

  @override
  Future<void> saveTargets(List<CaptureTarget> list) async {
    if (list.isEmpty) return;
    targets[list.first.stationId] = List<CaptureTarget>.of(list);
  }
}

/// A [GlbMeshSource] stand-in: the geometry is irrelevant to the maths under
/// test, and loading the real asset in a unit test would couple the alignment
/// suite to the asset bundle.
class FakeMeshSource extends GlbMeshSource {
  FakeMeshSource() : super(FakeModel(), FakeModel());

  @override
  bool matchesSpec() => true;

  @override
  MeshBuffers buffersFor(GlbModel model, {required bool skipCheckMarks}) =>
      MeshBuffers(
        positions: Float32List.fromList(const <double>[0, 0, 0]),
        colors: Float32List.fromList(const <double>[1, 1, 1, 1]),
        vertexCount: 1,
      );

  @override
  double get targetWidthM => 0.54;

  @override
  double get bezelWidthM => 0.70;

  @override
  int get targetTriangles => 1;

  @override
  int get bezelTriangles => 1;
}

/// A one-triangle [GlbModel]. `GlbMeshSource` only reads bounds, drawables and
/// triangle counts from it, so this is enough.
class FakeModel implements GlbModel {
  @override
  final List<GlbDrawable> drawables = const <GlbDrawable>[];

  @override
  Vec3 get min => Vec3.zero;

  @override
  Vec3 get max => Vec3.zero;

  @override
  int get triangleCount => 1;

  @override
  int get vertexCount => 3;

  @override
  double get widthX => 0.54;

  @override
  double get heightY => 0.54;

  @override
  double get depthZ => 0.006;

  @override
  Vec3 get center => Vec3.zero;

  @override
  int get nodeCount => 1;

  @override
  int get meshCount => 2;

  @override
  int get materialCount => 1;

  @override
  int get accessorCount => 3;

  @override
  int get bufferViewCount => 3;

  @override
  List<String> get unsupported => const <String>[];

  @override
  String get gltfVersion => '2.0';

  @override
  String? get generator => 'fake';
}
