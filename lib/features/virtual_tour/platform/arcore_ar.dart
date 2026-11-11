import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_payload.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The ARCore bridge: Dart ↔ the Kotlin host in `android/.../ar/`.
///
/// **Why a hand-written bridge.** Section 1 needs four capabilities that have to
/// work together: an arbitrary world-space raycast (not only a screen-tap hit
/// test), an anchor created at that raycast's hit, the camera pose at 30 Hz, and
/// a photograph taken *through* the running session — because ARCore owns the
/// camera device exclusively and CameraX cannot open it at the same time. No
/// maintained Flutter AR package exposes all four, and the ones that come
/// closest render through Sceneform, which would mean dropping the two shipped
/// `.glb` assets for a different model pipeline. See the class comment on
/// `ArCoreCapturePlugin.kt`.
///
/// **Coordinate contract, stated once.** ARCore reports:
///
/// * world: right-handed, `+Y` roughly up, metres;
/// * camera pose: a rigid transform whose local frame is `+X` screen-right,
///   `+Y` screen-up, `−Z` the optical axis.
///
/// This bridge hands that pose over **unchanged** as [ArPoseSample.raw]. The
/// conversion into this project's convention is [ArPoseSample.orientation],
/// which delegates to `arCameraPoseToCaptureWorld` — the single adapter, already
/// unit-tested in `domain/guidance/projection.dart`. Nothing else in the app is
/// allowed to reinterpret an ARCore quaternion.
///
/// Everything here is a thin, typed wrapper over one `MethodChannel` and one
/// `EventChannel`. All judgement — whether a hit is good enough, whether a pose
/// may be used — lives in the capture module above it.
/// What the capture module needs from an AR platform.
///
/// An interface, not a concrete class, so the controller, the anchoring service
/// and the photo service can be driven by a fake in a unit test. Anything that
/// depends on [ArCoreHost] directly would need a real device (or a mocked
/// `MethodChannel`) to be tested at all, which is how the interesting failure
/// paths — a missed raycast, a failed save — end up untested.
abstract class ArCoreGateway {
  Future<bool> get isAvailable;
  Future<ArCoreAvailability> checkAvailability();
  Future<ArCoreInstallResult> requestInstall();
  Future<ArCoreSession> start({
    required int width,
    required int height,
    bool planes = true,
  });
  Future<void> resize({required int width, required int height});
  Future<void> setDisplayRotation(int rotation);
  Future<void> setPlaneDetection(bool enabled);
  Future<void> setMesh({
    required String id,
    required Float32List positions,
    required Float32List colors,
  });
  Future<void> setScene(List<ArSceneDraw> draws);

  /// Probes every direction against the current frame, in one call.
  ///
  /// Returns one result per direction, in the order supplied. No anchors are
  /// created — this answers "is there a surface that way?" without committing to
  /// anything.
  Future<List<SurfaceProbe>> probeDirections({
    required Vec3 origin,
    required List<Vec3> directions,
    double maxDistanceM = 8,
  });

  /// Raycasts an arbitrary world ray.
  ///
  /// A miss comes back as an [ArRaycastOutcome] with a null
  /// [ArRaycastOutcome.hit] and a [ArRaycastMiss] reason — never as a guessed
  /// position, and never as a bare `null` that hides why nothing was placed.
  Future<ArRaycastOutcome> raycast({
    required Vec3 origin,
    required Vec3 direction,
    double maxDistanceM = 8,
    String surface = 'plane',
  });
  Future<ArCoreAnchor?> createAnchor({
    required Vec3 position,
    required Quat orientation,
    String? trackableKey,
  });
  Future<ArCoreAnchor?> refreshAnchor(String id);
  Future<void> detachAnchor(String id);

  /// Names the anchor the guidance is aimed at, so the pose stream can carry
  /// that anchor's **current** pose — the one the renderer draws.
  ///
  /// `null` clears it. One anchor at a time, because only one target is active.
  Future<void> setActiveAnchor(String? id);

  /// Re-arms surface detection and drops every anchor, without restarting the
  /// session — a restart would invalidate the station's world pose and with it
  /// every target direction.
  Future<void> rescan();

  /// Releases the camera while the app is in the background.
  ///
  /// The session — and therefore ARCore's world origin — is kept, so the
  /// station's target positions stay valid and [resume] continues the same
  /// capture rather than starting a new one. Any still or raycast still queued
  /// is answered with an error instead of being left to hang.
  Future<void> pause();

  /// Resumes a session [pause] released. Throws [ArCoreFailure] when the camera
  /// cannot be reopened; the caller must then require recalibration.
  Future<void> resume();

  Future<ArCoreStill> takeStill({int quality = 95});

  Future<void> stop();

  Stream<ArCoreEvent> get events;
}

class ArCoreHost implements ArCoreGateway {
  ArCoreHost({MethodChannel? channel, EventChannel? events})
    : _channel = channel ?? const MethodChannel(_methodChannelName),
      _events = events ?? const EventChannel(_eventChannelName);

  static const String _methodChannelName = 'adaptive360/arcore';
  static const String _eventChannelName = 'adaptive360/arcore/events';

  final MethodChannel _channel;
  final EventChannel _events;

  Stream<ArCoreEvent>? _stream;
  bool _sessionRunning = false;

  bool get sessionRunning => _sessionRunning;

  /// Whether the host exists on this platform at all.
  ///
  /// A `MissingPluginException` from the first call is the honest answer: this
  /// build is running somewhere the Kotlin host is not compiled in (iOS, or a
  /// desktop target), and the capture module has to say that out loud instead of
  /// pretending to track.
  @override
  Future<bool> get isAvailable async {
    try {
      await _channel.invokeMethod<dynamic>('resize', <String, dynamic>{
        'width': 1,
        'height': 1,
      });
      return true;
    } on MissingPluginException {
      return false;
    } on PlatformException {
      // The plugin is compiled in but has no session yet, which still counts as
      // "the host is present".
      return true;
    }
  }

  /// What the device reports before any session is created.
  @override
  Future<ArCoreAvailability> checkAvailability() async {
    final raw = await _channel.invokeMapMethod<String, dynamic>(
      'checkAvailability',
    );
    if (raw == null) {
      return const ArCoreAvailability.unavailable(
        'The AR host did not answer the availability check.',
      );
    }
    return ArCoreAvailability(
      supported: raw['supported'] as bool? ?? false,
      serviceInstalled: raw['serviceInstalled'] as bool? ?? false,
      reason: raw['reason'] as String?,
    );
  }

  /// Asks ARCore to install or update itself. Returns whether it is installed
  /// now.
  @override
  Future<ArCoreInstallResult> requestInstall() async {
    final raw = await _channel.invokeMapMethod<String, dynamic>(
      'requestInstall',
    );
    return ArCoreInstallResult(
      installed: raw?['installed'] as bool? ?? false,
      status: raw?['status'] as String?,
    );
  }

  /// Starts the session and returns the Flutter texture id the camera renders
  /// into.
  @override
  Future<ArCoreSession> start({
    required int width,
    required int height,
    bool planes = true,
  }) async {
    final raw = await _channel.invokeMapMethod<String, dynamic>(
      'start',
      <String, dynamic>{'width': width, 'height': height, 'planes': planes},
    );
    if (raw == null) throw const ArCoreFailure('The AR session did not start.');
    _sessionRunning = true;
    return ArCoreSession(
      textureId: (raw['textureId'] as num?)?.toInt() ?? -1,
      sessionId: _sessionIdOrNull(raw['sessionId']),
      planes: raw['planes'] as bool? ?? false,
      anchors: raw['anchors'] as bool? ?? false,
      cameraImage: raw['cameraImage'] as bool? ?? false,
      depth: raw['depth'] as bool? ?? false,
      reused: raw['reused'] as bool? ?? false,
    );
  }

  /// Resizes the Flutter texture backing surface after a layout change.
  @override
  Future<void> resize({required int width, required int height}) =>
      _channel.invokeMethod<void>('resize', <String, dynamic>{
        'width': width,
        'height': height,
      });

  @override
  Future<void> setDisplayRotation(int rotation) => _channel.invokeMethod<void>(
    'setDisplayRotation',
    <String, dynamic>{'rotation': rotation},
  );

  @override
  Future<void> setPlaneDetection(bool enabled) => _channel.invokeMethod<void>(
    'setPlaneDetection',
    <String, dynamic>{'enabled': enabled},
  );

  /// Uploads a mesh parsed from one of the shipped `.glb` files.
  ///
  /// [positions] is xyz per vertex and [colors] rgba per vertex. The native
  /// renderer only ever draws what Dart uploaded, so the geometry on screen is
  /// the same geometry the Dart GLB parser produced and the same geometry the
  /// unit tests assert on.
  @override
  Future<void> setMesh({
    required String id,
    required Float32List positions,
    required Float32List colors,
  }) => _channel.invokeMethod<void>('setMesh', <String, dynamic>{
    'id': id,
    'positions': positions,
    'colors': colors,
  });

  /// Replaces the whole AR scene: which meshes, on which anchors, with which
  /// tint. Passing an empty list hides everything.
  @override
  Future<void> setScene(List<ArSceneDraw> draws) => _channel.invokeMethod<void>(
    'setScene',
    <String, dynamic>{
      'draws': draws.map((ArSceneDraw d) => d.toMap()).toList(growable: false),
    },
  );

  /// Raycasts an arbitrary world ray. Returns `null` when nothing was hit.
  ///
  /// `null` is the normal "no surface here" answer and must never be turned into
  /// a guessed position.
  @override
  Future<List<SurfaceProbe>> probeDirections({
    required Vec3 origin,
    required List<Vec3> directions,
    double maxDistanceM = 8,
  }) async {
    final flat = <double>[];
    for (final d in directions) {
      flat
        ..add(d.x)
        ..add(d.y)
        ..add(d.z);
    }
    final raw = await _channel.invokeListMethod<dynamic>(
      'probeDirections',
      <String, dynamic>{
        'origin': <double>[origin.x, origin.y, origin.z],
        'directions': flat,
        'maxDistanceM': maxDistanceM,
      },
    );
    if (raw == null) {
      return List<SurfaceProbe>.filled(
        directions.length,
        const SurfaceProbe.miss(),
      );
    }
    return <SurfaceProbe>[
      for (final dynamic e in raw)
        if ((e as Map)['hit'] == true)
          SurfaceProbe.found(
            distanceM: (e['distanceM'] as num?)?.toDouble(),
            surfaceType: e['planeType'] as String?,
          )
        else
          const SurfaceProbe.miss(),
    ];
  }

  @override
  Future<ArRaycastOutcome> raycast({
    required Vec3 origin,
    required Vec3 direction,
    double maxDistanceM = 8,
    String surface = 'plane',
  }) async {
    final raw = await _channel
        .invokeMapMethod<String, dynamic>('raycast', <String, dynamic>{
          'ox': origin.x,
          'oy': origin.y,
          'oz': origin.z,
          'dx': direction.x,
          'dy': direction.y,
          'dz': direction.z,
          'maxDistanceM': maxDistanceM,
          'surface': surface,
        });
    if (raw == null) {
      return const ArRaycastOutcome.miss(ArRaycastMiss.unknown);
    }
    if (raw['hit'] != true) {
      return ArRaycastOutcome.miss(switch (raw['reason'] as String?) {
        'no_candidates' => ArRaycastMiss.noCandidates,
        'no_plane_within_range' => ArRaycastMiss.noPlaneInRange,
        _ => ArRaycastMiss.unknown,
      }, candidates: (raw['candidates'] as num?)?.toInt() ?? 0);
    }
    return ArRaycastOutcome.hit(ArRaycastHit.fromMap(raw));
  }

  /// Creates an anchor at a world pose, on the plane that was hit when
  /// [trackableKey] is supplied.
  @override
  Future<ArCoreAnchor?> createAnchor({
    required Vec3 position,
    required Quat orientation,
    String? trackableKey,
  }) async {
    final raw = await _channel
        .invokeMapMethod<String, dynamic>('createAnchor', <String, dynamic>{
          'px': position.x,
          'py': position.y,
          'pz': position.z,
          'qx': orientation.x,
          'qy': orientation.y,
          'qz': orientation.z,
          'qw': orientation.w,
          'trackableKey': trackableKey,
        });
    if (raw == null) return null;
    return ArCoreAnchor.fromMap(raw);
  }

  /// Re-reads an anchor's world position. ARCore keeps refining it, so this is
  /// how a target that has drifted is picked up.
  @override
  Future<ArCoreAnchor?> refreshAnchor(String id) async {
    final raw = await _channel.invokeMapMethod<String, dynamic>(
      'refreshAnchor',
      <String, dynamic>{'id': id},
    );
    if (raw == null) return null;
    return ArCoreAnchor.fromMap(raw);
  }

  @override
  Future<void> detachAnchor(String id) =>
      _channel.invokeMethod<void>('detachAnchor', <String, dynamic>{'id': id});

  /// See [ArCoreGateway.setActiveAnchor]. Fire-and-forget: the host records the
  /// id and starts carrying that anchor's pose on the next pose event, so there
  /// is nothing to wait for and nothing that can fail the selection.
  @override
  Future<void> setActiveAnchor(String? id) => _channel.invokeMethod<void>(
    'setActiveAnchor',
    <String, dynamic>{'id': id},
  );

  @override
  Future<void> rescan() => _channel.invokeMethod<void>('rescan');

  @override
  Future<void> pause() async {
    await _channel.invokeMethod<void>('pause');
    _sessionRunning = false;
  }

  @override
  Future<void> resume() async {
    await _channel.invokeMethod<void>('resume');
    _sessionRunning = true;
  }

  /// Takes a real photograph through the AR session.
  ///
  /// The bytes are ARCore's own camera image, encoded to JPEG on the raw YUV
  /// planes: no Flutter widget, no GLB marker and no overlay is in the frame.
  @override
  Future<ArCoreStill> takeStill({int quality = 95}) async {
    final raw = await _channel.invokeMapMethod<String, dynamic>(
      'takeStill',
      <String, dynamic>{'quality': quality},
    );
    if (raw == null) {
      throw const ArCoreFailure('The AR session produced no camera image.');
    }
    final jpeg = raw['jpeg'] as Uint8List?;
    if (jpeg == null) {
      throw const ArCoreFailure('The AR session produced no camera image.');
    }
    return ArCoreStill(
      jpeg: jpeg,
      width: (raw['imageWidth'] as num?)?.toInt() ?? 0,
      height: (raw['imageHeight'] as num?)?.toInt() ?? 0,
      sourceWidth: (raw['sourceImageWidth'] as num?)?.toInt() ?? 0,
      sourceHeight: (raw['sourceImageHeight'] as num?)?.toInt() ?? 0,
      rotationAppliedDeg: (raw['rotationAppliedDeg'] as num?)?.toInt() ?? 0,
      capturePose: parsePose(raw),
    );
  }

  /// Stops the session and destroys the texture.
  @override
  Future<void> stop() async {
    await _channel.invokeMethod<void>('stop');
    _sessionRunning = false;
  }

  /// The session's event stream: poses, the measured image orientation, and
  /// session errors.
  @override
  Stream<ArCoreEvent> get events {
    return _stream ??= _events
        .receiveBroadcastStream()
        .map((dynamic raw) => ArCoreEvent.fromMap(raw as Map<dynamic, dynamic>))
        .where((ArCoreEvent e) => e.type != ArCoreEventType.unknown);
  }
}

/// Why AR cannot run, before a session is even attempted.
class ArCoreAvailability {
  final bool supported;
  final bool serviceInstalled;
  final String? reason;

  const ArCoreAvailability({
    required this.supported,
    this.serviceInstalled = false,
    this.reason,
  });

  const ArCoreAvailability.unavailable(String message)
    : supported = false,
      serviceInstalled = false,
      reason = message;

  bool get usable => supported && serviceInstalled;
}

class ArCoreInstallResult {
  final bool installed;
  final String? status;
  const ArCoreInstallResult({required this.installed, this.status});
}

class ArCoreSession {
  final int textureId;

  /// Opaque identity of the native ARCore world, when the host reports one.
  ///
  /// It persists across `Session.pause()` / `resume()` and changes only when the
  /// Kotlin host constructs a new [Session]. A missing value is deliberately
  /// tolerated for compatibility with an older host.
  final String? sessionId;

  /// Each flag is what the host configured, i.e. what it *can* do — not what it
  /// has found. "Plane detection is enabled" and "a plane has been seen" are
  /// different facts and only the second one may unlock a button.
  final bool planes;
  final bool anchors;
  final bool cameraImage;
  final bool depth;
  final bool reused;

  const ArCoreSession({
    required this.textureId,
    required this.planes,
    required this.anchors,
    required this.cameraImage,
    required this.depth,
    this.sessionId,
    this.reused = false,
  });

  bool get canAnchor => planes && anchors;
  bool get canShoot => cameraImage;
}

/// The result of probing one direction for a surface, without creating an
/// anchor.
///
/// Deliberately not an [ArRaycastOutcome]: a probe has no world position and no
/// normal, because nothing is going to be placed. Giving it those fields would
/// invite code to anchor to a hit that was only ever a question.
class SurfaceProbe {
  final bool found;

  /// Distance to the surface along the ray, when one was found.
  final double? distanceM;

  /// `PLANE_VERTICAL`, `PLANE_HORIZONTAL_UPWARD_FACING`, …
  final String? surfaceType;

  const SurfaceProbe.found({this.distanceM, this.surfaceType}) : found = true;
  const SurfaceProbe.miss()
    : found = false,
      distanceM = null,
      surfaceType = null;

  /// A short human description: `wall · 2.4 m`.
  String get describe {
    if (!found) return 'no surface';
    final kind = switch (surfaceType) {
      'PLANE_VERTICAL' => 'wall',
      'PLANE_HORIZONTAL_UPWARD_FACING' => 'floor',
      'PLANE_HORIZONTAL_DOWNWARD_FACING' => 'ceiling',
      _ => 'surface',
    };
    final d = distanceM;
    return d == null ? kind : '$kind · ${d.toStringAsFixed(1)} m';
  }
}

/// Why a raycast found nothing.
enum ArRaycastMiss {
  /// ARCore returned no hits at all: nothing trackable along the ray.
  noCandidates,

  /// ARCore returned hits, but none was a plane within range.
  noPlaneInRange,

  /// The host did not answer.
  unknown;

  /// What to tell the user. Deliberately different per cause: "move the phone"
  /// and "point at that wall" are not the same instruction.
  String get advice => switch (this) {
    ArRaycastMiss.noCandidates =>
      'No surface along this direction yet. Move the phone slowly across the '
          'room so AR can map it, then scan again.',
    ArRaycastMiss.noPlaneInRange =>
      'AR sees surfaces, but none along this direction within range. Point the '
          'phone at that wall, floor or ceiling and scan again.',
    ArRaycastMiss.unknown => 'The surface lookup did not complete. Try again.',
  };
}

/// The result of a raycast: a hit, or a miss with its cause.
class ArRaycastOutcome {
  final ArRaycastHit? hit;
  final ArRaycastMiss? miss;

  /// How many raw hits ARCore returned, when it reported a miss.
  final int candidates;

  const ArRaycastOutcome.hit(ArRaycastHit this.hit)
    : miss = null,
      candidates = 0;

  const ArRaycastOutcome.miss(ArRaycastMiss this.miss, {this.candidates = 0})
    : hit = null;

  bool get isHit => hit != null;
}

/// Reads a number that must not poison arithmetic when it is not finite.
///
/// Same failure as in `arcore_payload.dart`: a platform channel can deliver
/// `double.nan`, and a NaN that reaches a rotation or a normal is silently
/// wrong rather than visibly broken. Here the contract is the file's existing
/// one — a missing value falls back — so a non-finite one falls back too,
/// instead of becoming a NaN quaternion in [ArRaycastHit.planeOrientation].
double _finiteOr(Object? v, double fallback) =>
    v is num && v.toDouble().isFinite ? v.toDouble() : fallback;

/// Reads a non-negative native continuity epoch without allowing a malformed
/// EventChannel value to manufacture a recovery boundary.
///
/// Older Kotlin hosts do not send this key, so `null` deliberately means
/// "continuity information unavailable", not epoch zero.  The codec normally
/// supplies an [int], but accepting an integral [num] or decimal-free [String]
/// keeps the bridge tolerant of host-version and test-payload differences.
int? _continuityEpochOrNull(Object? value) {
  final int? epoch;
  if (value is int) {
    epoch = value;
  } else if (value is num &&
      value.toDouble().isFinite &&
      value.toDouble() == value.toDouble().truncateToDouble()) {
    epoch = value.toInt();
  } else if (value is String) {
    epoch = int.tryParse(value.trim());
  } else {
    epoch = null;
  }
  return epoch != null && epoch >= 0 ? epoch : null;
}

/// Reads the native session identity conservatively.
///
/// The token has no mathematical meaning: it is an opaque native-session
/// provenance value.  Empty or malformed values must remain unavailable rather
/// than accidentally matching a station that was calibrated in another world.
String? _sessionIdOrNull(Object? value) {
  if (value is! String) return null;
  final id = value.trim();
  return id.isEmpty ? null : id;
}

/// Reads the native transition marker conservatively.
///
/// A malformed or absent value is `false`: only an explicit transition may
/// cause the capture controller to enter its recovery path.
bool _trackingTransitionOrFalse(Object? value) {
  if (value is bool) return value;
  if (value is num) return value.isFinite && value == 1;
  if (value is String) {
    final normalized = value.trim().toLowerCase();
    return normalized == 'true' || normalized == '1';
  }
  return false;
}

/// A display projection matrix emitted by the ARCore renderer.
///
/// It is deliberately parsed defensively: a malformed native event must fall
/// back to the intrinsics-based guidance projection, never leak NaN into a
/// Flutter [Canvas].
Float64List? _parseDisplayProjectionMatrix(Object? raw) {
  if (raw is! List || raw.length != 16) return null;
  final matrix = Float64List(16);
  for (var index = 0; index < matrix.length; index++) {
    final value = raw[index];
    if (value is! num || !value.toDouble().isFinite) return null;
    matrix[index] = value.toDouble();
  }
  return matrix;
}

/// A raycast result, in ARCore world metres.
class ArRaycastHit {
  final Vec3 worldPosition;

  /// The plane's orientation as ARCore reported it.
  final Quat planeOrientation;
  final double distanceM;
  final String planeType;

  /// Opaque handle to the plane that was hit, so the anchor can be attached to
  /// that plane rather than to the world.
  final String? trackableKey;
  final ({double x, double z})? extent;

  const ArRaycastHit({
    required this.worldPosition,
    required this.planeOrientation,
    required this.distanceM,
    required this.planeType,
    this.trackableKey,
    this.extent,
  });

  /// The plane's outward normal in world space.
  Vec3 get planeNormal => planeOrientation.rotate(Vec3.unitY);

  bool get isFloor => planeType.contains('UPWARD');
  bool get isCeiling => planeType.contains('DOWNWARD');
  bool get isWall => planeType.contains('VERTICAL');

  factory ArRaycastHit.fromMap(Map<Object?, Object?> m) {
    final parsed = parsePlaneHit(m)!;
    return ArRaycastHit(
      worldPosition: parsed.worldPosition,
      planeOrientation: Quat(
        _d(m['planeQx']),
        _d(m['planeQy']),
        _d(m['planeQz']),
        _d(m['planeQw']),
      ).normalized,
      distanceM: parsed.distanceM,
      planeType: parsed.planeType,
      trackableKey: m['trackableKey'] as String?,
      extent: parsed.extent,
    );
  }

  static double _d(Object? v) => _finiteOr(v, 0.0);
}

class ArCoreAnchor {
  final String id;
  /// Native ARCore session that owns this anchor, when the host reports it.
  final String? sessionId;
  final Vec3 worldPosition;
  final TrackingState tracking;

  const ArCoreAnchor({
    required this.id,
    required this.worldPosition,
    required this.tracking,
    this.sessionId,
  });

  /// Reads a native anchor payload.
  ///
  /// Re-implemented rather than routed through `parseAnchor`, which produces the
  /// older `ArAnchorHandle` type for the pre-existing sensor flow; this module
  /// keeps its own handle so the two never get confused.
  factory ArCoreAnchor.fromMap(Map<Object?, Object?> m) {
    final id = m['id'] as String?;
    final p = parseVec3(m);
    if (id == null || p == null) {
      return ArCoreAnchor(
        id: '',
        worldPosition: Vec3.zero,
        tracking: TrackingState.paused,
        sessionId: _sessionIdOrNull(m['sessionId']),
      );
    }
    return ArCoreAnchor(
      id: id,
      worldPosition: p,
      tracking: parseTracking(m['tracking'] as String?),
      sessionId: _sessionIdOrNull(m['sessionId']),
    );
  }

  bool get isUsable => id.isNotEmpty && tracking == TrackingState.tracking;
}

/// A photograph taken through the AR session.
class ArCoreStill {
  final Uint8List jpeg;
  /// Dimensions of the encoded JPEG after image-to-display rotation.
  final int width;
  final int height;

  /// Dimensions of the source ARCore CPU image before rotation. These are the
  /// dimensions its camera intrinsics refer to.
  final int sourceWidth;
  final int sourceHeight;

  /// How far the raw sensor image was rotated before encoding, in degrees.
  int rotationAppliedDeg;

  /// The ARCore camera pose at the moment the image was acquired.
  ///
  /// Raw ARCore values, like every other [ArPose]; use
  /// `capturePose.captureWorld` for this project's convention. `null` when the
  /// host is older than the metadata fields, in which case the caller cannot
  /// validate the capture pose and must not pretend it did.
  final ArPose? capturePose;

  ArCoreStill({
    required this.jpeg,
    required this.width,
    required this.height,
    required this.sourceWidth,
    required this.sourceHeight,
    required this.rotationAppliedDeg,
    this.capturePose,
  });
}

/// One mesh instance to draw, attached to an ARCore anchor.
class ArSceneDraw {
  final String meshId;
  final String anchorId;

  /// Mesh-local rotation, as `(x, y, z, w)`.
  final Quat localRotation;
  final double scale;
  final Vec3 offset;
  final ({double r, double g, double b, double a}) tint;
  final bool useTint;
  final double opacity;

  /// Hide the mesh while its anchor is not tracking, rather than drawing it at
  /// a stale pose.
  final bool requireTracking;

  /// The engine's **authoritative** local basis for this instance, column-major
  /// `[X | Y | Z | worldPosition]`, or `null` to let the host fall back to
  /// [localRotation].
  ///
  /// This is `VioSphereEngine.buildModelMatrix`'s output for the target: the
  /// verified orientation that puts the model's `+Z` on the inward facing
  /// normal. Only its rotation is consumed natively — the host zeroes the
  /// translation column, because the anchor carries the world translation — so
  /// the marker follows ARCore's refined world estimate while its orientation
  /// is the one the maths layer proved correct.
  ///
  /// Emitted as a `Float32List` so the platform channel decodes it to a Kotlin
  /// `float[]`; a `Float64List` would arrive as a `double[]` the host cannot
  /// read through its float-array path.
  final Float32List? modelMatrix;

  /// Keep the translation in [modelMatrix] when this draw is expressed in the
  /// coordinate system of a shared station anchor. Per-target anchors leave
  /// this false because their anchor already supplies the translation.
  final bool useModelTranslation;

  const ArSceneDraw({
    required this.meshId,
    required this.anchorId,
    required this.localRotation,
    this.scale = 1,
    this.offset = Vec3.zero,
    this.tint = (r: 1.0, g: 1.0, b: 1.0, a: 1.0),
    this.useTint = false,
    this.opacity = 1,
    this.requireTracking = true,
    this.modelMatrix,
    this.useModelTranslation = false,
  });

  Map<String, dynamic> toMap() => <String, dynamic>{
    'meshId': meshId,
    'anchorId': anchorId,
    'localRotation': Float32List.fromList(<double>[
      localRotation.x,
      localRotation.y,
      localRotation.z,
      localRotation.w,
    ]),
    if (modelMatrix != null) 'modelMatrix': modelMatrix,
    'useModelTranslation': useModelTranslation,
    'scale': scale,
    'offsetX': offset.x,
    'offsetY': offset.y,
    'offsetZ': offset.z,
    'tintR': tint.r,
    'tintG': tint.g,
    'tintB': tint.b,
    'tintA': tint.a,
    'useTint': useTint,
    'opacity': opacity,
    'requireTracking': requireTracking,
  };
}

enum ArCoreEventType { pose, imageOrientation, error, unknown }

/// One event from the AR session.
class ArCoreEvent {
  final ArCoreEventType type;
  final ArPoseSample? pose;
  final ({double x, double y})? imageAxis;
  final String? message;

  /// The first stack frame of a native failure, when the host reported one.
  ///
  /// This is what turns "Must use a native ord…" into
  /// `java.lang.IllegalArgumentException: Must use a native order direct buffer`
  /// plus the exact Kotlin line that threw it.
  final String? frame;

  /// Whether the failure stops the AR view from working at all.
  final bool fatal;

  const ArCoreEvent._(
    this.type, {
    this.pose,
    this.imageAxis,
    this.message,
    this.frame,
    this.fatal = false,
  });

  /// A pose event. Public so a test can push samples without a device; the
  /// platform path goes through [ArCoreEvent.fromMap].
  const ArCoreEvent.pose(ArPoseSample sample)
    : this._(ArCoreEventType.pose, pose: sample);

  /// A native failure. Public for the same reason as [ArCoreEvent.pose]: the
  /// fatal-render-error path has to be testable without a device.
  const ArCoreEvent.fault(String message, {String? frame, bool fatal = true})
    : this._(
        ArCoreEventType.error,
        message: message,
        frame: frame,
        fatal: fatal,
      );

  factory ArCoreEvent.fromMap(Map<dynamic, dynamic> raw) {
    final name = raw['type'] as String?;
    switch (name) {
      case 'pose':
        final pose = parsePose(raw);
        if (pose == null) return const ArCoreEvent._(ArCoreEventType.unknown);
        return ArCoreEvent._(
          ArCoreEventType.pose,
          pose: ArPoseSample(
            raw: pose,
            planeCount: (raw['planeCount'] as num?)?.toInt() ?? 0,
            planesTracked: (raw['planesTracked'] as num?)?.toInt() ?? 0,
            walls: (raw['walls'] as num?)?.toInt() ?? 0,
            floors: (raw['floors'] as num?)?.toInt() ?? 0,
            ceilings: (raw['ceilings'] as num?)?.toInt() ?? 0,
            anchorCount: (raw['anchorCount'] as num?)?.toInt() ?? 0,
            failureReason: raw['failureReason'] as String?,
            // A host that predates continuity reporting simply leaves these
            // absent. `null` / `false` preserve the previous pose contract;
            // the controller can require the signal only when the native host
            // has actually supplied it.
            sessionId: _sessionIdOrNull(raw['sessionId']),
            trackingEpoch: _continuityEpochOrNull(raw['trackingEpoch']),
            trackingTransition: _trackingTransitionOrFalse(
              raw['trackingTransition'],
            ),
            intrinsics: raw['imageWidth'] == null ? null : parseIntrinsics(raw),
            cameraQuality: raw['stillWidth'] == null
                ? null
                : parseCameraQuality(raw),
            displayProjectionMatrix: _parseDisplayProjectionMatrix(
              raw['displayProjectionMatrix'],
            ),
            // The active anchor's live pose. Absent on the first frames (the
            // controller names the anchor after placement) and on a host that
            // does not report it, both of which fall back to the placement.
            activeAnchorPose: raw['apx'] == null
                ? null
                : (
                    position: Vec3(
                      _finiteOr(raw['apx'], 0),
                      _finiteOr(raw['apy'], 0),
                      _finiteOr(raw['apz'], 0),
                    ),
                    orientation: Quat(
                      _finiteOr(raw['aqx'], 0),
                      _finiteOr(raw['aqy'], 0),
                      _finiteOr(raw['aqz'], 0),
                      _finiteOr(raw['aqw'], 1),
                    ).normalized,
                    tracking: parseTracking(
                      raw['activeAnchorTracking'] as String?,
                    ),
                  ),
            // A position without its identity is unsafe during a target
            // switch: an event can have been queued while the prior target was
            // active. The controller checks this id before using the pose.
            activeAnchorId: raw['activeAnchorId'] as String?,
          ),
        );
      case 'imageOrientation':
        return ArCoreEvent._(
          ArCoreEventType.imageOrientation,
          imageAxis: (
            // Finite, or the documented defaults: a NaN axis would rotate
            // every saved photograph by NaN degrees.
            x: _finiteOr(raw['axisX'], 1),
            y: _finiteOr(raw['axisY'], 0),
          ),
        );
      case 'error':
        return ArCoreEvent._(
          ArCoreEventType.error,
          message: raw['message'] as String?,
          frame: raw['frame'] as String?,
          fatal: raw['fatal'] as bool? ?? false,
        );
      default:
        return const ArCoreEvent._(ArCoreEventType.unknown);
    }
  }
}

/// A camera pose sample, with both coordinate readings kept apart.
class ArPoseSample {
  /// The display-oriented camera pose ARCore reported. Its local axes match
  /// the Flutter preview (`+X` screen-right, `+Y` screen-up, `-Z` forward), so
  /// the guidance arrow and native renderer use one screen convention.
  /// Never feed [ArPose.orientation] to app maths — use [orientation].
  final ArPose raw;

  final int planeCount;
  final int planesTracked;

  /// Tracked planes by type. "0 planes" alone cannot tell a user whether the
  /// floor was found, the walls were not, or the ceiling is out of reach — and
  /// those need different instructions.
  final int walls;
  final int floors;
  final int ceilings;

  final int anchorCount;
  final String? failureReason;

  /// Opaque native [Session] identity that produced this frame.
  ///
  /// It stays constant across an ordinary `Session.pause()` / `resume()` cycle
  /// and changes when a new ARCore world is created. `null` retains compatibility
  /// with hosts that predate session-provenance reporting.
  final String? sessionId;

  /// Native ARCore tracking-continuity epoch, when the host supports it.
  ///
  /// `null` is intentional for older hosts which predate the field. A changed
  /// epoch says the pose stream crossed a tracking/relocalisation boundary; it
  /// does not by itself claim that the frozen station remains trustworthy.
  final int? trackingEpoch;

  /// True only on the native event that marks a tracking continuity boundary.
  ///
  /// This is edge-triggered native metadata. Consumers must still validate the
  /// accompanying camera pose and station anchor before restoring guidance.
  final bool trackingTransition;

  /// The sensor intrinsics, on the event that carried them.
  final ArCameraIntrinsics? intrinsics;

  /// ARCore's display-geometry-aware camera projection for this pose event.
  ///
  /// This maps the display-oriented camera frame into the same viewport the
  /// native renderer uses. It is optional for compatibility with older hosts;
  /// callers fall back to [intrinsics] when absent.
  final Float64List? displayProjectionMatrix;

  /// The **current** pose of the active target's anchor, straight from the
  /// native render loop, or `null` when the host has none for this frame.
  ///
  /// This is the position the renderer draws the marker at — the same
  /// `anchor.pose` the render loop reads every frame — so aiming with it means
  /// the guidance and the marker cannot disagree about where the target is.
  /// [AnchoredTarget.worldPosition] remains the fallback for the frames before
  /// the host reports one and for hosts that do not report it at all.
  final ({Vec3 position, Quat orientation, TrackingState tracking})?
  activeAnchorPose;

  /// The native anchor that produced [activeAnchorPose]. A pose can be queued
  /// while Dart selects the next target after a capture, so its identity is
  /// required to reject a stale update.
  final String? activeAnchorId;

  /// The camera config (still size, focus mode), on the event that carried it.
  /// Present on exactly one event per session: the first after the host read
  /// ARCore's chosen config. `null` on every other event, and before it arrives.
  final ArCameraQuality? cameraQuality;

  const ArPoseSample({
    required this.raw,
    this.planeCount = 0,
    this.planesTracked = 0,
    this.walls = 0,
    this.floors = 0,
    this.ceilings = 0,
    this.anchorCount = 0,
    this.failureReason,
    this.sessionId,
    this.trackingEpoch,
    this.trackingTransition = false,
    this.intrinsics,
    this.displayProjectionMatrix,
    this.cameraQuality,
    this.activeAnchorPose,
    this.activeAnchorId,
  });

  Vec3 get position => raw.position;

  /// The orientation in this project's convention: `q·ẑ` is the optical axis,
  /// `q·ŷ` is up, and `−q·x̂` is screen-right.
  Quat get orientation => raw.captureWorld;

  TrackingState get tracking => raw.tracking;
}

/// A failure reported by the AR host, with the message the UI shows.
class ArCoreFailure implements Exception {
  final String message;
  const ArCoreFailure(this.message);

  @override
  String toString() => 'ArCoreFailure($message)';
}
