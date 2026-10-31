import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// Owns the ARCore session lifecycle and the pose stream.
///
/// Deliberately narrow: it starts and stops the session, hands out the pose
/// stream, and reports the tracking state it was actually given. Every decision
/// that depends on a pose — whether a station may be calibrated, whether a
/// surface exists, whether the shutter may fire — is made in the capture
/// controller, where it can be tested without a device.
class ArSessionService {
  ArSessionService(this._host);

  final ArCoreGateway _host;

  Stream<ArPoseSample>? _poses;
  ArCoreSession? _session;
  ArCoreAvailability? _availability;
  ArCoreInstallResult? _install;

  /// The orientation of the CPU image relative to the display, measured by the
  /// session itself and needed to write upright photographs.
  ({double x, double y}) _imageAxis = (x: 1, y: 0);
  bool _hasMeasuredImageAxis = false;

  /// The most recent pose, or `null` before the first one arrives.
  ArPoseSample? latest;

  /// The intrinsics the session last reported.
  ArCameraIntrinsics? intrinsics;

  /// The camera config the session reported: still stream size and focus mode.
  /// `null` until the first event that carries them, which arrives once, early.
  ArCameraQuality? cameraQuality;

  /// The last error the session raised, for the diagnostics panel.
  String? lastError;

  /// How many planes ARCore is tracking, from the latest pose event.
  int get planesTracked => latest?.planesTracked ?? 0;

  int get planeCount => latest?.planeCount ?? 0;

  int? get textureId => _session?.textureId;

  bool get isRunning => _session != null;

  ArCoreAvailability? get availability => _availability;

  ArCoreInstallResult? get installResult => _install;

  /// `null` until ARCore has measured the sensor-image orientation for this
  /// session.  The default axis is safe for encoding fallback, but it must not
  /// be presented to guidance as a real measurement.
  ({double x, double y})? get imageAxis =>
      _hasMeasuredImageAxis ? _imageAxis : null;

  /// How far the sensor image is rotated relative to the display, in degrees.
  ///
  /// Snapped to a quarter turn: the image→display transform is a rotation by a
  /// multiple of 90°, so any other value would be a measurement artifact.
  int get imageRotationDeg {
    final angle = math.atan2(_imageAxis.y, _imageAxis.x) * 180 / math.pi;
    return (((angle / 90).round() % 4) + 4) % 4 * 90;
  }

  TrackingState get tracking => latest?.tracking ?? TrackingState.unavailable;

  /// Names the anchor the guidance is aimed at, so the pose stream carries its
  /// current pose. See [ArCoreGateway.setActiveAnchor].
  Future<void> setActiveAnchor(String? id) => _host.setActiveAnchor(id);

  /// Whether a pose may be used to guide the user right now.
  bool get hasUsablePose =>
      latest != null && latest!.tracking == TrackingState.tracking;

  /// Probes the device without starting a session.
  ///
  /// Three separate facts, kept separate on purpose:
  /// * the host exists at all (false on iOS, where there is no Kotlin host);
  /// * ARCore is supported by this device;
  /// * the ARCore service is actually installed.
  ///
  /// Collapsing them would tell a Pixel user with an out-of-date Play services
  /// for AR that their phone cannot do AR, which is both wrong and unfixable.
  Future<ArCapabilityReport> probe() async {
    final hostPresent = await _host.isAvailable;
    if (!hostPresent) {
      _availability = const ArCoreAvailability.unavailable(
        'This build has no ARCore host. AR capture needs Android.',
      );
      return ArCapabilityReport(
        hostPresent: false,
        availability: _availability!,
        supported: false,
        installable: false,
      );
    }
    final availability = await _host.checkAvailability();
    _availability = availability;
    if (availability.supported && !availability.serviceInstalled) {
      _install = await _host.requestInstall();
      _availability = await _host.checkAvailability();
    }
    return ArCapabilityReport(
      hostPresent: true,
      availability: _availability!,
      supported: _availability!.supported,
      installable: true,
      installed: _availability!.serviceInstalled,
    );
  }

  /// Starts the session and the pose stream.
  Future<ArCoreSession> start({required int width, required int height}) async {
    final session = await _host.start(width: width, height: height);
    _session = session;
    latest = null;
    lastError = null;
    _imageAxis = (x: 1, y: 0);
    _hasMeasuredImageAxis = false;
    return session;
  }

  /// The pose stream. Every consumer shares one subscription: ARCore has one
  /// session and one frame clock, and duplicating it would double the callbacks
  /// for no benefit.
  Stream<ArPoseSample> get poses {
    return _poses ??= _host.events
        .where((ArCoreEvent e) => e.type == ArCoreEventType.pose)
        .map((ArCoreEvent e) => e.pose!)
        .map((ArPoseSample p) {
          latest = p;
          if (p.intrinsics != null) intrinsics = p.intrinsics;
          if (p.cameraQuality != null) cameraQuality = p.cameraQuality;
          return p;
        })
        .asBroadcastStream(
          onCancel: (StreamSubscription<ArPoseSample> _) {
            _poses = null;
          },
        );
  }

  /// Drains the non-pose events: the measured image orientation and session
  /// errors. Run once, alongside the pose subscription.
  Stream<ArCoreEvent> get diagnostics => _host.events
      .where((ArCoreEvent e) => e.type != ArCoreEventType.pose)
      .map((ArCoreEvent e) {
        if (e.type == ArCoreEventType.imageOrientation && e.imageAxis != null) {
          _imageAxis = e.imageAxis!;
          _hasMeasuredImageAxis = true;
        }
        if (e.type == ArCoreEventType.error) lastError = e.message;
        return e;
      });

  /// Uploads one mesh's vertex buffers to the native renderer.
  ///
  /// Done once per session, from the buffers [GlbMeshSource] produced out of the
  /// shipped `.glb` assets. The native side stores them and never reloads them,
  /// so switching targets is a scene update rather than a re-upload.
  Future<void> uploadMesh({
    required String id,
    required Float32List positions,
    required Float32List colors,
  }) => _host.setMesh(id: id, positions: positions, colors: colors);

  Future<void> resize({required int width, required int height}) =>
      _host.resize(width: width, height: height);

  /// Probes every direction against the current frame, in one host call.
  Future<List<SurfaceProbe>> probeDirections({
    required Vec3 origin,
    required List<Vec3> directions,
    double maxDistanceM = 8,
  }) => _host.probeDirections(
    origin: origin,
    directions: directions,
    maxDistanceM: maxDistanceM,
  );

  /// Re-arms surface detection and drops the host's anchors.
  ///
  /// Backs `CaptureController.rescanActive()`. It does not restart the
  /// session, so the station's
  /// reference pose — and every target direction derived from it — survives.
  Future<void> rescan() => _host.rescan();

  Future<void> setPlaneDetection(bool enabled) =>
      _host.setPlaneDetection(enabled);

  Future<void> stop() async {
    await _host.stop();
    _session = null;
    latest = null;
  }
}

/// What the device can do, before any session exists.
class ArCapabilityReport {
  /// Whether the Kotlin host is compiled into this build at all.
  final bool hostPresent;

  final ArCoreAvailability availability;
  final bool supported;
  final bool installable;
  final bool installed;

  const ArCapabilityReport({
    required this.hostPresent,
    required this.availability,
    required this.supported,
    required this.installable,
    this.installed = false,
  });

  /// Whether the guided capture flow may run at all.
  bool get usable => hostPresent && supported && installed;

  /// What to tell the user when it may not.
  String get message {
    if (!hostPresent) {
      return 'AR capture needs the Android ARCore host, which this build does '
          'not include.';
    }
    if (!supported) {
      return availability.reason ??
          'This device does not support ARCore, so guided AR capture is '
              'unavailable.';
    }
    if (!installed) {
      return 'Google Play services for AR is not installed or needs updating.';
    }
    return 'ARCore is ready.';
  }
}
