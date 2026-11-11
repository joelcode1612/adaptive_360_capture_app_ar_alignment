import 'dart:async';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/attitude_fusion.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/scalars.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_sources.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The pose source for a device with no ARCore: accelerometer plus gyroscope.
///
/// ## What it provides, and what it cannot
///
/// It estimates the phone's **attitude** — which way it is pointing — and nothing
/// else. There is no position, no plane and no anchor, so station displacement
/// diagnostics are unavailable and [ArCapabilities.supported] is `false`. The tier ladder reads exactly
/// that and reports [ArTier.poseOnly] with the "No AR — using the motion
/// sensors" headline, which is the outcome the brief asks for when real AR is
/// not available: a clearly labelled sensor-guided prototype, not a claim of AR
/// tracking.
///
/// ## Why the stored orientation is the device rotation, unconverted
///
/// `ArPose.orientation` is documented as ARCore's camera-local → world rotation:
/// `+X` right, `+Y` up the screen, `+Z` out of the *back* of the phone. That is
/// the Android sensor frame, so the estimate lands in the same convention by
/// construction, and [ArPose.captureWorld] applies the one audited conversion —
/// the same `Ry(180°)` it applies to a real ARCore pose. Nothing in the guidance
/// path learns which source it got.
///
/// The estimator itself is [AttitudeFusion], in the domain layer. It is not in
/// this file on purpose: this file imports `sensors_plus`, and the plain-`dart`
/// harness that checks every other piece of geometry in the project cannot
/// import a plugin — so maths that lived only here would have had no test at
/// all. The frame derivation, why gravity is read as *up*, and why the
/// correction is applied on the right are documented there.
///
/// ## Limits, stated before they bite
///
/// Heading has no reference: gravity constrains two of three degrees of freedom,
/// so the estimate keeps whatever heading the gyro integration gave it, and that
/// drifts by the gyro's bias. It is usable because every target direction is
/// measured *relative* to the heading locked at "Set Station Here", and screen 4
/// re-locks per station rather than reusing an old heading. [ArPose.position] is
/// identically zero and must never be read as metres travelled.
///
/// Every constant here is unvalidated on hardware — this machine has no device
/// and no emulator. `DEVICE_TEST_CHECKLIST.md` measures gyro bias, the beta gain
/// and the settle behaviour on a real phone.
class SensorPoseSource implements ArPoseSource {
  /// How long a fused sample stays usable before [tracking] reports `paused`.
  final Duration staleAfter;

  /// Sampling period requested from the sensor framework.
  ///
  /// 20 ms, which is what the state machine's 50 ms tick is fed by and the same
  /// period `GyroRateSource` already asks for, so the settle gate and the
  /// attitude are not competing for a faster stream.
  final Duration samplePeriod;

  /// The estimator. Injected so a test can drive a known attitude; otherwise
  /// this is the default complementary filter.
  final AttitudeFusion fusion;

  StreamSubscription<AccelerometerEvent>? _accelSub;
  StreamSubscription<GyroscopeEvent>? _gyroSub;
  final StreamController<ArPose> _out = StreamController<ArPose>.broadcast();

  Vec3? _accel;
  Vec3? _gyro;
  int? _lastSampleUs;
  int? _lastPushUs;
  DateTime? _lastSampleAt;
  int? _lastGyroUs;
  double? _achievedHz;
  Object? _error;
  bool _failed = false;

  SensorPoseSource({
    this.staleAfter = const Duration(milliseconds: 300),
    this.samplePeriod = const Duration(milliseconds: 20),
    AttitudeFusion? fusion,
  }) : fusion = fusion ?? AttitudeFusion();

  @override
  ArCapabilities get capabilities => ArCapabilities(
    supported: false,
    reason: _failed
        ? 'The motion sensors are not reporting: ${_error ?? 'unknown fault'}.'
        : 'ARCore is not available on this device. Direction comes from the '
              'accelerometer and gyroscope; position, planes and anchors are '
              'not available.',
  );

  @override
  TrackingState get tracking {
    if (_failed || _accelSub == null) return TrackingState.unavailable;
    final at = _lastSampleAt;
    if (at == null) return TrackingState.initialising;
    if (_accel == null || _gyro == null) return TrackingState.initialising;
    if (DateTime.now().difference(at) > staleAfter) return TrackingState.paused;
    return TrackingState.tracking;
  }

  /// The current fused pose, or `null` when there is nothing to guide by.
  ///
  /// `null` across a gap rather than the last good value, for the reason the
  /// capture screen already states: a held-over attitude would let the machine
  /// call a target aligned and fire the shutter at a phone that has since moved.
  @override
  ArPose? get latest {
    if (_failed) return null;
    final t = tracking;
    if (t != TrackingState.tracking) return null;
    final g = _gyro;
    final us = _lastSampleUs;
    return ArPose(
      position: Vec3.zero,
      orientation: fusion.deviceToWorld,
      tracking: t,
      angularRateDegPerSec: g == null ? null : g.length * radToDeg,
      timestampNs: us == null ? null : us * 1000,
    );
  }

  @override
  Stream<ArPose> get poses => _out.stream;

  /// The estimate in this project's world convention, for diagnostics.
  Quat get captureWorld => fusion.captureWorld;

  /// Whether the sensors themselves failed, as distinct from being idle.
  bool get sensorsFailed => _failed;

  /// What stopped the sensors, when they did.
  Object? get error => _error;

  /// The rate the gyroscope stream is actually delivering, smoothed over the
  /// recent samples, or `null` until there are enough to say.
  ///
  /// [samplePeriod] is a *request*. Android 12+ throttles motion sensors to 5 Hz
  /// unless `HIGH_SAMPLING_RATE_SENSORS` is declared, and the throttled stream is
  /// indistinguishable from the requested one in the event type, so the only way
  /// to know which one is running is to measure the arrivals. `staleAfter` and
  /// the angular-rate thresholds are tuned for the requested rate; this is the
  /// number that says whether that tuning is being honoured.
  double? get achievedSampleHz => _achievedHz;

  @override
  Future<ArCapabilities> start() async {
    if (_accelSub != null) return capabilities;
    _failed = false;
    _error = null;
    _accel = null;
    _gyro = null;
    _lastSampleUs = null;
    _lastSampleAt = null;
    _lastGyroUs = null;
    _achievedHz = null;
    fusion.reset();

    try {
      _accelSub = accelerometerEventStream(
        samplingPeriod: samplePeriod,
      ).listen(_onAccel, onError: (Object e) => _fail(e));
      _gyroSub = gyroscopeEventStream(
        samplingPeriod: samplePeriod,
      ).listen(_onGyro, onError: (Object e) => _fail(e));
    } catch (e) {
      _fail(e);
      return capabilities;
    }

    // Wait for one reading from each sensor before reporting capabilities, so a
    // device with no gyroscope fails here — where the caller can still show the
    // manual path — rather than three screens later with an empty reticle.
    final deadline = DateTime.now().add(const Duration(milliseconds: 800));
    while (_accel == null || _gyro == null) {
      if (_failed) break;
      if (DateTime.now().isAfter(deadline)) {
        _fail(
          _accel == null
              ? 'The accelerometer reported nothing within 800 ms.'
              : 'The gyroscope reported nothing within 800 ms.',
        );
        break;
      }
      await Future<void>.delayed(const Duration(milliseconds: 20));
    }
    return capabilities;
  }

  @override
  Future<void> stop() async {
    await _accelSub?.cancel();
    await _gyroSub?.cancel();
    _accelSub = null;
    _gyroSub = null;
    _accel = null;
    _gyro = null;
    _lastSampleUs = null;
    _lastSampleAt = null;
    _lastGyroUs = null;
    _achievedHz = null;
    // Cleared, not kept: a stale pose must not survive a stop(), because the
    // state machine reads `null` as "no information" and a held-over value as
    // "the phone is still".
  }

  void _fail(Object e) {
    _failed = true;
    _error = e;
  }

  void _onAccel(AccelerometerEvent e) {
    _accel = Vec3(e.x, e.y, e.z);
    _step();
  }

  void _onGyro(GyroscopeEvent e) {
    // rad/s, as the framework reports it; the estimator converts at the point
    // where the increment becomes an angle.
    _gyro = Vec3(e.x, e.y, e.z);
    _measureGyroRate();
    _step();
  }

  /// Times the gyroscope stream on its own.
  ///
  /// Deliberately not measured in [_step]: that fuses whichever of the two
  /// streams arrived, so its inter-arrival time is the *paired* rate and would
  /// read roughly double each sensor's. The angular-rate thresholds are tuned
  /// against the gyroscope, so the gyroscope is what has to be measured.
  void _measureGyroRate() {
    final nowUs = DateTime.now().microsecondsSinceEpoch;
    final previousUs = _lastGyroUs;
    _lastGyroUs = nowUs;
    if (previousUs == null) return;
    final dtSeconds = (nowUs - previousUs) / 1000000.0;
    // A gap is a lifecycle pause, not a slow sensor; averaging it in would
    // report a stop as a rate.
    if (dtSeconds <= 0 || dtSeconds > 0.25) return;
    final hz = 1 / dtSeconds;
    final previousHz = _achievedHz;
    _achievedHz = previousHz == null ? hz : previousHz + 0.2 * (hz - previousHz);
  }

  /// Fuses whichever pair of readings is currently available.
  ///
  /// Both streams are asked for the same period, so one timestamp serves the
  /// pair, and a sample from the faster of the two fuses with the older partner
  /// value — which is what a 20 ms complementary filter does in any case.
  void _step() {
    final a = _accel;
    final g = _gyro;
    if (a == null || g == null) return;
    final now = DateTime.now();
    final nowUs = now.microsecondsSinceEpoch;
    final previousUs = _lastSampleUs;
    _lastSampleUs = nowUs;
    _lastSampleAt = now;

    final dt = previousUs == null ? 0.0 : (nowUs - previousUs) / 1000000.0;
    // A first sample, or a gap after a pause. Integrating 40 s as one step would
    // be meaningless, so the gap is dropped rather than absorbed — `staleAfter`
    // has already told the caller not to trust the attitude across it.
    if (dt <= 0 || dt > 0.25) {
      _push(nowUs);
      return;
    }

    fusion.step(accelMs2: a, gyroRadSec: g, dtSeconds: dt);
    _push(nowUs);
  }

  void _push(int atUs) {
    final last = _lastPushUs;
    if (last != null && atUs - last < 15000) return;
    _lastPushUs = atUs;
    final pose = latest;
    if (pose != null) _out.add(pose);
  }

  @override
  String toString() => 'SensorPoseSource(${tracking.name}, no anchor)';
}
