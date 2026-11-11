import 'dart:math' as math;

import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart'
    show VioSphereConstants;
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/scalars.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';

/// The 26 spherical capture directions of the VIO sphere specification.
///
/// The layout is fixed by [VioSphereConstants] and is **not** adapted to
/// anything: 8 horizontal at 0° every 45°, 8 upper at +45° every 45°, 8 lower
/// at −45° every 45°, one zenith at +90°, one nadir at −90°.
///
/// ```
/// index  0 –  7   Horizontal   elevation   0°   azimuth  0° … 315°
/// index  8 – 15   Upper        elevation +45°   azimuth  0° … 315°
/// index 16 – 23   Lower        elevation −45°   azimuth  0° … 315°
/// index 24        Zenith       elevation +90°
/// index 25        Nadir        elevation −90°
/// ```
///
/// **The sequence starts at the FRONT target** — the forward horizontal
/// direction of the calibrated reference — because that is where the user is
/// already looking when they press Calibrate, and it is the index the
/// layout puts first.
///
/// The two poles sit at the **exact** ±90°, where yaw is mathematically
/// undefined. That is safe everywhere they are consumed: the direction vector
/// is exactly ±Y, `atan2(0, 0)` returns 0 rather than NaN, and every piece of
/// guidance recognises a pole by pitch alone (`CaptureTarget.isPole`,
/// `AlignmentCalculator.isPole`) and offers vertical-only wording.
///
/// Directions are **station-local** and produced by `directionFromYawPitch`,
/// i.e. `x = sin(yaw)·cos(pitch)`, `y = sin(pitch)`, `z = cos(yaw)·cos(pitch)`
/// with degrees converted to radians first. In the station frame `+X` is right,
/// `+Y` up and `+Z` forward, so a positive yaw is a direction to the user's
/// right — see `station_frame.dart` for why that needs saying.
///
/// That station-local direction, pushed through `StationFrame.localToWorld`, is
/// **the same world vector** `VioSphereEngine.buildNodeSphere` computes for the
/// same azimuth/elevation, so the placement here and the engine's frozen
/// positions cannot disagree.

/// The horizontal ring's eight azimuths, in degrees: every 45°, front first.
const List<double> kCaptureYawsDeg = <double>[
  0,
  45,
  90,
  135,
  180,
  -135,
  -90,
  -45,
];

/// The upper and lower rings' eight azimuths, in degrees: every 45°.
const List<double> kCaptureRingYawsDeg = <double>[
  0,
  45,
  90,
  135,
  180,
  -135,
  -90,
  -45,
];

/// The elevation of each ring: upper, horizontal, lower.
const List<double> kCaptureRingPitchesDeg = <double>[
  VioSphereConstants.upperElevationDeg,
  0,
  VioSphereConstants.lowerElevationDeg,
];

const String kUpperRingName = VioSphereConstants.upperRingName;
const String kMiddleRingName = VioSphereConstants.horizontalRingName;
const String kLowerRingName = VioSphereConstants.lowerRingName;
const String kZenithRingName = VioSphereConstants.zenithRingName;
const String kNadirRingName = VioSphereConstants.nadirRingName;

/// 1-based target ids of the two poles.
const int kZenithTargetId = VioSphereConstants.zenithIndex + 1;
const int kNadirTargetId = VioSphereConstants.nadirIndex + 1;

/// How many targets a complete station has.
const int kCaptureTargetCount = VioSphereConstants.nodeCount;

/// One planned direction, before any station or surface exists.
class CaptureTargetSpec {
  /// 1-based, 1…26.
  final int targetId;

  final double yawDegrees;
  final double pitchDegrees;

  /// Unit direction in station-local coordinates.
  final Vec3 direction;

  /// `Horizontal`, `Upper`, `Lower`, `Zenith` or `Nadir`.
  final String ringName;

  /// 1-based position within [ringName]; the two near-poles have no ring
  /// position and leave it `null`.
  final int? indexInRing;

  final bool isPole;
  final bool isZenith;
  final bool isNadir;

  const CaptureTargetSpec({
    required this.targetId,
    required this.yawDegrees,
    required this.pitchDegrees,
    required this.direction,
    required this.ringName,
    this.indexInRing,
    this.isPole = false,
    this.isZenith = false,
    this.isNadir = false,
  });

  /// A short label for the target chip, e.g. `T07 · yaw 90° pitch 0°`.
  String get label =>
      'T${targetId.toString().padLeft(2, '0')} · '
      '${yawDegrees.toStringAsFixed(0)}° / ${pitchDegrees.toStringAsFixed(0)}°';

  /// A name the guidance can say out loud: "Aim at the front target".
  ///
  /// The front target is the one at yaw 0 / pitch 0 — the forward horizontal
  /// direction of the capture reference. The poles are named for what they
  /// are; everything else is referred to by id, because "turn right 137°" is
  /// the useful instruction for a direction nobody has a word for.
  String get shortName {
    if (isZenith) return 'the zenith target';
    if (isNadir) return 'the nadir target';
    if (yawDegrees.abs() < 1e-9 && pitchDegrees.abs() < 1e-9) {
      return 'the front target';
    }
    return 'target $targetId';
  }
}

/// Builds the fixed 26-direction sphere.
///
/// Pure and deterministic, so the id→angle mapping is asserted once in the test
/// suite rather than re-derived per station.
///
/// The order is the capture order and the layout's index order:
/// **front first** across the horizontal ring, then the upper ring, then the
/// lower ring, then the two poles. Front first because the user is facing
/// that way when they calibrate, so the first photograph is the one they are
/// already looking at.
///
/// The counts, steps and elevations come from [VioSphereConstants], so the spec
/// list and `VioSphereEngine.buildNodeSphere` are driven by one set of
/// numbers and cannot drift apart.
List<CaptureTargetSpec> buildCaptureSphere() {
  final out = <CaptureTargetSpec>[];
  var id = 0;

  void add(
    double yaw,
    double pitch,
    String ringName, {
    int? indexInRing,
    bool pole = false,
    bool zenith = false,
    bool nadir = false,
  }) {
    id++;
    out.add(
      CaptureTargetSpec(
        targetId: id,
        yawDegrees: yaw,
        pitchDegrees: pitch,
        direction: directionFromYawPitch(yaw, pitch),
        ringName: ringName,
        indexInRing: indexInRing,
        isPole: pole,
        isZenith: zenith,
        isNadir: nadir,
      ),
    );
  }

  // Horizontal ring: front first, then the remaining seven, every 45°.
  for (var i = 0; i < VioSphereConstants.horizontalCount; i++) {
    add(kCaptureYawsDeg[i], 0, kMiddleRingName, indexInRing: i + 1);
  }
  // Upper ring, every 45°.
  for (var i = 0; i < VioSphereConstants.upperCount; i++) {
    add(
      kCaptureRingYawsDeg[i],
      kCaptureRingPitchesDeg[0],
      kUpperRingName,
      indexInRing: i + 1,
    );
  }
  // Lower ring, every 45°.
  for (var i = 0; i < VioSphereConstants.lowerCount; i++) {
    add(
      kCaptureRingYawsDeg[i],
      kCaptureRingPitchesDeg[2],
      kLowerRingName,
      indexInRing: i + 1,
    );
  }
  // The two poles close the sequence.
  add(
    0,
    VioSphereConstants.zenithElevationDeg,
    kZenithRingName,
    pole: true,
    zenith: true,
  );
  add(
    0,
    VioSphereConstants.nadirElevationDeg,
    kNadirRingName,
    pole: true,
    nadir: true,
  );
  return out;
}

/// Configuration for how a target direction is turned into a real target.
///
/// ## Placement: computed, not discovered
///
/// A target's world position is **arithmetic**, not the result of a raycast:
///
/// ```
/// targetWorldPosition = stationWorldPosition + targetRadiusM × direction
/// ```
///
/// No wall, floor or ceiling is consulted, and none is required: a room ARCore
/// has not mapped yet, an empty unit, or a user pointing at open air all place
/// the target identically. ARCore anchors the computed pose in world space and
/// keeps it fixed as the phone moves.
///
/// [rayDistanceM] survives for one purpose only: the **informational probe**
/// ("how many of the 26 directions happen to have a surface"), which the target
/// sheet shows as context. It is the *maximum length of the ray* cast from the
/// station origin along a target direction. Nothing is placed from its result,
/// and capture never waits for it — the brief is explicit that surface
/// detection must not be a prerequisite, so a raycast that misses changes no
/// behaviour at all.
class CaptureGeometryConfig {
  /// How far in front of the station every target sits, in metres.
  ///
  /// **1.0 m, the specification's value.** Drawn at [targetDiameterM]
  /// (0.22 m) the marker subtends `2·atan(0.11/1.0) ≈ 12.6°` here — large
  /// enough to aim at without being so close that turning towards it exits the
  /// frame. It is a per-station constant, deliberately: the 26 targets are
  /// directions, and a direction plus a radius is a position.
  final double targetRadiusM;

  /// Maximum probe raycast length, in metres. Informational only — see the
  /// class doc.
  final double rayDistanceM;

  /// How far off the ray a hit may be and still count as "this direction", in
  /// metres. Zero: ARCore returns the closest hit along the ray.
  final double hitToleranceM;

  /// Scale applied to the GLB marker when it is drawn at an anchor.
  ///
  /// **A multiplier on the physical size, not the physical size itself.**
  /// [markerScaleFor] derives the draw scale from the asset's measured width
  /// and [targetDiameterM]; this stays as a fine-tune (1.0 = exactly the
  /// configured diameter). The old code used it as the whole story — a bare
  /// `1.0` on a plate authored 0.54 m across, which at 1.0 m subtends ~30° and
  /// filled half the camera view.
  final double markerScale;

  /// The physical diameter of the target plate, in metres.
  ///
  /// 0.22 m at the 1.0 m capture radius subtends `2·atan(0.11/1.0) ≈ 12.6°` —
  /// about a fifth of a phone's vertical field of view: unmistakable to aim
  /// at, nowhere near dominating the frame. The authored asset is 0.54 m
  /// across, so the draw scale is `0.22 / 0.54 ≈ 0.41`; the bezel is scaled by
  /// the same factor, which keeps the two concentric and coplanar exactly as
  /// they were authored.
  final double targetDiameterM;

  /// How far the bezel is pushed behind the target plate, in metres.
  ///
  /// **Derived from the assets, not tuned.** They are concentric and meet:
  ///
  /// ```
  /// plate : disc   r 0 → 0.270,  z 0 → 0.006
  /// bezel : torus  r 0.270 → 0.350, z 0 → 0.012
  /// ```
  ///
  /// The ring surrounds the plate and abuts it on `r = 0.270`; they never overlap
  /// in x/y. But recessing the ring pushes it away from the viewer, so it
  /// projects *smaller* than the plate in front of it — and the nearer plate then
  /// overlaps the ring's inner edge. The error crosses zero at exactly one value:
  ///
  /// ```
  /// 0.020 → plate overlaps the ring by 3.8 mm
  /// 0.014 → overlaps by 2.2 mm
  /// 0.006 → exact: neither gap nor overlap
  /// 0.002 → a 1.1 mm ring of daylight
  /// ```
  ///
  /// 0.006 is that crossover — and it is also `0.012 − 0.006`, the difference
  /// between the two front faces, because at the crossover they are coplanar.
  ///
  /// The legacy 2D overlay uses the same value for the same reason.
  final double bezelSetbackM;

  /// Alignment tolerance, in degrees.
  ///
  /// The shutter requires this strictly: the 3-D error at the instant of
  /// capture must be inside it.
  final double alignmentToleranceDeg;

  /// How far past [alignmentToleranceDeg] the error may drift before the
  /// stability window is **thrown away**, in degrees.
  ///
  /// The window and the shutter ask different questions. The shutter asks "is
  /// this frame inside tolerance right now?" — a strict, instantaneous test.
  /// The window asks "has the user been holding it?" — and on a real phone the
  /// raw error wanders a degree or two around the tolerance while the user
  /// holds perfectly still. Refusing the window on every excursion above the
  /// strict bound is what left the app stuck on "Hold still…" forever; with
  /// hysteresis the window survives hand tremor and only a genuine turn breaks
  /// it.
  final double alignmentHysteresisDeg;

  /// How far the orientation may wander across the window, in degrees.
  ///
  /// The VIO capture tier permits at most 1.5° across the full 600 ms hold.
  /// This remains independent of the instantaneous target-alignment limit so
  /// a slow turn cannot be accepted merely because it ends on the target.
  final double maxOrientationDriftDeg;

  /// How long the pose must stay inside tolerance before the shutter may fire.
  ///
  /// **600 ms, the specification's value**, enforced by
  /// `VioSphereEngine.evaluateFrame`'s stability window. Long enough that a hand
  /// tremor cannot cross it, short enough that holding still does not feel like
  /// waiting.
  final Duration stabilityWindow;

  /// Display smoothing time constant, in seconds. See
  /// `AdaptiveQuaternionSmoother` for what it does and does not buy.
  final double smootherTauSeconds;

  /// A display jump larger than this bypasses smoothing, in degrees. A
  /// deliberate turn or a re-acquisition must not be lagged.
  final double smootherSnapAngleDeg;

  /// The fastest rotation that still counts as "held still", in degrees/second.
  final double maxAngularVelocityDegPerSec;

  /// The oldest reading that may still be acted on.
  final Duration maxMeasurementAge;

  /// The baseline the derived angular rate is measured over. See
  /// `StabilityMonitor.rateBaseline` for why it is not the frame interval.
  final Duration rateBaseline;

  /// How far the phone may wander in position across the window, in metres.
  final double maxPositionDriftM;

  /// The largest gap between two consecutive poses that may still count as a
  /// continuous hold.
  ///
  /// ARCore's pose stream runs at roughly 30–60 Hz, so 150 ms is four to nine
  /// missed frames: long enough to tolerate a scheduling stall, short enough
  /// that a dropped stretch cannot be interpolated over and then claimed as a
  /// 600 ms hold. Without it, two samples 649 ms apart would "span the window"
  /// while containing almost no evidence.
  final Duration maxSampleGap;

  /// The maximum time allowed to re-establish a retained station reference.
  ///
  /// This is a failure deadline, never evidence of recovery: the controller
  /// still requires the original session and shared anchor to report fresh
  /// `TRACKING` samples for its verification hold. It is configurable so device
  /// testing can tune the user-facing wait without changing capture geometry.
  final Duration stationRecoveryTimeout;

  const CaptureGeometryConfig({
    this.targetRadiusM = VioSphereConstants.targetRadiusM,
    this.rayDistanceM = 8.0,
    this.hitToleranceM = 0.0,
    this.markerScale = 1.0,
    this.targetDiameterM = 0.22,
    this.bezelSetbackM = 0.006,
    this.alignmentToleranceDeg = VioSphereConstants.maxAlignmentErrorDeg,
    this.alignmentHysteresisDeg = 2.0,
    this.stabilityWindow = VioSphereConstants.stabilityWindow,
    this.maxOrientationDriftDeg =
        VioSphereConstants.maxWindowOrientationDriftDeg,
    this.maxPositionDriftM = VioSphereConstants.maxWindowPositionDriftM,
    this.maxSampleGap = VioSphereConstants.maxSampleGap,
    this.smootherTauSeconds = 0.12,
    this.smootherSnapAngleDeg = 25,
    this.maxAngularVelocityDegPerSec =
        VioSphereConstants.maxAngularVelocityDegPerSec,
    this.maxMeasurementAge = VioSphereConstants.maxPoseAge,
    this.rateBaseline = VioSphereConstants.rateBaseline,
    this.stationRecoveryTimeout = const Duration(seconds: 15),
  });

  /// The draw scale for a plate of [modelWidthM] authored metres.
  ///
  /// Normalises the model to the configured physical diameter instead of
  /// applying a blind constant: `0.22 / 0.54 ≈ 0.407` for the shipped asset.
  /// The bezel uses the same number, so the ring keeps its authored
  /// relationship to the plate.
  double markerScaleFor(double modelWidthM) {
    if (modelWidthM <= 0) return markerScale;
    return (targetDiameterM / modelWidthM) * markerScale;
  }
}

/// Where one target actually is, in the AR world.
///
/// **The single authority for target placement.** The AR scene and the tests
/// both read this; the sphere map projects the same [CaptureTargetSpec.direction]
/// through the same station frame, so a marker on the map and its GLB in the
/// world are the same target by construction rather than by coincidence.
///
/// The position is `station.origin + radius × station.localToWorld(direction)`:
/// gravity-aligned, frozen at calibration, and never derived from the live
/// camera pose.
({Vec3 worldPosition, Vec3 facingNormal, Quat localRotation}) resolveTarget({
  required StationFrame station,
  required CaptureTargetSpec spec,
  required double radiusM,
}) {
  final worldDirection = station.localToWorld(spec.direction).normalized;
  // The plate's face points back at the station: the user standing there is
  // who aims at it, however far they step away afterwards.
  final facing = (-worldDirection).normalized;
  return (
    worldPosition: station.origin + worldDirection * radiusM,
    facingNormal: facing,
    localRotation: facingRotation(facing),
  );
}

/// The rotation that takes the model's `+Z` (the plate's face normal) onto
/// [normal].
///
/// `Quat.axisAngle` about `ẑ × n`, by the angle between them. `+Z → −Z` is the
/// degenerate case (an anti-parallel normal), where any perpendicular axis gives
/// a valid 180° rotation; `x̂` is chosen so the result is deterministic.
///
/// Pure and static because it is the one piece of orientation maths in the whole
/// anchoring path, and it is the piece that decides whether the marker faces the
/// user or the inside of the wall.
///
/// The `+Z` face is **verified against the shipped asset**, not assumed:
/// `ar_capture_target.glb` is a disc in the `XY` plane with its thickness along
/// `Z` — the front face sits at `+0.006`, and the bezel's own front face at
/// `+0.012` is what the `bezelSetbackM` of `0.006` makes coplanar with it. Both
/// are parsed from the GLB by `GlbMeshSource`, and `docs/` records the
/// measurement. A model whose face were on `−Z` would need the negation here.
Quat facingRotation(Vec3 normal) {
  final n = normal.normalized;
  if (n.length == 0) return Quat.identity;
  final dot = clampDouble(Vec3.unitZ.dot(n), -1, 1);
  // 1e-9 rather than an exact compare: n comes from a tracked plane and is never
  // exact.
  if (dot > 1 - 1e-9) return Quat.identity;
  if (dot < -1 + 1e-9) return Quat.axisAngle(180, Vec3.unitX);
  final axis = Vec3.unitZ.cross(n);
  final angleDeg = _acosDeg(dot);
  return Quat.axisAngle(angleDeg, axis.normalized);
}

double _acosDeg(double v) {
  // atan2 form, stable at both ends of the range.
  final s = math.sqrt(math.max(0, 1 - v * v));
  return math.atan2(s, v) * radToDeg;
}
