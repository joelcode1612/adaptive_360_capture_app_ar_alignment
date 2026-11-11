import 'package:adaptive_360_capture_app/features/virtual_tour/domain/environment/station_anchor.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The runtime half of the station-anchor diagnostic.
///
/// `domain/environment/station_anchor.dart` holds the geometry: given an origin
/// and a camera position, how far has the camera moved. This class holds the
/// *state* around it — plus the two cases the pure function cannot see: no
/// anchor at all, and an anchor whose own tracking has degraded.
///
/// Distance is recorded for diagnostics only. A lost anchor is different: its
/// tracking state is evidence the AR reference is unreliable, so callers can
/// pause safely until it returns.
class StationDriftGate {
  /// The station origin, or `null` when no anchor was placed.
  final StationOrigin? origin;

  StationDrift? _last;

  /// Set when the anchor's own tracking has degraded, which makes the measured
  /// distance meaningless rather than merely inaccurate.
  bool _anchorUntracked = false;

  StationDriftGate({required this.origin});

  /// A gate that is off, for the [ArTier.poseOnly] and
  /// [ArTier.unavailable] paths.
  static StationDriftGate disabled() => StationDriftGate(origin: null);

  /// Whether this diagnostic has an anchor-backed reference.
  bool get enabled =>
      origin != null && origin!.confidence != StationConfidence.manual;

  /// Whether capture should be held because the anchor itself is no longer
  /// tracked. Physical distance from the station never changes this value.
  bool get paused => _anchorUntracked;

  /// The most recent measurement, or `null` when the gate is off.
  StationDrift? get last => _last;

  /// What to show the user, or `null` when there is nothing to show.
  String? get banner {
    if (!enabled) return null;
    if (_anchorUntracked) {
      return 'The station marker lost tracking. Point the camera back at the '
          'floor where you started and move slowly.';
    }
    return null;
  }

  /// Feeds the current camera position.
  ///
  /// [anchorTracking] is the anchor's own tracking state. When it is not
  /// [TrackingState.tracking] the distance is not trustworthy, so the gate
  /// pauses and says why instead of reporting a number that looks fine.
  StationDrift? update({
    required Vec3? cameraPosition,
    TrackingState anchorTracking = TrackingState.tracking,
  }) {
    if (!enabled) return null;

    if (anchorTracking != TrackingState.tracking) {
      _anchorUntracked = true;
      _last = StationDrift(
        horizontalM: double.nan,
        totalM: double.nan,
      );
      return _last;
    }
    _anchorUntracked = false;

    if (cameraPosition == null) return _last;

    final d = evaluateStationDrift(
      origin: origin!,
      cameraPosition: cameraPosition,
    );
    _last = d;
    return d;
  }

  /// Clears the latch, e.g. after the user re-anchored at a new spot.
  void reset() {
    _anchorUntracked = false;
    _last = null;
  }

  /// Builds a [StationOrigin] from a placed anchor.
  ///
  /// [cameraHeightM] is the camera's height above the anchor's plane, which the
  /// platform supplies as the difference between the camera position and the
  /// hit point. It is recorded because the brief's station geometry is
  /// three-dimensional: a user who crouches has drifted in `y` too, and the
  /// horizontal test alone would not notice.
  static StationOrigin originFrom({
    required ArAnchorHandle anchor,
    required double lockYawDeg,
    required double cameraHeightM,
    StationConfidence confidence = StationConfidence.plane,
  }) => StationOrigin(
    worldPosition: anchor.worldPosition,
    lockYawDeg: lockYawDeg,
    lockCameraHeightM: cameraHeightM,
    confidence: confidence,
  );

  @override
  String toString() {
    if (!enabled) return 'StationDriftGate(off)';
    final h = _last == null || _last!.horizontalM.isNaN
        ? 'n/a'
        : '${_last!.horizontalM.toStringAsFixed(3)} m';
    return 'StationDriftGate($h, '
        '${_anchorUntracked ? 'anchor untracked' : 'diagnostic'})';
  }
}

/// Whether the camera is close enough to a hit point to be a usable station.
///
/// The brief requires the hit-test to *miss* rather than silently guess, and
/// requires the button to be blocked when there is no plane. This is the small
/// pure predicate behind that: a hit that is too far away, or on a surface that
/// is not horizontal, is rejected with a reason the UI can show.
({bool ok, String? reason}) validateStationHit(
  PlaneHit? hit, {
  double minDistanceM = 0.4,
  double maxDistanceM = 4.0,
}) {
  if (hit == null) {
    return (
      ok: false,
      reason:
          'Point the centre of the screen at the floor and move the phone '
          'slowly until a surface is found.',
    );
  }
  if (!hit.isHorizontal) {
    return (
      ok: false,
      reason: 'That surface is not horizontal. Aim slightly down at the floor.',
    );
  }
  if (hit.distanceM < minDistanceM) {
    return (ok: false, reason: 'Too close. Step back a little.');
  }
  if (hit.distanceM > maxDistanceM) {
    return (
      ok: false,
      reason: 'Too far away to place the station reliably. Move closer.',
    );
  }
  return (ok: true, reason: null);
}

/// How high the camera is above the plane it was hit-tested against.
///
/// The signed distance along the plane normal, not the difference in `y`: on a
/// slightly sloped floor those differ, and the recorded value is meant to be
/// the geometric height. Reported rather than assumed, because it is the one
/// quantity the AR layer can supply for free that the parallax estimate and the
/// three-dimensional drift test both want.
double cameraHeightAbovePlane(Vec3 cameraPosition, PlaneHit hit) {
  final n = hit.planeNormal.normalized;
  final d = cameraPosition - hit.worldPosition;
  return d.dot(n);
}
