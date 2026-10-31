import 'package:flutter/foundation.dart';

import 'package:adaptive_360_capture_app/core/math/vio_sphere_math.dart' as vio;
import 'package:adaptive_360_capture_app/features/capture/domain/services/station_frame.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/quaternions.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The bezel colour for a target that is not captured yet: the prototype's own
/// `#f4f4f4`.
const ({double r, double g, double b, double a}) kBezelIdleTint = (
  r: 0.957,
  g: 0.957,
  b: 0.957,
  a: 1.0,
);

/// The bezel colour once the target is aligned: `#d8f6e3`.
const ({double r, double g, double b, double a}) kBezelAlignedTint = (
  r: 0.847,
  g: 0.965,
  b: 0.890,
  a: 1.0,
);

/// The bezel colour after a successful capture: `#d8f6e3` again, with the
/// check-mark meshes shown by the target mesh variant.
const ({double r, double g, double b, double a}) kBezelDoneTint = (
  r: 0.847,
  g: 0.965,
  b: 0.890,
  a: 1.0,
);

/// The bezel colour on a failure: `#f2a93b`.
const ({double r, double g, double b, double a}) kBezelFailedTint = (
  r: 0.949,
  g: 0.663,
  b: 0.231,
  a: 1.0,
);

/// The active target's colour while the phone has left the calibrated station.
///
/// This intentionally uses the same high-contrast amber as an unsuccessful
/// attempt, but names a different condition: the target may be perfectly
/// centred and still cannot be photographed until the camera returns within
/// the station-distance limit.  Tinting both the bezel and the green centre
/// prevents the marker from looking capture-ready while the shutter is locked.
const ({double r, double g, double b, double a}) kBezelStationDriftTint = (
  r: 0.949,
  g: 0.663,
  b: 0.231,
  a: 1.0,
);

/// Magenta — the debug marker's colour, chosen because neither shipped asset
/// contains a pixel of it: white plates and green centres are never magenta,
/// so it can never be mistaken for the real marker.
const ({double r, double g, double b, double a}) kDebugMarkerTint = (
  r: 1.0,
  g: 0.0,
  b: 1.0,
  a: 1.0,
);

/// Whether the diagnostic marker is compiled in. Off unless the run says so:
/// `--dart-define=glb_debug_marker=true`.
const bool kGlbDebugMarker = bool.fromEnvironment('glb_debug_marker');

/// Puts the GLB markers at their computed world positions and keeps them
/// there — all 30 of them, for the whole capture.
///
/// ## The arrangement is created once and lives for the session
///
/// The 30 targets are *directions* from the calibrated station. A direction
/// and a radius are a position:
///
/// ```
/// targetWorldPosition = station.origin + targetRadiusM × worldDirection
/// ```
///
/// **No surface is consulted and none is required.** No raycast finds a wall,
/// floor or ceiling; no plane has to be tracked; a room ARCore has not mapped
/// yet places its targets exactly as a fully-mapped one does. Every anchor is
/// a world-space anchor (`trackableKey: null`) created **once, at calibration
/// time**, and kept: a photograph never deletes, recreates or repositions a
/// node, and advancing to the next target only moves the highlight. The
/// positions are computed once, at placement time; ARCore refines each anchor
/// in place as it maps the room, and the renderer reads the anchor's tracked
/// pose every frame.
///
/// ## A placed anchor is not a *tracked* anchor
///
/// ARCore pauses world anchors on its own — a fast rotation or a featureless
/// wall — and a paused anchor's pose is its **last known** pose, not a current
/// one. Capture markers are not drawn from that pose: the controller clears
/// the scene and the native renderer independently skips untracked anchors.
/// The placement remains in memory, so the same frozen target reappears once
/// ARCore again reports a trustworthy anchor pose.
class PlaneAnchorService {
  PlaneAnchorService(this._host, {required this.geometry});

  final ArCoreGateway _host;
  final CaptureGeometryConfig geometry;

  /// One world anchor at the calibrated station. Every target is a frozen
  /// local transform below it, so ARCore refines one coherent reference.
  ArCoreAnchor? _stationAnchor;

  /// The placement each target's anchor was created with, keyed by target id.
  final Map<int, AnchoredTarget> _placementsById = <int, AnchoredTarget>{};

  /// Anchor ids whose `detachAnchor` call is currently in flight.
  ///
  /// [release] and [releaseAll] can both reach the same anchor from different
  /// flows (a pause teardown racing a scan-again), and ARCore rejects a second
  /// detach of an already-detached anchor. The id is marked before the first
  /// `await` — everything after that is a yield point — and unmarked once the
  /// call has settled, so a duplicate detach is skipped instead of doubled.
  final Set<String> _detachInFlight = <String>{};

  /// The in-flight [dispose] teardown, observed rather than re-run by an
  /// overlapping call. Reset in `finally` because the service survives a
  /// pause/resume and is used again after the next boot.
  Future<void>? _disposeInFlight;

  /// Mesh ids the controller registers with the native renderer.
  static const String targetMeshId = 'target';
  static const String bezelMeshId = 'bezel';

  /// The surface label a world-space placement records.
  ///
  /// There is no surface, and the record must not pretend there is: the field
  /// exists on [CaptureTarget]/[CaptureRecord] for schema stability, so it
  /// carries the honest value instead of a plane type.
  static const String worldSurfaceLabel = 'WORLD';

  /// Opacity of a target that is neither active nor already captured.
  ///
  /// The whole arrangement stays in the scene — the brief's requirement — so
  /// "which one do I shoot now" is answered by opacity: the active target is
  /// opaque, the rest are ghosts. A captured target keeps the done tint at
  /// full opacity so progress is legible at a glance.
  static const double kInactiveTargetOpacity = 0.35;

  /// The draw scale applied to both models, normalised to the configured
  /// physical diameter.
  ///
  /// Set by the controller once the assets are loaded, from
  /// `CaptureGeometryConfig.markerScaleFor(plateWidthM)`. 1.0 until then, which
  /// is the authored size — the same value the old code used permanently, and
  /// the reason the marker filled half the view.
  double markerScale = 1;

  int get placedCount => _placementsById.length;

  bool isPlaced(int targetId) => _placementsById.containsKey(targetId);

  AnchoredTarget? placementFor(int targetId) => _placementsById[targetId];

  ArCoreAnchor? anchorFor(int targetId) => _placementsById[targetId]?.anchor;

  /// Target ids that have a live anchor, in id order.
  List<int> get placedTargetIds {
    final ids = _placementsById.keys.toList()..sort();
    return ids;
  }

  /// Creates one station anchor then derives every target from its immutable
  /// station-local transform. No target requires a raycast or its own anchor.
  Future<PlacementOutcome> placeAll({
    required StationFrame station,
    required List<CaptureTargetSpec> specs,
  }) async {
    final placed = <int, AnchoredTarget>{};
    final failures = <int, String>{};
    var stationAnchor = _stationAnchor;
    if (stationAnchor == null) {
      try {
        stationAnchor = await _host.createAnchor(
          position: station.origin,
          orientation: Quat.identity,
          trackableKey: null,
        );
      } catch (e) {
        stationAnchor = null;
        for (final spec in specs) {
          failures[spec.targetId] = 'Station anchor could not be created: $e';
        }
        return PlacementOutcome(placed: placed, failures: failures);
      }
      if (stationAnchor == null) {
        for (final spec in specs) {
          failures[spec.targetId] =
              'AR would not hold the calibrated station anchor.';
        }
        return PlacementOutcome(placed: placed, failures: failures);
      }
      _stationAnchor = stationAnchor;
    }
    for (final spec in specs) {
      final existing = _placementsById[spec.targetId];
      if (existing != null) {
        placed[spec.targetId] = existing;
        continue;
      }
      // One authority for placement: the same function the tests and the
      // sphere map's contract are written against.
      final resolved = resolveTarget(
        station: station,
        spec: spec,
        radiusM: geometry.targetRadiusM,
      );
      if (resolved.worldPosition.length == 0) {
        failures[spec.targetId] =
            'Target ${spec.targetId} has no direction from this station.';
        continue;
      }
      final worldPosition = resolved.worldPosition;
      // The engine's own model matrix for this target, from the same two
      // points `resolveTarget` used, so the orientation the renderer draws is
      // the one the maths layer proved correct rather than the quaternion's
      // minimal-rotation approximation.
      final modelMatrix = vio.VioSphereEngine.buildModelMatrix(
        targetWorldPosition: vio.Vec3(
          worldPosition.x,
          worldPosition.y,
          worldPosition.z,
        ),
        stationOriginWorld: vio.Vec3(
          station.origin.x,
          station.origin.y,
          station.origin.z,
        ),
      );
      // The renderer composes this station-local translation with the station
      // anchor, rather than creating thirty independently refinable anchors.
      modelMatrix[12] = worldPosition.x - station.origin.x;
      modelMatrix[13] = worldPosition.y - station.origin.y;
      modelMatrix[14] = worldPosition.z - station.origin.z;
      final placement = AnchoredTarget(
        anchor: stationAnchor,
        worldPosition: worldPosition,
        stationLocalPosition: Vec3(modelMatrix[12], modelMatrix[13], modelMatrix[14]),
        facingNormal: resolved.facingNormal,
        localRotation: resolved.localRotation,
        modelMatrix: modelMatrix,
        surfaceType: worldSurfaceLabel,
      );
      _placementsById[spec.targetId] = placement;
      placed[spec.targetId] = placement;
    }
    return PlacementOutcome(placed: placed, failures: failures);
  }

  /// Re-reads the shared station anchor. The selected target only chooses
  /// which immutable local transform guidance aims at.
  ///
  /// Returns `gone` **only** when the anchor this refresh asked about is still
  /// the live one and the host no longer has it — tracking lost long enough
  /// that ARCore dropped it, or the session restarted. A *paused* anchor comes
  /// back normally with [AnchoredTarget.isTracking] false: the two states need
  /// different recoveries (re-place vs. wait), and collapsing them into one
  /// null is what made a paused marker look like a vanished one.
  ///
  /// Returns `stale` when the answer describes an anchor that was replaced
  /// while the refresh was in flight. The check lives here, before any state
  /// is mutated, because a caller-visible map has already been overwritten by
  /// the time the answer would reach it.
  Future<AnchorRefreshOutcome> refresh(int targetId) async {
    final current = _stationAnchor;
    final placement = _placementsById[targetId];
    if (current == null || placement == null) {
      return const (placement: null, gone: false, stale: false);
    }
    final requested = current.id;
    final refreshed = await _host.refreshAnchor(requested);
    if (_stationAnchor?.id != requested) {
      return const (placement: null, gone: false, stale: true);
    }
    if (refreshed == null) {
      _stationAnchor = null;
      _placementsById.clear();
      return (placement: null, gone: true, stale: false);
    }
    _stationAnchor = refreshed;
    for (final entry in _placementsById.entries.toList()) {
      _placementsById[entry.key] = entry.value.copyWith(anchor: refreshed);
    }
    return (placement: _placementsById[targetId], gone: false, stale: false);
  }

  /// Pushes the whole arrangement: bezel behind, target in front, for every
  /// placed target.
  ///
  /// [activeTargetId] is the one target drawn at full opacity with [bezelTint];
  /// [doneTargets] keep the done tint at full opacity so progress is legible;
  /// everything else is a ghost at [kInactiveTargetOpacity]. Nothing is added
  /// or removed between photographs — advancing only changes which id is
  /// active, which is a scene push, not a placement.
  ///
  /// [activeTracking] is ARCore's current confidence in the active anchor.
  /// The controller only publishes a scene when it is true; native code also
  /// checks the anchor immediately before drawing, so stale poses cannot move
  /// a visible marker while recovery is pending.
  ///
  /// [activeTargetTint] is applied only to the active target's centre mesh.
  /// It is normally absent so the asset's authored green remains visible. The
  /// station-drift gate supplies amber here, making a capture-blocked target
  /// visibly different from a ready target even when it is perfectly aligned.
  ///
  /// [hiddenTargets] are **omitted from the draw list entirely** — not faded,
  /// not moved, not detached. Their shared station anchor stays alive and their
  /// placements stay in [_placementsById], so a hidden target is still placed
  /// and still aims the guidance; only its mesh and bezel leave the frame.
  /// That is what "the target disappears when the ring completes" means: the
  /// node survives, the draw commands do not.
  ///
  /// [debugMarker] adds a deliberate, unmistakable plate at the **active
  /// anchor** — the target mesh at 3.5× in magenta, no setback. It answers one
  /// question on a device: is the placement/anchor right and the GLB rendering
  /// wrong, or is the marker not in the scene at all? If the magenta plate
  /// appears, the anchor, the scene push and the projection are all correct
  /// and the defect is in the real mesh's geometry or material; if it does
  /// not appear, nothing about the GLB is relevant and the problem is the
  /// anchor. Off by default, enabled with
  /// `--dart-define=glb_debug_marker=true`.
  Future<void> pushScene({
    required int activeTargetId,
    ({double r, double g, double b, double a}) bezelTint = kBezelIdleTint,
    double opacity = 1,
    Set<int> doneTargets = const <int>{},
    Set<int> hiddenTargets = const <int>{},
    bool activeTracking = true,
    ({double r, double g, double b, double a})? activeTargetTint,
    bool debugMarker = false,
  }) async {
    final draws = <ArSceneDraw>[];
    for (final entry in _placementsById.entries) {
      final targetId = entry.key;
      final placement = entry.value;
      // Omitted, not faded: the anchor and the placement stay exactly where
      // they are, only the geometry leaves the frame.
      if (hiddenTargets.contains(targetId)) continue;
      final isActive = targetId == activeTargetId;
      final isDone = doneTargets.contains(targetId);
      // The controller publishes a scene only when the active anchor is
      // tracked. This opacity is retained for a direct caller's presentation
      // choice; native rendering still rejects an untracked anchor.
      final authoritative = isDone || (isActive && activeTracking);
      final targetOpacity = authoritative ? opacity : kInactiveTargetOpacity;
      final tint = isDone ? kBezelDoneTint : bezelTint;
      final markerTint = isActive && !isDone ? activeTargetTint : null;
      // The engine's basis, downcast once per target for transport: the host
      // reads a Kotlin `float[]`, not a `double[]`.
      final modelMatrix = Float32List.fromList(placement.modelMatrix);
      if (debugMarker && isActive) {
        draws.add(
          ArSceneDraw(
            meshId: targetMeshId,
            anchorId: placement.anchor.id,
            localRotation: placement.localRotation,
            modelMatrix: modelMatrix,
            useModelTranslation: true,
            scale: markerScale * 3.5,
            // No setback: the debug plate must not hide behind the bezel, and
            // its colour must be impossible to mistake for the real marker.
            tint: kDebugMarkerTint,
            useTint: true,
            opacity: opacity,
          ),
        );
      }
      // The bezel is recessed along the plate's own normal by
      // [CaptureGeometryConfig.bezelSetbackM] — derived from the assets, not
      // tuned: 0.006 = 0.012 − 0.006 puts the two front faces coplanar, so the
      // ring surrounds the plate with neither a gap nor an overlap.
      draws.add(
        ArSceneDraw(
          meshId: bezelMeshId,
          anchorId: placement.anchor.id,
          localRotation: placement.localRotation,
          modelMatrix: modelMatrix,
          useModelTranslation: true,
          scale: markerScale,
          offset: Vec3(0, 0, -geometry.bezelSetbackM),
          tint: tint,
          useTint: true,
          opacity: targetOpacity,
          // A capture target is a world anchor at a computed position. It is
          // rendered only while ARCore tracks that anchor; a stale last-known
          // pose must never look like a target moving through the room.
          requireTracking: true,
        ),
      );
      draws.add(
        ArSceneDraw(
          meshId: targetMeshId,
          anchorId: placement.anchor.id,
          localRotation: placement.localRotation,
          modelMatrix: modelMatrix,
          useModelTranslation: true,
          scale: markerScale,
          tint: markerTint ?? kBezelIdleTint,
          useTint: markerTint != null,
          opacity: targetOpacity,
          requireTracking: true,
        ),
      );
    }
    await _host.setScene(draws);
  }

  /// Empties the scene without releasing anything.
  Future<void> clearScene() async {
    await _host.setScene(const <ArSceneDraw>[]);
  }

  /// Removes one target draw placement. The shared station anchor stays alive.
  Future<void> release(int targetId) async {
    _placementsById.remove(targetId);
  }

  /// Releases every anchor. Used by an arrangement rebuild, by a station reset
  /// (`recalibrateHere`) and by session teardown, never by a photograph.
  ///
  /// The map is snapshotted and emptied synchronously, before any `await`:
  /// iterating the live map across awaited detaches crashed with a
  /// `ConcurrentModificationError` whenever teardown ran while a photograph
  /// or a dropped anchor mutated it. Clearing first also means a [release]
  /// racing this can never double-detach — the in-flight set covers the rest.
  Future<void> releaseAll() async {
    final anchor = _stationAnchor;
    if (anchor == null) return;
    _stationAnchor = null;
    _placementsById.clear();
    await _detachAnchorOnce(anchor.id);
  }

  /// Detaches [anchorId] at most once, shared by [release] and [releaseAll].
  ///
  /// The in-flight mark is taken synchronously — before the first `await` —
  /// so overlapping calls can never both pass. A missing or already-detached
  /// anchor, or a session that is already closed by the time the call runs,
  /// surfaces here as an error thrown by the gateway; it is logged and
  /// swallowed because there is nothing left to undo.
  Future<void> _detachAnchorOnce(String anchorId) async {
    if (!_detachInFlight.add(anchorId)) {
      return;
    }
    try {
      await _host.detachAnchor(anchorId);
    } catch (error) {
      debugPrint('ARCORE_ANCHOR: detach of $anchorId failed or the anchor '
          'was already gone: $error');
    } finally {
      _detachInFlight.remove(anchorId);
    }
  }

  Future<void> dispose() async {
    final existing = _disposeInFlight;
    if (existing != null) {
      return existing;
    }
    final inFlight = _teardown();
    _disposeInFlight = inFlight;
    try {
      return await inFlight;
    } finally {
      _disposeInFlight = null;
    }
  }

  /// The teardown body of [dispose]: clear the scene, then release detectors.
  Future<void> _teardown() async {
    await clearScene();
    await releaseAll();
  }
}

/// What a [PlaneAnchorService.placeAll] call did: which targets now have a
/// node (placed now *or* reused), and which could not be placed, with the
/// reason for each.
class PlacementOutcome {
  final Map<int, AnchoredTarget> placed;
  final Map<int, String> failures;

  const PlacementOutcome({required this.placed, required this.failures});

  bool get allPlaced => failures.isEmpty;
  int get placedCount => placed.length;
  int get failureCount => failures.length;
}

/// What a [PlaneAnchorService.refresh] call learned.
///
/// Three outcomes, because the caller must act differently on each:
/// * [placement] non-null — the live placement, possibly refined by ARCore;
/// * [gone] — the anchor that was asked about is still the live one and the
///   host no longer has it, so a re-placement is the right recovery;
/// * [stale] — the answer describes an anchor that was replaced while the
///   refresh was in flight, so it applies to nothing that is live.
typedef AnchorRefreshOutcome = ({
  AnchoredTarget? placement,
  bool gone,
  bool stale,
});

/// A target placed at its calculated world position.
class AnchoredTarget {
  final ArCoreAnchor anchor;

  /// Where the marker is, in AR world metres. Computed once, at placement;
  /// the renderer then follows the anchor, not this number.
  final Vec3 worldPosition;

  /// Fixed target offset in the station anchor's local axes.
  final Vec3 stationLocalPosition;

  /// The direction the marker's face points: back towards the station.
  final Vec3 facingNormal;

  /// The mesh-local rotation that maps the model's own `+Z` onto
  /// [facingNormal].
  final Quat localRotation;

  /// The engine's column-major `[X | Y | Z | worldPosition]` model matrix for
  /// this target, computed once at placement by
  /// `VioSphereEngine.buildModelMatrix`.
  ///
  /// This is the transform the native renderer consumes: it carries the
  /// verified orientation (model `+Z` on [facingNormal], with a defined basis
  /// at the poles) that [localRotation] only approximates. Its translation
  /// column matches [worldPosition]; the host zeroes it and uses the anchor's,
  /// so the marker still follows ARCore's refinement.
  final Float64List modelMatrix;

  /// Always [PlaneAnchorService.worldSurfaceLabel] on this path — there is no
  /// surface; the field survives for schema stability on the target and
  /// capture records.
  final String surfaceType;

  const AnchoredTarget({
    required this.anchor,
    required this.worldPosition,
    required this.stationLocalPosition,
    required this.facingNormal,
    required this.localRotation,
    required this.modelMatrix,
    required this.surfaceType,
  });

  bool get isTracking => anchor.tracking == TrackingState.tracking;

  AnchoredTarget copyWith({ArCoreAnchor? anchor, Vec3? worldPosition}) =>
      AnchoredTarget(
        anchor: anchor ?? this.anchor,
        worldPosition: worldPosition ?? this.worldPosition,
        stationLocalPosition: stationLocalPosition,
        facingNormal: facingNormal,
        localRotation: localRotation,
        // The basis is a per-target constant; only the anchor's pose is
        // refreshed, so the matrix is carried through unchanged.
        modelMatrix: modelMatrix,
        surfaceType: surfaceType,
      );
}
