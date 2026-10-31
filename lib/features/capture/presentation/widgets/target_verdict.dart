import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';

/// What actually happened to one target.
///
/// Four states, because they need four different actions from the user. A target
/// that was never attempted because no surface was found is *not* a failure —
/// telling the user it failed would send them looking for a bug instead of
/// pointing the phone at a wall.
enum TargetVerdict {
  /// A photograph is on disk and recorded.
  saved,

  /// An attempt was made and it failed. Re-takeable.
  failed,

  /// Probed, and there is no surface along this direction.
  noSurface,

  /// Never probed — the station has not been calibrated, or the scan has not
  /// run. Distinct from [noSurface] because "I have not looked" and "I looked
  /// and there is nothing" call for different actions.
  unscanned,

  /// Anchored, nothing attempted, not the current target.
  ready,

  /// The target currently being guided towards.
  active;

  /// Whether this target can be captured without changing the environment.
  bool get isActionable => this == ready || this == active;
}

/// The verdict for [target], plus the one line of detail that explains it.
///
/// Pure, so the four branches can be tested without a widget tree — which
/// matters because the difference between [TargetVerdict.failed] and
/// [TargetVerdict.noSurface] is the difference between "report a bug" and "move
/// the phone".
({TargetVerdict verdict, String? detail}) targetVerdict(
  CaptureTarget target, {
  SurfaceProbe? probe,
}) {
  if (target.status.isDone) {
    final file = target.imagePath;
    return (
      verdict: TargetVerdict.saved,
      detail: file == null ? null : 'photo ${_basename(file)}',
    );
  }
  if (target.status == CaptureTargetStatus.failed) {
    return (
      verdict: TargetVerdict.failed,
      detail: target.message ?? 'No reason recorded.',
    );
  }
  final isActive = target.status == CaptureTargetStatus.active;

  if (target.isAnchored) {
    final surface = target.anchorSurfaceType == null
        ? 'anchored'
        : 'anchored · ${target.anchorSurfaceType}';
    return (
      verdict: isActive ? TargetVerdict.active : TargetVerdict.ready,
      detail: surface,
    );
  }

  // Not anchored. The probe decides whether that is "nothing there" or "not
  // looked yet" — and a surface that was found is still actionable, because
  // selecting the target will anchor to it.
  if (probe == null) {
    return (
      verdict: TargetVerdict.unscanned,
      detail: target.message ?? 'Not scanned yet — scan all directions.',
    );
  }
  if (!probe.found) {
    return (
      verdict: TargetVerdict.noSurface,
      detail:
          target.message ??
          'No surface along this direction. Point at that part of the room '
              'and re-scan all directions.',
    );
  }
  return (
    verdict: isActive ? TargetVerdict.active : TargetVerdict.ready,
    detail: 'surface found · ${probe.describe}',
  );
}

String _basename(String path) {
  final slash = path.lastIndexOf('/');
  final backslash = path.lastIndexOf('\\');
  final cut = slash > backslash ? slash : backslash;
  return cut < 0 ? path : path.substring(cut + 1);
}
