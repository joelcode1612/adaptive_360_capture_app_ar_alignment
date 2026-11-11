import 'package:adaptive_360_capture_app/features/virtual_tour/domain/environment/viewpoint.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/plan/plan_generator.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/session/capture_state_machine.dart';

/// The plain data behind the capture flow.
///
/// This file, and `capture_flow_engine.dart` next to it, make up the `flow/`
/// layer: the only place in the feature that is allowed to know about `domain/`,
/// `ar/`, `camera/` and `data/` at the same time. `ui/` above it is purely
/// presentational and `domain/` below it knows nothing about any of them, which
/// is what keeps the layers from turning into a web of shortcuts.
///
/// **Pure Dart, on purpose.** The flow logic has to be testable without a
/// Flutter binding, because `flutter test` cannot run in every environment
/// (see `tool/verify_all.dart`). The screens wrap this in a `ChangeNotifier`,
/// which is the only Flutter part.

/// The room-size answer from screen 2.
enum RoomSize {
  small('small', 'Small / Compact', 24, 18),
  medium('medium', 'Medium', 36, 26),
  large('large', 'Large / Open-plan', 52, 32);

  const RoomSize(this.key, this.label, this.iconWidth, this.iconHeight);
  final String key;
  final String label;

  /// The little rectangle the prototype draws inside the card.
  final double iconWidth;
  final double iconHeight;

  static RoomSize fromKey(String? k) =>
      values.firstWhere((e) => e.key == k, orElse: () => RoomSize.medium);
}

/// The obstruction answer from screen 2.
enum Obstruction {
  open('open', 'Mostly open and visible'),
  part('part', 'Some areas have partitions, corners or fixed furniture');

  const Obstruction(this.key, this.label);
  final String key;
  final String label;

  static Obstruction fromKey(String? k) =>
      values.firstWhere((e) => e.key == k, orElse: () => Obstruction.open);
}

/// The recommended plan from screen 3.
///
/// Ported from the prototype's `plan()`. **The station count is an estimate**
/// and the UI says so; nothing here measures the room.
class PlanRecommendation {
  final bool multiStation;
  final String estimate;

  const PlanRecommendation({
    required this.multiStation,
    required this.estimate,
  });

  String get headline => multiStation
      ? 'Multiple panorama stations may be needed'
      : 'One panorama station';

  String get body => multiStation
      ? 'You will capture a separate panorama at each selected station, then '
            'link the panoramas in the virtual-tour viewer.'
      : 'You will capture one panorama around the marked position.';

  String reason(Obstruction obstruction) {
    if (!multiStation) {
      return 'A small or medium open room is usually visible from one central '
          'point.';
    }
    return obstruction == Obstruction.part
        ? 'Partitions, corners or fixed furniture can hide areas from a single '
              'point.'
        : 'A large open-plan room may be too wide to represent from one point.';
  }
}

/// Ported from the prototype's `plan()`.
PlanRecommendation recommendPlan(RoomSize size, Obstruction obstruction) {
  final multi = obstruction == Obstruction.part || size == RoomSize.large;
  return PlanRecommendation(
    multiStation: multi,
    estimate: (size == RoomSize.large && obstruction == Obstruction.part)
        ? '3 to 4'
        : multi
        ? '2'
        : '1',
  );
}

/// A candidate position for a later station, ported from the prototype's
/// `cands()`. Normalised 0…1 coordinates inside the floor-plan rectangle.
class StationCandidate {
  final double x;
  final double y;
  final String label;

  const StationCandidate(this.x, this.y, this.label);
}

/// Ported from the prototype's `cands()`.
List<StationCandidate> stationCandidates(Obstruction obstruction) =>
    obstruction == Obstruction.part
    ? const [
        StationCandidate(0.8, 0.3, 'Back area'),
        StationCandidate(0.8, 0.75, 'Side area'),
        StationCandidate(0.2, 0.8, 'Corner area'),
      ]
    : const [
        StationCandidate(0.8, 0.5, 'Far end'),
        StationCandidate(0.2, 0.5, 'Near end'),
      ];

/// Ported from the prototype's `firstPos()`.
({double x, double y}) firstStationPosition(Obstruction obstruction) =>
    obstruction == Obstruction.part ? (x: 0.35, y: 0.5) : (x: 0.5, y: 0.5);

/// Which screen the flow is showing.
///
/// The prototype's screen numbers are kept in [wireNumber] so the port can be
/// checked against it, and because the docs reference them.
enum CaptureScreen {
  intro(1),
  roomDetails(2),
  plan(3),
  stationSetup(4),
  arCapture(5),
  progressMap(6),
  stationReview(7),
  addStation(8),
  tourPreview(9);

  const CaptureScreen(this.wireNumber);
  final int wireNumber;
}

/// The status of one station, mirroring the prototype's `acc` flag plus the
/// review outcome.
enum StationOutcome {
  /// Still capturing.
  inProgress,

  /// Finished, and the user accepted the panorama.
  accepted,

  /// Finished, but coverage needs review (the prototype's outcome B).
  needsReview,
}

/// One station's record.
///
/// Deliberately holds the plan and the per-frame statuses so a resumed session
/// can rebuild the progress map without re-running the planner.
class StationRecord {
  final int index;
  final RoomSize roomSize;
  final Obstruction obstruction;

  /// The plan generated for this station.
  final CapturePlan plan;

  /// Normalised position on the floor plan.
  final double planX;
  final double planY;

  /// Per-target status, indexed by `PlanFrame.index`.
  final List<TargetStatus> statuses;

  StationOutcome outcome;

  /// How many frames are on disk for this station.
  int savedFrames;

  /// Non-empty when the review found something to say.
  final List<String> warnings;

  /// Non-empty when the review could not resolve something.
  final List<String> unresolved;

  /// The measured spherical coverage, once VERIFY has run.
  double? coveragePct;

  /// Where VERIFY said another station would help, in metres on the room model's
  /// own axes.
  ///
  /// Only ever set from a **measured** room: without one there is nothing to
  /// search over, and the review says so instead of showing a number. The room
  /// itself is the user's estimate, so this is planning advice, not a
  /// measurement of the space.
  ViewpointSuggestion? suggestedStation;

  /// Whether a station anchor was placed for AR reference tracking.
  bool anchorPlaced;

  StationRecord({
    required this.index,
    required this.roomSize,
    required this.obstruction,
    required this.plan,
    required this.planX,
    required this.planY,
    List<TargetStatus>? statuses,
    this.outcome = StationOutcome.inProgress,
    this.savedFrames = 0,
    List<String>? warnings,
    List<String>? unresolved,
    this.coveragePct,
    this.anchorPlaced = false,
  }) : statuses =
           statuses ??
           // Growable on purpose. VERIFY can append coverage-repair targets, and
           // the flow then extends this list to account for them
           // (`capture_flow_engine.dart`, `_onStationComplete` and the accept
           // path). `List.filled` is fixed-length, so the plain default threw
           // `UnsupportedError: Cannot add to a fixed-length list` the moment a
           // station grew past its planned 26 views — and a *resumed* station,
           // built with `List.generate`, did not, so the two paths disagreed.
           List<TargetStatus>.filled(
             plan.frames.length,
             TargetStatus.pending,
             growable: true,
           ),
       warnings = warnings ?? <String>[],
       unresolved = unresolved ?? <String>[];

  int get total => plan.frames.length;

  int get capturedCount => statuses.where((s) => s == TargetStatus.ok).length;

  int get skippedCount =>
      statuses.where((s) => s == TargetStatus.skipped).length;

  /// Whether every target has been either captured or explicitly skipped.
  bool get isFullyAddressed => !statuses.contains(TargetStatus.pending);

  /// The next target that still needs a frame, or `-1`.
  int nextPending() {
    for (var i = 0; i < statuses.length; i++) {
      if (statuses[i] == TargetStatus.pending) return i;
    }
    return -1;
  }

  /// The prototype's default display name for a station.
  String get displayName => 'Station ${index + 1}';

  /// The area label, when the position came from a candidate.
  String areaLabel(Obstruction o) {
    if (index == 0) return 'Suggested centre';
    final c = stationCandidates(o);
    // Nearest candidate to the recorded position.
    var best = c.first;
    var bestD = double.infinity;
    for (final cand in c) {
      final d = (cand.x - planX).abs() + (cand.y - planY).abs();
      if (d < bestD) {
        bestD = d;
        best = cand;
      }
    }
    return best.label;
  }
}

/// A row in the spherical progress map, matching the prototype's `RN`/`RP`.
class ProgressRing {
  final String label;

  /// Inclusive pitch range, in degrees.
  final double fromPitchDeg;
  final double toPitchDeg;

  const ProgressRing(this.label, this.fromPitchDeg, this.toPitchDeg);
}

/// The prototype's three rings plus the poles.
const List<ProgressRing> progressRings = <ProgressRing>[
  ProgressRing('Upper ring', 20, 60),
  ProgressRing('Middle ring', -20, 20),
  ProgressRing('Lower ring', -60, -20),
];
