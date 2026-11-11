import 'package:flutter/material.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/frame_integrity.dart';
// The status→colour-role mapping used to be inlined in `_FrameGrid`; it now
// lives in the pure-Dart viz module so the sphere map and the grid cannot drift
// apart, and so the plain-`dart` suite can check it.
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/session/capture_state_machine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/viz/sphere_map.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/capture_copy.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/capture_flow_engine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/flow_state.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/review_outcome.dart';
// The 360° viewer. It only opens once a stitched panorama exists, which is why
// the button that opens it is disabled by default rather than hidden.
// (`TourViewpoint` is not imported: the engine hands over the list already
// built, so this screen never names the type.)
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/screens/tour_screen.dart';
// The pre-upload gate. It reads the same StationGroup the flow writes, so the
// review screen is where it belongs: the answer to "this will not stitch" is to
// retake a frame, and retaking is possible here and awkward later.
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/stitch/stitch_validator.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/widgets/flow_widgets.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/widgets/frame_gallery.dart';

/// Screens 7–9: the station review, adding a station, and the tour preview.
///
/// Ported from `V[7]`, `V[8]` and `V[9]` of `360° Capture Prototype.html`.
///
/// The prototype's review has a simulated processing bar and a simulated
/// panorama preview. **Both are gone.** The brief forbids claiming that
/// stitching works, and a progress bar that always reaches 100% is exactly that
/// claim in animated form. What replaces them is what the flow actually knows:
/// the per-view grid, the measured coverage, and the unresolved items from
/// VERIFY. Where the prototype shows a preview image, this says plainly that the
/// panorama is built after upload.

/// Screen 7.
class StationReviewScreen extends StatefulWidget {
  const StationReviewScreen({super.key, required this.controller});

  final dynamic controller;

  @override
  State<StationReviewScreen> createState() => _StationReviewScreenState();
}

class _StationReviewScreenState extends State<StationReviewScreen> {
  CaptureFlowEngine get engine => widget.controller.engine;

  /// The last read-back, with the station it was run against.
  ///
  /// Held here rather than in the engine: it is a fact about the filesystem the
  /// capture flow has no use for, and it goes stale the moment another station
  /// is opened, which is why the station index travels with it.
  ({int stationIndex, Map<int, FrameIntegrity> byFrame})? _readBack;

  bool _reading = false;

  Future<void> _readBackAll(int stationIndex) async {
    setState(() => _reading = true);
    final byFrame = await engine.readBackStation(engine.stations[stationIndex]);
    if (!mounted) return;
    setState(() {
      _reading = false;
      _readBack = (stationIndex: stationIndex, byFrame: byFrame);
    });
  }

  void _openFrame(StationRecord record, GalleryFrame frame) {
    openFramePreview(
      context,
      title: frame.title,
      evidence: () => engine.readFrameEvidence(record, frame.index),
    );
  }

  @override
  Widget build(BuildContext context) {
    final record = engine.stations[engine.currentStation];
    final needsReview = record.outcome == StationOutcome.needsReview;
    final accepted = record.outcome == StationOutcome.accepted;
    final preStitch = engine.reviewPreStitch;

    // The prototype's outcome B marks views 3 and 11 as the ones the review
    // sends back. One set, used by the grid and the gallery together, so the
    // two can never disagree about which photograph is being questioned.
    final retake = reviewOutcome(
      needsReview: needsReview,
    ).badFrameIndices.toSet();

    return FlowScaffold(
      title: '${record.displayName} review',
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // A Wrap, not a Row: two side-by-side labels do not shrink, and this
          // row was the one place in the flow that overflowed — by 36 dp on a
          // 360 dp screen and 76 dp on a 320 dp one, which `flutter test
          // test/ui_layout_test.dart` now measures.
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: [
              // `.sim` — the disclaimer badge, not the `.chip` status badge.
              ChipBadge(
                label: 'Not stitched here',
                color: CaptureColors.warn,
                outlined: true,
              ),
              ChipBadge(
                label: accepted
                    ? 'Accepted'
                    : needsReview
                    ? 'Needs review'
                    : 'In progress',
                color: accepted
                    ? CaptureColors.ok
                    : needsReview
                    ? CaptureColors.warn
                    : null,
              ),
            ],
          ),
          const SizedBox(height: 14),
          _FrameGrid(record: record, retake: retake),
          const SizedBox(height: 10),
          Text(
            'Captured ${record.capturedCount} of ${record.total}'
            '${record.skippedCount > 0 ? ' · skipped ${record.skippedCount}' : ''}',
            style: TextStyle(color: CaptureColors.mutedOnDark),
          ),
          if (record.coveragePct != null) ...[
            const SizedBox(height: 6),
            Text(
              'Measured spherical coverage: '
              '${record.coveragePct!.toStringAsFixed(1)}%',
              style: TextStyle(color: CaptureColors.mutedOnDark),
            ),
            const SizedBox(height: 4),
            const ProvisionalLabel(
              text:
                  'Coverage is a geometric test of the planned directions '
                  'against a 4000-point sphere. It does not prove the frames '
                  'will stitch.',
            ),
          ],
          const SizedBox(height: 16),
          FrameGallery(
            frames: _gallery(record, retake),
            onOpen: (frame) => _openFrame(record, frame),
          ),
          const SizedBox(height: 12),
          SecondaryButton(
            label: _reading
                ? 'Reading the files back…'
                : 'Check every photograph on disk',
            enabled: !_reading && record.capturedCount > 0,
            onPressed: () => _readBackAll(record.index),
          ),
          if (_readBack != null && _readBack!.stationIndex == record.index)
            _ReadBackCard(byFrame: _readBack!.byFrame, record: record),
          const SizedBox(height: 16),
          if (preStitch != null) _PreStitchCard(result: preStitch),
          if (needsReview) ...[
            NoteCard(
              tone: NoteTone.warn,
              icon: Icons.report_problem_outlined,
              child: const Text(
                'Some views could not be resolved. Review or retake the '
                'affected frames.',
              ),
            ),
            for (final u in record.unresolved)
              NoteCard(
                tone: NoteTone.warn,
                icon: Icons.warning_amber,
                child: Text(u),
              ),
            if (record.suggestedStation case final suggested?)
              NoteCard(
                tone: NoteTone.warn,
                icon: Icons.add_location_alt_outlined,
                child: Text(
                  'A second station about '
                  '${suggested.x.toStringAsFixed(1)} m along the room and '
                  '${suggested.z.toStringAsFixed(1)} m across it would see '
                  'more of what this one misses. That position comes from the '
                  'room size you entered, which is an estimate rather than a '
                  'measurement.',
                ),
              ),
          ],
          for (final w in record.warnings)
            NoteCard(
              tone: NoteTone.warn,
              icon: Icons.info_outline,
              child: Text(w),
            ),
          if (record.anchorPlaced)
            NoteCard(
              tone: NoteTone.ok,
              icon: Icons.anchor,
              child: Text(
                'This station was anchored, so camera-to-station displacement '
                'was available as a diagnostic while capturing.',
              ),
            )
          else
            const NoteCard(
              tone: NoteTone.warn,
              icon: Icons.explore_off,
              child: Text(
                'This station has no anchor, so camera-to-station displacement '
                'could not be measured. The frames are still usable.',
              ),
            ),
          const NoteCard(
            icon: Icons.cloud_upload_outlined,
            child: Text(
              'The panorama is assembled from these frames after you upload. '
              'Nothing has been uploaded yet.',
            ),
          ),
        ],
      ),
      footer: _reviewFooter(context, record, accepted, needsReview),
    );
  }

  /// The gallery's contents: one cell per planned view, with the file the store
  /// would have written for it.
  ///
  /// A view that was never accepted gets no path — the tile then reads as an
  /// empty slot, which is the truth, rather than pointing at a file from an
  /// earlier take of the same target.
  List<GalleryFrame> _gallery(StationRecord record, Set<int> retake) =>
      <GalleryFrame>[
        for (var i = 0; i < record.statuses.length; i++)
          GalleryFrame(
            index: i,
            title: i < record.plan.frames.length
                ? frameTitle(record.plan.frames[i])
                : 'View ${i + 1}',
            status: record.statuses[i],
            imagePath: record.statuses[i] == TargetStatus.ok
                ? engine.paths.frameImage(engine.sessionId, record.index, i)
                : null,
            needsRetake: retake.contains(i),
          ),
      ];

  Widget _reviewFooter(
    BuildContext context,
    StationRecord record,
    bool accepted,
    bool needsReview,
  ) {
    if (accepted) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          PrimaryButton(
            label: 'Add another station',
            onPressed: () => widget.controller.goTo(CaptureScreen.addStation),
          ),
          const SizedBox(height: 8),
          SecondaryButton(
            label: 'Finish room',
            onPressed: () => _finish(context),
          ),
        ],
      );
    }
    if (needsReview) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          PrimaryButton(
            label: 'Retake the frames that are not saved',
            onPressed: () => _retake(),
          ),
          const SizedBox(height: 8),
          SecondaryButton(
            label: 'Add another station',
            onPressed: () => widget.controller.goTo(CaptureScreen.addStation),
          ),
          const SizedBox(height: 4),
          LinkButton(label: 'Accept anyway', onPressed: () => _accept()),
        ],
      );
    }
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        PrimaryButton(label: 'Accept station', onPressed: () => _accept()),
        const SizedBox(height: 8),
        SecondaryButton(
          label: 'Add another station instead',
          onPressed: () => widget.controller.goTo(CaptureScreen.addStation),
        ),
      ],
    );
  }

  Future<void> _accept() async {
    await engine.acceptStation();
    widget.controller.refresh();
  }

  Future<void> _retake() async {
    engine.retakeStation();
    await engine.beginCapture(fromTarget: 0);
    widget.controller.goTo(CaptureScreen.arCapture);
  }

  Future<void> _finish(BuildContext context) async {
    await engine.finish();
    widget.controller.goTo(CaptureScreen.tourPreview);
  }
}

/// The verdict of a file-by-file read-back of the station.
///
/// The gallery shows thumbnails, which prove a file decodes *while it is on
/// screen*. This proves it for every frame at once, including the ones that
/// scrolled past, and separates "no file" from "file that stops halfway" —
/// the two failures a manifest cannot tell apart.
class _ReadBackCard extends StatelessWidget {
  const _ReadBackCard({required this.byFrame, required this.record});

  final Map<int, FrameIntegrity> byFrame;
  final StationRecord record;

  @override
  Widget build(BuildContext context) {
    final expected = <int>[];
    for (var i = 0; i < record.statuses.length; i++) {
      if (record.statuses[i] == TargetStatus.ok) expected.add(i);
    }
    String verdictFor(int frameIndex) {
      final check = byFrame[frameIndex];
      if (check == null) return 'no file on disk';
      if (check.isStored) return '';
      return check.label;
    }

    final unreadable = <String>[
      for (final i in expected)
        if (verdictFor(i).isNotEmpty) 'View ${i + 1}: ${verdictFor(i)}',
    ];
    final sized = expected
        .where((i) => byFrame[i]?.dimensionsKnown ?? false)
        .length;

    if (expected.isEmpty) {
      return const NoteCard(
        icon: Icons.photo_library_outlined,
        child: Text(
          'Nothing to read back: this station has no accepted frames.',
        ),
      );
    }
    if (unreadable.isEmpty) {
      return NoteCard(
        tone: NoteTone.ok,
        icon: Icons.fact_check_outlined,
        child: Text(
          'Read-back passed: all ${expected.length} accepted photographs are on '
          'disk, start with a JPEG header and end with the end-of-image marker. '
          'Dimensions recovered for $sized of ${expected.length}.',
        ),
      );
    }
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        NoteCard(
          tone: NoteTone.danger,
          icon: Icons.error_outline,
          child: Text(
            '${unreadable.length} of ${expected.length} accepted '
            'photographs are not usable. Use the retake below to shoot them '
            'again.',
          ),
        ),
        for (final u in unreadable)
          NoteCard(
            tone: NoteTone.danger,
            icon: Icons.broken_image_outlined,
            child: Text(u),
          ),
      ],
    );
  }
}

/// The local pre-stitch gate, shown before the user accepts the station.
///
/// This is *not* a claim that the panorama will stitch. Passing means "nothing
/// is obviously wrong with the geometry" — every planned frame is on disk and
/// each one is close enough to a neighbour to share features. A textureless wall
/// or a moving object can still defeat the stitcher, which is why the wording
/// stops at "passed the check".
class _PreStitchCard extends StatelessWidget {
  const _PreStitchCard({required this.result});

  final StitchReadiness result;

  @override
  Widget build(BuildContext context) {
    if (!result.ready) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          NoteCard(
            tone: NoteTone.danger,
            icon: Icons.report_outlined,
            child: Text(
              'Pre-stitch check found ${result.problems.length} problem(s). '
              'These frames will probably not stitch:',
            ),
          ),
          for (final p in result.problems)
            NoteCard(
              tone: NoteTone.danger,
              icon: Icons.error_outline,
              child: Text(p),
            ),
        ],
      );
    }
    if (result.warnings.isEmpty) {
      return const NoteCard(
        tone: NoteTone.ok,
        icon: Icons.check_circle_outline,
        child: Text(
          'Pre-stitch check passed: every planned frame is on disk, and each '
          'one is close enough to a neighbour to share features.',
        ),
      );
    }
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        const NoteCard(
          tone: NoteTone.ok,
          icon: Icons.check_circle_outline,
          child: Text('Pre-stitch check passed, with things to be aware of:'),
        ),
        for (final w in result.warnings)
          NoteCard(
            tone: NoteTone.warn,
            icon: Icons.info_outline,
            child: Text(w),
          ),
      ],
    );
  }
}

/// The per-view grid: one cell per planned frame, coloured by status.
///
/// The prototype's `.fr` grid. It is the most honest summary the flow has —
/// every cell is a target, and a cell is only green when its image is on disk.
/// The gallery below it is the same truth with the photographs attached.
class _FrameGrid extends StatelessWidget {
  const _FrameGrid({required this.record, required this.retake});

  final StationRecord record;

  /// The views the review is questioning, from [reviewOutcome].
  final Set<int> retake;

  @override
  Widget build(BuildContext context) {
    // `.fr` is a 13-column grid of 16 dp-tall cells with a 3 px gap, and it sits
    // straight on the page — the reference gives it no panel. A `Wrap` of fixed
    // squares was the first port's approximation; it put 17 cells on a row on a
    // wide phone, so the two rings did not line up.
    return GridView.count(
      crossAxisCount: CaptureMetrics.frameGridColumns,
      shrinkWrap: true,
      primary: false,
      padding: const EdgeInsets.only(bottom: 6),
      crossAxisSpacing: 3,
      mainAxisSpacing: 3,
      childAspectRatio: 22 / 16,
      children: [
        for (var i = 0; i < record.statuses.length; i++)
          Tooltip(
            message: retake.contains(i)
                ? 'View ${i + 1}: needs retake'
                : 'View ${i + 1}: ${record.statuses[i].name}',
            child: Container(
              decoration: BoxDecoration(
                color: retake.contains(i)
                    ? CaptureColors.warn
                    : switch (sphereDotRole(record.statuses[i])) {
                        SphereDotRole.ok => CaptureColors.ok,
                        SphereDotRole.warn => CaptureColors.warn,
                        SphereDotRole.neutral => CaptureColors.frameCellEmpty,
                      },
                border: retake.contains(i)
                    ? Border.all(color: CaptureColors.danger, width: 1.5)
                    : null,
                borderRadius: BorderRadius.circular(3),
              ),
            ),
          ),
      ],
    );
  }
}

/// Screen 8.
class AddStationScreen extends StatelessWidget {
  const AddStationScreen({super.key, required this.controller});

  final dynamic controller;

  CaptureFlowEngine get engine => controller.engine;

  @override
  Widget build(BuildContext context) {
    final accepted = engine.acceptedStations;
    final candidates = engine.candidates;
    final next = engine.stations.length + 1;

    return FlowScaffold(
      title: 'Add station',
      onBack: () => controller.goTo(CaptureScreen.stationReview),
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          FloorPlanDiagram(
            stations: [
              for (var i = 0; i < engine.stations.length; i++)
                PlanStation(
                  x: engine.stations[i].planX,
                  y: engine.stations[i].planY,
                  label: engine.stations[i].displayName,
                  accepted:
                      engine.stations[i].outcome == StationOutcome.accepted,
                  current: i == engine.currentStation,
                ),
            ],
            highlight: Rect.fromCenter(
              center: Offset(
                candidates[engine.selectedCandidate].x,
                candidates[engine.selectedCandidate].y,
              ),
              width: 0.28,
              height: 0.28,
            ),
            onTap: (x, y) {
              var best = 0;
              var bestD = double.infinity;
              for (var i = 0; i < candidates.length; i++) {
                final d =
                    (candidates[i].x - x).abs() + (candidates[i].y - y).abs();
                if (d < bestD) {
                  bestD = d;
                  best = i;
                }
              }
              engine.selectedCandidate = best;
              controller.refresh();
            },
          ),
          const SizedBox(height: 14),
          NoteCard(
            icon: Icons.place_outlined,
            child: Text(
              'Station $next will represent the '
              '${candidates[engine.selectedCandidate].label.toLowerCase()}. '
              'Each station makes its own panorama; images are never merged '
              'across stations.',
            ),
          ),
          const SizedBox(height: 8),
          for (var i = 0; i < engine.stations.length; i++)
            _StationRow(
              record: engine.stations[i],
              onView: () {
                engine.currentStation = i;
                controller.goTo(CaptureScreen.stationReview);
              },
            ),
          const SizedBox(height: 10),
          Text(
            '${accepted.length} accepted · estimated total '
            '${engine.recommendation.estimate}',
            style: TextStyle(color: CaptureColors.mutedOnDark),
          ),
          const SizedBox(height: 6),
          const ProvisionalLabel(
            text:
                'The estimated total comes from your room answers, not from '
                'a measurement.',
          ),
        ],
      ),
      footer: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          PrimaryButton(
            label: 'Start capture at Station $next',
            onPressed: () => controller.goTo(CaptureScreen.stationSetup),
          ),
          const SizedBox(height: 8),
          SecondaryButton(
            label: 'Finish room',
            enabled: accepted.isNotEmpty,
            onPressed: () async {
              await engine.finish();
              controller.goTo(CaptureScreen.tourPreview);
            },
          ),
        ],
      ),
    );
  }
}

class _StationRow extends StatelessWidget {
  const _StationRow({required this.record, required this.onView});

  final StationRecord record;
  final VoidCallback onView;

  @override
  Widget build(BuildContext context) => Container(
    // `.sl>div` — one row per station, separated by a hairline rather than by a
    // card, because the reference lists stations the way it lists settings.
    padding: const EdgeInsets.symmetric(vertical: 8),
    decoration: const BoxDecoration(
      border: Border(bottom: BorderSide(color: CaptureColors.hairlineFaint)),
    ),
    child: Row(
      children: [
        Expanded(
          child: Text(
            '${record.displayName} · '
            '${record.capturedCount}/${record.total}',
            style: const TextStyle(fontSize: 13),
          ),
        ),
        LinkButton(label: record.outcome.name, onPressed: onView),
      ],
    ),
  );
}

/// Screen 9.
class TourPreviewScreen extends StatelessWidget {
  const TourPreviewScreen({super.key, required this.controller});

  final dynamic controller;

  CaptureFlowEngine get engine => controller.engine;

  @override
  Widget build(BuildContext context) {
    final accepted = engine.acceptedStations;
    return FlowScaffold(
      title: 'Virtual tour preview',
      onBack: () => controller.goTo(CaptureScreen.addStation),
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          if (accepted.isEmpty)
            const NoteCard(
              tone: NoteTone.warn,
              icon: Icons.info_outline,
              child: Text(
                'No station has been accepted yet. Go back and accept one to '
                'see it here.',
              ),
            )
          else ...[
            for (final s in accepted)
              Container(
                // `.sl>div` again — the same list treatment station 8 uses, so
                // the two screens do not give one station two shapes.
                padding: const EdgeInsets.symmetric(vertical: 8),
                decoration: const BoxDecoration(
                  border: Border(
                    bottom: BorderSide(color: CaptureColors.hairlineFaint),
                  ),
                ),
                child: Row(
                  children: [
                    const Icon(
                      Icons.panorama,
                      color: CaptureColors.ok,
                      size: 20,
                    ),
                    const SizedBox(width: 10),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            s.displayName,
                            style: const TextStyle(fontSize: 13),
                          ),
                          Text(
                            '${s.savedFrames} frames'
                            '${s.coveragePct == null ? '' : ' · ${s.coveragePct!.toStringAsFixed(1)}% coverage'}',
                            style: const TextStyle(
                              color: CaptureColors.mutedOnDark,
                              fontSize: 12,
                            ),
                          ),
                        ],
                      ),
                    ),
                    LinkButton(
                      label: 'Return',
                      onPressed: () {
                        engine.currentStation = engine.stations.indexOf(s);
                        controller.goTo(CaptureScreen.stationReview);
                      },
                    ),
                  ],
                ),
              ),
            const SizedBox(height: 10),
            NoteCard(
              icon: Icons.link,
              child: Text(
                accepted.length > 1
                    ? 'Each station is a separate panorama. They are linked by '
                          'hotspots in the viewer — they are not one sphere and '
                          'not a 3D model.'
                    : 'One station makes one panorama. Add another station to '
                          'link them with hotspots.',
              ),
            ),
          ],
          const NoteCard(
            icon: Icons.cloud_off_outlined,
            child: Text(
              'Nothing has been uploaded. Uploading sends your photos to the '
              'stitching service, and only happens when you choose it.',
            ),
          ),
        ],
      ),
      footer: _tourFooter(context),
    );
  }

  Widget _tourFooter(BuildContext context) {
    final stops = engine.tourViewpoints;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        PrimaryButton(
          label: stops.isEmpty
              ? 'Viewer opens after the panorama comes back'
              : 'Open the 360° viewer',
          enabled: stops.isNotEmpty,
          onPressed: () => Navigator.of(context).push(
            MaterialPageRoute<void>(
              builder: (_) => TourScreen(viewpoints: stops),
            ),
          ),
        ),
        if (stops.isEmpty) ...[
          const SizedBox(height: 8),
          const Text(
            'The viewer needs a stitched equirectangular panorama, and nothing '
            'is stitched on the phone. It opens once the processing service has '
            'returned one for an accepted station.',
            style: TextStyle(fontSize: 12),
          ),
        ],
        const SizedBox(height: 8),
        SecondaryButton(
          label: 'Complete',
          onPressed: () => controller.goTo(CaptureScreen.intro),
        ),
      ],
    );
  }
}
