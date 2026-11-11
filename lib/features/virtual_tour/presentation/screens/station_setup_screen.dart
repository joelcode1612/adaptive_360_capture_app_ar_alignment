import 'dart:async';
import 'package:flutter/material.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_availability.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/capture_flow_engine.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/application/flow_state.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/widgets/camera_feed.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/widgets/flow_widgets.dart';

/// Screen 4: the station position, and the calibration that locks it.
///
/// Ported from the prototype's four-step station setup, with the anchor work
/// the prototype does not have. The prototype marks an approximate position on a
/// diagram; the brief requires the position to be *anchored* — ARCore scans
/// horizontal planes, the user aims slightly down and taps Set Station Here, and
/// a hit-test from the screen centre has to land on a tracked plane.
///
/// The prototype really does block that tap on a miss (`V[4]` step 3 disables the
/// button while no floor is under the reticle), so on the AR path the button
/// stays disabled and says why rather than guessing a point.
///
/// **On the sensor path the gate is different, and the screen says so.** A
/// device with no ARCore has no plane to hit, so there is nothing to aim a ray
/// at; the gate becomes "the motion sensors have a direction", and what the tap
/// locks is a heading rather than a point on the floor. The wording, the note
/// card and the button label all change with the tier, because calling a heading
/// lock "anchored" would be the one thing the brief forbids.
///
/// **The anchor does not stop the user from moving.** It supplies an AR
/// reference and a diagnostic camera-to-station displacement; capture quality
/// still comes from live tracking, alignment and stability checks.
class StationSetupScreen extends StatefulWidget {
  const StationSetupScreen({super.key, required this.controller});

  final dynamic controller;

  @override
  State<StationSetupScreen> createState() => _StationSetupScreenState();
}

class _StationSetupScreenState extends State<StationSetupScreen> {
  CaptureFlowEngine get engine => widget.controller.engine;

  Timer? _hitPoller;
  bool _placing = false;
  bool _showAr = false;

  @override
  void dispose() {
    _hitPoller?.cancel();
    super.dispose();
  }

  bool get _isFirstStation => engine.stations.isEmpty;

  double get _planX => _isFirstStation
      ? engine.firstPosition.x
      : engine.candidates[engine.selectedCandidate].x;

  double get _planY => _isFirstStation
      ? engine.firstPosition.y
      : engine.candidates[engine.selectedCandidate].y;

  Future<void> _beginAr() async {
    setState(() => _placing = true);
    await engine.createStation(planX: _planX, planY: _planY);
    final ok = await engine.beginStationSetup();
    if (!mounted) return;
    setState(() {
      _placing = false;
      _showAr = ok;
    });
    if (!ok) {
      // The station still works without AR; the message says why. Degrading is
      // the point, not an error path.
      widget.controller.refresh();
      return;
    }
    _hitPoller?.cancel();
    _hitPoller = Timer.periodic(const Duration(milliseconds: 250), (_) async {
      await engine.pollStationHit();
      if (mounted) setState(() {});
    });
  }

  Future<void> _setStationHere() async {
    final result = await engine.setStationHere();
    _hitPoller?.cancel();
    _hitPoller = null;
    if (!mounted) return;
    if (!result.ok) {
      setState(() {});
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text(result.message)));
      return;
    }
    widget.controller.goTo(CaptureScreen.arCapture);
  }

  @override
  Widget build(BuildContext context) {
    if (_showAr && (engine.arRunning || engine.poseOnlyGuidance)) {
      return _buildCalibrationView();
    }
    return _buildPlanView();
  }

  // ── the plan view ──────────────────────────────────────────────────────

  Widget _buildPlanView() {
    final index = engine.stations.length;
    final candidates = engine.candidates;

    return FlowScaffold(
      title: 'Station ${index + 1} position',
      onBack: widget.controller.back,
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // `.dg` — the diagram sits in its own translucent group, not on the
          // page background.
          Container(
            width: double.infinity,
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: CaptureColors.groupOnDark,
              borderRadius: BorderRadius.circular(14),
            ),
            child: _planDiagram(candidates),
          ),
          // `V[4]`'s `h3`: the station number, and the word "suggested" only on
          // the first one.
          Padding(
            padding: const EdgeInsets.only(top: 14, bottom: 8),
            child: Text(
              'Station ${index + 1}'
              '${_isFirstStation ? ' · suggested position' : ''}',
              style: const TextStyle(fontSize: 15, fontWeight: FontWeight.w600),
            ),
          ),
          // `.li` — the reference's bullets are a lighter white than the muted
          // paragraphs, because they are instructions, not caveats.
          _bullet(
            _isFirstStation
                ? 'Stand near the marked point in the room.'
                : 'Stand inside the highlighted area: '
                      '${candidates[engine.selectedCandidate].label}.',
          ),
          _bullet('Plant your feet and turn on the spot.'),
          _bullet('Stay close to this point for every view.'),
          const SizedBox(height: 18),
          _buildAnchorSection(),
        ],
      ),
      footer: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          PrimaryButton(
            // The brief requires sensor-based and ARCore tracking to be told
            // apart, so the button names whichever one is about to run.
            label: _placing
                ? 'Starting…'
                : engine.arTier.tier == ArTier.full
                ? 'Begin AR Guidance'
                : 'Begin Sensor Guidance',
            enabled: !_placing,
            onPressed: _beginAr,
          ),
        ],
      ),
    );
  }

  Widget _planDiagram(List<StationCandidate> candidates) => FloorPlanDiagram(
    stations: [
      for (var i = 0; i < engine.stations.length; i++)
        PlanStation(
          x: engine.stations[i].planX,
          y: engine.stations[i].planY,
          label: engine.stations[i].displayName,
          accepted: engine.stations[i].outcome == StationOutcome.accepted,
        ),
    ],
    highlight: _isFirstStation
        ? null
        : Rect.fromCenter(
            center: Offset(
              candidates[engine.selectedCandidate].x,
              candidates[engine.selectedCandidate].y,
            ),
            width: 0.28,
            height: 0.28,
          ),
    onTap: _isFirstStation
        ? null
        : (x, y) {
            // Pick the nearest candidate area. Later stations are chosen on the
            // diagram, as the prototype does.
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
            setState(() => engine.selectedCandidate = best);
          },
  );

  Widget _bullet(String text) => Padding(
    padding: const EdgeInsets.only(bottom: 4),
    child: Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Padding(
          padding: EdgeInsets.only(top: 8, right: 10),
          child: Icon(Icons.circle, size: 5, color: CaptureColors.bulletOnDark),
        ),
        Expanded(
          child: Text(
            text,
            style: const TextStyle(color: CaptureColors.bulletOnDark),
          ),
        ),
      ],
    ),
  );

  Widget _buildAnchorSection() {
    final tier = engine.arTier;
    if (!tier.guidanceEnabled) {
      // "If the device does not support ARCore, disable the anchor check and
      // say so." This is that message, and it is not an error: the station is
      // placed by position only and the capture runs on the manual path.
      return NoteCard(
        tone: NoteTone.warn,
        icon: Icons.explore_off,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'The station cannot be anchored',
              style: TextStyle(fontWeight: FontWeight.w600),
            ),
            const SizedBox(height: 4),
            Text(tier.limitation ?? 'AR is not available on this device.'),
            const SizedBox(height: 6),
            const Text(
              'You can still capture: aim by eye and use the manual button. '
              'No anchor-backed station displacement is available.',
            ),
          ],
        ),
      );
    }
    return NoteCard(
      tone: tier.tier == ArTier.full ? NoteTone.neutral : NoteTone.warn,
      icon: tier.tier == ArTier.full ? Icons.anchor : Icons.explore_off,
      child: tier.tier == ArTier.full
          ? const Text(
              'AR will scan the floor when you continue. Aim slightly down at '
              'the floor and tap Set Station Here when a surface is found — '
              'the button stays disabled until the hit-test actually lands on '
              'one.',
            )
          : Text(
              '${tier.headline}. ${tier.detail}\n\n'
              'The station is then locked to the direction you are facing, not '
              'to a point on the floor. ${tier.limitation ?? ''}',
            ),
    );
  }

  // ── the calibration view ───────────────────────────────────────────────

  /// The aim-and-set screen, in whichever mode this device got.
  ///
  /// Two modes share this widget because they differ in exactly one gate: with
  /// AR the tap needs a hit-test that landed on the floor, without AR it needs a
  /// direction from the sensors. [CaptureFlowEngine.stationLockReady] is the only
  /// place that decides, so the screen cannot disagree with the engine about when
  /// a station may be locked.
  Widget _buildCalibrationView() {
    final poseOnly = engine.poseOnlyGuidance;
    final ready = engine.stationLockReady;
    return CameraScaffold(
      body: Column(
        children: [
          FlowHeader(
            // `currentStation`, not `stations.length + 1`: this view is only
            // reachable after `createStation` has appended the station being
            // anchored, so the length is already one ahead of it.
            title: 'Station ${engine.currentStation + 1}',
            onBack: _cancelCalibration,
          ),
          Expanded(
            child: Stack(
              fit: StackFit.expand,
              children: [
                // The same widget the capture screen uses, for the same reason:
                // this view used to ask the camera for a preview and, when the
                // camera was closed, quietly paint a black box with no message
                // and no way to re-open it.
                CameraFeed(
                  camera: engine.activeCamera,
                  arOwnsSensor: engine.arRunning,
                  reason: engine.fatal,
                  onReopen: () async => widget.controller.reopenCamera(),
                ),
                Center(
                  child: _Reticle(
                    found: ready,
                    label: poseOnly ? 'heading' : 'floor',
                  ),
                ),
                Positioned(
                  left: 12,
                  top: 12,
                  right: 12,
                  child: Center(
                    child: Container(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 14,
                        vertical: 8,
                      ),
                      decoration: BoxDecoration(
                        color: CaptureColors.cameraChrome.withValues(
                          alpha: 0.85,
                        ),
                        borderRadius: BorderRadius.circular(999),
                      ),
                      child: Text(
                        engine.status.isEmpty
                            ? 'Move the phone slowly so AR can find the floor.'
                            : engine.status,
                        style: const TextStyle(
                          color: Colors.white,
                          fontSize: 13,
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
      footer: Container(
        color: CaptureColors.cameraChrome,
        padding: const EdgeInsets.fromLTRB(18, 12, 18, 18),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const SizedBox(height: 6),
            Text(
              poseOnly
                  ? 'Set the station at your heading'
                  : 'Set the station where you stand',
              style: const TextStyle(
                color: Colors.white,
                fontSize: 22,
                fontWeight: FontWeight.w600,
              ),
            ),
            const SizedBox(height: 4),
            Text(
              poseOnly
                  ? 'The direction you are facing becomes the start of the '
                        'capture plan. Nothing is anchored to the floor, so the '
                        'app cannot tell you when you have stepped away.'
                  : 'Point the camera slightly down at the floor. The app casts '
                        'a ray from the centre of the screen, and the button '
                        'stays disabled until it lands on a tracked surface.',
              style: const TextStyle(
                color: CaptureColors.mutedOnDark,
                fontSize: 13,
                height: 1.35,
              ),
            ),
            const SizedBox(height: 12),
            PrimaryButton(
              label: ready
                  ? 'Set Station Here'
                  : poseOnly
                  ? 'Waiting for the sensors…'
                  : 'Looking for the floor…',
              enabled: ready,
              onPressed: _setStationHere,
            ),
            const SizedBox(height: 8),
            SecondaryButton(label: 'Back', onPressed: _cancelCalibration),
          ],
        ),
      ),
    );
  }

  /// Leaves the calibration view without losing the station that was just made.
  Future<void> _cancelCalibration() async {
    _hitPoller?.cancel();
    _hitPoller = null;
    await engine.endAr();
    if (mounted) setState(() => _showAr = false);
  }
}

/// The aiming reticle: a ring that fills in when the gate for this mode is met.
class _Reticle extends StatelessWidget {
  const _Reticle({required this.found, this.label});

  final bool found;

  /// What the ring is waiting for, so the two modes do not read identically.
  final String? label;

  @override
  Widget build(BuildContext context) => SizedBox(
    width: CaptureColors.reticleSize,
    height: CaptureColors.reticleSize,
    child: Stack(
      alignment: Alignment.center,
      children: [
        CustomPaint(
          painter: _ReticlePainter(
            color: found ? CaptureColors.ok : Colors.white70,
          ),
        ),
        if (label != null)
          Padding(
            padding: const EdgeInsets.only(top: CaptureColors.reticleSize - 16),
            child: Text(
              label!,
              style: TextStyle(
                color: found ? CaptureColors.ok : Colors.white70,
                fontSize: 10,
              ),
            ),
          ),
      ],
    ),
  );
}

class _ReticlePainter extends CustomPainter {
  _ReticlePainter({required this.color});

  final Color color;

  @override
  void paint(Canvas canvas, Size size) {
    final c = Offset(size.width / 2, size.height / 2);
    final r = size.width / 2 - 2;
    canvas.drawCircle(
      c,
      r,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..color = color,
    );
    final tick = Paint()
      ..strokeWidth = 2
      ..color = color;
    canvas.drawLine(c - Offset(r * 0.5, 0), c - Offset(r * 0.2, 0), tick);
    canvas.drawLine(c + Offset(r * 0.5, 0), c + Offset(r * 0.2, 0), tick);
    canvas.drawLine(c - Offset(0, r * 0.5), c - Offset(0, r * 0.2), tick);
    canvas.drawLine(c + Offset(0, r * 0.5), c + Offset(0, r * 0.2), tick);
  }

  @override
  bool shouldRepaint(_ReticlePainter old) => old.color != color;
}

/// The camera view is not available while AR holds the sensor.
///
/// Kept as a named type so the reason is greppable: on a device where ARCore
/// cannot supply a camera image, the capture screen shows the still camera's
/// preview instead, and this is the flag that says which one is in play.
bool arOwnsTheCamera(ArCapabilities caps) => caps.canShootWhileRunning;
