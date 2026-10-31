import 'package:flutter/material.dart';

import 'package:adaptive_360_capture_app/core/permissions/permission_service.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_controller.dart';
import 'package:adaptive_360_capture_app/features/capture/application/capture_state.dart';
import 'package:adaptive_360_capture_app/features/capture/data/repositories/capture_repository_impl.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/ar_session_service.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/photo_capture_service.dart';
import 'package:adaptive_360_capture_app/features/capture/data/services/plane_anchor_service.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/shot_phase.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/capture/presentation/widgets/capture_progress_widget.dart';
import 'package:adaptive_360_capture_app/features/capture/presentation/widgets/spherical_target_map.dart';
import 'package:adaptive_360_capture_app/features/capture/presentation/widgets/target_verdict.dart';
import 'package:adaptive_360_capture_app/features/capture/presentation/widgets/target_alignment_overlay.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/arcore_ar.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';

/// Section 1: the real AR capture screen.
///
/// The camera view is a Flutter [Texture] fed by the ARCore session's own GL
/// renderer — the camera image and the anchored GLB marker are drawn by the
/// native host into that one texture. Nothing in this file simulates a pose, and
/// nothing here decides where the target is: the marker's position comes from an
/// ARCore anchor and the guidance comes from the alignment calculator, both of
/// which are pure and tested.
class CaptureModuleScreen extends StatefulWidget {
  const CaptureModuleScreen({
    super.key,
    this.geometry = const CaptureGeometryConfig(),
  });

  final CaptureGeometryConfig geometry;

  @override
  State<CaptureModuleScreen> createState() => _CaptureModuleScreenState();
}

class _CaptureModuleScreenState extends State<CaptureModuleScreen>
    with WidgetsBindingObserver {
  CaptureController? _controller;
  String? _bootError;
  bool _booted = false;
  int? _surfaceWidth;
  int? _surfaceHeight;
  bool _surfaceSyncQueued = false;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
  }

  Future<void> _boot(Size logical) async {
    if (_booted || _controller != null) return;
    _booted = true;
    final ratio = MediaQuery.of(context).devicePixelRatio;
    try {
      // The camera permission is asked before the session is created: on Android
      // the session fails with a generic "camera not available", which does not
      // tell the user that they have to grant anything.
      await PermissionService().ensureCapturePermissions();

      final repository = await FileCaptureRepository.create();
      final host = ArCoreHost();
      final session = ArSessionService(host);
      final anchors = PlaneAnchorService(host, geometry: widget.geometry);
      final photos = PhotoCaptureService(host: host, repository: repository);

      final controller = CaptureController(
        session: session,
        anchors: anchors,
        photos: photos,
        repository: repository,
        geometry: widget.geometry,
      );
      if (!mounted) {
        controller.dispose();
        return;
      }
      setState(() => _controller = controller);
      controller.addListener(_onChanged);
      await controller.boot(
        width: (logical.width * ratio).round(),
        height: (logical.height * ratio).round(),
      );
      _surfaceWidth = (logical.width * ratio).round();
      _surfaceHeight = (logical.height * ratio).round();
    } catch (e) {
      if (!mounted) return;
      setState(() => _bootError = '$e');
    }
  }

  void _onChanged() {
    if (mounted) setState(() {});
  }

  @override
  void didChangeMetrics() {
    super.didChangeMetrics();
    if (_surfaceSyncQueued) return;
    _surfaceSyncQueued = true;
    // Metrics may change before Flutter has laid out the Texture. Waiting for
    // that frame makes the physical ARCore viewport match the actual Flutter
    // viewport, rather than a stale pre-rotation or multi-window size.
    WidgetsBinding.instance.addPostFrameCallback((_) async {
      _surfaceSyncQueued = false;
      if (!mounted) return;
      final controller = _controller;
      if (controller == null) return;

      final physical = View.of(context).physicalSize;
      final width = physical.width.round();
      final height = physical.height.round();
      if (width <= 0 || height <= 0 ||
          (width == _surfaceWidth && height == _surfaceHeight)) {
        return;
      }

      _surfaceWidth = width;
      _surfaceHeight = height;
      try {
        await controller.resize(width: width, height: height);
      } catch (_) {
        // The controller reports native-session failures through its state. A
        // metric notification must not crash Flutter while that happens.
      }
    });
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    final controller = _controller;
    if (controller == null) return;
    switch (state) {
      case AppLifecycleState.paused:
      case AppLifecycleState.hidden:
      case AppLifecycleState.detached:
        controller.onAppPaused();
      case AppLifecycleState.resumed:
        // **`inactive` deliberately does not resume.** It fires for the
        // notification shade, the app-switcher preview and the camera
        // permission dialog — none of which stopped the AR session. Treating it
        // as a resume re-ran `boot()`, which reset the phase to `scanning` while
        // keeping the calibrated station and its targets: the screen then showed
        // a live target, live tracking and a shutter disabled with "AR is still
        // starting." for the rest of the session.
        final size = MediaQuery.of(context).size;
        final ratio = MediaQuery.of(context).devicePixelRatio;
        controller.onAppResumed(
          width: (size.width * ratio).round(),
          height: (size.height * ratio).round(),
        );
      case AppLifecycleState.inactive:
        // Nothing to do: the session is still running and the phase still
        // describes it.
        break;
    }
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _controller?.removeListener(_onChanged);
    _controller?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_bootError != null) return _Failure(message: _bootError!);

    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        final size = Size(constraints.maxWidth, constraints.maxHeight);
        if (_controller == null) {
          // Boot after layout, because the AR surface has to be created at the
          // size the viewfinder actually is.
          WidgetsBinding.instance.addPostFrameCallback((_) => _boot(size));
        }
        final controller = _controller;
        if (controller == null) {
          return const Scaffold(
            backgroundColor: Colors.black,
            body: Center(child: CircularProgressIndicator()),
          );
        }
        return _CaptureBody(controller: controller);
      },
    );
  }
}

/// Shown when the module could not even be built.
class _Failure extends StatelessWidget {
  const _Failure({required this.message});

  final String message;

  @override
  Widget build(BuildContext context) => Scaffold(
    backgroundColor: CaptureColors.cameraChrome,
    body: SafeArea(
      child: Center(
        child: Padding(
          padding: const EdgeInsets.all(24),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Icon(
                Icons.view_in_ar_outlined,
                size: 48,
                color: Colors.white70,
              ),
              const SizedBox(height: 16),
              const Text(
                'AR capture could not start',
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 20,
                  fontWeight: FontWeight.w600,
                ),
              ),
              const SizedBox(height: 8),
              Text(
                message,
                textAlign: TextAlign.center,
                style: const TextStyle(color: CaptureColors.mutedOnDark),
              ),
            ],
          ),
        ),
      ),
    ),
  );
}

class _CaptureBody extends StatelessWidget {
  const _CaptureBody({required this.controller});

  final CaptureController controller;

  @override
  Widget build(BuildContext context) {
    final state = controller.state;
    final textureId = state.textureId;

    return PopScope(
      canPop: false,
      child: Scaffold(
        backgroundColor: Colors.black,
        body: Stack(
          fit: StackFit.expand,
          children: [
            // ── the AR camera scene ──────────────────────────────────────
            if (textureId != null && textureId >= 0)
              Texture(textureId: textureId, filterQuality: FilterQuality.none)
            else
              const ColoredBox(color: Colors.black),

            // Keep the physical aim ring on screen for the whole calibrated
            // station. After a successful shot the controller deliberately
            // clears the active target (so its arrow cannot point at a saved
            // node), but removing the ring made that safe transition look like
            // an AR failure. With no target the painter draws only this neutral
            // reticle; the status line says “Select a target”.
            if (state.station != null && state.phase == CapturePhase.capturing)
              TargetAlignmentOverlay(
                // One alignment for the arrow, the ring and the shutter. It was
                // `displayAlignment` here — a filtered copy of the same pose —
                // so the pointer lagged the arithmetic it was meant to picture.
                alignment: state.alignment,
                withinTolerance: state.alignment?.withinTolerance ?? false,
                // The ring fills with the stability window — but **only when
                // every other capture condition already holds**. A ring at 100%
                // now means `readyToCapture`, so a full ring can never sit above
                // a shutter that is blocked on drift or an untracked anchor.
                stabilityProgress:
                    state.displayCaptureBlockReasonBeforeSettle == null
                    ? state.stabilityProgress
                    : 0,
                // Alignment alone cannot arm a shot: walking away from the
                // calibrated station invalidates the composition. Keep the
                // central reticle visually honest with the disabled shutter.
                // With no target selected this is a neutral, persistent
                // reticle after a completed shot.  A prior target's distance
                // warning must not keep it amber; selecting the next target
                // re-enables the real station-distance gate.
                captureBlocked:
                    state.guidanceTarget != null &&
                    state.stationDriftWarningVisible &&
                    // The amber reticle is presentation, not safety: the
                    // shutter stays blocked by `captureBlockReason` either
                    // way. Gated while tracking is debugged on a device.
                    kStationDriftWarningVisualsEnabled,
                // A paused anchor is now hidden rather than drawn from its
                // stale pose, so the overlay must remain neutral too. The
                // marker and its guidance therefore always appear and disappear
                // together.
                anchored:
                    state.activeAnchored && state.activeAnchorTracking,
                // What the arrow *cannot* work without is a camera pose: its
                // direction is the projection of the target through that pose.
                // When AR is not tracking, no arrow — the instruction line
                // carries the tracking state instead.
                // During tracking recovery the camera has a pose again, but
                // it has not yet been proven continuous with the frozen
                // station. Keep the reticle neutral until that check passes.
                poseValid:
                    state.tracking == TrackingState.tracking &&
                    !state.trackingRecovering,
                // The arrow projects the target with the same field of view the
                // session reported, falling back to it only when the renderer
                // could not provide its display-geometry-aware projection.
                displayFov: state.displayFov,
                displayProjectionMatrix: state.displayProjectionMatrix,
                // The ring *is* the shipped bezel, so the reticle and the marker
                // on the wall are the same shape.
                bezel: controller.bezelGeometry,
                completed: state.shotPhase == ShotPhase.accept,
              ),

            // A native render failure is not a hint, it is the reason the
            // preview is black — so it takes the whole top band, states the
            // exception, and offers the one action that can fix it.
            if (state.arFault != null)
              Positioned(
                left: 12,
                right: 12,
                top: MediaQuery.paddingOf(context).top + 8,
                child: _FaultBanner(
                  message: state.arFault!,
                  frame: state.arFaultFrame,
                  onRetry: controller.retrySession,
                ),
              )
            else
              // One line of status, expandable. Always present: "is AR
              // tracking?" and "how many planes?" are asked before anything
              // else works, and both are answered on the collapsed line.
              _StatusBar(
                controller: controller,
                top: MediaQuery.paddingOf(context).top + 8,
                right: 108,
              ),

            Positioned(
              right: 12,
              top: MediaQuery.paddingOf(context).top + 8,
              child: _MiniMapButton(controller: controller),
            ),

            Align(
              alignment: Alignment.bottomCenter,
              child: _ControlPanel(controller: controller),
            ),
          ],
        ),
      ),
    );
  }
}

class _FaultBanner extends StatelessWidget {
  const _FaultBanner({
    required this.message,
    required this.frame,
    required this.onRetry,
  });

  final String message;
  final String? frame;
  final Future<void> Function() onRetry;

  @override
  Widget build(BuildContext context) => Container(
    padding: const EdgeInsets.fromLTRB(12, 10, 12, 10),
    decoration: BoxDecoration(
      color: CaptureColors.cameraChrome.withValues(alpha: 0.94),
      border: Border.all(color: CaptureColors.danger),
      borderRadius: BorderRadius.circular(10),
    ),
    child: Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            const Icon(
              Icons.error_outline,
              color: CaptureColors.danger,
              size: 18,
            ),
            const SizedBox(width: 6),
            const Expanded(
              child: Text(
                'The AR view could not render',
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 13,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ),
            TextButton(
              onPressed: () => onRetry(),
              style: TextButton.styleFrom(
                foregroundColor: CaptureColors.warn,
                padding: const EdgeInsets.symmetric(horizontal: 8),
                minimumSize: Size.zero,
                tapTargetSize: MaterialTapTargetSize.shrinkWrap,
              ),
              child: const Text('Retry'),
            ),
          ],
        ),
        const SizedBox(height: 4),
        Text(
          message,
          maxLines: 3,
          overflow: TextOverflow.ellipsis,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 11,
            height: 1.35,
            fontFamily: 'monospace',
          ),
        ),
        if (frame != null) ...[
          const SizedBox(height: 2),
          Text(
            frame!,
            maxLines: 1,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(
              color: CaptureColors.mutedOnDark,
              fontSize: 10,
              fontFamily: 'monospace',
            ),
          ),
        ],
        const SizedBox(height: 4),
        const Text(
          'Retry restarts the AR session. The station will need calibrating '
          'again, because the new session has a new world origin.',
          style: TextStyle(
            color: CaptureColors.mutedOnDark,
            fontSize: 10,
            height: 1.3,
          ),
        ),
      ],
    ),
  );
}

class _MiniMapButton extends StatelessWidget {
  const _MiniMapButton({required this.controller});

  final CaptureController controller;

  @override
  Widget build(BuildContext context) {
    final state = controller.state;
    return SizedBox.square(
      dimension: CaptureMetrics.miniMapSize,
      child: FilledButton(
        style: FilledButton.styleFrom(
          backgroundColor: CaptureColors.cameraChrome.withValues(alpha: 0.72),
          padding: const EdgeInsets.all(5),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(CaptureMetrics.miniMapRadius),
          ),
        ),
        onPressed: () => _openTargetSheet(context, controller),
        child: SphericalTargetMap(
          specs: controller.specs,
          targets: state.targets,
          activeTargetId: state.activeTargetId,
          onSelect: (_) {},
          size: CaptureMetrics.miniMapSize - 10,
        ),
      ),
    );
  }
}

Future<void> _openTargetSheet(
  BuildContext context,
  CaptureController controller,
) => showModalBottomSheet<void>(
  context: context,
  backgroundColor: CaptureColors.sheet,
  isScrollControlled: true,
  builder: (BuildContext sheetContext) => _TargetSheet(controller: controller),
);

/// The 30 targets: the sphere for choosing one, and a list that says, for each,
/// whether it works.
///
/// The list exists because the sphere shows *where* a target is and whether it is
/// captured, but not *why* one is not. A target can be un-capturable for three
/// different reasons — no surface found, the save failed, or nothing attempted —
/// and those need different actions. Each row therefore carries an explicit
/// verdict rather than a colour to interpret.
class _TargetSheet extends StatelessWidget {
  const _TargetSheet({required this.controller});

  final CaptureController controller;

  @override
  Widget build(BuildContext context) {
    final state = controller.state;
    final targets = state.targets;

    final ready = targets
        .where(
          (CaptureTarget t) =>
              !t.status.isDone &&
              targetVerdict(
                t,
                probe: state.probes[t.targetId],
              ).verdict.isActionable,
        )
        .length;
    final blocked = targets.length - state.capturedCount - ready;

    return SafeArea(
      child: ConstrainedBox(
        constraints: BoxConstraints(
          maxHeight: MediaQuery.sizeOf(context).height * 0.88,
        ),
        child: Padding(
          padding: const EdgeInsets.fromLTRB(18, 14, 18, 16),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Text(
                'Capture targets',
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 16,
                  fontWeight: FontWeight.w600,
                ),
              ),
              const SizedBox(height: 4),
              Text(
                targets.isEmpty
                    ? 'Calibrate the station to create the 30 targets.'
                    : '${state.capturedCount} of ${targets.length} saved · '
                          '$ready with a surface · $blocked blocked',
                style: const TextStyle(
                  color: CaptureColors.mutedOnDark,
                  fontSize: 12,
                ),
              ),
              const SizedBox(height: 12),
              if (targets.isNotEmpty) ...[
                Center(
                  child: SphericalTargetMap(
                    specs: controller.specs,
                    targets: targets,
                    activeTargetId: state.activeTargetId,
                    size: 220,
                    showLabels: true,
                    onSelect: (int id) {
                      // A saved target is not selectable: its guidance is
                      // finished. Re-taking one is deliberate, from its row.
                      if (id < 1 || id > targets.length) return;
                      if (targets[id - 1].status.isDone) return;
                      controller.selectTarget(id);
                      Navigator.of(context).pop();
                    },
                  ),
                ),
                const SizedBox(height: 12),
                CaptureProgressWidget(
                  targets: targets,
                  specs: controller.specs,
                ),
                const SizedBox(height: 8),
                _GhostButton(
                  label: state.probes.isEmpty
                      ? 'Scan all directions'
                      : 'Re-scan all directions',
                  enabled: state.station != null,
                  onPressed: () => controller.probeTargets(),
                ),
                const SizedBox(height: 8),
                const Divider(color: Color(0x22FFFFFF), height: 1),
                const SizedBox(height: 6),
                Flexible(
                  child: ListView.builder(
                    shrinkWrap: true,
                    itemCount: targets.length,
                    itemBuilder: (BuildContext context, int i) => _TargetRow(
                      target: targets[i],
                      probe: state.probes[targets[i].targetId],
                      // The only way back into a captured target. The
                      // controller refuses to *select* a saved one, so the
                      // list has to offer the explicit reset instead of
                      // hiding the action behind an active selection.
                      onRetake: targets[i].status.isDone
                          ? () {
                              controller.retake(targets[i].targetId);
                              Navigator.of(context).pop();
                            }
                          : null,
                    ),
                  ),
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }
}

class _ControlPanel extends StatefulWidget {
  const _ControlPanel({required this.controller});

  final CaptureController controller;

  @override
  State<_ControlPanel> createState() => _ControlPanelState();
}

/// The bottom "dropup".
///
/// Collapsed it shows only what a shot needs: the instruction, the phase, the
/// shutter, and one line saying why the shutter is shut. Everything else — the
/// secondary action and the numeric alignment — is one tap away.
///
/// **What never collapses, and why.** The block reason stays on screen in both
/// states. A collapsed panel that hides *why* the button is disabled would be a
/// greyed-out button with no explanation, which this project has explicitly
/// ruled out. The instruction stays too: it is the guidance, not the diagnostics.
class _ControlPanelState extends State<_ControlPanel> {
  bool _expanded = false;

  @override
  Widget build(BuildContext context) {
    final controller = widget.controller;
    final state = controller.state;
    final calibrationReadiness = controller.calibrationReadiness;
    final alignment = state.alignment;
    final reason = state.displayCaptureBlockReason;

    return Container(
      width: double.infinity,
      color: CaptureColors.cameraChrome,
      padding: const EdgeInsets.fromLTRB(18, 4, 18, 14),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          _CollapseHandle(
            expanded: _expanded,
            label: _expanded ? 'Hide controls' : 'Show controls',
            onTap: () => setState(() => _expanded = !_expanded),
          ),
          Text(
            state.instruction,
            maxLines: _expanded ? 2 : 1,
            overflow: TextOverflow.ellipsis,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 18,
              fontWeight: FontWeight.w600,
            ),
          ),
          const SizedBox(height: 8),
          // The capture status, from the controller's own published state —
          // not recomputed here. `CaptureState.captureStatus` reads the same
          // fields the block reason and the button do, in the same order, so
          // the sentence, the greyed-out button and the shutter cannot
          // contradict each other.
          Text(
            state.captureStatus,
            maxLines: 1,
            overflow: TextOverflow.ellipsis,
            style: TextStyle(
              color: state.stationDriftWarningVisible
                  ? CaptureColors.warn
                  : state.readyToCapture
                  ? CaptureColors.ok
                  : CaptureColors.mutedOnDark,
              fontSize: 12,
              fontWeight: FontWeight.w600,
              letterSpacing: 0.2,
            ),
          ),
          const SizedBox(height: 8),
          Row(
            children: [
              _PhaseChip(
                phase: state.shotPhase,
                stationDriftWarningActive: state.stationDriftWarningVisible,
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  '${state.capturedCount} of ${state.totalTargets} captured'
                  '${state.station == null ? '' : ' · drift ${state.driftM.toStringAsFixed(2)} m'}',
                  maxLines: 1,
                  overflow: TextOverflow.ellipsis,
                  style: const TextStyle(
                    color: CaptureColors.mutedOnDark,
                    fontSize: 12,
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 10),
          if (state.station == null) ...[
            _PrimaryButton(
              label: 'Calibrate station',
              enabled: calibrationReadiness.isReady,
              onPressed: () => controller.calibrateStation(),
            ),
            if (!calibrationReadiness.isReady &&
                calibrationReadiness.reason != null) ...[
              const SizedBox(height: 6),
              Text(
                calibrationReadiness.reason!,
                style: const TextStyle(
                  color: CaptureColors.warn,
                  fontSize: 11,
                  height: 1.3,
                ),
              ),
            ],
            // Continue a station that already has photographs on disk. The
            // count shown comes from the saved records, so what the button
            // promises is what the continue restores. In a new AR session the
            // saved world coordinates are meaningless, so the label and the
            // action say "recalibrate" instead of pretending they are valid.
            if (controller.resumableStationId != null) ...[
              const SizedBox(height: 8),
              _GhostButton(
                label: controller.resumeKeepsWorldFrame
                    ? 'Resume last capture'
                    : 'Recalibrate & continue',
                // Both actions ultimately record/reuse a camera pose.  Use
                // the same controller-owned validation as the primary
                // calibration button so recovery cannot offer an enabled
                // action for a stale pose or an undefined horizontal yaw.
                enabled: calibrationReadiness.isReady,
                onPressed: () => controller.continueSavedStation(),
              ),
            ],
          ] else ...[
            const Text(
              'Automatic capture — hold the aim circle on the target.',
              style: TextStyle(color: CaptureColors.mutedOnDark, fontSize: 12),
            ),
            if (_expanded) ...[
              const SizedBox(height: 8),
              Row(
                children: [
                  Expanded(
                    child: state.stationDriftWarningVisible
                        // A drifted user has two honest options: walk back, or
                        // anchor a new station here. "Scan again" is neither —
                        // it re-places the arrangement around the station they
                        // have left.
                        ? _GhostButton(
                            label: 'Recalibrate here',
                            enabled:
                                !state.capturing &&
                                state.tracking == TrackingState.tracking,
                            onPressed: () => controller.recalibrateHere(),
                          )
                        : state.guidanceTarget == null
                        ? _GhostButton(
                            label: 'Review targets',
                            enabled: !state.capturing,
                            onPressed: () => _openTargetSheet(context, controller),
                          )
                        // "Scan again" removed from the post-calibration interface:
                        // calibration runs once per session; rescan is not offered.
                        : const SizedBox.shrink(),
                  ),
                ],
              ),
            ],
          ],
          // Always shown, in both states. See the class comment.
          if (reason != null) ...[
            const SizedBox(height: 6),
            Text(
              reason,
              maxLines: _expanded ? 3 : 1,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(
                color: CaptureColors.warn,
                fontSize: 11,
                height: 1.3,
              ),
            ),
          ],
          if (_expanded && alignment != null && alignment.usable) ...[
            const SizedBox(height: 8),
            Text(
              'yaw ${_signed(alignment.yawCorrectionDeg)}° · '
              'pitch ${_signed(alignment.pitchCorrectionDeg)}° · '
              'error ${alignment.angularErrorDeg.toStringAsFixed(1)}° '
              '(tol ${state.geometry.alignmentToleranceDeg.toStringAsFixed(1)}°)',
              style: const TextStyle(
                color: CaptureColors.mutedOnDark,
                fontSize: 11,
              ),
            ),
          ],
        ],
      ),
    );
  }
}

/// The top "dropdown": one line of status, expandable to the full detail.
///
/// It is always present, because "is AR tracking?" and "how many planes?" are
/// questions the user asks *before* anything else works — and on the collapsed
/// line those two answers are already visible.
class _StatusBar extends StatefulWidget {
  const _StatusBar({
    required this.controller,
    required this.top,
    this.right = 12,
  });

  final CaptureController controller;
  final double top;
  final double right;

  @override
  State<_StatusBar> createState() => _StatusBarState();
}

class _StatusBarState extends State<_StatusBar> {
  bool _expanded = false;

  @override
  Widget build(BuildContext context) {
    final state = widget.controller.state;
    final detail = _detailText(state);

    return Positioned(
      left: 12,
      right: widget.right,
      top: widget.top,
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          borderRadius: BorderRadius.circular(10),
          onTap: () => setState(() => _expanded = !_expanded),
          child: Container(
            padding: const EdgeInsets.fromLTRB(12, 8, 8, 8),
            decoration: BoxDecoration(
              color: CaptureColors.cameraChrome.withValues(alpha: 0.92),
              borderRadius: BorderRadius.circular(10),
              border: Border.all(
                color: CaptureColors.mutedOnDark.withValues(alpha: 0.35),
              ),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    Icon(
                      _statusIcon(state),
                      size: 16,
                      color: _statusColor(state),
                    ),
                    const SizedBox(width: 6),
                    Expanded(
                      child: Text(
                        _statusLine(state),
                        maxLines: 1,
                        overflow: TextOverflow.ellipsis,
                        style: const TextStyle(
                          color: Colors.white,
                          fontSize: 12,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                    ),
                    Icon(
                      _expanded
                          ? Icons.keyboard_arrow_up
                          : Icons.keyboard_arrow_down,
                      size: 18,
                      color: CaptureColors.mutedOnDark,
                    ),
                  ],
                ),
                AnimatedSize(
                  duration: const Duration(milliseconds: 160),
                  curve: Curves.easeOutCubic,
                  alignment: Alignment.topCenter,
                  child: _expanded
                      ? Padding(
                          padding: const EdgeInsets.only(top: 6),
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                detail,
                                style: const TextStyle(
                                  color: CaptureColors.mutedOnDark,
                                  fontSize: 11,
                                  height: 1.4,
                                ),
                              ),
                              const SizedBox(height: 8),
                              Align(
                                alignment: Alignment.centerLeft,
                                child: _GhostButton(
                                  label: 'All ${state.totalTargets} targets',
                                  enabled: true,
                                  onPressed: () =>
                                      _openTargets(context, widget.controller),
                                ),
                              ),
                            ],
                          ),
                        )
                      : const SizedBox(width: double.infinity),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

Future<void> _openTargets(BuildContext context, CaptureController c) =>
    showModalBottomSheet<void>(
      context: context,
      backgroundColor: CaptureColors.sheet,
      isScrollControlled: true,
      builder: (BuildContext sheetContext) => _TargetSheet(controller: c),
    );

IconData _statusIcon(CaptureState state) {
  if (state.arFault != null) return Icons.error_outline;
  if (state.tracking != TrackingState.tracking) return Icons.gps_off;
  if (state.planesTracked == 0) return Icons.grid_off;
  if (state.readyToCapture) return Icons.check_circle_outline;
  return Icons.my_location;
}

Color _statusColor(CaptureState state) {
  if (state.arFault != null) return CaptureColors.danger;
  if (state.tracking != TrackingState.tracking) return CaptureColors.warn;
  if (state.planesTracked == 0) return CaptureColors.warn;
  if (state.readyToCapture) return CaptureColors.ok;
  return CaptureColors.mutedOnDark;
}

/// The one line shown when the status bar is collapsed.
String _statusLine(CaptureState state) {
  final tracking = state.tracking == TrackingState.tracking
      ? 'AR tracking OK'
      : _trackingLabel(state.tracking);
  return '$tracking · ${state.planeSummary} · '
      '${state.capturedCount}/${state.totalTargets}';
}

String _signed(double v) => '${v >= 0 ? '+' : ''}${v.toStringAsFixed(1)}';

/// What to do about the current tracking state, in one sentence, or `null`
/// when tracking is fine and nothing needs saying — a permanently-on advice
/// line is advice nobody reads.
String? _trackingAdvice(CaptureState state) {
  if (state.tracking == TrackingState.tracking) {
    // Tracking is fine. Nothing else is needed for a target: placement is
    // computed from the station, not discovered from the room.
    //
    // No guidance target means either nothing is selected yet or every
    // photograph is saved. In both cases advice about placing "the target"
    // names an action with no target to place.
    if (state.guidanceTarget == null) return null;
    if (!state.activeAnchored) {
      return 'Tracking is live — calibrate the station to place the target.';
    }
    return null;
  }
  // Every other state's wording lives with the state, so the instruction line
  // and this detail can never drift apart.
  return state.trackingGuidance;
}

/// `tracking` on its own reads as a stutter in "Tracking tracking".
String _trackingLabel(TrackingState tracking) => switch (tracking) {
  TrackingState.tracking => 'AR tracking OK',
  TrackingState.initialising => 'AR starting',
  TrackingState.paused => 'AR tracking paused',
  TrackingState.stopped => 'AR tracking stopped',
  TrackingState.unavailable => 'AR unavailable',
};

String _detailText(CaptureState state) {
  final target = state.activeTarget;
  final parts = <String>[];
  if (target != null) {
    parts.add(
      'Target ${target.targetId} · yaw ${target.yawDegrees.toStringAsFixed(0)}° '
      '· pitch ${target.pitchDegrees.toStringAsFixed(0)}°',
    );
  }
  parts.add(
    !state.activeAnchored
        ? 'Target not anchored'
        // Placed is not the same as tracked, and both are drawn: a paused
        // anchor keeps its world position and its mesh — ARCore is refining
        // the world, not moving the marker. What a pause does cost is the
        // shutter, which stays blocked until tracking is back.
        : state.activeAnchorTracking
        ? 'Placed in the AR world'
        : 'Placed — AR is refining it',
  );
  // "0 planes" alone is not actionable; the breakdown says *what* is missing.
  parts.add('${_trackingLabel(state.tracking)} · ${state.planeSummary}');
  // "no planes yet" reads like a blocker, and it is not one: every target is
  // placed at a *computed* world position on a world anchor with no trackable,
  // so an unmapped room changes nothing about placement, capture or the
  // sphere. Saying so is cheaper than the user pointing the phone at a wall.
  if (state.tracking == TrackingState.tracking && state.planesTracked == 0) {
    parts.add('Planes are not needed — targets are placed from the station');
  }
  if (state.meshTriangles > 0) {
    parts.add('GLB ${state.meshTriangles} tris in the scene');
  }
  // What the hardware actually delivered: the resolution every saved
  // photograph is written from, and whether the lens may autofocus. Without
  // this line a blurry result has no explanation on screen — "soft lens" and
  // "small stream" look identical and need opposite fixes.
  if (state.cameraQuality != null) {
    parts.add(state.cameraQuality!.describe());
  }
  if (state.lastSavedPath != null) {
    parts.add('Last photo ${_basename(state.lastSavedPath!)}');
  }
  if (!state.assetsMatchSpec && state.meshTriangles > 0) {
    parts.add('GLB assets differ from the recorded dimensions');
  }
  if (state.probes.isNotEmpty) {
    parts.add(
      '${state.probesWithSurface} of ${state.probes.length} directions have a '
      'surface',
    );
  }
  if (state.lastError != null) parts.add(state.lastError!);
  // The physical size of the marker, and how far away it is. **The world scale
  // is fixed** — every target is `targetDiameterM` across, applied once when
  // the scene is pushed and never re-derived from the camera. What changes as
  // the phone moves is the *distance*, and with it the apparent size, which is
  // ordinary perspective. These two numbers are what tell the two apart, so a
  // marker that "got smaller" can be read off the screen instead of guessed.
  parts.add(
    'Plate ${state.geometry.targetDiameterM.toStringAsFixed(2)} m across'
    '${_distanceSuffix(state)}',
  );
  // The advice is the actionable half of the status, so it goes last — but
  // only when there is something to act on.
  if (_trackingAdvice(state) case final advice?) parts.add(advice);
  return parts.join(' · ');
}

/// ` · 1.48 m away`, or nothing while there is no alignment to measure from.
String _distanceSuffix(CaptureState state) {
  final alignment = state.alignment;
  if (alignment == null || !alignment.usable) return '';
  return ' · ${alignment.distanceM.toStringAsFixed(2)} m away';
}

String _basename(String path) {
  final index = path.lastIndexOf('/');
  final backslash = path.lastIndexOf('\\');
  final cut = index > backslash ? index : backslash;
  return cut < 0 ? path : path.substring(cut + 1);
}

/// A tappable grab handle with a chevron.
class _CollapseHandle extends StatelessWidget {
  const _CollapseHandle({
    required this.expanded,
    required this.label,
    required this.onTap,
  });

  final bool expanded;
  final String label;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) => Semantics(
    button: true,
    label: label,
    child: InkWell(
      onTap: onTap,
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 4),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Container(
              width: 34,
              height: 3,
              decoration: BoxDecoration(
                color: CaptureColors.mutedOnDark.withValues(alpha: 0.6),
                borderRadius: BorderRadius.circular(2),
              ),
            ),
            const SizedBox(width: 6),
            Icon(
              expanded ? Icons.keyboard_arrow_down : Icons.keyboard_arrow_up,
              size: 16,
              color: CaptureColors.mutedOnDark,
            ),
          ],
        ),
      ),
    ),
  );
}

/// One target, with an explicit verdict rather than a colour the user must
/// interpret.
///
/// The four verdicts are the four states that actually differ in practice:
/// a saved photograph, a failure with a reason, a target that can be attempted,
/// and a target that cannot be attempted because no surface was found for it.
class _TargetRow extends StatelessWidget {
  const _TargetRow({required this.target, this.probe, this.onRetake});

  final CaptureTarget target;
  final SurfaceProbe? probe;

  /// Present only for a saved target: the explicit reset that lets it be
  /// aligned and captured again.
  final VoidCallback? onRetake;

  @override
  Widget build(BuildContext context) {
    final (:verdict, :detail) = targetVerdict(target, probe: probe);
    final (label, color) = _appearance(verdict);
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 5),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          SizedBox(
            width: 30,
            child: Text(
              '${target.targetId}',
              style: const TextStyle(
                color: CaptureColors.mutedOnDark,
                fontSize: 12,
                fontFeatures: [FontFeature.tabularFigures()],
              ),
            ),
          ),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'yaw ${target.yawDegrees.toStringAsFixed(0)}° · '
                  'pitch ${target.pitchDegrees.toStringAsFixed(0)}°'
                  '${target.isZenith ? ' · zenith' : ''}'
                  '${target.isNadir ? ' · nadir' : ''}',
                  style: const TextStyle(color: Colors.white, fontSize: 12),
                ),
                if (detail != null)
                  Text(
                    detail,
                    maxLines: 2,
                    overflow: TextOverflow.ellipsis,
                    style: const TextStyle(
                      color: CaptureColors.mutedOnDark,
                      fontSize: 10,
                      height: 1.3,
                    ),
                  ),
              ],
            ),
          ),
          const SizedBox(width: 8),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 1),
            decoration: BoxDecoration(
              color: color.withValues(alpha: 0.16),
              borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
              border: Border.all(color: color.withValues(alpha: 0.5)),
            ),
            child: Text(
              label,
              style: TextStyle(
                color: color,
                fontSize: 10,
                fontWeight: FontWeight.w600,
              ),
            ),
          ),
          if (onRetake != null) ...[
            const SizedBox(width: 2),
            TextButton(
              onPressed: onRetake,
              style: TextButton.styleFrom(
                foregroundColor: CaptureColors.accent,
                minimumSize: Size.zero,
                padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                tapTargetSize: MaterialTapTargetSize.shrinkWrap,
                textStyle: const TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                ),
              ),
              child: const Text('Retake'),
            ),
          ],
        ],
      ),
    );
  }

  /// How a verdict is presented. Colour and wording only — the decision itself
  /// lives in [targetVerdict], where it is tested without a widget tree.
  static (String, Color) _appearance(TargetVerdict verdict) =>
      switch (verdict) {
        TargetVerdict.saved => ('Saved', CaptureColors.ok),
        TargetVerdict.failed => ('Failed', CaptureColors.danger),
        TargetVerdict.noSurface => ('No surface', CaptureColors.pendingDot),
        TargetVerdict.unscanned => ('Not scanned', CaptureColors.pendingDot),
        TargetVerdict.ready => ('Ready', CaptureColors.accent),
        TargetVerdict.active => ('Active', CaptureColors.accent),
      };
}

/// The shot phase, as a chip.
///
/// Colour follows the phase, and only [ShotPhase.capture] and
/// [ShotPhase.accept] are green — a phone that is aimed but still moving shows
/// amber "Hold still", never green.
class _PhaseChip extends StatelessWidget {
  const _PhaseChip({
    required this.phase,
    this.stationDriftWarningActive = false,
  });

  final ShotPhase phase;
  final bool stationDriftWarningActive;

  @override
  Widget build(BuildContext context) {
    final color = stationDriftWarningActive
        ? CaptureColors.warn
        : switch (phase) {
            ShotPhase.capture || ShotPhase.accept => CaptureColors.ok,
            ShotPhase.validate => CaptureColors.accent,
            ShotPhase.retry => CaptureColors.danger,
            ShotPhase.settle => CaptureColors.warn,
            ShotPhase.align => CaptureColors.warn,
            ShotPhase.initialising => CaptureColors.pendingDot,
          };
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.16),
        borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
        border: Border.all(color: color.withValues(alpha: 0.55)),
      ),
      child: Text(
        stationDriftWarningActive ? 'Return to station' : phase.label,
        style: TextStyle(
          color: color,
          fontSize: 10,
          fontWeight: FontWeight.w600,
          letterSpacing: 0.2,
        ),
      ),
    );
  }
}

class _PrimaryButton extends StatelessWidget {
  const _PrimaryButton({
    required this.label,
    required this.enabled,
    required this.onPressed,
  });

  final String label;
  final bool enabled;
  final VoidCallback onPressed;

  @override
  Widget build(BuildContext context) => FilledButton(
    onPressed: enabled ? onPressed : null,
    style: FilledButton.styleFrom(
      backgroundColor: CaptureColors.accent,
      disabledBackgroundColor: CaptureColors.trackOnDark,
      minimumSize: const Size.fromHeight(CaptureMetrics.buttonHeight),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
      ),
    ),
    child: Text(label),
  );
}

class _GhostButton extends StatelessWidget {
  const _GhostButton({
    required this.label,
    required this.onPressed,
    this.enabled = true,
  });

  final String label;
  final VoidCallback onPressed;
  final bool enabled;

  @override
  Widget build(BuildContext context) => OutlinedButton(
    onPressed: enabled ? onPressed : null,
    style: OutlinedButton.styleFrom(
      foregroundColor: Colors.white,
      disabledForegroundColor: CaptureColors.mutedOnDark,
      side: BorderSide(
        color: enabled
            ? CaptureColors.ghostButtonBorder
            : CaptureColors.ghostButtonBorderDisabled,
      ),
      minimumSize: const Size.fromHeight(CaptureMetrics.buttonHeight),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(CaptureMetrics.smallRadius),
      ),
    ),
    child: Text(label),
  );
}
