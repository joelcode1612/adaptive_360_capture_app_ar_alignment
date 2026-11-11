import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:adaptive_360_capture_app/features/capture/presentation/screens/ar_capture_screen.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_flow_screen.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/screens/virtual_tour_camera_screen.dart';

/// 360° Virtual Tour — the capture app.
///
/// Portrait only. The capture plans are defined by yaw and pitch around a
/// standing person. Both grids — the guided flow's 26 points and the capture
/// module's 26 nodes — assume the phone's long side is vertical, and the
/// prototype is a phone-shaped column, so rotating the activity would change
/// what the guidance means, not merely how it looks. The lock is applied here
/// rather than in the manifest so the same Dart entry behaves identically under
/// `flutter test`.
///
/// **Portrait-up, not both portraits.** The capture plan assumes a vertical
/// phone. The AR screen still forwards metric changes to Android, where the
/// host refreshes ARCore's display geometry, but portrait-up keeps the capture
/// directions and the user-facing guidance semantically stable.
void main() {
  runApp(const VirtualTourApp());
}

class VirtualTourApp extends StatelessWidget {
  const VirtualTourApp({super.key});

  @override
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations(const [DeviceOrientation.portraitUp]);
    return MaterialApp(
      // The brief's §2 display name. `android:label` in the manifest is what the
      // launcher shows; this is what the OS task switcher and the window title
      // use, and the two agreeing is the only reason either is trustworthy.
      title: '360° Virtual Tour',
      debugShowCheckedModeBanner: false,
      // One theme for the whole app. The prototype keeps every screen —
      // settings and review included — inside `.phone{background:#0f1418}`, so
      // there is no light variant to follow the system into and no `darkTheme`
      // to switch on.
      theme: captureAppTheme(),
      home: const CaptureLauncher(),
    );
  }
}

/// Entry point: choose the single production path or a diagnostic tool.
///
/// **AR capture is the authoritative production route.** The other two
/// entries remain available for investigation and compatibility work, but they
/// do not share its anchor, local-manifest, or 26-frame upload contract.
/// Their diagnostic status is deliberately visible in the launcher so an
/// operator cannot mistake a successful legacy run for a production station.
///
/// The three paths differ as follows:
///
/// * **AR capture** is Section 1 — a real ARCore session, surface anchors, GLB
///   marker, live pose and real photographs. Android only, because the ARCore
///   host is a Kotlin host in this app and there is no ARKit equivalent yet; on
///   any other platform the screen says so instead of degrading silently.
/// * **Virtual tour** is a diagnostic 30-node station: a 1.0 m world-locked
///   sphere, the GLB target at every node, an automatic shutter, then upload
///   and stitching. Also Android/ARCore only, for the same reason.
/// * **Guided flow** is the legacy sensor-guided prototype port, which runs
///   anywhere and needs no AR.
class CaptureLauncher extends StatelessWidget {
  const CaptureLauncher({super.key});

  @override
  Widget build(BuildContext context) => Scaffold(
    backgroundColor: CaptureColors.cameraChrome,
    body: SafeArea(
      child: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const SizedBox(height: 12),
            const Text(
              '360° Virtual Tour',
              style: TextStyle(
                color: Colors.white,
                fontSize: 24,
                fontWeight: FontWeight.w700,
              ),
            ),
            const SizedBox(height: 6),
            const Text(
              'Capture a commercial space one station at a time.',
              style: TextStyle(color: CaptureColors.mutedOnDark),
            ),
            const SizedBox(height: 28),
            _LaunchCard(
              title: 'AR capture · Production · 26 frames',
              subtitle:
                  'The authoritative ARCore station flow: one shared anchor, '
                  'verified local capture manifest, and 26 photographs. '
                  'Android with ARCore only.',
              icon: Icons.view_in_ar,
              onTap: () => Navigator.of(context).push(
                MaterialPageRoute<void>(
                  builder: (_) => const CaptureModuleScreen(),
                ),
              ),
            ),
            const SizedBox(height: 12),
            _LaunchCard(
              title: 'Virtual tour · Diagnostic · 30 nodes',
              subtitle:
                  'Legacy diagnostic path with a separate 30-node contract. '
                  'Do not use for production AR Capture stations.',
              icon: Icons.threesixty,
              onTap: () => Navigator.of(context).push(
                MaterialPageRoute<void>(
                  builder: (_) => const VirtualTourCameraScreen(),
                ),
              ),
            ),
            const SizedBox(height: 12),
            _LaunchCard(
              title: 'Guided flow · Legacy diagnostic',
              subtitle:
                  'Sensor-guided prototype for diagnostics only; it does not '
                  'provide the production AR anchor contract.',
              icon: Icons.photo_camera_outlined,
              onTap: () => Navigator.of(context).push(
                MaterialPageRoute<void>(
                  builder: (_) => const CaptureFlowScreen(),
                ),
              ),
            ),
          ],
        ),
      ),
    ),
  );
}

class _LaunchCard extends StatelessWidget {
  const _LaunchCard({
    required this.title,
    required this.subtitle,
    required this.icon,
    required this.onTap,
  });

  final String title;
  final String subtitle;
  final IconData icon;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) => Card(
    color: CaptureColors.sheet,
    shape: RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(CaptureMetrics.cornerRadius),
    ),
    child: InkWell(
      borderRadius: BorderRadius.circular(CaptureMetrics.cornerRadius),
      onTap: onTap,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: [
            Icon(icon, color: CaptureColors.accent, size: 30),
            const SizedBox(width: 14),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    title,
                    style: const TextStyle(
                      color: Colors.white,
                      fontSize: 16,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                  const SizedBox(height: 4),
                  Text(
                    subtitle,
                    style: const TextStyle(
                      color: CaptureColors.mutedOnDark,
                      fontSize: 12,
                      height: 1.4,
                    ),
                  ),
                ],
              ),
            ),
            const Icon(Icons.chevron_right, color: CaptureColors.mutedOnDark),
          ],
        ),
      ),
    ),
  );
}
