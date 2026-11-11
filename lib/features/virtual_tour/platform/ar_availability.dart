import 'package:adaptive_360_capture_app/features/virtual_tour/platform/ar_types.dart';

/// The fallback ladder.
///
/// The brief is specific about this: *"If the device does not support ARCore,
/// disable the anchor check and say so."* Three tiers, and the decision is a
/// pure function of what was measured, so it can be tested for every
/// combination without a device.
enum ArTier {
  /// ARCore, planes and anchors all work. Full guidance and an anchor-backed
  /// station reference are available.
  full,

  /// A pose is available but there is no usable anchor. Guidance still runs,
  /// but there is no anchor-backed station reference.
  poseOnly,

  /// No AR and no usable pose. The guided flow cannot run; the user is offered
  /// the manual path instead of a screen that silently does nothing.
  unavailable;

  /// Whether the guided capture flow may start at all.
  bool get canCapture => this != ArTier.unavailable;

}

/// The outcome of the ladder, with the wording the UI shows.
class ArTierDecision {
  final ArTier tier;

  /// One line, for the top of the calibration screen.
  final String headline;

  /// Two or three lines explaining what is and is not available.
  final String detail;

  /// What is missing, or `null` when nothing is. Never empty when [tier] is not
  /// [ArTier.full] — "something is degraded" without saying what is exactly the
  /// behaviour the brief forbids.
  final String? limitation;

  /// Whether the flow may show turn-by-turn guidance.
  final bool guidanceEnabled;

  /// Whether a second, cheaper pose source is available to fall back to.
  final bool sensorFallback;

  const ArTierDecision({
    required this.tier,
    required this.headline,
    required this.detail,
    required this.guidanceEnabled,
    required this.sensorFallback,
    this.limitation,
  });

  @override
  String toString() =>
      'ArTierDecision(${tier.name}, '
      'guidance: $guidanceEnabled)';
}

/// Chooses the tier from what was measured.
///
/// [poseSensorsAvailable] means a non-AR pose source exists — the platform
/// rotation vector or a fused gyro. It is a separate input because it is a
/// separate capability: a phone with no ARCore still has a perfectly good
/// orientation sensor, and using it (with an honest caveat) is better than
/// refusing to run.
ArTierDecision decideArTier({
  required ArCapabilities ar,
  required bool poseSensorsAvailable,
}) {
  if (ar.canAnchor) {
    return ArTierDecision(
      tier: ArTier.full,
      headline: 'AR ready',
      detail:
          'Plane detection and anchors are available. The app will place a '
          'station anchor to keep the AR reference stable.',
      guidanceEnabled: true,
      sensorFallback: poseSensorsAvailable,
    );
  }

  if (ar.canPose) {
    return ArTierDecision(
      tier: ArTier.poseOnly,
      headline: 'AR pose only',
      detail:
          'The camera pose is available, so guidance works. The station '
          'anchor is not, so its AR reference cannot be verified.',
      guidanceEnabled: true,
      sensorFallback: poseSensorsAvailable,
      limitation: ar.planes
          ? 'Anchors could not be created on this device.'
          : 'Plane detection is unavailable, so no station anchor can be '
                'placed.',
    );
  }

  if (poseSensorsAvailable) {
    return ArTierDecision(
      tier: ArTier.poseOnly,
      headline: 'No AR — using the motion sensors',
      detail:
          'AR is not available on this device, so the motion sensors provide '
          'the direction. There is no anchor and no '
          'plane detection. Near the ceiling and the floor the direction is '
          'less reliable, so those two frames may need the manual button.',
      guidanceEnabled: true,
      sensorFallback: true,
      limitation:
          'ARCore is not available${ar.reason == null ? '' : ': ${ar.reason}'}. '
          'The station anchor check is disabled.',
    );
  }

  return ArTierDecision(
    tier: ArTier.unavailable,
    headline: 'Guided capture unavailable',
    detail:
        'This device reported neither AR support nor a usable motion sensor, '
        'so the app cannot tell which way the phone is pointing. Use the '
        'manual capture path instead.',
    guidanceEnabled: false,
    sensorFallback: false,
    limitation:
        'No AR and no pose sensor. Guided capture is disabled on this device.',
  );
}
