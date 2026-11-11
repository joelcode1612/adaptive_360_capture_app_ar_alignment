/// The shot's own state machine.
///
/// Deliberately **derived**, not stored: every field is computed from state the
/// controller already holds, so this cannot become a second source of truth that
/// disagrees with `CaptureState.captureBlockReason`. The brief asks for the state
/// machine *and* for one central readiness decision; deriving the label from the
/// decision satisfies both.
///
/// ```
/// INITIALISING → ALIGN → SETTLE → CAPTURE → VALIDATE → ACCEPT
///                    ↑        ↑         │
///                    └────────┴─────────┴── RETRY
/// ```
enum ShotPhase {
  /// No trustworthy measurement yet: tracking is down, or the last reading is
  /// older than the accepted age.
  initialising,

  /// A valid measurement, but the phone is not aimed at the target.
  align,

  /// Aimed, but not yet continuously aligned and steady for the whole window.
  settle,

  /// Aimed and steady: the shutter may fire.
  capture,

  /// A photograph is being taken and written.
  validate,

  /// The photograph was written, verified and recorded; the target is accepted.
  accept,

  /// The last attempt failed. The target is still available.
  retry;

  /// Whether the shutter is open to a manual press in this phase.
  bool get isReady => this == ShotPhase.capture;

  /// Whether the UI should show the green "aligned" treatment.
  ///
  /// Only [capture] and [accept]. **Not** [settle]: a phone that is aimed but
  /// still moving is not aligned, and colouring it green is the exact false
  /// positive the brief forbids.
  bool get showsAligned =>
      this == ShotPhase.capture || this == ShotPhase.accept;

  /// A short label for the guidance panel.
  String get label => switch (this) {
    ShotPhase.initialising => 'Initialising',
    ShotPhase.align => 'Aim at target',
    ShotPhase.settle => 'Hold still',
    ShotPhase.capture => 'Ready',
    ShotPhase.validate => 'Validating image',
    ShotPhase.accept => 'Accepted',
    ShotPhase.retry => 'Retry',
  };
}

/// Derives the phase from the conditions the controller already evaluates.
///
/// The order of the tests *is* the precedence, and three pairs are easy to get
/// wrong:
///
/// * [ShotPhase.validate] outranks everything except [ShotPhase.accept] — while a
///   JPEG is being written the UI must not fall back to "Aim at target" just
///   because the phone drifted.
/// * [ShotPhase.retry] outranks [ShotPhase.align] — a failed attempt is more
///   useful information than "aim at it again".
ShotPhase deriveShotPhase({
  required bool trackingValid,
  required bool measurementFresh,
  required bool targetSelected,
  required bool targetAnchored,
  required bool withinTolerance,
  bool alignmentHoldComplete = true,
  required bool stable,
  required bool saving,
  required bool lastAttemptFailed,
  required bool targetAccepted,
}) {
  if (targetAccepted) return ShotPhase.accept;
  if (saving) return ShotPhase.validate;
  if (lastAttemptFailed) return ShotPhase.retry;
  if (!trackingValid || !measurementFresh) return ShotPhase.initialising;
  if (!targetSelected || !targetAnchored) return ShotPhase.align;
  if (!withinTolerance) return ShotPhase.align;
  if (!alignmentHoldComplete || !stable) return ShotPhase.settle;
  return ShotPhase.capture;
}
