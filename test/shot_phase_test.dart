import 'package:flutter_test/flutter_test.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/motion/shot_phase.dart';

/// The shot state machine.
///
/// The tests are written as "given these conditions, which phase?" rather than
/// "does capture succeed?", because the failure modes that matter are the
/// *ordering* ones — a failed attempt that reads as "aim again", or a save that
/// reads as "move to target" while the JPEG is being written.
void main() {
  ShotPhase phase({
    bool trackingValid = true,
    bool measurementFresh = true,
    bool targetSelected = true,
    bool targetAnchored = true,
    bool withinTolerance = true,
    bool stable = true,
    bool saving = false,
    bool lastAttemptFailed = false,
    bool targetAccepted = false,
  }) => deriveShotPhase(
    trackingValid: trackingValid,
    measurementFresh: measurementFresh,
    targetSelected: targetSelected,
    targetAnchored: targetAnchored,
    withinTolerance: withinTolerance,
    stable: stable,
    saving: saving,
    lastAttemptFailed: lastAttemptFailed,
    targetAccepted: targetAccepted,
  );

  group('the happy path', () {
    test('no tracking → initialising', () {
      expect(phase(trackingValid: false), ShotPhase.initialising);
    });

    test('a stale reading → initialising', () {
      expect(phase(measurementFresh: false), ShotPhase.initialising);
    });

    test('not aimed → align', () {
      expect(phase(withinTolerance: false), ShotPhase.align);
    });

    test('aimed but moving → settle, never capture', () {
      expect(phase(stable: false), ShotPhase.settle);
    });

    test('aimed and steady → capture', () {
      expect(phase(), ShotPhase.capture);
      expect(phase().isReady, isTrue);
    });

    test('a missing target or anchor → align', () {
      expect(phase(targetSelected: false), ShotPhase.align);
      expect(phase(targetAnchored: false), ShotPhase.align);
    });
  });

  group('precedence', () {
    test('saving outranks a drifted pose', () {
      // The phone drifting mid-write must not flip the panel back to "aim".
      expect(
        phase(saving: true, withinTolerance: false, stable: false),
        ShotPhase.validate,
      );
    });

    test('a failed attempt outranks "aim again"', () {
      expect(
        phase(lastAttemptFailed: true, withinTolerance: false),
        ShotPhase.retry,
      );
    });

    test('saving outranks a failure', () {
      expect(phase(saving: true, lastAttemptFailed: true), ShotPhase.validate);
    });

    test('accept outranks everything', () {
      expect(
        phase(
          targetAccepted: true,
          saving: true,
          lastAttemptFailed: true,
          trackingValid: false,
        ),
        ShotPhase.accept,
      );
    });
  });

  group('aligned colour', () {
    test('only capture and accept show the aligned treatment', () {
      expect(ShotPhase.capture.showsAligned, isTrue);
      expect(ShotPhase.accept.showsAligned, isTrue);
      // The one that matters: aimed but still moving must not look aligned.
      expect(ShotPhase.settle.showsAligned, isFalse);
      expect(ShotPhase.align.showsAligned, isFalse);
      expect(ShotPhase.retry.showsAligned, isFalse);
      expect(ShotPhase.validate.showsAligned, isFalse);
      expect(ShotPhase.initialising.showsAligned, isFalse);
    });

    test('every phase has a distinct label', () {
      final labels = ShotPhase.values.map((p) => p.label).toSet();
      expect(labels.length, ShotPhase.values.length);
    });
  });

  group('failure and retry', () {
    test('a failed save leaves the target retryable', () {
      final p = phase(lastAttemptFailed: true);
      expect(p, ShotPhase.retry);
      expect(p.isReady, isFalse);
    });

    test('a retry that succeeds reaches capture again', () {
      expect(phase(lastAttemptFailed: false), ShotPhase.capture);
    });
  });
}
