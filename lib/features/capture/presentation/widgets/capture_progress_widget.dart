import 'package:flutter/material.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';

/// Capture progress, overall and per ring.
///
/// Counts **saved photographs**, not requests: a target only enters
/// [CaptureTargetStatus.captured] after its JPEG is on disk and has been read
/// back, so this can never show 26/26 while a file is missing.
///
/// Takes [specs] rather than rebuilding the sphere itself, so the ring a target
/// belongs to is decided in exactly one place.
class CaptureProgressWidget extends StatelessWidget {
  const CaptureProgressWidget({
    super.key,
    required this.targets,
    required this.specs,
  });

  final List<CaptureTarget> targets;
  final List<CaptureTargetSpec> specs;

  @override
  Widget build(BuildContext context) {
    final total = targets.isEmpty ? kCaptureTargetCount : targets.length;
    final done = targets.where((CaptureTarget t) => t.status.isDone).length;
    final failed = targets
        .where((CaptureTarget t) => t.status == CaptureTargetStatus.failed)
        .length;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Row(
          children: [
            Expanded(
              child: ClipRRect(
                borderRadius: BorderRadius.circular(3),
                child: LinearProgressIndicator(
                  value: total == 0 ? 0 : done / total,
                  minHeight: 6,
                  backgroundColor: CaptureColors.trackOnDark,
                  valueColor: const AlwaysStoppedAnimation<Color>(
                    CaptureColors.ok,
                  ),
                ),
              ),
            ),
            const SizedBox(width: 10),
            Text(
              '$done / $total',
              style: const TextStyle(
                color: Colors.white,
                fontSize: 13,
                fontWeight: FontWeight.w600,
              ),
            ),
          ],
        ),
        if (failed > 0) ...[
          const SizedBox(height: 6),
          Text(
            '$failed need${failed == 1 ? 's' : ''} another attempt',
            style: const TextStyle(color: CaptureColors.warn, fontSize: 12),
          ),
        ],
        const SizedBox(height: 10),
        for (final name in <String>[
          kUpperRingName,
          kMiddleRingName,
          kLowerRingName,
          kZenithRingName,
          kNadirRingName,
        ])
          _RingRow(
            label: name,
            done: _count(name, (CaptureTarget t) => t.status.isDone),
            failed: _count(
              name,
              (CaptureTarget t) => t.status == CaptureTargetStatus.failed,
            ),
            total: _count(name, (CaptureTarget t) => true),
          ),
      ],
    );
  }

  int _count(String ring, bool Function(CaptureTarget) test) {
    var n = 0;
    for (var i = 0; i < targets.length && i < specs.length; i++) {
      if (specs[i].ringName == ring && test(targets[i])) n++;
    }
    return n;
  }
}

class _RingRow extends StatelessWidget {
  const _RingRow({
    required this.label,
    required this.done,
    required this.failed,
    required this.total,
  });

  final String label;
  final int done;
  final int failed;
  final int total;

  @override
  Widget build(BuildContext context) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 2),
    child: Row(
      children: [
        SizedBox(
          width: 74,
          child: Text(
            label,
            style: const TextStyle(
              color: CaptureColors.mutedOnDark,
              fontSize: 12,
            ),
          ),
        ),
        Expanded(
          child: ClipRRect(
            borderRadius: BorderRadius.circular(2),
            child: LinearProgressIndicator(
              value: total == 0 ? 0 : done / total,
              minHeight: 4,
              backgroundColor: CaptureColors.trackOnDark,
              valueColor: const AlwaysStoppedAnimation<Color>(
                CaptureColors.accent,
              ),
            ),
          ),
        ),
        const SizedBox(width: 8),
        Text(
          '$done/$total',
          style: const TextStyle(
            color: CaptureColors.mutedOnDark,
            fontSize: 12,
          ),
        ),
        if (failed > 0) ...[
          const SizedBox(width: 6),
          Text(
            '$failed failed',
            style: const TextStyle(color: CaptureColors.danger, fontSize: 11),
          ),
        ],
      ],
    ),
  );
}
