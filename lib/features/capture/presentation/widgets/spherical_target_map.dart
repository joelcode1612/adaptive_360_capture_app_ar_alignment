import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/services/target_sphere.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/math/vectors.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/presentation/capture_theme.dart';

/// The 26 capture directions, drawn as a sphere.
///
/// Three rings (upper +45°, middle 0°, lower −45°) plus the zenith and the
/// nadir, laid out by the *same* direction vectors the guidance uses — not by a
/// second, decorative mapping. Each dot is the orthographic projection of its
/// station-local unit vector, so a dot that the guidance calls "up and to the
/// right" is up and to the right here.
///
/// The view is tilted 18° about the horizontal axis, which is what makes the
/// three rings read as ellipses of different sizes rather than as three parallel
/// lines; the back half of the sphere is dimmed so the front is unambiguous.
class SphericalTargetMap extends StatelessWidget {
  const SphericalTargetMap({
    super.key,
    required this.specs,
    required this.targets,
    required this.activeTargetId,
    required this.onSelect,
    this.size = 220,
    this.showLabels = false,
  });

  final List<CaptureTargetSpec> specs;

  /// Live status per target; may be shorter than [specs] before boot.
  final List<CaptureTarget> targets;

  final int activeTargetId;
  final ValueChanged<int> onSelect;

  /// Width and height, in logical pixels.
  final double size;
  final bool showLabels;

  /// Tilt of the sphere's axis, in degrees. Chosen once: any value between
  /// 10° and 25° reads as "a sphere seen slightly from above".
  static const double tiltDeg = 18;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: size,
      height: size,
      child: GestureDetector(
        onTapDown: (TapDownDetails d) {
          final id = _hitTest(d.localPosition, size);
          if (id != null) onSelect(id);
        },
        child: CustomPaint(
          painter: _SpherePainter(
            specs: specs,
            statuses: _statuses(),
            activeTargetId: activeTargetId,
            tiltDeg: tiltDeg,
            showLabels: showLabels,
          ),
        ),
      ),
    );
  }

  List<CaptureTargetStatus> _statuses() => <CaptureTargetStatus>[
    for (final spec in specs)
      targets.length >= spec.targetId
          ? targets[spec.targetId - 1].status
          : CaptureTargetStatus.pending,
  ];

  int? _hitTest(Offset local, double size) {
    final radius = size / 2 - 12;
    final centre = Offset(size / 2, size / 2);
    final tilt = tiltDeg * math.pi / 180;
    final cosT = math.cos(tilt);
    final sinT = math.sin(tilt);

    var bestId = 0;
    var bestDistance = 26.0;
    for (final spec in specs) {
      final d = spec.direction;
      final y = d.y * cosT - d.z * sinT;
      final x = d.x;
      final p = centre + Offset(x * radius, -y * radius);
      final distance = (p - local).distance;
      if (distance < bestDistance) {
        bestDistance = distance;
        bestId = spec.targetId;
      }
    }
    return bestId == 0 ? null : bestId;
  }
}

class _SpherePainter extends CustomPainter {
  _SpherePainter({
    required this.specs,
    required this.statuses,
    required this.activeTargetId,
    required this.tiltDeg,
    required this.showLabels,
  });

  final List<CaptureTargetSpec> specs;
  final List<CaptureTargetStatus> statuses;
  final int activeTargetId;
  final double tiltDeg;
  final bool showLabels;

  @override
  void paint(Canvas canvas, Size size) {
    final radius = size.width / 2 - 12;
    final centre = Offset(size.width / 2, size.height / 2);
    final tilt = tiltDeg * math.pi / 180;
    final cosT = math.cos(tilt);
    final sinT = math.sin(tilt);

    Offset project(Vec3 d) {
      final y = d.y * cosT - d.z * sinT;
      return centre + Offset(d.x * radius, -y * radius);
    }

    double depth(Vec3 d) => d.y * sinT + d.z * cosT;

    // The sphere itself.
    canvas.drawCircle(
      centre,
      radius,
      Paint()
        ..color = CaptureColors.sphereFill
        ..style = PaintingStyle.fill,
    );
    canvas.drawCircle(
      centre,
      radius,
      Paint()
        ..color = CaptureColors.sphereOutline
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );

    // Three latitude rings, sampled rather than drawn as ellipses: sampling is
    // the same projection the dots use, so a ring and its dots cannot disagree.
    for (final pitchDeg in kCaptureRingPitchesDeg) {
      final pitch = pitchDeg * math.pi / 180;
      final path = Path();
      var started = false;
      for (var step = 0; step <= 72; step++) {
        final yaw = step * 360 / 72 * math.pi / 180;
        final d = Vec3(
          math.cos(pitch) * math.sin(yaw),
          math.sin(pitch),
          math.cos(pitch) * math.cos(yaw),
        );
        final p = project(d);
        if (!started) {
          path.moveTo(p.dx, p.dy);
          started = true;
        } else {
          path.lineTo(p.dx, p.dy);
        }
      }
      path.close();
      canvas.drawPath(
        path,
        Paint()
          ..color = CaptureColors.sphereOutline.withValues(alpha: 0.35)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1,
      );
    }

    // A vertical meridian, so "front" and "back" are still readable when a
    // ring's dots all sit on one line.
    final meridian = Path();
    for (var step = 0; step <= 72; step++) {
      final a = -math.pi / 2 + step * math.pi / 72;
      final p = project(Vec3(0, math.sin(a), math.cos(a)));
      if (step == 0) {
        meridian.moveTo(p.dx, p.dy);
      } else {
        meridian.lineTo(p.dx, p.dy);
      }
    }
    canvas.drawPath(
      meridian,
      Paint()
        ..color = CaptureColors.sphereOutline.withValues(alpha: 0.25)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );

    // Dots, back half first so the front ones overlap them.
    final ordered = List<int>.generate(specs.length, (int i) => i)
      ..sort(
        (int a, int b) =>
            depth(specs[a].direction).compareTo(depth(specs[b].direction)),
      );

    for (final index in ordered) {
      final spec = specs[index];
      final status = index < statuses.length
          ? statuses[index]
          : CaptureTargetStatus.pending;
      final p = project(spec.direction);
      final front = depth(spec.direction) >= 0;
      // A completed target keeps its saved style; the amber "you are on this
      // one" halo never outlives its capture, even if a stale id still names
      // it for a frame.
      final isActive = spec.targetId == activeTargetId && !status.isDone;

      final color = switch (status) {
        CaptureTargetStatus.captured => CaptureColors.ok,
        CaptureTargetStatus.failed => CaptureColors.danger,
        CaptureTargetStatus.aligned => CaptureColors.accent,
        CaptureTargetStatus.active => CaptureColors.warn,
        // A shutter request is in flight: the same amber as "you are on this
        // one", because it is still the active target and the photograph has
        // not been recorded yet.
        CaptureTargetStatus.capturing => CaptureColors.warn,
        CaptureTargetStatus.pending => CaptureColors.pendingDot,
      };

      final alpha = front ? 1.0 : 0.45;
      if (isActive) {
        canvas.drawCircle(
          p,
          11,
          Paint()
            ..color = CaptureColors.warn.withValues(alpha: 0.28)
            ..style = PaintingStyle.fill,
        );
        canvas.drawCircle(
          p,
          11,
          Paint()
            ..color = CaptureColors.warn
            ..style = PaintingStyle.stroke
            ..strokeWidth = 1.5,
        );
      }

      canvas.drawCircle(
        p,
        isActive ? 6 : 5,
        Paint()
          ..color = color.withValues(alpha: alpha)
          ..style = PaintingStyle.fill,
      );
      canvas.drawCircle(
        p,
        isActive ? 6 : 5,
        Paint()
          ..color = Colors.black.withValues(alpha: front ? 0.35 : 0.2)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1,
      );

      // A captured target gets a tick rather than a second colour, so the map
      // stays readable for a red/green colour-blind user.
      if (status == CaptureTargetStatus.captured) {
        canvas.drawLine(
          p + const Offset(-2.5, 0),
          p + const Offset(-0.5, 2.5),
          Paint()
            ..color = Colors.white.withValues(alpha: alpha)
            ..strokeWidth = 1.6
            ..strokeCap = StrokeCap.round,
        );
        canvas.drawLine(
          p + const Offset(-0.5, 2.5),
          p + const Offset(3, -3),
          Paint()
            ..color = Colors.white.withValues(alpha: alpha)
            ..strokeWidth = 1.6
            ..strokeCap = StrokeCap.round,
        );
      }

      if (showLabels) {
        final painter = TextPainter(
          text: TextSpan(
            text: '${spec.targetId}',
            style: TextStyle(
              color: Colors.white.withValues(alpha: front ? 0.85 : 0.4),
              fontSize: 9,
              fontWeight: FontWeight.w600,
            ),
          ),
          textDirection: TextDirection.ltr,
        )..layout();
        painter.paint(canvas, p + const Offset(7, -4));
      }
    }
  }

  @override
  bool shouldRepaint(_SpherePainter old) =>
      old.statuses != statuses ||
      old.activeTargetId != activeTargetId ||
      old.specs.length != specs.length;
}
