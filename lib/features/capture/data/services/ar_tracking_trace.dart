import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';

/// Append-only record of the ARCore tracking events the app already knows about.
///
/// Every field written here is already computed and already printed with
/// `debugPrint`, which reaches logcat and then vanishes. A session that lost
/// visual-inertial tracking an hour earlier therefore leaves no way to answer
/// the only question that matters: *which* `TrackingFailureReason` fired, at
/// which continuity epoch, and what the drift read at that instant.
///
/// Event-driven, not per-frame: a steady session writes nothing, so the file
/// stays small enough to read on the phone.
///
/// Writes are serialised through one future. Several transitions can land in
/// the same Flutter frame, and interleaved `FileMode.append` calls are not
/// ordered.
class ArTrackingTrace {
  ArTrackingTrace._(this._file);

  final File? _file;

  Future<void> _writes = Future<void>.value();

  /// Set once a write has failed. One honest report is better than a failing
  /// line per transition.
  bool _disabled = false;

  /// A trace that records nowhere, for tests and for non-Android hosts.
  factory ArTrackingTrace.disabled() => ArTrackingTrace._(null);

  /// Opens the trace beside the captured photographs.
  ///
  /// A failure to open is reported and then ignored: the trace is a diagnostic,
  /// and losing it must not stop a capture session.
  static Future<ArTrackingTrace> createDefault() async {
    try {
      final docs = await getApplicationDocumentsDirectory();
      final dir = Directory(p.join(docs.path, 'ar_capture'));
      if (!await dir.exists()) await dir.create(recursive: true);
      return ArTrackingTrace._(
        File(p.join(dir.path, 'ar_tracking_trace.jsonl')),
      );
    } catch (e) {
      debugPrint('ARCORE_TRACE_OFF: could not open the tracking trace: $e');
      return ArTrackingTrace._(null);
    }
  }

  /// For tests: a trace at an explicit path, with no platform dependency.
  @visibleForTesting
  static ArTrackingTrace atFilePath(String path) =>
      ArTrackingTrace._(File(path));

  bool get enabled => _file != null && !_disabled;

  /// The absolute path of the file, or `null` when this trace records nothing.
  String? get path => _file?.path;

  /// Appends one JSON object. Fire-and-forget: a diagnostic must never block
  /// the pose handler that produced it.
  void record(String event, Map<String, Object?> fields) {
    final file = _file;
    if (file == null || _disabled) return;
    final line = jsonEncode({'event': event, ...fields});
    _writes = _writes.then((_) {
      file.writeAsStringSync('$line\n', mode: FileMode.append, flush: true);
    }).catchError((Object e) {
      if (_disabled) return;
      _disabled = true;
      debugPrint(
        'ARCORE_TRACE_OFF: writing the tracking trace failed ($e); '
        'the rest of this session is unrecorded.',
      );
    });
  }

  /// Completes once every queued line is on disk.
  Future<void> flush() => _writes;

  @override
  String toString() => 'ArTrackingTrace(${enabled ? _file!.path : 'off'})';
}
