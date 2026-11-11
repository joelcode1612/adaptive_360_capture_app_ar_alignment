import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:crypto/crypto.dart';
import 'package:path/path.dart' as p;
import 'package:path_provider/path_provider.dart';
import 'package:adaptive_360_capture_app/core/error/failures.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_record.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_station.dart';
import 'package:adaptive_360_capture_app/features/capture/domain/models/capture_target.dart';

/// Local persistence for Section 1 capture data.
///
/// Filesystem layout, under the app's private documents directory (the same
/// choice the existing session store makes: these are the user's photographs of
/// their own premises and they must not land in shared storage):
///
/// ```
/// <appDocs>/ar_capture/
///   <stationId>/
///     manifests/        immutable, complete capture-state snapshots
///       <generation>.json
///     frames/
///       t01/<generation>.jpg
/// ```
///
/// No backend is involved in capturing a photograph. Everything a review screen
/// needs is on disk before the target is marked done.
abstract class CaptureRepository {
  /// Reads one internally coherent capture snapshot. Resume flows must use
  /// this instead of independently reading station, target and record files:
  /// a commit between those independent reads would otherwise mix generations.
  Future<CapturePersistedState?> loadCaptureState(String stationId);

  /// Atomically publishes the complete durable state for a station. New code
  /// uses this rather than independently replacing station, target and record
  /// files; the three rows then either all refer to one snapshot or none do.
  Future<void> saveCaptureState({
    required CaptureStation station,
    required List<CaptureTarget> targets,
    required List<CaptureRecord> records,
  });

  /// Writes an immutable JPEG version and atomically publishes the snapshot
  /// that accepts it. If the snapshot cannot be published, the JPEG is an
  /// ignored orphan; it never replaces an already accepted record.
  Future<CaptureCommitResult> commitCapture({
    required CaptureStation station,
    required CaptureRecord record,
    required Uint8List jpegBytes,
  });

  Future<void> saveStation(CaptureStation station);

  Future<void> saveTargets(List<CaptureTarget> targets);

  Future<void> saveRecord(CaptureRecord record);

  Future<CaptureStation?> loadStation(String stationId);

  Future<List<CaptureTarget>> loadTargets(String stationId);

  Future<List<CaptureRecord>> loadRecords(String stationId);

  /// Allocates an absolute, immutable JPEG path for [stationId]/[targetId].
  /// A later call for the same target deliberately returns a different path so
  /// a retake cannot overwrite the only accepted image.
  Future<String> framePath(String stationId, int targetId);

  /// Every station that has been saved, newest first.
  ///
  /// The resume entry point needs this to offer "continue where you left off"
  /// without the caller knowing anything about the store's layout.
  Future<List<String>> stationIds();
}

/// The single result of a durable capture commit. The controller may update its
/// live AR state only after receiving this value.
class CaptureCommitResult {
  const CaptureCommitResult({
    required this.record,
    required this.station,
    required this.targets,
  });

  final CaptureRecord record;
  final CaptureStation station;
  final List<CaptureTarget> targets;
}

/// One verified, self-contained persisted capture state.
///
/// This deliberately exposes no manifest implementation details. The
/// repository may use immutable manifest generations, while callers get the
/// one station/target/record set that was accepted together.
class CapturePersistedState {
  CapturePersistedState({
    required this.station,
    required List<CaptureTarget> targets,
    required List<CaptureRecord> records,
  }) : targets = List<CaptureTarget>.unmodifiable(targets),
       records = List<CaptureRecord>.unmodifiable(records);

  final CaptureStation station;
  final List<CaptureTarget> targets;
  final List<CaptureRecord> records;
}

class FileCaptureRepository implements CaptureRepository {
  FileCaptureRepository(this._root);

  static const int _manifestSchemaVersion = 1;

  final String _root;
  final Map<String, Future<void>> _stationWriteTails = <String, Future<void>>{};
  final math.Random _random = math.Random.secure();
  int _imageSequence = 0;

  /// Builds one against the platform's documents directory.
  static Future<FileCaptureRepository> create() async {
    final docs = await getApplicationDocumentsDirectory();
    return FileCaptureRepository(p.join(docs.path, 'ar_capture'));
  }

  String _stationDir(String stationId) => p.join(_root, stationId);

  String _manifestDir(String stationId) =>
      p.join(_stationDir(stationId), 'manifests');

  String _legacyStationPath(String stationId) =>
      p.join(_stationDir(stationId), 'station.json');

  String _legacyTargetsPath(String stationId) =>
      p.join(_stationDir(stationId), 'targets.json');

  String _legacyRecordsPath(String stationId) =>
      p.join(_stationDir(stationId), 'records.json');

  String _targetFramesDir(String stationId, int targetId) => p.join(
    _stationDir(stationId),
    'frames',
    't${targetId.toString().padLeft(2, '0')}',
  );

  String _nextImageVersion() {
    final micros = DateTime.now()
        .toUtc()
        .microsecondsSinceEpoch
        .toString()
        .padLeft(20, '0');
    final sequence = (++_imageSequence).toString().padLeft(6, '0');
    final entropy = _random
        .nextInt(0x100000000)
        .toRadixString(16)
        .padLeft(8, '0');
    return '$micros-$sequence-$entropy';
  }

  @override
  Future<String> framePath(String stationId, int targetId) async {
    _validateTargetId(targetId);
    final dir = Directory(_targetFramesDir(stationId, targetId));
    await dir.create(recursive: true);
    return p.join(dir.path, '${_nextImageVersion()}.jpg');
  }

  /// Serialises all read-modify-write state transitions for one station.
  ///
  /// A JPEG can be staged concurrently with unrelated station work, but two
  /// snapshots for the same station must never both read one old base and each
  /// publish only its own change. The tail survives an earlier failure, so one
  /// failed save cannot deadlock the following retry.
  Future<T> _withStationWriteLock<T>(
    String stationId,
    Future<T> Function() operation,
  ) async {
    final previous = _stationWriteTails[stationId] ?? Future<void>.value();
    final settledPrevious = previous.then<void>(
      (_) {},
      onError: (Object error, StackTrace stackTrace) {},
    );
    final release = Completer<void>();
    final tail = settledPrevious.then<void>((_) => release.future);
    _stationWriteTails[stationId] = tail;
    try {
      await settledPrevious;
      return await operation();
    } finally {
      release.complete();
      if (identical(_stationWriteTails[stationId], tail)) {
        _stationWriteTails.remove(stationId);
      }
    }
  }

  @override
  Future<void> saveCaptureState({
    required CaptureStation station,
    required List<CaptureTarget> targets,
    required List<CaptureRecord> records,
  }) => _withStationWriteLock(station.stationId, () async {
    final previous = await _loadCommittedSnapshot(station.stationId);
    final verifiedRecords = await _normaliseRecords(station.stationId, records);
    _validateState(
      station: station,
      targets: targets,
      records: verifiedRecords,
    );
    await _publishSnapshot(
      station: station,
      targets: targets,
      records: verifiedRecords,
      previousGeneration: previous?.generation,
    );
  });

  @override
  Future<CaptureCommitResult> commitCapture({
    required CaptureStation station,
    required CaptureRecord record,
    required Uint8List jpegBytes,
  }) async {
    if (record.stationId != station.stationId) {
      throw const StorageFailure(
        'The photograph does not belong to the station being saved.',
      );
    }
    _validateTargetId(record.targetId);

    return _withStationWriteLock(station.stationId, () async {
      try {
        final previous = await _loadCommittedSnapshot(station.stationId);
        final hasFinalManifest =
            previous != null || await _hasFinalManifest(station.stationId);
        final base =
            previous ??
            (hasFinalManifest
                ? null
                : await _loadLegacySnapshot(station.stationId));
        if (base == null) {
          throw const StorageFailure(
            'The station was not durably initialised before the photograph was saved.',
          );
        }
        final targetIndex = base.targets.indexWhere(
          (CaptureTarget target) => target.targetId == record.targetId,
        );
        if (targetIndex < 0) {
          throw StorageFailure(
            'Target ${record.targetId} is not part of the saved station.',
          );
        }
        final previousRecords = await _normaliseRecords(
          station.stationId,
          base.records,
        );

        final image = await _writeImmutableJpeg(
          stationId: station.stationId,
          targetId: record.targetId,
          bytes: jpegBytes,
        );
        if ((record.imageWidth > 0 && record.imageWidth != image.width) ||
            (record.imageHeight > 0 && record.imageHeight != image.height)) {
          throw const StorageFailure(
            'The encoded JPEG dimensions do not match the camera metadata.',
          );
        }
        final acceptedRecord = record.copyWith(
          imagePath: image.path,
          imageByteLength: image.byteLength,
          imageSha256: image.sha256,
        );
        final records = List<CaptureRecord>.of(previousRecords)
          ..removeWhere(
            (CaptureRecord existing) => existing.targetId == record.targetId,
          )
          ..add(acceptedRecord)
          ..sort(
            (CaptureRecord a, CaptureRecord b) =>
                a.targetId.compareTo(b.targetId),
          );
        final targets = List<CaptureTarget>.of(base.targets);
        targets[targetIndex] = targets[targetIndex].copyWith(
          status: CaptureTargetStatus.captured,
          imagePath: acceptedRecord.imagePath,
          clearMessage: true,
        );
        final updatedStation = station.copyWith(
          capturedCount: targets
              .where((CaptureTarget target) => target.status.isDone)
              .length,
        );

        await _publishSnapshot(
          station: updatedStation,
          targets: targets,
          records: records,
          previousGeneration: previous?.generation,
        );
        return CaptureCommitResult(
          record: acceptedRecord,
          station: updatedStation,
          targets: List<CaptureTarget>.unmodifiable(targets),
        );
      } on StorageFailure {
        rethrow;
      } on Object catch (error) {
        throw StorageFailure('Could not durably save the photograph: $error');
      }
    });
  }

  /// Compatibility adapters for records written before immutable manifests.
  /// Production capture paths enter through [saveCaptureState] and
  /// [commitCapture]; these adapters never delete an existing file to make a
  /// replacement fit.
  @override
  Future<void> saveStation(CaptureStation station) =>
      _withStationWriteLock(station.stationId, () async {
        final committed = await _loadCommittedSnapshot(station.stationId);
        if (committed != null) {
          await _publishSnapshot(
            station: station,
            targets: committed.targets,
            records: committed.records,
            previousGeneration: committed.generation,
          );
          return;
        }
        await _writeJsonAtomically(
          File(_legacyStationPath(station.stationId)),
          station.toJson(),
        );
      });

  @override
  Future<void> saveTargets(List<CaptureTarget> targets) async {
    if (targets.isEmpty) return;
    final stationId = targets.first.stationId;
    if (targets.any((CaptureTarget target) => target.stationId != stationId)) {
      throw const StorageFailure('All target rows must belong to one station.');
    }
    await _withStationWriteLock(stationId, () async {
      final committed = await _loadCommittedSnapshot(stationId);
      if (committed != null) {
        await _publishSnapshot(
          station: committed.station,
          targets: targets,
          records: committed.records,
          previousGeneration: committed.generation,
        );
        return;
      }
      await _writeJsonAtomically(
        File(_legacyTargetsPath(stationId)),
        targets
            .map((CaptureTarget target) => target.toJson())
            .toList(growable: false),
      );
    });
  }

  @override
  Future<void> saveRecord(CaptureRecord record) =>
      _withStationWriteLock(record.stationId, () async {
        final committed = await _loadCommittedSnapshot(record.stationId);
        if (committed != null) {
          final records = List<CaptureRecord>.of(committed.records)
            ..removeWhere(
              (CaptureRecord existing) => existing.targetId == record.targetId,
            )
            ..add(record)
            ..sort(
              (CaptureRecord a, CaptureRecord b) =>
                  a.targetId.compareTo(b.targetId),
            );
          await _publishSnapshot(
            station: committed.station,
            targets: committed.targets,
            records: await _normaliseRecords(record.stationId, records),
            previousGeneration: committed.generation,
          );
          return;
        }
        final records =
            List<CaptureRecord>.of(await _readLegacyRecords(record.stationId))
              ..removeWhere(
                (CaptureRecord existing) =>
                    existing.targetId == record.targetId,
              )
              ..add(record)
              ..sort(
                (CaptureRecord a, CaptureRecord b) =>
                    a.targetId.compareTo(b.targetId),
              );
        await _writeJsonAtomically(
          File(_legacyRecordsPath(record.stationId)),
          records
              .map((CaptureRecord item) => item.toJson())
              .toList(growable: false),
        );
      });

  @override
  Future<CapturePersistedState?> loadCaptureState(String stationId) async {
    final committed = await _loadCommittedSnapshot(stationId);
    if (committed != null) return committed.toPersistedState();
    // Once a final manifest exists, even an invalid one supersedes the split
    // legacy files. Falling back would resurrect older, unverified evidence.
    if (await _hasFinalManifest(stationId)) return null;
    return (await _loadLegacySnapshot(stationId))?.toPersistedState();
  }

  @override
  Future<CaptureStation?> loadStation(String stationId) async =>
      (await loadCaptureState(stationId))?.station;

  @override
  Future<List<CaptureTarget>> loadTargets(String stationId) async =>
      (await loadCaptureState(stationId))?.targets ?? const <CaptureTarget>[];

  @override
  Future<List<CaptureRecord>> loadRecords(String stationId) async =>
      (await loadCaptureState(stationId))?.records ?? const <CaptureRecord>[];

  /// Every station with a valid committed snapshot (or a readable legacy
  /// station) is resumable. Orphaned JPEGs and incomplete `.part` files do not
  /// make a station appear in this list.
  @override
  Future<List<String>> stationIds() async {
    final root = Directory(_root);
    if (!await root.exists()) return const <String>[];
    final entries = <Directory>[];
    await for (final entity in root.list(followLinks: false)) {
      if (entity is Directory) entries.add(entity);
    }
    final stations = <({String id, int generation, DateTime modified})>[];
    for (final entry in entries) {
      final id = p.basename(entry.path);
      final committed = await _loadCommittedSnapshot(id);
      if (committed != null) {
        stations.add((
          id: id,
          generation: _generationOrder(committed.generation),
          modified: await entry.stat().then((stat) => stat.modified),
        ));
      } else if (!await _hasFinalManifest(id) &&
          await _loadLegacySnapshot(id) != null) {
        stations.add((
          id: id,
          generation: 0,
          modified: await entry.stat().then((stat) => stat.modified),
        ));
      }
    }
    stations.sort((a, b) {
      final generation = b.generation.compareTo(a.generation);
      return generation != 0 ? generation : b.modified.compareTo(a.modified);
    });
    return stations.map((entry) => entry.id).toList(growable: false);
  }

  Future<_CaptureSnapshot?> _loadLegacySnapshot(String stationId) async {
    final station = await _readLegacyStation(stationId);
    if (station == null || station.stationId != stationId) return null;
    final targets = await _readLegacyTargets(stationId);
    final records = await _readLegacyRecords(stationId);
    if (targets.any((CaptureTarget target) => target.stationId != stationId) ||
        records.any((CaptureRecord record) => record.stationId != stationId)) {
      return null;
    }
    return _CaptureSnapshot(
      generation: null,
      previousGeneration: null,
      station: station,
      targets: targets,
      records: records,
    );
  }

  /// Reads the newest internally valid snapshot. A partially written manifest,
  /// a manifest whose image was lost, and an orphaned image are all ignored;
  /// the previous valid generation remains authoritative.
  Future<_CaptureSnapshot?> _loadCommittedSnapshot(String stationId) async {
    final files = await _finalManifestFiles(stationId);
    final snapshots = <String, _CaptureSnapshot>{};
    for (final file in files) {
      final snapshot = await _readValidatedManifest(file, stationId);
      final generation = snapshot?.generation;
      if (generation != null) snapshots[generation] = snapshot!;
    }
    final candidates = snapshots.values.toList(growable: false)
      ..sort(
        (_CaptureSnapshot a, _CaptureSnapshot b) =>
            _compareGenerations(b.generation, a.generation),
      );
    for (final candidate in candidates) {
      if (_hasCompleteLineage(candidate, snapshots)) return candidate;
    }
    return null;
  }

  Future<List<File>> _finalManifestFiles(String stationId) async {
    final dir = Directory(_manifestDir(stationId));
    if (!await dir.exists()) return const <File>[];
    final files = <File>[];
    await for (final entity in dir.list(followLinks: false)) {
      if (entity is File && entity.path.endsWith('.json')) files.add(entity);
    }
    return files;
  }

  Future<bool> _hasFinalManifest(String stationId) async =>
      (await _finalManifestFiles(stationId)).isNotEmpty;

  bool _hasCompleteLineage(
    _CaptureSnapshot candidate,
    Map<String, _CaptureSnapshot> snapshots,
  ) {
    var current = candidate;
    final seen = <String>{};
    while (true) {
      final generation = current.generation;
      if (generation == null || !seen.add(generation)) return false;
      final previous = current.previousGeneration;
      if (previous == null) return true;
      final parent = snapshots[previous];
      if (parent == null ||
          _compareGenerations(parent.generation, current.generation) >= 0) {
        return false;
      }
      current = parent;
    }
  }

  int _generationOrder(String? generation) {
    if (generation == null) return 0;
    return int.tryParse(generation.split('-').first) ?? 0;
  }

  int _compareGenerations(String? left, String? right) {
    final byOrder = _generationOrder(left).compareTo(_generationOrder(right));
    if (byOrder != 0) return byOrder;
    return (left ?? '').compareTo(right ?? '');
  }

  Future<String> _nextManifestGeneration(String stationId) async {
    var maximum = 0;
    for (final file in await _finalManifestFiles(stationId)) {
      final generation = p.basenameWithoutExtension(file.path);
      maximum = math.max(maximum, _generationOrder(generation));
    }
    return (maximum + 1).toString().padLeft(20, '0');
  }

  Future<_CaptureSnapshot?> _readValidatedManifest(
    File file,
    String stationId,
  ) async {
    try {
      final decoded = jsonDecode(await file.readAsString());
      if (decoded is! Map<String, dynamic>) return null;
      final snapshot = _CaptureSnapshot.fromJson(decoded);
      if (snapshot.generation != p.basenameWithoutExtension(file.path) ||
          !_isStateConsistent(
            station: snapshot.station,
            targets: snapshot.targets,
            records: snapshot.records,
            requireImageIntegrityMetadata: true,
          ) ||
          snapshot.station.stationId != stationId) {
        return null;
      }
      for (final record in snapshot.records) {
        if (!await _recordIsIntact(stationId, record)) return null;
      }
      return snapshot;
    } catch (_) {
      // A bad snapshot never throws through resume. Older immutable snapshots
      // are still candidates; split legacy files cannot supersede a manifest.
      return null;
    }
  }

  bool _hasDistinctTargetIds(List<CaptureTarget> targets) =>
      targets.map((CaptureTarget target) => target.targetId).toSet().length ==
      targets.length;

  bool _hasDistinctRecordIds(List<CaptureRecord> records) =>
      records.map((CaptureRecord record) => record.targetId).toSet().length ==
      records.length;

  Future<void> _publishSnapshot({
    required CaptureStation station,
    required List<CaptureTarget> targets,
    required List<CaptureRecord> records,
    required String? previousGeneration,
  }) async {
    _validateState(station: station, targets: targets, records: records);
    final generation = await _nextManifestGeneration(station.stationId);
    final snapshot = _CaptureSnapshot(
      generation: generation,
      previousGeneration: previousGeneration,
      station: station,
      targets: List<CaptureTarget>.of(targets),
      records: List<CaptureRecord>.of(records),
    );
    final finalFile = File(
      p.join(_manifestDir(station.stationId), '$generation.json'),
    );
    await finalFile.parent.create(recursive: true);
    final part = File('${finalFile.path}.part');
    await part.writeAsString(jsonEncode(snapshot.toJson()), flush: true);
    // The destination name is a new generation, so rename never needs to
    // replace the prior manifest. Same-directory rename is the commit point.
    await part.rename(finalFile.path);
  }

  Future<_StoredImage> _writeImmutableJpeg({
    required String stationId,
    required int targetId,
    required Uint8List bytes,
  }) async {
    if (!_hasJpegMarkers(bytes)) {
      throw const StorageFailure('The camera bytes are not a complete JPEG.');
    }
    final finalPath = await framePath(stationId, targetId);
    final part = File('$finalPath.part');
    await part.parent.create(recursive: true);
    await part.writeAsBytes(bytes, flush: true);
    final readBack = await part.readAsBytes();
    if (readBack.length != bytes.length || !_sameBytes(readBack, bytes)) {
      throw const StorageFailure(
        'The temporary photograph did not read back intact.',
      );
    }
    final decoded = await _decodeJpeg(readBack);
    await part.rename(finalPath);
    return _StoredImage(
      path: finalPath,
      byteLength: readBack.length,
      sha256: sha256.convert(readBack).toString(),
      width: decoded.width,
      height: decoded.height,
    );
  }

  Future<List<CaptureRecord>> _normaliseRecords(
    String stationId,
    List<CaptureRecord> records,
  ) async {
    final result = <CaptureRecord>[];
    for (final record in records) {
      if (record.stationId != stationId) {
        throw const StorageFailure(
          'A capture record belongs to another station.',
        );
      }
      final image = await _readVerifiedImage(stationId, record);
      if (image == null) {
        throw StorageFailure(
          'An existing accepted image for target ${record.targetId} is no longer valid.',
        );
      }
      result.add(
        record.copyWith(
          imageByteLength: image.byteLength,
          imageSha256: image.sha256,
        ),
      );
    }
    return result;
  }

  Future<bool> _recordIsIntact(String stationId, CaptureRecord record) async =>
      await _readVerifiedImage(stationId, record) != null;

  Future<_StoredImage?> _readVerifiedImage(
    String stationId,
    CaptureRecord record,
  ) async {
    final expectedLength = record.imageByteLength;
    final expectedSha256 = record.imageSha256;
    // Old split-file records predate digest metadata. They may be promoted only
    // by reading the real bytes, validating markers and a full decode, then
    // writing the calculated length/hash into the first manifest. A partially
    // populated digest is never trusted as legacy data.
    final missingBoth = expectedLength == null && expectedSha256 == null;
    if (!missingBoth &&
        (expectedLength == null ||
            expectedLength <= 0 ||
            expectedSha256 == null)) {
      return null;
    }
    final path = record.imagePath;
    if (!_isPathWithinStation(stationId, path)) return null;
    try {
      final file = File(path);
      if (!await file.exists()) return null;
      final bytes = await file.readAsBytes();
      final actualSha256 = sha256.convert(bytes).toString();
      if ((expectedLength != null && bytes.length != expectedLength) ||
          (expectedSha256 != null && actualSha256 != expectedSha256) ||
          !_hasJpegMarkers(bytes)) {
        return null;
      }
      final decoded = await _decodeJpeg(bytes);
      if ((record.imageWidth > 0 && record.imageWidth != decoded.width) ||
          (record.imageHeight > 0 && record.imageHeight != decoded.height)) {
        return null;
      }
      return _StoredImage(
        path: path,
        byteLength: bytes.length,
        sha256: actualSha256,
        width: decoded.width,
        height: decoded.height,
      );
    } catch (_) {
      return null;
    }
  }

  bool _isPathWithinStation(String stationId, String path) {
    final root = p.normalize(_stationDir(stationId));
    final candidate = p.normalize(path);
    return p.isWithin(root, candidate);
  }

  Future<_DecodedImage> _decodeJpeg(Uint8List bytes) async {
    try {
      final codec = await ui.instantiateImageCodec(bytes);
      try {
        final frame = await codec.getNextFrame();
        final image = frame.image;
        try {
          if (image.width <= 0 || image.height <= 0) {
            throw const StorageFailure(
              'The JPEG decoder reported empty dimensions.',
            );
          }
          return _DecodedImage(width: image.width, height: image.height);
        } finally {
          image.dispose();
        }
      } finally {
        codec.dispose();
      }
    } on StorageFailure {
      rethrow;
    } on Object catch (error) {
      throw StorageFailure('The JPEG could not be decoded: $error');
    }
  }

  Future<void> _writeJsonAtomically(File target, Object json) async {
    await target.parent.create(recursive: true);
    final part = File('${target.path}.${_nextImageVersion()}.part');
    await part.writeAsString(jsonEncode(json), flush: true);
    try {
      await part.rename(target.path);
    } on FileSystemException catch (error) {
      // Do not delete an old legacy record just to make a replacement fit on a
      // platform with restrictive rename semantics. The new `.part` is ignored
      // on restart and the old committed state remains authoritative.
      throw StorageFailure(
        'Could not atomically replace ${target.path}: $error',
      );
    }
  }

  Future<CaptureStation?> _readLegacyStation(String stationId) async {
    try {
      final file = File(_legacyStationPath(stationId));
      if (!await file.exists()) return null;
      final decoded = jsonDecode(await file.readAsString());
      if (decoded is! Map<String, dynamic>) return null;
      return CaptureStation.fromJson(decoded);
    } catch (_) {
      return null;
    }
  }

  Future<List<CaptureTarget>> _readLegacyTargets(String stationId) async {
    try {
      final file = File(_legacyTargetsPath(stationId));
      if (!await file.exists()) return const <CaptureTarget>[];
      final decoded = jsonDecode(await file.readAsString());
      if (decoded is! List<dynamic>) return const <CaptureTarget>[];
      return decoded
          .whereType<Map<String, dynamic>>()
          .map(CaptureTarget.fromJson)
          .toList(growable: false);
    } catch (_) {
      return const <CaptureTarget>[];
    }
  }

  Future<List<CaptureRecord>> _readLegacyRecords(String stationId) async {
    try {
      final file = File(_legacyRecordsPath(stationId));
      if (!await file.exists()) return const <CaptureRecord>[];
      final decoded = jsonDecode(await file.readAsString());
      if (decoded is! List<dynamic>) return const <CaptureRecord>[];
      return decoded
          .whereType<Map<String, dynamic>>()
          .map(CaptureRecord.fromJson)
          .toList(growable: false);
    } catch (_) {
      return const <CaptureRecord>[];
    }
  }

  void _validateState({
    required CaptureStation station,
    required List<CaptureTarget> targets,
    required List<CaptureRecord> records,
  }) {
    if (!_isStateConsistent(
      station: station,
      targets: targets,
      records: records,
      requireImageIntegrityMetadata: true,
    )) {
      throw const StorageFailure(
        'The capture snapshot has inconsistent station, target, or image '
        'records.',
      );
    }
  }

  bool _isStateConsistent({
    required CaptureStation station,
    required List<CaptureTarget> targets,
    required List<CaptureRecord> records,
    required bool requireImageIntegrityMetadata,
  }) {
    if (targets.any(
          (CaptureTarget target) =>
              target.stationId != station.stationId || target.targetId <= 0,
        ) ||
        records.any(
          (CaptureRecord record) =>
              record.stationId != station.stationId || record.targetId <= 0,
        ) ||
        !_hasDistinctTargetIds(targets) ||
        !_hasDistinctRecordIds(records)) {
      return false;
    }
    final recordsByTarget = <int, CaptureRecord>{
      for (final record in records) record.targetId: record,
    };
    final capturedCount = targets
        .where((CaptureTarget target) => target.status.isDone)
        .length;
    if (station.capturedCount != capturedCount) return false;

    for (final target in targets) {
      final record = recordsByTarget[target.targetId];
      if (target.status.isDone) {
        if (record == null ||
            target.imagePath == null ||
            target.imagePath != record.imagePath) {
          return false;
        }
      } else if (record != null) {
        return false;
      }
    }
    // A record for an id that is not present in targets is never evidence for
    // this snapshot, even when it points at a readable JPEG.
    if (records.length != recordsByTarget.length ||
        records.any(
          (CaptureRecord record) => !targets.any(
            (CaptureTarget target) => target.targetId == record.targetId,
          ),
        )) {
      return false;
    }
    if (requireImageIntegrityMetadata &&
        records.any(
          (CaptureRecord record) =>
              record.imageByteLength == null ||
              record.imageByteLength! <= 0 ||
              record.imageSha256 == null ||
              record.imageSha256!.isEmpty,
        )) {
      return false;
    }
    return true;
  }

  void _validateTargetId(int targetId) {
    if (targetId <= 0) {
      throw const StorageFailure('Capture target identifiers start at 1.');
    }
  }

  static bool _hasJpegMarkers(List<int> bytes) =>
      bytes.length >= 4 &&
      bytes[0] == 0xff &&
      bytes[1] == 0xd8 &&
      bytes[bytes.length - 2] == 0xff &&
      bytes.last == 0xd9;

  static bool _sameBytes(List<int> left, List<int> right) {
    if (left.length != right.length) return false;
    for (var index = 0; index < left.length; index++) {
      if (left[index] != right[index]) return false;
    }
    return true;
  }
}

class _CaptureSnapshot {
  const _CaptureSnapshot({
    required this.generation,
    required this.previousGeneration,
    required this.station,
    required this.targets,
    required this.records,
  });

  final String? generation;
  final String? previousGeneration;
  final CaptureStation station;
  final List<CaptureTarget> targets;
  final List<CaptureRecord> records;

  CapturePersistedState toPersistedState() => CapturePersistedState(
    station: station,
    targets: targets,
    records: records,
  );

  Map<String, dynamic> toJson() => <String, dynamic>{
    'schemaVersion': FileCaptureRepository._manifestSchemaVersion,
    'generation': generation,
    'previousGeneration': previousGeneration,
    'station': station.toJson(),
    'targets': targets
        .map((CaptureTarget target) => target.toJson())
        .toList(growable: false),
    'records': records
        .map((CaptureRecord record) => record.toJson())
        .toList(growable: false),
  };

  static _CaptureSnapshot fromJson(Map<String, dynamic> json) {
    if (json['schemaVersion'] != FileCaptureRepository._manifestSchemaVersion ||
        json['generation'] is! String ||
        json['station'] is! Map<String, dynamic> ||
        json['targets'] is! List<dynamic> ||
        json['records'] is! List<dynamic>) {
      throw const FormatException('Unsupported capture manifest.');
    }
    return _CaptureSnapshot(
      generation: json['generation'] as String,
      previousGeneration: json['previousGeneration'] as String?,
      station: CaptureStation.fromJson(json['station'] as Map<String, dynamic>),
      targets: (json['targets'] as List<dynamic>)
          .map(
            (dynamic item) =>
                CaptureTarget.fromJson(item as Map<String, dynamic>),
          )
          .toList(growable: false),
      records: (json['records'] as List<dynamic>)
          .map(
            (dynamic item) =>
                CaptureRecord.fromJson(item as Map<String, dynamic>),
          )
          .toList(growable: false),
    );
  }
}

class _StoredImage {
  const _StoredImage({
    required this.path,
    required this.byteLength,
    required this.sha256,
    required this.width,
    required this.height,
  });

  final String path;
  final int byteLength;
  final String sha256;
  final int width;
  final int height;
}

class _DecodedImage {
  const _DecodedImage({required this.width, required this.height});

  final int width;
  final int height;
}
