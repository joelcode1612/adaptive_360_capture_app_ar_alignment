import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_local_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/data/datasources/virtual_tour_remote_data_source.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/repositories/virtual_tour_repository.dart';

/// The repository: backend first, local rows after, and never the other way.
///
/// Order inside [uploadCaptures] is the part that matters:
///
/// 1. create (or re-read) the backend session and **persist its id
///    immediately**;
/// 2. send each JPEG, and only after the server accepts it write its row with
///    the `storage_key` and `sha256` the server returned;
/// 3. update the tour row with the server's own status and counts.
///
/// So a crash between two captures loses nothing a retry cannot rebuild — the
/// app still knows which backend tour to resume — and no local row ever claims
/// an acceptance that did not happen. Local state is written *from* backend
/// answers, which is what keeps the two consistent. Every write is an upsert
/// keyed by a deterministic id, so retrying a tour replaces rows instead of
/// duplicating them.
class VirtualTourRepositoryImpl implements VirtualTourRepository {
  VirtualTourRepositoryImpl({
    required VirtualTourRemoteDataSource remote,
    required VirtualTourLocalDataSource local,
  }) : _remote = remote, // ignore: prefer_initializing_formals
       _local = local; // ignore: prefer_initializing_formals

  final VirtualTourRemoteDataSource _remote;
  final VirtualTourLocalDataSource _local;

  @override
  Future<VirtualTourTicket> uploadCaptures({
    required String tourId,
    required String propertyId,
    required String stationId,
    required List<TourCapture> captures,
    VirtualTourUploadProgress? onProgress,
  }) async {
    if (captures.isEmpty) {
      throw const VirtualTourFailure(
        'There are no photographs to upload.',
        retryable: false,
      );
    }
    _checkTargetIndices(captures);

    final existing = await _local.virtualTour(tourId);
    final session = await _ensureSession(
      existing,
      tourId: tourId,
      propertyId: propertyId,
      stationId: stationId,
      expectedCaptureCount: captures.length,
    );

    // Keyed by target index rather than by capture, because the backend stores
    // one capture per index; two rows for the same index must not count twice.
    final accepted = <int>{};
    for (final capture in captures) {
      // Every capture goes on the wire even if a local row already marks it
      // uploaded: the backend's `(tour_id, target_index)` uniqueness plus its
      // SHA-256 comparison answers `duplicate: true` for bytes it already
      // holds, which beats a device-side guess about whether the server's
      // database still exists.
      final ack = await _remote.uploadCapture(
        tourId: session.tourId,
        capture: capture,
      );
      accepted.add(ack.targetIndex);
      await _local.upsertPropertyPhoto(
        _photoRow(
          tourId: tourId,
          propertyId: propertyId,
          capture: capture,
          ack: ack,
        ),
      );
      onProgress?.call(accepted.length, captures.length);
    }

    final ticket = VirtualTourTicket(
      tourId: session.tourId,
      propertyId: session.propertyId,
      // Still `uploading`: the server only moves past it when finalise writes
      // the manifest, and anything else here would be this app promoting
      // itself.
      status: VirtualTourStatus.uploading,
      expectedCaptureCount: session.expectedCaptureCount,
      uploadedCaptureCount: accepted.length,
      stitchManifestKey: session.stitchManifestKey,
      simulated: session.simulated,
    );
    await _writeTour(
      await _local.virtualTour(tourId),
      ticket,
      tourId: tourId,
      propertyId: propertyId,
      stationId: stationId,
      photoCount: captures.length,
    );
    return ticket;
  }

  @override
  Future<VirtualTourTicket> finaliseTour({required String tourId}) async {
    final existing = await _local.virtualTour(tourId);
    final backendTourId = existing?.backendTourId;
    if (existing == null || backendTourId == null) {
      throw const VirtualTourFailure(
        'This tour has never reached the backend, so there is nothing to '
        'finalise. Upload its photographs first.',
        retryable: false,
      );
    }
    final ticket = await _remote.finaliseTour(tourId: backendTourId);
    await _writeTour(
      existing,
      ticket,
      tourId: tourId,
      propertyId: existing.propertyId,
      stationId: existing.stationId,
      photoCount: existing.photoCount,
    );
    return ticket;
  }

  @override
  Future<VirtualTourTicket> refreshTourStatus({required String tourId}) async {
    final existing = await _local.virtualTour(tourId);
    final backendTourId = existing?.backendTourId;
    if (existing == null || backendTourId == null) {
      throw const VirtualTourFailure(
        'This tour has never reached the backend, so there is no server state '
        'to read.',
        retryable: false,
      );
    }
    final ticket = await _remote.tourStatus(tourId: backendTourId);
    await _writeTour(
      existing,
      ticket,
      tourId: tourId,
      propertyId: existing.propertyId,
      stationId: existing.stationId,
      photoCount: existing.photoCount,
    );
    return ticket;
  }

  @override
  Future<VirtualTourRecord?> tourRecord(String tourId) =>
      _local.virtualTour(tourId);


  /// The backend tour these captures upload into: reused when one exists.
  ///
  /// Reusing it is the whole point — a second `POST /virtual-tours` for the
  /// same frames would file a duplicate tour. An existing session is verified
  /// against the server rather than trusted, because the server may already
  /// have finalised it or may no longer have it at all.
  Future<VirtualTourTicket> _ensureSession(
    VirtualTourRecord? existing, {
    required String tourId,
    required String propertyId,
    required String stationId,
    required int expectedCaptureCount,
  }) async {
    final row = existing;
    final backendTourId = row?.backendTourId;
    if (row == null || backendTourId == null) {
      final created = await _remote.createTour(
        propertyId: propertyId,
        expectedCaptureCount: expectedCaptureCount,
      );
      // Before any JPEG moves: this write is the only thing that lets a retry
      // or a relaunched app find the session again.
      await _writeTour(
        row,
        created,
        tourId: tourId,
        propertyId: propertyId,
        stationId: stationId,
        photoCount: 0,
      );
      return created;
    }

    if (row.expectedCaptureCount != expectedCaptureCount) {
      throw VirtualTourFailure(
        'The backend session for this tour expects '
        '${row.expectedCaptureCount} captures, and this station is ready '
        'with $expectedCaptureCount. Start a new capture session — the count '
        'is fixed when the tour is created.',
        retryable: false,
      );
    }

    final current = await _remote.tourStatus(tourId: backendTourId);
    if (current.status != VirtualTourStatus.uploading) {
      throw VirtualTourFailure(
        'The backend already has this tour as "${current.status}", so it will '
        'not accept more captures. Start a new capture session.',
        retryable: false,
      );
    }
    await _writeTour(
      row,
      current,
      tourId: tourId,
      propertyId: propertyId,
      stationId: stationId,
      photoCount: row.photoCount,
    );
    return current;
  }

  void _checkTargetIndices(List<TourCapture> captures) {
    final seen = <int>{};
    for (final capture in captures) {
      if (capture.targetIndex < 1) {
        throw VirtualTourFailure(
          'Node ${capture.nodeId} has target index ${capture.targetIndex}; '
          'the backend numbers captures from 1.',
          retryable: false,
        );
      }
      if (!seen.add(capture.targetIndex)) {
        throw VirtualTourFailure(
          'Two photographs claim target index ${capture.targetIndex}, which '
          'the backend stores once per tour. Recapture this station.',
          retryable: false,
        );
      }
    }
  }

  Future<void> _writeTour(
    VirtualTourRecord? existing,
    VirtualTourTicket ticket, {
    required String tourId,
    required String propertyId,
    required String stationId,
    required int photoCount,
  }) {
    final now = DateTime.now().toUtc();
    return _save('The state of tour $tourId', () => _local.upsertVirtualTour(
      VirtualTourRecord(
        id: tourId,
        propertyId: propertyId,
        // Local only: the backend groups captures by tour, not by station,
        // but the device needs to know which physical station these frames
        // came from to lay out its capture plan.
        stationId: stationId,
        status: ticket.status,
        backendTourId: ticket.tourId,
        backendStatus: ticket.status,
        stitchManifestKey: ticket.stitchManifestKey,
        expectedCaptureCount: ticket.expectedCaptureCount,
        uploadedCaptureCount: ticket.uploadedCaptureCount,
        photoCount: photoCount,
        createdAt: existing?.createdAt ?? now,
        updatedAt: now,
        simulated: ticket.simulated,
      ),
    ));
  }

  PropertyPhotoRecord _photoRow({
    required String tourId,
    required String propertyId,
    required TourCapture capture,
    required VirtualTourCaptureAck ack,
  }) => PropertyPhotoRecord(
    id: '${tourId}_${capture.nodeId}',
    virtualTourId: tourId,
    propertyId: propertyId,
    nodeId: capture.nodeId,
    kind: PropertyPhotoRecord.kindCapture,
    location: capture.filePath,
    targetIndex: ack.targetIndex,
    yawDeg: capture.yawDeg,
    pitchDeg: capture.pitchDeg,
    width: ack.width,
    height: ack.height,
    storageKey: ack.storageKey,
    sha256: ack.sha256,
    capturedAt: capture.capturedAt.toUtc(),
    createdAt: DateTime.now().toUtc(),
  );

  /// Runs a local write, keeping the failure honest about which half broke.
  ///
  /// The backend already has the data at this point, so the sentence says so
  /// rather than reporting the whole operation as if it had failed.
  Future<void> _save(String what, Future<void> Function() write) async {
    try {
      await write();
    } on VirtualTourFailure {
      rethrow;
    } catch (e) {
      throw VirtualTourFailure(
        '$what is recorded by the backend, but this device could not save '
        'it: $e',
      );
    }
  }
}
