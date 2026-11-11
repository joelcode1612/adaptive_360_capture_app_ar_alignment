import 'dart:async';

import 'package:flutter_bloc/flutter_bloc.dart';

import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/repositories/virtual_tour_repository.dart';

/// REQ-807 / REQ-808: the ingestion workflow as an explicit state machine.
///
/// The states follow the backend's own lifecycle — session, captures,
/// manifest — and stop there. Nothing is called "stitched", because stitching
/// is a worker that has not been built: the furthest honest state is
/// [VirtualTourAwaitingStitching], which reports the server's
/// `stitch_pending` and the manifest key it wrote.
sealed class VirtualTourState {
  const VirtualTourState();
}

/// Nothing has been requested yet.
class VirtualTourInitial extends VirtualTourState {
  final String? message;

  const VirtualTourInitial({this.message});
}

/// Work is in flight.
///
/// [stage] names which of the backend's operations is running; during
/// [VirtualTourStage.uploading] the counts say how many captures the server has
/// accepted so far — real numbers from real acknowledgements, not a simulated
/// percentage.
class VirtualTourProcessing extends VirtualTourState {
  final VirtualTourStage stage;
  final String message;
  final int uploaded;
  final int total;

  const VirtualTourProcessing({
    required this.stage,
    required this.message,
    this.uploaded = 0,
    this.total = 0,
  });
}

/// Every capture is stored and accepted by the backend; the tour has not been
/// finalised, so no stitch manifest exists yet.
class VirtualTourUploaded extends VirtualTourState {
  final VirtualTourTicket ticket;

  const VirtualTourUploaded({required this.ticket});
}

/// The backend has validated the capture count and written the stitch
/// manifest. This is **not** a finished panorama: it says a worker has to run.
class VirtualTourAwaitingStitching extends VirtualTourState {
  final VirtualTourTicket ticket;

  const VirtualTourAwaitingStitching({required this.ticket});
}

/// Everything failed with a message the user can act on.
class VirtualTourError extends VirtualTourState {
  final String message;
  final VirtualTourStage stage;
  final bool retryable;

  /// Whatever the backend had already confirmed before this failure.
  ///
  /// Keeping it is what lets the screen say "24 of 30 uploaded" next to the
  /// error instead of pretending the session is empty — the state a retry
  /// depends on being true. Only a *completed* operation produces a ticket, so
  /// an upload that failed halfway through has none; [uploaded] and [total]
  /// cover that case.
  final VirtualTourTicket? ticket;

  /// Captures the server acknowledged inside the operation that failed.
  ///
  /// These are counts, not a ticket: the operation never finished, so there is
  /// no confirmed server state to carry — only the acknowledgements that
  /// arrived before the failure. Zero when the failure had nothing accepted.
  final int uploaded;
  final int total;

  const VirtualTourError({
    required this.message,
    required this.stage,
    required this.retryable,
    this.ticket,
    this.uploaded = 0,
    this.total = 0,
  });
}

sealed class VirtualTourEvent {
  const VirtualTourEvent();
}

/// Create the backend session if needed and upload the station's captures.
class VirtualTourUploadRequested extends VirtualTourEvent {
  final String tourId;
  final String propertyId;
  final String stationId;
  final List<TourCapture> captures;

  const VirtualTourUploadRequested({
    required this.tourId,
    required this.propertyId,
    required this.stationId,
    required this.captures,
  });
}

/// Ask the backend to validate the count and write the stitch manifest.
class VirtualTourFinaliseRequested extends VirtualTourEvent {
  final String tourId;

  const VirtualTourFinaliseRequested({required this.tourId});
}

/// Re-read the server's status for a tour.
class VirtualTourRefreshRequested extends VirtualTourEvent {
  final String tourId;

  const VirtualTourRefreshRequested({required this.tourId});
}

/// Retry the last failed operation, with the same payload.
class VirtualTourRetryRequested extends VirtualTourEvent {
  const VirtualTourRetryRequested();
}

/// Drop every result and go back to [VirtualTourInitial].
class VirtualTourResetRequested extends VirtualTourEvent {
  const VirtualTourResetRequested();
}

/// The Bloc from the brief.
///
/// It owns three things the screen must not:
///
/// * **duplicate protection** — a second upload or finalise while one is
///   running is ignored rather than sent twice;
/// * **out-of-order protection** — every operation carries a token, and a
///   response that arrives after a reset or a newer operation is discarded,
///   including the per-capture progress callbacks;
/// * **retry** — only the last failed operation is retried, with its original
///   captures, so "Try again" cannot finalise a tour that was never uploaded.
///
/// What it does *not* own is truth about the backend: every state below this
/// line comes from a `TourResponse` the repository read off the server, so a
/// failed request leaves the machine in an error state rather than a hopeful
/// one.
class VirtualTourBloc extends Bloc<VirtualTourEvent, VirtualTourState> {
  VirtualTourBloc({required VirtualTourRepository repository})
    : _repository = repository, // ignore: prefer_initializing_formals
       super(const VirtualTourInitial()) {
    on<VirtualTourUploadRequested>(_onUpload);
    on<VirtualTourFinaliseRequested>(_onFinalise);
    on<VirtualTourRefreshRequested>(_onRefresh);
    on<VirtualTourRetryRequested>(_onRetry);
    on<VirtualTourResetRequested>(_onReset);
  }

  final VirtualTourRepository _repository;

  /// Incremented by every new operation and by reset. A completion whose token
  /// no longer matches is stale and must not emit.
  int _operation = 0;

  /// The last operation that failed, for [VirtualTourRetryRequested].
  VirtualTourEvent? _lastFailed;

  /// The ticket the most recent operation confirmed, carried into errors so a
  /// failure cannot erase what the backend already accepted.
  VirtualTourTicket? _lastTicket;

  Future<void> _onUpload(
    VirtualTourUploadRequested event,
    Emitter<VirtualTourState> emit,
  ) async {
    if (state is VirtualTourProcessing) return;
    await _runUpload(event, emit);
  }

  Future<void> _runUpload(
    VirtualTourUploadRequested event,
    Emitter<VirtualTourState> emit,
  ) async {
    if (event.captures.isEmpty) {
      _lastFailed = null;
      emit(
        const VirtualTourError(
          message: 'There are no photographs to upload.',
          stage: VirtualTourStage.validation,
          retryable: false,
        ),
      );
      return;
    }

    final token = ++_operation;
    _lastFailed = null;
    emit(
      VirtualTourProcessing(
        stage: VirtualTourStage.creating,
        message:
            'Opening the upload session for ${event.captures.length} '
            'photographs…',
        total: event.captures.length,
      ),
    );

    var uploaded = 0;
    void onProgress(int accepted, int total) {
      // The repository calls this from inside its own loop; a token that no
      // longer matches means reset ran, and this progress belongs to nothing.
      if (isClosed || token != _operation) return;
      uploaded = accepted;
      emit(
        VirtualTourProcessing(
          stage: VirtualTourStage.uploading,
          message: 'Uploading photograph $accepted of $total…',
          uploaded: accepted,
          total: total,
        ),
      );
    }

    try {
      final ticket = await _repository.uploadCaptures(
        tourId: event.tourId,
        propertyId: event.propertyId,
        stationId: event.stationId,
        captures: List<TourCapture>.unmodifiable(event.captures),
        onProgress: onProgress,
      );
      if (isClosed || token != _operation) return;
      _lastTicket = ticket;
      emit(VirtualTourUploaded(ticket: ticket));
    } on VirtualTourFailure catch (failure) {
      if (isClosed || token != _operation) return;
      _lastFailed = event;
      emit(
        VirtualTourError(
          message: failure.message,
          stage: _uploadStageFor(uploaded, event.captures.length),
          retryable: failure.retryable,
          ticket: _lastTicket,
          uploaded: uploaded,
          total: event.captures.length,
        ),
      );
    } catch (e, _) {
      if (isClosed || token != _operation) return;
      _lastFailed = event;
      emit(
        VirtualTourError(
          message: 'The photographs could not be uploaded: $e',
          stage: _uploadStageFor(uploaded, event.captures.length),
          retryable: true,
          ticket: _lastTicket,
          uploaded: uploaded,
          total: event.captures.length,
        ),
      );
    }
  }

  /// Where an upload failure happened, from the counter the backend drove.
  ///
  /// Zero accepted means the session itself never worked, which is a different
  /// message to a user than "capture 17 was rejected".
  VirtualTourStage _uploadStageFor(int uploaded, int total) =>
      uploaded == 0 && total > 0 ? VirtualTourStage.creating
                                  : VirtualTourStage.uploading;

  Future<void> _onFinalise(
    VirtualTourFinaliseRequested event,
    Emitter<VirtualTourState> emit,
  ) async {
    if (state is VirtualTourProcessing) return;
    await _runFinalise(event, emit);
  }

  Future<void> _runFinalise(
    VirtualTourFinaliseRequested event,
    Emitter<VirtualTourState> emit,
  ) async {
    final token = ++_operation;
    _lastFailed = null;
    emit(
      const VirtualTourProcessing(
        stage: VirtualTourStage.finalising,
        message: 'Asking the backend to check the captures and write the '
            'stitch manifest…',
      ),
    );

    try {
      final ticket = await _repository.finaliseTour(tourId: event.tourId);
      if (isClosed || token != _operation) return;
      _lastTicket = ticket;
      emit(_stateForTicket(ticket));
    } on VirtualTourFailure catch (failure) {
      if (isClosed || token != _operation) return;
      _lastFailed = event;
      emit(
        VirtualTourError(
          message: failure.message,
          stage: VirtualTourStage.finalising,
          retryable: failure.retryable,
          ticket: _lastTicket,
        ),
      );
    } catch (e, _) {
      if (isClosed || token != _operation) return;
      _lastFailed = event;
      emit(
        VirtualTourError(
          message: 'The tour could not be finalised: $e',
          stage: VirtualTourStage.finalising,
          retryable: true,
          ticket: _lastTicket,
        ),
      );
    }
  }

  Future<void> _onRefresh(
    VirtualTourRefreshRequested event,
    Emitter<VirtualTourState> emit,
  ) async {
    if (state is VirtualTourProcessing) return;
    final token = ++_operation;
    _lastFailed = null;
    emit(
      const VirtualTourProcessing(
        stage: VirtualTourStage.status,
        message: 'Reading the tour\'s status from the backend…',
      ),
    );
    try {
      final ticket = await _repository.refreshTourStatus(tourId: event.tourId);
      if (isClosed || token != _operation) return;
      _lastTicket = ticket;
      emit(_stateForTicket(ticket));
    } on VirtualTourFailure catch (failure) {
      if (isClosed || token != _operation) return;
      _lastFailed = event;
      emit(
        VirtualTourError(
          message: failure.message,
          stage: VirtualTourStage.status,
          retryable: failure.retryable,
          ticket: _lastTicket,
        ),
      );
    } catch (e, _) {
      if (isClosed || token != _operation) return;
      _lastFailed = event;
      emit(
        VirtualTourError(
          message: 'The backend could not report this tour: $e',
          stage: VirtualTourStage.status,
          retryable: true,
          ticket: _lastTicket,
        ),
      );
    }
  }

  /// Where the server says the tour is, expressed as a state.
  ///
  /// `uploading` still means the captures need finalising; anything past it is
  /// a manifest that exists. A status the app has never heard of is reported as
  /// awaiting stitching with its own word shown, not forced into a known one.
  VirtualTourState _stateForTicket(VirtualTourTicket ticket) =>
      ticket.status == VirtualTourStatus.uploading
      ? VirtualTourUploaded(ticket: ticket)
      : VirtualTourAwaitingStitching(ticket: ticket);

  Future<void> _onRetry(
    VirtualTourRetryRequested event,
    Emitter<VirtualTourState> emit,
  ) async {
    if (state is VirtualTourProcessing) return;
    final failed = _lastFailed;
    if (failed == null) return;
    // A failure classified as final — a 409, a fixed capture count, a frame
    // that is not a JPEG — produces the same answer on a second attempt, so
    // the event is ignored exactly as the screen omits the button.
    if (state case final VirtualTourError error when !error.retryable) return;
    _lastFailed = null;
    switch (failed) {
      case final VirtualTourUploadRequested upload:
        await _runUpload(upload, emit);
      case final VirtualTourFinaliseRequested finalise:
        await _runFinalise(finalise, emit);
      case final VirtualTourRefreshRequested refresh:
        await _onRefresh(refresh, emit);
      case final VirtualTourRetryRequested _:
      case final VirtualTourResetRequested _:
        return;
    }
  }

  void _onReset(
    VirtualTourResetRequested event,
    Emitter<VirtualTourState> emit,
  ) {
    _operation++;
    _lastFailed = null;
    _lastTicket = null;
    emit(const VirtualTourInitial());
  }
}
