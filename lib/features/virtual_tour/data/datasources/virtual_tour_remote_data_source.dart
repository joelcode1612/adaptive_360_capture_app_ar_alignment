import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:http/http.dart' as http;
import 'package:http_parser/http_parser.dart';

import 'package:adaptive_360_capture_app/features/virtual_tour/data/services/virtual_tour_photo_store.dart';
import 'package:adaptive_360_capture_app/features/virtual_tour/domain/models/virtual_tour.dart';

/// Whether this build talks to the SiteSense API or to nothing at all.
///
/// Declared, never inferred. The previous build chose by asking "is a base URL
/// defined?", which meant a mistyped URL — or a backend build that forgot one
/// flag — silently produced a *simulated* panorama and a green success panel.
/// [backend] therefore fails at construction when it cannot reach a real
/// server, and [mock] says so in every string the UI shows.
enum VirtualTourMode {
  /// No network. Captures are stored on this device and the tour is reported as
  /// a simulation, never as a backend result.
  mock,

  /// Every tour id, acceptance and status comes from the FastAPI service.
  backend;

  static const String _environmentKey = 'VIRTUAL_TOUR_MODE';

  /// The mode compiled into this build. Unset means [mock].
  ///
  /// An unrecognised value throws instead of defaulting: `--dart-define`
  /// typos are otherwise invisible until a "successful" upload turns out to
  /// have gone nowhere.
  static VirtualTourMode fromEnvironment() {
    final raw = String.fromEnvironment(_environmentKey).trim().toLowerCase();
    if (raw.isEmpty) return VirtualTourMode.mock;
    return VirtualTourMode.values.firstWhere(
      (mode) => mode.name == raw,
      orElse: () => throw VirtualTourFailure(
        '$_environmentKey must be "mock" or "backend", not "$raw".',
        retryable: false,
      ),
    );
  }
}

/// Where the SiteSense API lives and how this app identifies itself to it.
///
/// The endpoints are not invented here: they are the four routes in
/// `backend/src/sitesense/api/v1/virtual_tours.py`. Paths stay configurable so
/// a deployment behind a prefix works without a code change, but the shape of
/// every request — multipart per capture, JSON for the rest — is the server's.
class VirtualTourApiConfig {
  final Uri baseUri;

  /// Value of the `X-User-Id` development identity header.
  ///
  /// The backend only honours it while `SITESENSE_ALLOW_DEVELOPMENT_IDENTITY`
  /// is true, and it must parse as a UUID (`dependencies.py:get_current_user`).
  /// It is a *development* boundary, not authentication: production replaces it
  /// with verified JWT and this field becomes meaningless.
  final String userId;

  /// Prefix of the tour routes, `/api/v1/virtual-tours`.
  final String toursPath;

  /// Per-request deadline. Uploads are single JPEGs, so this is a ceiling on a
  /// stalled connection rather than a stitch timeout.
  final Duration timeout;

  /// Extra headers on every request — a real deployment puts its bearer token
  /// here rather than in this file.
  final Map<String, String> headers;

  const VirtualTourApiConfig({
    required this.baseUri,
    required this.userId,
    this.toursPath = '/api/v1/virtual-tours',
    this.timeout = const Duration(seconds: 60),
    this.headers = const <String, String>{},
  });

  static const String _environmentBase = 'VIRTUAL_TOUR_API_BASE_URL';
  static const String _environmentUser = 'VIRTUAL_TOUR_USER_ID';
  static const String _environmentTimeout = 'VIRTUAL_TOUR_TIMEOUT_SECONDS';

  /// The shared developer identity, used when the build defines no other.
  ///
  /// Deliberately a valid UUID: the backend rejects anything else with a 401,
  /// and a failure that says "set SITESENSE_ALLOW_DEVELOPMENT_IDENTITY" is only
  /// useful if the app's own header is not the thing that is wrong.
  static const String defaultDevelopmentUserId =
      '11111111-1111-1111-1111-111111111111';

  /// The property a tour is filed under when no route parameter supplies one.
  ///
  /// `POST /virtual-tours` takes `property_id: uuid.UUID`, so the old
  /// placeholder string `"property_local"` is now a guaranteed 422; a
  /// development build reads `VIRTUAL_TOUR_PROPERTY_ID` instead and defaults to
  /// this UUID.
  static const String defaultDevelopmentPropertyId =
      '22222222-2222-2222-2222-222222222222';

  static const String _environmentProperty = 'VIRTUAL_TOUR_PROPERTY_ID';

  /// The property id compiled into this build, falling back to the shared
  /// development property.
  static String propertyIdFromEnvironment() {
    final raw = String.fromEnvironment(_environmentProperty).trim();
    if (raw.isEmpty) return defaultDevelopmentPropertyId;
    if (!isUuid(raw)) {
      throw VirtualTourFailure(
        '$_environmentProperty must be a UUID; got "$raw".',
        retryable: false,
      );
    }
    return raw;
  }

  static final RegExp _uuidPattern = RegExp(
    r'^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-'
    r'[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$',
  );

  /// Whether [value] is something the backend can parse as `uuid.UUID`.
  static bool isUuid(String value) => _uuidPattern.hasMatch(value);

  /// The endpoint set compiled into this build, or `null` when none was given.
  ///
  /// A malformed URL throws rather than returning `null`, because `null` reads
  /// as "no backend configured" to the caller while the developer's intent was
  /// the opposite.
  static VirtualTourApiConfig? fromEnvironment() {
    final raw = String.fromEnvironment(_environmentBase).trim();
    if (raw.isEmpty) return null;
    final uri = Uri.tryParse(raw);
    if (uri == null || !uri.hasScheme || uri.host.isEmpty) {
      throw VirtualTourFailure(
        '$_environmentBase is not a valid absolute URL: "$raw".',
        retryable: false,
      );
    }
    final user = String.fromEnvironment(_environmentUser).trim();
    if (user.isNotEmpty && !_uuidPattern.hasMatch(user)) {
      throw VirtualTourFailure(
        '$_environmentUser must be a UUID; got "$user".',
        retryable: false,
      );
    }
    final seconds = int.tryParse(
      String.fromEnvironment(_environmentTimeout).trim(),
    );
    return VirtualTourApiConfig(
      baseUri: uri,
      userId: user.isEmpty ? defaultDevelopmentUserId : user,
      timeout: seconds == null || seconds <= 0
          ? const Duration(seconds: 60)
          : Duration(seconds: seconds),
    );
  }

  Uri get toursUri => baseUri.resolve(toursPath);

  Uri tourUri(String tourId) => baseUri.resolve(_tourPath(tourId));

  Uri capturesUri(String tourId) => baseUri.resolve('${_tourPath(tourId)}/captures');

  Uri finaliseUri(String tourId) => baseUri.resolve('${_tourPath(tourId)}/finalise');

  String _tourPath(String tourId) =>
      '$toursPath/${Uri.encodeComponent(tourId)}';

  /// The filename part of a capture upload.
  ///
  /// The server ignores filenames and orders by `target_index`, so this exists
  /// only to keep a captured frame recognisable in a proxy log.
  static String fileNameFor(int targetIndex) =>
      't${targetIndex.toString().padLeft(3, '0')}.jpg';
}

/// The network seam. The repository is written against this, so the Bloc and
/// its tests never open a socket.
///
/// Four operations, one per route. There is no "stitch" method here on purpose:
/// stitching is a worker behind `finalise`, and this phase must not pretend an
/// image exists before that worker has run.
abstract class VirtualTourRemoteDataSource {
  /// `POST /api/v1/virtual-tours` — the server generates the tour id.
  Future<VirtualTourTicket> createTour({
    required String propertyId,
    required int expectedCaptureCount,
  });

  /// `POST /api/v1/virtual-tours/{tourId}/captures` — one JPEG per call.
  Future<VirtualTourCaptureAck> uploadCapture({
    required String tourId,
    required TourCapture capture,
  });

  /// `POST /api/v1/virtual-tours/{tourId}/finalise` — validates the count and
  /// writes the stitch manifest.
  Future<VirtualTourTicket> finaliseTour({required String tourId});

  /// `GET /api/v1/virtual-tours/{tourId}` — the server's answer, used to
  /// recover after a crash or a lost response.
  Future<VirtualTourTicket> tourStatus({required String tourId});

  Future<void> dispose() async {}
}

/// Talks to the FastAPI service.
///
/// The wire shape is the server's, documented against
/// `backend/src/sitesense/api/v1/virtual_tours.py`:
///
/// * create — `POST {toursPath}` JSON `{property_id, expected_capture_count}`,
///   201 `TourResponse`;
/// * upload — `POST {toursPath}/{tour_id}/captures` `multipart/form-data` with
///   fields `target_index` (1-based), `captured_at` (ISO-8601) and `pose_json`,
///   plus one file part named `image` whose **content type must be
///   `image/jpeg`** — the service refuses anything else;
/// * finalise — `POST {toursPath}/{tour_id}/finalise`, no body;
/// * status — `GET {toursPath}/{tour_id}`.
///
/// Every request carries `X-User-Id`. Errors keep the server's own message
/// (`.error.message` for domain failures, `.detail` for validation) so a 409
/// "a different image already exists for this target_index" reaches the user
/// instead of a generic "upload failed".
class HttpVirtualTourRemoteDataSource implements VirtualTourRemoteDataSource {
  HttpVirtualTourRemoteDataSource({required this.config, http.Client? client})
    : _client = client ?? http.Client();

  final VirtualTourApiConfig config;
  final http.Client _client;

  @override
  Future<VirtualTourTicket> createTour({
    required String propertyId,
    required int expectedCaptureCount,
  }) async {
    _requireUuid(
      propertyId,
      what: 'The property id',
      hint: '--dart-define=VIRTUAL_TOUR_PROPERTY_ID',
    );
    if (expectedCaptureCount < 1) {
      throw const VirtualTourFailure(
        'A tour must expect at least one capture.',
        retryable: false,
      );
    }
    final request = http.Request('POST', config.toursUri)
      ..headers.addAll(_headers(contentType: 'application/json'))
      ..body = jsonEncode(<String, Object?>{
        'property_id': propertyId,
        'expected_capture_count': expectedCaptureCount,
      });
    final response = await _send(request, what: 'The tour service');
    return _ticket(_decodeJson(response, what: 'The tour service'));
  }

  @override
  Future<VirtualTourCaptureAck> uploadCapture({
    required String tourId,
    required TourCapture capture,
  }) async {
    if (capture.targetIndex < 1) {
      throw VirtualTourFailure(
        'Node ${capture.nodeId} has target index ${capture.targetIndex}; the '
        'backend numbers captures from 1.',
        retryable: false,
      );
    }
    final file = File(capture.filePath);
    if (!await file.exists()) {
      throw VirtualTourFailure(
        'The photograph for node ${capture.nodeId} is missing from '
        '${capture.filePath}.',
        retryable: false,
      );
    }
    // Read once. The bytes go on the wire, and checking them before the request
    // turns "the camera wrote a truncated frame" into a local message instead
    // of a 409 from the server's image decoder.
    final bytes = await file.readAsBytes();
    if (bytes.isEmpty) {
      throw VirtualTourFailure(
        'The photograph for node ${capture.nodeId} is empty.',
        retryable: false,
      );
    }
    try {
      VirtualTourPhotoStore.inspectJpeg(bytes);
    } on VirtualTourFailure catch (e) {
      // Not retryable: the frame is on this disk and every retry re-reads the
      // same bytes. Only recapturing the target changes the outcome.
      throw VirtualTourFailure(
        'The photograph for node ${capture.nodeId} is unusable (${e.message}). '
        'Recapture this target.',
        retryable: false,
      );
    }

    final request = http.MultipartRequest('POST', config.capturesUri(tourId))
      ..headers.addAll(_headers())
      ..headers['Idempotency-Key'] = '$tourId/${capture.targetIndex}'
      ..fields.addAll(<String, String>{
        'target_index': capture.targetIndex.toString(),
        'captured_at': capture.capturedAt.toUtc().toIso8601String(),
        // Required by the form even when nothing was measured: every field of
        // the server's CapturePose is optional, so `{}` validates.
        'pose_json': jsonEncode((capture.pose ?? const CapturePose()).toJson()),
      })
      ..files.add(
        http.MultipartFile.fromBytes(
          'image',
          bytes,
          filename: VirtualTourApiConfig.fileNameFor(capture.targetIndex),
          contentType: MediaType('image', 'jpeg'),
        ),
      );

    final response = await _send(request, what: 'The capture service');
    final json = _decodeJson(response, what: 'The capture service');
    final ack = VirtualTourCaptureAck.fromJson(json);
    if (ack.targetIndex != capture.targetIndex) {
      // The ack is keyed by target_index; a mismatch means the response belongs
      // to another upload, so it must not be recorded against this node.
      throw VirtualTourFailure(
        'The capture service acknowledged target ${ack.targetIndex} for a '
        'photograph sent as target ${capture.targetIndex}.',
      );
    }
    return ack;
  }

  @override
  Future<VirtualTourTicket> finaliseTour({required String tourId}) async {
    try {
      final request = http.Request('POST', config.finaliseUri(tourId))
        ..headers.addAll(_headers());
      final response = await _send(request, what: 'The finalise service');
      return _ticket(_decodeJson(response, what: 'The finalise service'));
    } on VirtualTourFailure catch (failure) {
      // A conflict here is ambiguous: "you already finalised" (success, lost
      // response) or "you are missing captures" (nothing to fix by retrying).
      // Asking for the status settles it instead of guessing from the message.
      if (failure.statusCode != 409) rethrow;
      final current = await tourStatus(tourId: tourId);
      if (current.status == VirtualTourStatus.uploading) rethrow;
      return current;
    }
  }

  @override
  Future<VirtualTourTicket> tourStatus({required String tourId}) async {
    final request = http.Request('GET', config.tourUri(tourId))
      ..headers.addAll(_headers());
    final response = await _send(request, what: 'The tour service');
    return _ticket(_decodeJson(response, what: 'The tour service'));
  }

  @override
  Future<void> dispose() async => _client.close();

  Map<String, String> _headers({String? contentType}) {
    final headers = <String, String>{
      'X-User-Id': config.userId,
      ...config.headers,
    };
    // A multipart request sets its own boundary-bearing content type, so this
    // is only added for the JSON calls.
    if (contentType != null) headers['Content-Type'] = contentType;
    return headers;
  }

  void _requireUuid(
    String value, {
    required String what,
    required String hint,
  }) {
    if (VirtualTourApiConfig.isUuid(value)) return;
    throw VirtualTourFailure(
      '$what "$value" is not a UUID, so the backend cannot accept it. '
      'Give the app a real one ($hint=<uuid>).',
      retryable: false,
    );
  }

  VirtualTourTicket _ticket(Map<String, Object?> json) {
    try {
      return VirtualTourTicket.fromJson(json);
    } on TypeError {
      throw VirtualTourFailure(
        'The tour service answered with an unexpected shape: '
        '${jsonEncode(json)}',
      );
    }
  }

  Future<http.Response> _send(
    http.BaseRequest request, {
    required String what,
  }) async {
    try {
      final streamed = await _client.send(request).timeout(config.timeout);
      final response = await http.Response.fromStream(
        streamed,
      ).timeout(config.timeout);
      if (response.statusCode < 200 || response.statusCode >= 300) {
        throw _apiFailure(response, what);
      }
      return response;
    } on TimeoutException {
      throw VirtualTourFailure(
        '$what did not answer within ${config.timeout.inSeconds}s.',
      );
    } on IOException catch (e) {
      throw VirtualTourFailure(
        '$what could not be reached at ${config.baseUri}: $e',
      );
    } on http.ClientException catch (e) {
      throw VirtualTourFailure(
        '$what could not be reached: ${e.message}',
      );
    }
  }

  /// Maps an HTTP error onto the two things the UI needs: a sentence and a
  /// verdict on whether trying again could possibly help.
  VirtualTourFailure _apiFailure(http.Response response, String what) {
    final code = response.statusCode;
    final serverMessage = _serverMessage(response);
    final detail = serverMessage == null ? '' : ' — $serverMessage';
    return switch (code) {
      401 => VirtualTourFailure(
        'The backend refused this app\'s identity (HTTP 401)$detail. It only '
        'accepts the development header while SITESENSE_ALLOW_DEVELOPMENT_'
        'IDENTITY=true, with X-User-Id as a UUID.',
        retryable: false,
        statusCode: code,
      ),
      403 => VirtualTourFailure(
        'The backend says this app may not touch that tour (HTTP 403)$detail.',
        retryable: false,
        statusCode: code,
      ),
      404 => VirtualTourFailure(
        'The backend has no tour with this id (HTTP 404)$detail. If its '
        'database was rebuilt, start a new capture session.',
        retryable: false,
        statusCode: code,
      ),
      409 => VirtualTourFailure(
        'The backend rejected this request (HTTP 409)$detail.',
        retryable: false,
        statusCode: code,
      ),
      422 => VirtualTourFailure(
        'The backend could not read this request (HTTP 422)$detail.',
        retryable: false,
        statusCode: code,
      ),
      _ => VirtualTourFailure(
        '$what answered HTTP $code$detail',
        retryable: code >= 500 || code == 408 || code == 429,
        statusCode: code,
      ),
    };
  }

  /// Pulls the server's own explanation out of either error envelope it uses:
  /// `{"error": {"code", "message"}}` for domain failures and `{"detail": …}`
  /// for FastAPI's validation and auth errors.
  String? _serverMessage(http.Response response) {
    final body = response.body.trim();
    if (body.isEmpty) return null;
    final Object? decoded;
    try {
      decoded = jsonDecode(body);
    } on FormatException {
      return _clip(body);
    }
    if (decoded is Map) {
      final error = decoded['error'];
      if (error is Map && error['message'] is String) {
        return _clip(error['message']! as String);
      }
      final detail = decoded['detail'];
      if (detail is String) return _clip(detail);
      if (detail is List && detail.isNotEmpty) return _clip(jsonEncode(detail));
    }
    return _clip(body);
  }

  String _clip(String text) => text.length > 300
      ? '${text.substring(0, 300)}…'
      : text;

  Map<String, Object?> _decodeJson(
    http.Response response, {
    required String what,
  }) {
    final body = response.body.trim();
    if (body.isEmpty) {
      throw VirtualTourFailure(
        '$what returned an empty response.',
        statusCode: response.statusCode,
      );
    }
    final Object? decoded;
    try {
      decoded = jsonDecode(body);
    } on FormatException catch (e) {
      throw VirtualTourFailure(
        '$what returned invalid JSON: ${e.message}',
        statusCode: response.statusCode,
      );
    }
    if (decoded is! Map) {
      throw VirtualTourFailure(
        '$what returned JSON that is not an object.',
        statusCode: response.statusCode,
      );
    }
    return decoded.cast<String, Object?>();
  }
}

/// The stand-in for a build that was **declared** as `VIRTUAL_TOUR_MODE=mock`.
///
/// It is not a fake stitcher and it holds no panorama: it keeps one tour in
/// memory, answers with `simulated: true`, and leaves `stitchManifestKey` null
/// because no manifest was written anywhere. Its `sha256` is empty — nothing
/// hashed these bytes — which is what stops a local row from looking like an
/// accepted upload.
class LocalVirtualTourRemoteDataSource implements VirtualTourRemoteDataSource {
  /// Tours this instance has handed out, so a later call answers with the
  /// counts it gave at create time instead of inventing new ones.
  final Map<String, _LocalSession> _sessions = <String, _LocalSession>{};

  @override
  Future<VirtualTourTicket> createTour({
    required String propertyId,
    required int expectedCaptureCount,
  }) async {
    final session = _LocalSession(
      tourId: 'local-${DateTime.now().toUtc().millisecondsSinceEpoch}',
      propertyId: propertyId,
      expectedCaptureCount: expectedCaptureCount,
    );
    _sessions[session.tourId] = session;
    return session.ticket();
  }

  @override
  Future<VirtualTourCaptureAck> uploadCapture({
    required String tourId,
    required TourCapture capture,
  }) async {
    if (capture.targetIndex < 1) {
      throw VirtualTourFailure(
        'Node ${capture.nodeId} has target index ${capture.targetIndex}; a '
        'real backend numbers captures from 1.',
        retryable: false,
      );
    }
    final session = _requireSession(tourId);
    if (session.status != VirtualTourStatus.uploading) {
      throw VirtualTourFailure(
        'This local session is already ${session.status}, so it cannot '
        'accept images.',
        retryable: false,
      );
    }
    final file = File(capture.filePath);
    if (!await file.exists()) {
      throw VirtualTourFailure(
        'The photograph for node ${capture.nodeId} is missing from '
        '${capture.filePath}.',
        retryable: false,
      );
    }
    VirtualTourJpegInfo info;
    try {
      info = VirtualTourPhotoStore.inspectJpeg(await file.readAsBytes());
    } on VirtualTourFailure catch (e) {
      // Same rule as the HTTP remote: the frame is on this disk, so a retry
      // would re-read the same bytes and fail identically.
      throw VirtualTourFailure(
        'The photograph for node ${capture.nodeId} is unusable (${e.message}). '
        'Recapture this target.',
        retryable: false,
      );
    }
    final duplicate = session.targets.contains(capture.targetIndex);
    session.targets.add(capture.targetIndex);
    return VirtualTourCaptureAck(
      targetIndex: capture.targetIndex,
      storageKey: file.uri.toString(),
      sha256: '',
      width: info.width,
      height: info.height,
      duplicate: duplicate,
    );
  }

  @override
  Future<VirtualTourTicket> finaliseTour({required String tourId}) async {
    final session = _requireSession(tourId);
    if (session.status != VirtualTourStatus.uploading) {
      return session.ticket(); // already finalised in this session
    }
    if (session.targets.length < session.expectedCaptureCount) {
      throw VirtualTourFailure(
        'Only ${session.targets.length} of '
        '${session.expectedCaptureCount} photographs are in this local '
        'session, so there is nothing to finalise.',
        retryable: false,
      );
    }
    // Stops at `stitch_pending` because that is as far as anything can honestly
    // go with no worker behind it: there is no panorama to report.
    session.status = VirtualTourStatus.stitchPending;
    return session.ticket();
  }

  @override
  Future<VirtualTourTicket> tourStatus({required String tourId}) async =>
      _requireSession(tourId).ticket();

  @override
  Future<void> dispose() async => _sessions.clear();

  _LocalSession _requireSession(String tourId) {
    final session = _sessions[tourId];
    if (session != null) return session;
    throw VirtualTourFailure(
      'There is no local session for tour $tourId, and this build is in mock '
      'mode, so there is no server to ask.',
      retryable: false,
    );
  }
}

/// One mock tour: the counters and status a real server would own, in memory.
class _LocalSession {
  _LocalSession({
    required this.tourId,
    required this.propertyId,
    required this.expectedCaptureCount,
  });

  final String tourId;
  final String propertyId;
  final int expectedCaptureCount;
  final Set<int> targets = <int>{};

  String status = VirtualTourStatus.uploading;

  VirtualTourTicket ticket() => VirtualTourTicket(
    tourId: tourId,
    propertyId: propertyId,
    status: status,
    expectedCaptureCount: expectedCaptureCount,
    uploadedCaptureCount: targets.length,
    simulated: true,
  );
}

/// Builds the data source this build declared itself as.
///
/// There is no fallback here. `backend` with no `VIRTUAL_TOUR_API_BASE_URL`
/// throws instead of quietly simulating, which is the failure the old
/// "is a URL defined?" check made invisible: a whole capture session that
/// looked uploaded and reached no server.
VirtualTourRemoteDataSource createVirtualTourRemoteDataSource({
  VirtualTourMode? mode,
  VirtualTourApiConfig? config,
}) {
  final resolved = mode ?? VirtualTourMode.fromEnvironment();
  if (resolved == VirtualTourMode.mock) {
    return LocalVirtualTourRemoteDataSource();
  }
  final resolvedConfig = config ?? VirtualTourApiConfig.fromEnvironment();
  if (resolvedConfig == null) {
    throw const VirtualTourFailure(
      'This build is in backend mode but defines no API endpoint. Rebuild '
      'with --dart-define=VIRTUAL_TOUR_API_BASE_URL=http://… or declare '
      '--dart-define=VIRTUAL_TOUR_MODE=mock.',
      retryable: false,
    );
  }
  return HttpVirtualTourRemoteDataSource(config: resolvedConfig);
}
