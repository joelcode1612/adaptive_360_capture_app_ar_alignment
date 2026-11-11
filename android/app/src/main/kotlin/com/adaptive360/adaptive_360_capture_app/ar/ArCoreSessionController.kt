package com.adaptive360.adaptive_360_capture_app.ar

import android.content.Context
import android.graphics.SurfaceTexture
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import com.google.ar.core.Anchor
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Camera
import com.google.ar.core.Config
import com.google.ar.core.Coordinates2d
import com.google.ar.core.Frame
import com.google.ar.core.HitResult
import com.google.ar.core.Plane
import com.google.ar.core.Pose
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import io.flutter.view.TextureRegistry
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference

/** The outcome of a call that ran on the AR thread. */
internal data class ArOutcome(
    val value: Map<String, Any?>? = null,
    val errorCode: String? = null,
    val errorMessage: String? = null,
)

/**
 * Owns the ARCore session, the GL thread and the Flutter texture it renders into.
 *
 * **Everything ARCore owns, this class owns.** A `Session` may only be touched
 * from one thread, `Session.update()` must be followed by a draw for the frame
 * it produced, and `Frame.acquireCameraImage()` is only valid until the next
 * `update()`. Keeping all of it behind one handler is what makes those rules
 * enforceable; every call from Dart is posted onto that handler and answered
 * from there.
 *
 * The camera image is written by ARCore into an external OES texture, which is
 * then drawn — together with the target mesh — into an EGL window surface backed
 * by the Flutter `SurfaceTexture`. So the pixels the user sees are ARCore's
 * camera feed, and the target's transform is ARCore's own anchor pose, re-read
 * every frame.
 */
internal class ArCoreSessionController(
    private val appContext: Context,
    private val textureEntry: TextureRegistry.SurfaceTextureEntry,
    private val onEvent: (Map<String, Any?>) -> Unit,
) {
    private val thread = HandlerThread("arcore-capture").apply { start() }
    private val handler = Handler(thread.looper)

    /** Off-AR-thread encoder for still captures: one daemon worker. */
    private val stillEncodeExecutor: ExecutorService = Executors.newSingleThreadExecutor { r ->
        Thread(r, "arcore-still-encode").apply { isDaemon = true }
    }

    private val renderer = ArGlRenderer()
    private val surfaceTexture: SurfaceTexture = textureEntry.surfaceTexture()

    @Volatile
    private var running = false

    /** Whether [pause] released the camera and stopped the render loop. */
    @Volatile
    private var paused = false

    @Volatile
    private var session: Session? = null

    /**
     * Process-unique identity of the live ARCore [Session].
     *
     * ARCore's tracking epoch deliberately resets for every new Session, so it
     * cannot distinguish a resumed world from a newly-created epoch zero on
     * the Flutter side. This token is assigned exactly once per native Session,
     * remains stable across [pause]/[resume], and is carried with every pose and
     * anchor result that Dart may use to verify the frozen station reference.
     */
    private var sessionId: String? = null

    private var eglDisplay = EGL14.EGL_NO_DISPLAY
    private var eglContext = EGL14.EGL_NO_CONTEXT
    private var eglSurface = EGL14.EGL_NO_SURFACE
    private var eglConfig: EGLConfig? = null

    private var surfaceWidth = 1
    private var surfaceHeight = 1
    private var pendingResize = true

    private var displayRotation = 0
    private var pendingDisplayGeometry = true

    /**
     * ARCore continues to update and render at [FRAME_PERIOD_MS], but Flutter
     * does not need every camera frame to draw guidance. Sending one Map across
     * JNI for every frame overloaded the Xiaomi's main thread (thousands of
     * "Attempt to remove non-JNI local reference" warnings) and coincided with
     * VIO pauses. 15 Hz still leaves a 67 ms pose gap, well inside the capture
     * gate's 150 ms continuity limit.
     */
    private var lastPoseEventTimestampNs = Long.MIN_VALUE

    /**
     * The continuity epoch is a session-scoped generation number for Flutter.
     *
     * A pose after ARCore pauses, relocalises, or otherwise has a broken frame
     * sequence must not look like an ordinary later pose.  The epoch is reset
     * only for a newly created [Session], never for app pause/resume, so Dart
     * can refuse to continue a calibrated capture until it has re-established
     * continuity against its frozen reference.
     */
    private var trackingContinuityEpoch = 0L
    private var lastObservedTrackingState: TrackingState? = null
    private var lastObservedFrameTimestampNs = Long.MIN_VALUE
    private var continuityBreakLatched = false

    private val anchors = HashMap<String, Anchor>()
    private var anchorCounter = 0

    /** The scene Dart asked for; read by the render loop. */
    private val drawRequests = AtomicReference<List<DrawSpec>>(emptyList())

    /**
     * The anchor the guidance is currently aimed at, or null.
     *
     * The render loop draws every target from `anchor.pose`, re-read each
     * frame. Dart's alignment used to aim at a position refreshed on a timer,
     * which is a *different* number from the one on screen — the arrow then
     * points a few centimetres off the marker while ARCore refines it. Naming
     * the active anchor here lets the pose event that already flows at 30–60 Hz
     * carry that anchor's current pose, so the guidance and the renderer read
     * one position instead of two. The event delivery is rate-limited, but the
     * active anchor is always sampled from the same ARCore frame as the pose.
     */
    private val activeAnchorId = AtomicReference<String?>(null)

    /** Names the anchor the guidance aims at; see [activeAnchorId]. */
    fun setActiveAnchor(id: String?, onDone: (ArOutcome) -> Unit) = post {
        activeAnchorId.set(id)
        onDone(ArOutcome(value = mapOf<String, Any?>()))
    }

    /** A shutter request waiting for the next live ARCore frame. */
    private val pendingStillRequest = AtomicBoolean(false)

    /**
     * A still has been consumed from a frame and is being snapshotted or JPEG
     * encoded. This is deliberately distinct from [pendingStillRequest]: an
     * encoder in flight must block a second request, but must never make the
     * render loop take another image on every subsequent frame.
     */
    private val stillCaptureInFlight = AtomicBoolean(false)
    /**
     * Invalidates an encoder result when the session pauses, stops, or is
     * released after ARCore handed a camera image to the worker.  The worker
     * owns a local callback copy at that point, so clearing pending callbacks
     * alone cannot prevent an obsolete JPEG from completing a capture.
     */
    private val stillLifecycleGeneration = AtomicLong(0L)
    private val pendingStillQuality = AtomicReference<Int>(95)
    private val pendingRaycast = AtomicReference<PendingRaycast?>(null)

    /**
     * Guards against releasing the Flutter texture twice.
     *
     * `stop()` is followed by `dispose()` from the plugin, engine detach can
     * arrive independently, and a failed `start()` asks for both. Releasing a
     * `SurfaceTextureEntry` is not idempotent, so exactly one of those paths
     * may win.
     */
    private val disposed = AtomicBoolean(false)
    private val textureReleased = AtomicBoolean(false)

    /** Where the image's +x axis points in display coordinates; measured once. */
    private var imageAxisXInView = floatArrayOf(1f, 0f)
    private var imageAxisMeasured = false

    private var lastIntrinsicsKey = ""
    private var lastFrame: Frame? = null

    /** `auto` or `fixed`, as granted when the session was configured. */
    private var focusModeName = "fixed"

    /** The camera ARCore picked, read once; reported on the first pose event. */
    private var cameraConfigRead = false
    private var cameraConfigReported = false
    private var stillW = 0
    private var stillH = 0
    private var previewW = 0
    private var previewH = 0
    private var fpsMin = 0
    private var fpsMax = 0

    // ── lifecycle ────────────────────────────────────────────────────────

    /** Checks ARCore availability. Runs off the AR thread: it is a binder call. */
    fun checkAvailability(): Map<String, Any?> {
        val availability = try {
            ArCoreApk.getInstance().checkAvailability(appContext)
        } catch (e: Exception) {
            return mapOf(
                "supported" to false,
                "serviceInstalled" to false,
                "reason" to "ARCore availability could not be checked: ${e.message}",
            )
        }
        val supported = availability.isSupported
        val installed = availability == ArCoreApk.Availability.SUPPORTED_INSTALLED
        return mapOf(
            "supported" to supported,
            "serviceInstalled" to installed,
            "reason" to if (supported) null else "This device does not support ARCore.",
        )
    }

    fun start(
        withPlanes: Boolean,
        rotation: Int,
        width: Int,
        height: Int,
        onDone: (ArOutcome) -> Unit,
    ) {
        post {
            if (running) {
                onDone(ArOutcome(value = capabilitiesMap()))
                return@post
            }
            try {
                displayRotation = rotation
                surfaceWidth = width.coerceAtLeast(1)
                surfaceHeight = height.coerceAtLeast(1)
                pendingResize = true
                pendingDisplayGeometry = true
                // A new session is a new camera: the config must be read again,
                // or the second session would inherit the first session's
                // "already reported" flag and say nothing.
                cameraConfigRead = false
                cameraConfigReported = false
                imageAxisMeasured = false

                setupEgl()
                renderer.onSurfaceCreated()
                renderer.onSurfaceChanged(surfaceWidth, surfaceHeight)

                val created = Session(appContext)
                val config = Config(created)
                // Only what the capture module actually uses. Depth, cloud
                // anchors and HDR light estimation are left at their defaults
                // rather than explicitly disabled: nothing reads them, and
                // naming them here would imply they matter.
                config.updateMode = Config.UpdateMode.BLOCKING
                config.planeFindingMode = if (withPlanes) {
                    Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
                } else {
                    Config.PlaneFindingMode.DISABLED
                }
                // **Autofocus, not the default.** ARCore's default focus mode is
                // FIXED, which parks the lens near its hyperfocal distance: a
                // subject at the distance this app captures from — arm's length,
                // wall 30–70 cm away — is then outside the depth of field and
                // every frame is soft. It is soft in the saved photographs too,
                // because `acquireCameraImage()` reads the same lens, not just
                // the preview. A device that refuses AUTO (some report no AF
                // unit at configure time) falls back to the default rather than
                // losing the session, and which mode actually took effect is
                // reported with the camera config on the first pose event.
                try {
                    config.focusMode = Config.FocusMode.AUTO
                    created.configure(config)
                    focusModeName = created.config.focusMode.name.lowercase()
                } catch (e: Exception) {
                    val fallback = Config(created)
                    fallback.updateMode = config.updateMode
                    fallback.planeFindingMode = config.planeFindingMode
                    created.configure(fallback)
                    focusModeName = "fixed"
                }
                created.setCameraTextureName(renderer.cameraTextureId)
                created.resume()
                resetTrackingContinuityForNewSession()
                session = created
                sessionId = "arcore-session-${NEXT_SESSION_ID.incrementAndGet()}"
                paused = false
                running = true
                startRenderLoop()
                Log.d("ARCORE_SESSION", "Session started id=$sessionId")
                onDone(ArOutcome(value = capabilitiesMap()))
            } catch (e: Exception) {
                onDone(
                    ArOutcome(
                        errorCode = mapSessionError(e),
                        errorMessage = describeSessionError(e),
                    ),
                )
            }
        }
    }

    fun stop(onDone: () -> Unit) {
        post {
            running = false
            paused = false
            val stoppedSessionId = sessionId
            handler.removeCallbacks(renderRunnable)
            stillLifecycleGeneration.incrementAndGet()
            // A still or a raycast queued but not yet consumed will never be
            // consumed now. Answering both is what turns a hang into a
            // retryable failure.
            answerPendingStills("The AR session stopped before the photograph "
                + "was taken.")
            answerPendingRaycast("The AR session stopped before the surface "
                + "lookup completed.")
            try {
                session?.close()
            } catch (_: Exception) {
            }
            session = null
            sessionId = null
            Log.d(
                "ARCORE_SESSION",
                "Session stopped id=$stoppedSessionId; camera and EGL released.",
            )
            anchors.clear()
            renderer.clearMeshes()
            renderer.release()
            releaseEgl()
            onDone()
        }
    }

    /**
     * Releases the camera and stops the render loop, keeping the session alive.
     *
     * This is what the app runs when it goes to the background. Closing and
     * recreating the session would move ARCore's world origin, and every one of
     * the station's capture-target positions is expressed in that origin — a restart
     * would silently aim all of them at the wrong walls. `Session.pause()`
     * releases the camera device without discarding the world, so a resume
     * continues the same station.
     */
    fun pause(onDone: (ArOutcome) -> Unit) = post {
        val s = session
        if (s == null) {
            onDone(
                ArOutcome(
                    errorCode = "no_session",
                    errorMessage = "No AR session is running.",
                ),
            )
            return@post
        }
        if (paused) {
            onDone(ArOutcome(value = mapOf("paused" to true)))
            return@post
        }
        running = false
        handler.removeCallbacks(renderRunnable)
        stillLifecycleGeneration.incrementAndGet()
        answerPendingStills("The app was paused before the photograph was taken.")
        answerPendingRaycast(
            "The app was paused before the surface lookup completed.",
        )
        try {
            s.pause()
            paused = true
            Log.d(
                "ARCORE_SESSION",
                "Session paused id=$sessionId; camera released, world preserved.",
            )
            // A frame is only valid until the next update; after a pause there
            // is no next update until resume, and an old frame must not be
            // handed to anything that asks.
            lastFrame = null
            // The display may have changed size or orientation while away, and
            // the image axis belongs to a camera stream that was stopped. Both
            // are re-established on the first frame after resume.
            imageAxisMeasured = false
            lastIntrinsicsKey = ""
            pendingResize = true
            pendingDisplayGeometry = true
            onDone(ArOutcome(value = mapOf("paused" to true)))
        } catch (e: Exception) {
            onDone(
                ArOutcome(
                    errorCode = "pause_failed",
                    errorMessage = "The AR session could not be paused: $e",
                ),
            )
        }
    }

    /**
     * Resumes a session [pause] stopped, without changing its world origin.
     *
     * A failed resume is reported, not retried here: the Dart side turns it
     * into "recalibrate", which is the only honest recovery when the camera
     * cannot be reopened on this session.
     */
    fun resume(onDone: (ArOutcome) -> Unit) = post {
        val s = session
        if (s == null) {
            onDone(
                ArOutcome(
                    errorCode = "no_session",
                    errorMessage = "No AR session is running.",
                ),
            )
            return@post
        }
        if (running) {
            onDone(ArOutcome(value = mapOf("resumed" to true)))
            return@post
        }
        try {
            s.resume()
            paused = false
            running = true
            // Allow an error that was reported before the pause to be reported
            // again: it may have been the pause itself, and suppressing it
            // would hide a real render failure after the resume.
            lastRenderFailure = null
            pendingResize = true
            pendingDisplayGeometry = true
            imageAxisMeasured = false
            lastIntrinsicsKey = ""
            startRenderLoop()
            Log.d("ARCORE_SESSION", "Session resumed id=$sessionId; render loop restarted.")
            onDone(ArOutcome(value = mapOf("resumed" to true)))
        } catch (e: Exception) {
            onDone(
                ArOutcome(
                    errorCode = "resume_failed",
                    errorMessage = "The AR session could not be resumed: $e",
                ),
            )
        }
    }

    /** Whether [pause] released the camera and stopped the render loop. */
    val isPaused: Boolean get() = paused

    /**
     * Releases everything the controller owns and stops its thread.
     *
     * Called after a normal [stop], by the plugin when the engine detaches, and
     * on the failure path of [start]. Before this did only half the job: a
     * failed start left the AR thread, the EGL surface and the texture entry
     * alive, so a retry opened a second camera on top of the first.
     */
    fun dispose() {
        if (!disposed.compareAndSet(false, true)) return
        handler.post {
            running = false
            paused = false
            val disposedSessionId = sessionId
            handler.removeCallbacks(renderRunnable)
            stillLifecycleGeneration.incrementAndGet()
            answerPendingStills("The AR session was released before the "
                + "photograph was taken.")
            answerPendingRaycast("The AR session was released before the "
                + "surface lookup completed.")
            try {
                session?.close()
            } catch (_: Exception) {
            }
            session = null
            sessionId = null
            anchors.clear()
            Log.d("ARCORE_SESSION", "Session disposed id=$disposedSessionId.")
            try {
                renderer.clearMeshes()
                renderer.release()
                releaseEgl()
            } finally {
                releaseTexture()
                // Shut the encoder pool down *through* the pool: every queued
                // still encode completes first, then the worker exits.
                stillEncodeExecutor.execute { stillEncodeExecutor.shutdown() }
            }
        }
        thread.quitSafely()
    }

    /** The Flutter texture id the Dart `Texture` widget renders. */
    val textureId: Long get() = textureEntry.id()

    /** What the session reports it can do; also returned by a reused start. */
    fun capabilities(): Map<String, Any?> = capabilitiesMap()

    private fun releaseTexture() {
        if (!textureReleased.compareAndSet(false, true)) return
        try {
            textureEntry.release()
        } catch (_: Exception) {
        }
    }

    // ── Dart → AR thread ─────────────────────────────────────────────────

    fun resize(width: Int, height: Int) = post {
        val w = width.coerceAtLeast(1)
        val h = height.coerceAtLeast(1)
        if (w != surfaceWidth || h != surfaceHeight) {
            surfaceWidth = w
            surfaceHeight = h
            pendingResize = true
        }
    }

    fun setDisplayRotation(rotation: Int) = post {
        if (rotation != displayRotation) {
            displayRotation = rotation
            // `transformCoordinates2d(IMAGE_PIXELS → VIEW)` depends on the
            // display geometry.  Keeping the previous measurement after a
            // rotation swaps the display FOV axes incorrectly, so Flutter's
            // guidance arrow no longer lands on the ARCore-rendered target.
            // Measure again from the first frame using the new geometry.
            imageAxisMeasured = false
            pendingDisplayGeometry = true
        }
    }

    fun setPlaneDetection(enabled: Boolean, onDone: (ArOutcome) -> Unit) = post {
        val s = session
        if (s == null) {
            onDone(ArOutcome(errorCode = "no_session", errorMessage = "No AR session is running."))
            return@post
        }
        try {
            val config = s.config
            config.planeFindingMode = if (enabled) {
                Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            } else {
                Config.PlaneFindingMode.DISABLED
            }
            s.configure(config)
            onDone(ArOutcome(value = mapOf("planes" to enabled)))
        } catch (e: Exception) {
            onDone(ArOutcome(errorCode = "configure_failed", errorMessage = "$e"))
        }
    }

    fun setMesh(id: String, positions: FloatArray, colors: FloatArray) = post {
        renderer.setMesh(id, positions, colors)
    }

    /** Replaces the whole scene description atomically. */
    fun setScene(draws: List<DrawSpec>) {
        drawRequests.set(draws)
    }

    /**
     * Raycasts an arbitrary world ray against tracked planes and geometry.
     *
     * Runs inside the render loop rather than against a saved `Frame`, because a
     * `Frame` is only valid until the next `Session.update()`.
     */
    fun raycast(spec: RaycastSpec, onDone: (ArOutcome) -> Unit) {
        post {
            pendingRaycast.set(PendingRaycast(spec, onDone))
        }
    }

    fun createAnchor(
        pose: FloatArray,
        planeKey: String?,
        onDone: (ArOutcome) -> Unit,
    ) = post {
        val s = session
        // A calibration tap is asynchronous: by the time this work reaches the
        // AR thread, the app may have paused or ARCore may have lost tracking.
        // Creating an anchor from the last pre-pause Frame would turn a stale
        // camera pose into a durable station reference.  Require the same live
        // evidence as the still-capture path before ARCore accepts the request.
        if (s == null || !running || paused) {
            onDone(
                ArOutcome(
                    errorCode = "no_live_session",
                    errorMessage = "No live AR session is available to create an anchor.",
                ),
            )
            return@post
        }
        val frame = lastFrame
        if (frame == null) {
            onDone(
                ArOutcome(
                    errorCode = "no_live_frame",
                    errorMessage = "No live AR frame is available to create an anchor.",
                ),
            )
            return@post
        }
        if (frame.camera.trackingState != TrackingState.TRACKING) {
            onDone(
                ArOutcome(
                    errorCode = "tracking_paused",
                    errorMessage = "AR tracking is not valid for anchor creation.",
                ),
            )
            return@post
        }
        try {
            val p = Pose(floatArrayOf(pose[0], pose[1], pose[2]), floatArrayOf(pose[3], pose[4], pose[5], pose[6]))
            val anchor = if (planeKey != null) {
                val plane = findPlane(planeKey)
                plane?.createAnchor(p) ?: s.createAnchor(p)
            } else {
                s.createAnchor(p)
            }
            val id = "anchor-${++anchorCounter}"
            anchors[id] = anchor
            onDone(ArOutcome(value = anchorMap(id, anchor)))
        } catch (e: Exception) {
            onDone(ArOutcome(errorCode = "anchor_failed", errorMessage = "$e"))
        }
    }

    /**
     * Probes every direction against the current frame, **without creating
     * anchors**.
     *
     * This is what makes "which of the 26 directions actually works" answerable
     * the moment the station is calibrated: 26 raycasts in one frame is
     * microseconds, whereas 26 anchors would be a real resource cost and would
     * pin markers the user has not asked for.
     *
     * Returns one entry per direction, in the order supplied.
     */
    fun probeDirections(
        origin: FloatArray,
        directions: FloatArray,
        maxDistanceM: Float,
        onDone: (ArOutcome) -> Unit,
    ) = post {
        val frame = lastFrame
        if (frame == null) {
            onDone(
                ArOutcome(
                    errorCode = "no_frame",
                    errorMessage = "No AR frame yet — tracking has not started.",
                ),
            )
            return@post
        }
        val results = ArrayList<Map<String, Any?>>(directions.size / 3)
        for (i in directions.indices step 3) {
            val dir = floatArrayOf(directions[i], directions[i + 1], directions[i + 2])
            var best: HitResult? = null
            try {
                for (h in frame.hitTest(origin, 0, dir, 0)) {
                    if (h.trackable !is Plane) continue
                    if (h.distance > maxDistanceM) continue
                    if (best == null || h.distance < best.distance) best = h
                }
            } catch (_: Exception) {
                // A raycast that throws is a miss for that direction only.
            }
            val hit = best
            results.add(
                if (hit == null) {
                    mapOf("hit" to false)
                } else {
                    mapOf(
                        "hit" to true,
                        "distanceM" to hit.distance,
                        "planeType" to (hit.trackable as Plane).type.name,
                    )
                },
            )
        }
        // Wrapped in a map because `ArOutcome.value` is a map: one shape for
        // every host answer keeps the Dart side from branching on the method.
        onDone(ArOutcome(value = mapOf("results" to results)))
    }

    fun refreshAnchor(id: String, onDone: (ArOutcome) -> Unit) = post {
        val anchor = anchors[id]
        if (anchor == null) {
            onDone(ArOutcome(errorCode = "unknown_anchor", errorMessage = "Anchor $id is gone."))
        } else {
            onDone(ArOutcome(value = anchorMap(id, anchor)))
        }
    }

    fun detachAnchor(id: String) = post {
        try {
            anchors.remove(id)?.detach()
        } catch (e: Exception) {
            // A hang-up or an anchor ARCore already released on its own
            // ("The anchor is already detached") are both terminal states with
            // nothing left to undo, so they are logged and not rethrown.
            Log.d(
                "ARCORE_ANCHOR",
                "Detach of $id failed or the anchor was already gone: $e",
            )
        }
    }

    /**
     * Re-arms surface detection and drops every anchor.
     *
     * This is what "Scan again" runs. It is deliberately not a session restart:
     * a restart would invalidate the station's world pose and with it the 26
     * target directions. Instead it re-applies the plane-finding configuration
     * (which some devices drop after a tracking pause), forces the display
     * geometry to be re-sent, and detaches the anchors — a stale anchor would
     * otherwise keep the marker pinned to a surface the user has walked away
     * from. Captured photographs are untouched.
     */
    fun rescan(onDone: (ArOutcome) -> Unit) = post {
        val s = session
        if (s == null) {
            onDone(
                ArOutcome(
                    errorCode = "no_session",
                    errorMessage = "The AR session is not running.",
                ),
            )
            return@post
        }
        try {
            for (anchor in anchors.values) {
                runCatching { anchor.detach() }
            }
            anchors.clear()
            val config = s.config
            config.planeFindingMode = Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            s.configure(config)
            pendingDisplayGeometry = true
            lastRenderFailure = null
            onDone(ArOutcome(value = mapOf("planes" to true)))
        } catch (e: Exception) {
            onDone(ArOutcome(errorCode = "rescan_failed", errorMessage = "$e"))
        }
    }

    /**
     * Requests a still from the AR session's camera image.
     *
     * The request is consumed by the render loop, which is the only place a live
     * `Frame` exists. The JPEG therefore contains ARCore's camera image and
     * nothing else — no Flutter overlay, no GLB marker, no UI.
     */
    fun takeStill(quality: Int, onDone: (ArOutcome) -> Unit) {
        post {
            if (!running || session == null) {
                // No loop, no frame, no photograph. Answering immediately beats
                // leaving the caller waiting on a request nothing will consume.
                onDone(
                    ArOutcome(
                        errorCode = "no_session",
                        errorMessage = "The AR session is not running.",
                    ),
                )
                return@post
            }
            // One frame can only produce one image. The old code accepted a
            // second request while the first was between "queued" and
            // "consumed"; `captureStill` then drained both callbacks and handed
            // the same JPEG to two callers. Refusing is the honest answer —
            // the Dart side already treats an error as a retryable failure.
            if (pendingStillRequest.get() ||
                stillCaptureInFlight.get() ||
                stillCallbacks.isNotEmpty()
            ) {
                onDone(
                    ArOutcome(
                        errorCode = "still_busy",
                        errorMessage = "A photograph is already being captured.",
                    ),
                )
                return@post
            }
            pendingStillQuality.set(quality.coerceIn(1, 100))
            pendingStillRequest.set(true)
            stillCallbacks.add(onDone)
        }
    }

    private val stillCallbacks = ArrayList<(ArOutcome) -> Unit>()

    /**
     * Answers every queued still request with a stated failure.
     *
     * A request that is queued while the loop runs and then abandoned — stop,
     * release, a paused app — used to leave its caller waiting for a frame
     * nothing would produce. That is a hang the UI cannot distinguish from a
     * slow encoder, so it is answered explicitly instead.
     */
    private fun answerPendingStills(
        message: String,
        errorCode: String = "no_session",
    ) {
        val callbacks = ArrayList(stillCallbacks)
        stillCallbacks.clear()
        pendingStillRequest.set(false)
        val outcome = ArOutcome(errorCode = errorCode, errorMessage = message)
        for (callback in callbacks) {
            try {
                callback(outcome)
            } catch (_: Exception) {
            }
        }
    }

    /**
     * Answers a raycast that was queued but will never meet a frame.
     *
     * The render loop is the only consumer, so a pause or a stop between the
     * request and the next frame leaves the Dart caller waiting forever unless
     * it is answered here.
     */
    private fun answerPendingRaycast(message: String) {
        val pending = pendingRaycast.getAndSet(null) ?: return
        try {
            pending.onDone(
                ArOutcome(errorCode = "no_session", errorMessage = message),
            )
        } catch (_: Exception) {
        }
    }

    // ── render loop ──────────────────────────────────────────────────────

    private val renderRunnable = Runnable { renderFrame() }

    private fun startRenderLoop() {
        handler.removeCallbacks(renderRunnable)
        handler.post(renderRunnable)
    }

    private fun renderFrame() {
        if (!running) return
        val s = session
        if (s == null) {
            scheduleNext()
            return
        }
        try {
            applyResize()
            if (pendingDisplayGeometry) {
                s.setDisplayGeometry(displayRotation, surfaceWidth, surfaceHeight)
                pendingDisplayGeometry = false
            }

            val frame = s.update()
            lastFrame = frame
            val camera = frame.getCamera()

            // Each of these is independently recoverable. A failure in one must
            // not skip `eglSwapBuffers`, because a skipped swap is a black
            // camera — which is exactly how the original buffer bug presented.
            try {
                measureImageAxis(frame)
            } catch (e: Exception) {
                reportRenderFailure(e)
            }
            // Read the camera ARCore picked before the pose that will carry
            // it: the still stream's size is what the saved photographs are
            // measured against, and the preview stream's size is what the
            // preview gets scaled from.
            if (!cameraConfigRead) readCameraConfig(s)
            try {
                emitPose(frame, camera)
            } catch (e: Exception) {
                reportRenderFailure(e)
            }

            // Consume a pending still request *here*, before anything that
            // can throw. The shutter used to be consumed after the render
            // calls, so a persistent failure in `transformCoordinates2d` or
            // `renderer.draw` left the pending request set and the Dart future
            // hanging forever — the device recording's "Saving…" stuck at
            // 0 of 26.
            if (pendingStillRequest.compareAndSet(true, false)) {
                if (!stillCaptureInFlight.compareAndSet(false, true)) {
                    // The request should have been rejected before it reached
                    // the loop. Do not turn an already-busy encoder into a
                    // second image acquisition; answer the queued caller.
                    answerPendingStills(
                        "A photograph is already being captured.",
                        errorCode = "still_busy",
                    )
                } else if (camera.trackingState != TrackingState.TRACKING) {
                    // The Dart gate can only authorise the preceding pose
                    // event. A tracking pause between that event and this
                    // frame must reject the shutter here, before a stale
                    // camera image can reach storage.
                    stillCaptureInFlight.set(false)
                    answerPendingStills(
                        "AR tracking was not valid when the photograph was taken.",
                        errorCode = "tracking_paused",
                    )
                } else {
                    try {
                        captureStill(frame)
                    } catch (e: Exception) {
                        stillCaptureInFlight.set(false)
                        reportRenderFailure(e)
                    }
                }
            }

            val uvs = FloatArray(8)
            frame.transformCoordinates2d(
                Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES,
                NDC_QUAD,
                Coordinates2d.TEXTURE_NORMALIZED,
                uvs,
            )
            renderer.setCameraUvs(uvs)

            val view = FloatArray(16)
            val projection = FloatArray(16)
            camera.getViewMatrix(view, 0)
            // near 0.05 / far 50, not 0.02 / 100: with a 16-bit depth buffer the
            // precision at 2 m is proportional to near⁻¹, and the marker sits
            // 1–8 m away with its bezel 14 mm behind the plate. A 0.02 near
            // plane put that 14 mm within the depth quantisation step.
            camera.getProjectionMatrix(projection, 0, 0.05f, 50f)
            renderer.setViewProjection(view, projection)

            // Flutter clears its scene as soon as it receives a tracking-loss
            // event, but that EventChannel hop is asynchronous. Never draw an
            // anchor from this frame while ARCore itself says the camera is not
            // TRACKING: its pose is only a last/refining estimate and can make
            // a supposedly frozen marker visibly slide before Dart reacts.
            renderer.draw(
                buildDraws(camera.trackingState == TrackingState.TRACKING),
            )

            val pending = pendingRaycast.getAndSet(null)
            if (pending != null) {
                runRaycast(frame, pending)
            }

            EGL14.eglSwapBuffers(eglDisplay, eglSurface)
            // A frame reached the texture. Anything reported earlier was
            // transient, so allow it to be reported again if it comes back.
            lastRenderFailure = null
        } catch (e: Exception) {
            reportRenderFailure(e)
        }
        scheduleNext()
    }

    /** The signature of the last render failure, so it is reported once. */
    private var lastRenderFailure: String? = null

    /**
     * Reports a render-loop exception to Dart **with its origin**.
     *
     * The previous build swallowed these into a generic "error" and kept going,
     * which is how `Must use a native order direct buffer` reached the screen as
     * a truncated sentence with no class name and no frame. A render failure is
     * the one thing that produces a black camera, so it has to arrive intact.
     */
    private fun reportRenderFailure(e: Exception) {
        val signature = "${e.javaClass.name}: ${e.message}"
        if (signature == lastRenderFailure) return
        lastRenderFailure = signature
        emit(
            mapOf(
                "type" to "error",
                "message" to signature,
                "frame" to e.stackTrace.firstOrNull()?.toString(),
                "fatal" to true,
            ),
        )
    }

    private fun scheduleNext() {
        if (running) handler.postDelayed(renderRunnable, FRAME_PERIOD_MS)
    }

    private fun applyResize() {
        if (!pendingResize) return
        pendingResize = false
        surfaceTexture.setDefaultBufferSize(surfaceWidth, surfaceHeight)
        val display = eglDisplay
        if (display != EGL14.EGL_NO_DISPLAY && eglConfig != null) {
            if (eglSurface != EGL14.EGL_NO_SURFACE) {
                EGL14.eglDestroySurface(display, eglSurface)
            }
            val surfaceAttribs = intArrayOf(EGL14.EGL_NONE)
            eglSurface = EGL14.eglCreateWindowSurface(
                display,
                eglConfig,
                Surface(surfaceTexture),
                surfaceAttribs,
                0,
            )
            if (eglSurface == EGL14.EGL_NO_SURFACE) {
                throw RuntimeException("eglCreateWindowSurface failed: ${EGL14.eglGetError()}")
            }
            if (!EGL14.eglMakeCurrent(display, eglSurface, eglSurface, eglContext)) {
                throw RuntimeException("eglMakeCurrent failed: ${EGL14.eglGetError()}")
            }
            renderer.onSurfaceChanged(surfaceWidth, surfaceHeight)
        }
    }

    private fun buildDraws(cameraTracking: Boolean): List<ArGlRenderer.DrawRequest> {
        if (!cameraTracking) return emptyList()
        val specs = drawRequests.get()
        val out = ArrayList<ArGlRenderer.DrawRequest>(specs.size)
        for (spec in specs) {
            val anchor = anchors[spec.anchorId] ?: continue
            // A paused anchor exposes only its last/refining pose. Rendering it
            // as a ghost still makes a capture marker appear to walk across
            // the room while Flutter is waiting for recovery verification.
            // Capture targets are therefore never drawn from an untracked
            // anchor. `requireTracking` remains in the bridge payload for
            // compatibility, but safety takes precedence for this renderer.
            if (anchor.trackingState != TrackingState.TRACKING) continue
            val matrix = FloatArray(16)
            anchor.pose.toMatrix(matrix, 0)
            out.add(
                ArGlRenderer.DrawRequest(
                    meshId = spec.meshId,
                    anchorMatrix = matrix,
                    localRotation = spec.localRotation,
                    scale = spec.scale,
                    offsetX = spec.offsetX,
                    offsetY = spec.offsetY,
                    offsetZ = spec.offsetZ,
                    tintR = spec.tintR,
                    tintG = spec.tintG,
                    tintB = spec.tintB,
                    tintA = spec.tintA,
                    useTint = spec.useTint,
                    opacity = spec.opacity,
                    modelMatrix = spec.modelMatrix,
                    useModelTranslation = spec.useModelTranslation,
                ),
            )
        }
        return out
    }

    /**
     * Measures which way the CPU image's +x axis lies in display coordinates.
     *
     * This is the input [YuvJpegEncoder] needs to write an upright photograph,
     * and it is read from ARCore's own coordinate transform rather than from a
     * sensor-orientation constant, so it is correct for whichever way the
     * manufacturer mounted the sensor.
     */
    private fun measureImageAxis(frame: Frame) {
        if (imageAxisMeasured) return
        val out = FloatArray(4)
        try {
            frame.transformCoordinates2d(
                Coordinates2d.IMAGE_PIXELS,
                floatArrayOf(0f, 0f, 1f, 0f),
                Coordinates2d.VIEW,
                out,
            )
        } catch (_: Exception) {
            return
        }
        val dx = out[2] - out[0]
        val dy = out[3] - out[1]
        val length = kotlin.math.sqrt(dx * dx + dy * dy)
        if (length < 1e-6f) return
        imageAxisXInView = floatArrayOf(dx / length, dy / length)
        imageAxisMeasured = true
        emit(
            mapOf(
                "type" to "imageOrientation",
                "axisX" to imageAxisXInView[0],
                "axisY" to imageAxisXInView[1],
            ),
        )
    }

    /**
     * Reads the camera ARCore chose, once per session.
     *
     * The app does not select a camera config — it takes ARCore's automatic
     * choice — but that choice decides two things the user cares about and
     * could otherwise only guess: the resolution of the still stream every
     * saved photograph is written from (`acquireCameraImage()` hands over the
     * CPU image at this size), and the resolution of the preview stream the
     * display is scaled from. Reporting it turns "the photos look blurry"
     * into "the photos are 640×480" or "the lens is at fixed focus", which
     * are different problems with different fixes.
     *
     * A session that has not produced a frame yet reports nothing: a zero here
     * would be indistinguishable from "no camera".
     */
    private fun readCameraConfig(session: Session) {
        cameraConfigRead = true
        try {
            val camera = session.cameraConfig
            stillW = camera.imageSize.width
            stillH = camera.imageSize.height
            previewW = camera.textureSize.width
            previewH = camera.textureSize.height
            fpsMin = camera.fpsRange.lower
            fpsMax = camera.fpsRange.upper
        } catch (_: Exception) {
            // Unreadable is a missing field, never a zero: the Dart side treats
            // a null as "not reported" rather than believing a 0×0 camera.
        }
    }

    /**
     * Emits one pose event.
     *
     * **Order matters.** Everything that can fail is isolated, because a pose
     * event that never arrives is indistinguishable in the UI from "ARCore found
     * no planes and never tracked": `planesTracked` stays 0, the tracking state
     * never changes, and the target can never be anchored. That is precisely how
     * the "0 planes" report looked, and it was caused by this method throwing
     * before `emit()` — `camera.imageIntrinsics` was read unconditionally.
     *
     * So: the pose and the plane counts are built first and are always emitted;
     * the intrinsics are attached best-effort, and their absence costs nothing
     * but a field.
     */
    private fun emitPose(frame: Frame, camera: Camera) {
        val tracking = camera.trackingState
        val previousTracking = lastObservedTrackingState
        val trackingTransition = tracking != previousTracking
        val previousFrameTimestampNs = lastObservedFrameTimestampNs
        val timestampRegressed =
            previousFrameTimestampNs != Long.MIN_VALUE &&
                frame.timestamp < previousFrameTimestampNs
        val longContinuityGap =
            previousFrameTimestampNs != Long.MIN_VALUE &&
                !timestampRegressed &&
                frame.timestamp - previousFrameTimestampNs > CONTINUITY_GAP_NS
        val leftTracking =
            previousTracking == TrackingState.TRACKING &&
                tracking != TrackingState.TRACKING
        val continuityBroken =
            leftTracking || timestampRegressed || longContinuityGap
        val continuityReason = when {
            leftTracking -> "tracking_left"
            timestampRegressed -> "timestamp_regressed"
            longContinuityGap -> "frame_gap"
            else -> null
        }

        // Several bad frames during one PAUSED period are one broken sequence,
        // not a new recovery generation each.  The latch clears after the next
        // ordinary TRACKING frame, ready to identify a later, separate break.
        if (continuityBroken) {
            if (!continuityBreakLatched) {
                trackingContinuityEpoch++
                Log.w(
                    "ARCORE_RECOVERY",
                    "Pose continuity break session=$sessionId epoch=$trackingContinuityEpoch " +
                        "reason=$continuityReason tracking=${tracking.name} " +
                        "frameNs=${frame.timestamp}",
                )
            }
            continuityBreakLatched = true
        } else if (tracking == TrackingState.TRACKING) {
            continuityBreakLatched = false
        }
        lastObservedTrackingState = tracking
        lastObservedFrameTimestampNs = frame.timestamp

        val elapsedNs = frame.timestamp - lastPoseEventTimestampNs
        if (!trackingTransition &&
            !continuityBroken &&
            lastPoseEventTimestampNs != Long.MIN_VALUE &&
            elapsedNs in 0 until POSE_EVENT_INTERVAL_NS
        ) {
            return
        }
        lastPoseEventTimestampNs = frame.timestamp

        val event = HashMap<String, Any?>()
        event["type"] = "pose"

        // Flutter guidance is drawn in display coordinates. The renderer below
        // uses `getViewMatrix`, which ARCore derives from this same
        // display-oriented pose. Forwarding the physical image-readout pose
        // instead rotates left/right and up/down guidance by the camera
        // sensor's quarter-turn on portrait devices.
        val pose = camera.displayOrientedPose
        val t = FloatArray(3)
        val q = FloatArray(4)
        pose.getTranslation(t, 0)
        pose.getRotationQuaternion(q, 0)
        event["px"] = t[0]
        event["py"] = t[1]
        event["pz"] = t[2]
        event["qx"] = q[0]
        event["qy"] = q[1]
        event["qz"] = q[2]
        event["qw"] = q[3]
        event["tracking"] = tracking.name
        event["failureReason"] = camera.trackingFailureReason.name
        event["timestampNs"] = frame.timestamp
        event["sessionId"] = sessionId
        // Keep the bridge key concise and stable: Dart treats this as a
        // generation number, not an ARCore timestamp.
        event["trackingEpoch"] = trackingContinuityEpoch
        event["trackingTransition"] = trackingTransition
        event["trackingContinuityBroken"] = continuityBroken
        if (continuityReason != null) {
            event["trackingContinuityReason"] = continuityReason
        }
        event["anchorCount"] = anchors.size

        // Flutter's guidance overlay must project with the same calibrated
        // camera matrix as the native renderer.  A FOV reconstructed from the
        // sensor image alone misses ARCore's display geometry (rotation,
        // crop, and principal point), which is why the arrow could diverge from
        // a rendered target.  This is the display-geometry-aware projection
        // matrix for the exact frame that supplied the pose above.
        try {
            val displayProjection = FloatArray(16)
            camera.getProjectionMatrix(displayProjection, 0, 0.05f, 50f)
            event["displayProjectionMatrix"] = displayProjection.toList()
        } catch (_: Exception) {
            // The Dart side falls back to the validated intrinsics FOV when a
            // device cannot provide a projection matrix for this frame.
        }

        // The active target's current pose — the same `anchor.pose` the render
        // loop is about to draw with. Carried on the pose event rather than a
        // separate channel call, so it costs nothing per frame.
        val activeId = activeAnchorId.get()
        val active = if (activeId == null) null else anchors[activeId]
        if (active != null) {
            val at = FloatArray(3)
            val aq = FloatArray(4)
            active.pose.getTranslation(at, 0)
            active.pose.getRotationQuaternion(aq, 0)
            event["activeAnchorId"] = activeId
            event["apx"] = at[0]
            event["apy"] = at[1]
            event["apz"] = at[2]
            event["aqx"] = aq[0]
            event["aqy"] = aq[1]
            event["aqz"] = aq[2]
            event["aqw"] = aq[3]
            event["activeAnchorTracking"] = active.trackingState.name
        }

        // Plane counts, broken down by type. "0 planes" on its own cannot tell a
        // user whether the floor was found, the walls were not, or the ceiling
        // is out of reach — and those need different instructions.
        var total = 0
        var tracked = 0
        var walls = 0
        var floors = 0
        var ceilings = 0
        try {
            val planes = session?.getAllTrackables(Plane::class.java) ?: emptyList()
            total = planes.size
            for (plane in planes) {
                if (plane.trackingState != TrackingState.TRACKING) continue
                if (plane.subsumedBy != null) continue
                tracked++
                when (plane.type) {
                    Plane.Type.VERTICAL -> walls++
                    Plane.Type.HORIZONTAL_UPWARD_FACING -> floors++
                    Plane.Type.HORIZONTAL_DOWNWARD_FACING -> ceilings++
                    else -> Unit
                }
            }
        } catch (_: Exception) {
            // A plane query failing must not cost the pose.
        }
        event["planeCount"] = total
        event["planesTracked"] = tracked
        event["walls"] = walls
        event["floors"] = floors
        event["ceilings"] = ceilings

        // The camera config rides on exactly one event, the first one after it
        // was read. `stillW > 0` is the guard: a session whose config could not
        // be read reports nothing rather than a believable-looking 0×0.
        if (cameraConfigRead && !cameraConfigReported && stillW > 0 && stillH > 0) {
            cameraConfigReported = true
            event["stillWidth"] = stillW
            event["stillHeight"] = stillH
            event["previewWidth"] = previewW
            event["previewHeight"] = previewH
            event["fpsMin"] = fpsMin
            event["fpsMax"] = fpsMax
            event["focusMode"] = focusModeName
        }

        try {
            val intrinsics = camera.imageIntrinsics
            val dims = intrinsics.imageDimensions
            val focal = intrinsics.focalLength
            val principal = intrinsics.principalPoint
            lastImageWidth = dims[0]
            lastImageHeight = dims[1]
            val key = "${dims[0]}x${dims[1]}/${focal[0]}/${focal[1]}/${displayRotation}"
            if (key != lastIntrinsicsKey) {
                lastIntrinsicsKey = key
                event["imageWidth"] = dims[0]
                event["imageHeight"] = dims[1]
                event["fx"] = focal[0]
                event["fy"] = focal[1]
                event["cx"] = principal[0]
                event["cy"] = principal[1]
                event["displayRotation"] = displayRotation
            }
        } catch (_: Exception) {
            // No intrinsics this frame. The pose is still valid and the fallback
            // field of view is reported as provisional by the UI.
        }

        emit(event)
    }

    private fun captureStill(frame: Frame) {
        val callbacks = ArrayList(stillCallbacks)
        stillCallbacks.clear()
        // Snapshot this before the CPU work begins. A pause/stop/dispose may
        // happen while JPEG encoding is in progress on the worker thread.
        val captureGeneration = stillLifecycleGeneration.get()
        try {
            // Everything the encoder needs has to be copied on the AR thread:
            // the `Frame`, its camera image, and even its pose are only valid
            // inside this render call. The encode runs off-thread and must not
            // reach back into any of them.
            val quality = pendingStillQuality.get()
            val timestampNs = frame.timestamp
            val camera = frame.camera
            // The JPEG is rotated into display coordinates below, so record the
            // matching display-oriented pose.  EventChannel guidance uses this
            // same ARCore pose; using camera.pose here made stored image
            // orientation differ by the portrait display rotation.
            val pose = camera.displayOrientedPose
            val ptX = pose.tx()
            val ptY = pose.ty()
            val ptZ = pose.tz()
            val pqX = pose.qx()
            val pqY = pose.qy()
            val pqZ = pose.qz()
            val pqW = pose.qw()
            val tracking = camera.trackingState.toString()
            val width = lastImageWidth ?: 0
            val height = lastImageHeight ?: 0
            val rotationDeg = rotationAppliedDeg()
            // A resumed session re-measures the axis; the encode must use the
            // axis these pixels were actually taken under, so it gets a copy,
            // not a reference the next frame could overwrite.
            val axisXInView = imageAxisXInView.clone()
            var image: android.media.Image? = null
            val snapshot = try {
                image = frame.acquireCameraImage()
                if (image == null) null else YuvJpegEncoder.snapshot(image)
            } catch (e: Exception) {
                null
            } finally {
                try {
                    image?.close()
                } catch (e: Exception) {
                    Log.d("ARCORE_CAPTURE", "Camera image close failed: $e")
                }
            }
            if (snapshot == null) {
                // A failed frame must free the shutter for a retry, not leave
                // the in-flight flag parked on an image that never existed.
                stillCaptureInFlight.set(false)
                Log.d(
                    "ARCORE_CAPTURE",
                    "No usable camera image this frame; answering still_unavailable.",
                )
                val outcome = ArOutcome(
                    errorCode = "still_unavailable",
                    errorMessage = "ARCore did not hand over a camera image for this frame.",
                )
                for (callback in callbacks) {
                    try {
                        callback(outcome)
                    } catch (e: Exception) {
                        Log.d("ARCORE_CAPTURE", "Still callback failed: $e")
                    }
                }
                return
            }
            stillEncodeExecutor.execute {
                try {
                    val encoded = if (captureGeneration == stillLifecycleGeneration.get()) {
                        try {
                            YuvJpegEncoder.encode(snapshot, axisXInView, quality)
                        } catch (e: Exception) {
                            Log.d("ARCORE_CAPTURE", "JPEG encode failed: $e")
                            null
                        }
                    } else {
                        null
                    }
                    val outcome = if (captureGeneration != stillLifecycleGeneration.get()) {
                        Log.d(
                            "ARCORE_CAPTURE",
                            "Discarding still result because the AR session changed during encoding.",
                        )
                        ArOutcome(
                            errorCode = "no_session",
                            errorMessage = "The AR session changed before the photograph finished encoding.",
                        )
                    } else if (encoded == null) {
                        ArOutcome(
                            errorCode = "still_unavailable",
                            errorMessage =
                                "ARCore delivered the camera image but the photograph " +
                                    "could not be encoded.",
                        )
                    } else {
                        ArOutcome(
                            value = mapOf(
                                "jpeg" to encoded.bytes,
                                "byteCount" to encoded.bytes.size,
                                // These dimensions describe the JPEG after the
                                // image-to-display rotation, not the landscape
                                // CPU input. Stitching must consume the file's
                                // real pixel geometry.
                                "imageWidth" to encoded.width,
                                "imageHeight" to encoded.height,
                                "sourceImageWidth" to snapshot.width,
                                "sourceImageHeight" to snapshot.height,
                                "rotationAppliedDeg" to rotationDeg,
                                // The pose that actually produced these pixels. The Dart
                                // side compares it with the pose that opened the shutter
                                // before the node is marked captured.
                                "timestampNs" to timestampNs,
                                "tracking" to tracking,
                                "px" to ptX,
                                "py" to ptY,
                                "pz" to ptZ,
                                "qx" to pqX,
                                "qy" to pqY,
                                "qz" to pqZ,
                                "qw" to pqW,
                            ),
                        )
                    }
                    for (callback in callbacks) {
                        try {
                            callback(outcome)
                        } catch (e: Exception) {
                            Log.d("ARCORE_CAPTURE", "Still callback failed: $e")
                        }
                    }
                } finally {
                    stillCaptureInFlight.set(false)
                }
            }
        } catch (e: Exception) {
            // Reaching here means the capture aborted before a task was
            // enqueued either from a bad read while snapshotting, or from
            // `execute` rejecting (dispose racing a capture). The worker's
            // `finally` never runs for an enqueue that never happened, so the
            // busy flag is cleared here and the drained callbacks get an
            // immediate failure instead of hanging.
            Log.d("ARCORE_CAPTURE", "Still capture aborted before encode: $e")
            stillCaptureInFlight.set(false)
            val outcome = ArOutcome(
                errorCode = "capture_failed",
                errorMessage = "The still capture aborted before the photograph could be encoded.",
            )
            for (callback in callbacks) {
                try {
                    callback(outcome)
                } catch (x: Exception) {
                    Log.d("ARCORE_CAPTURE", "Still callback failed: $x")
                }
            }
        }
    }

    private var lastImageWidth: Int? = null
    private var lastImageHeight: Int? = null

    private fun rotationAppliedDeg(): Int {
        val angle = Math.atan2(imageAxisXInView[1].toDouble(), imageAxisXInView[0].toDouble())
        val quarters = ((Math.round(angle / (Math.PI / 2)).toInt()) % 4 + 4) % 4
        return quarters * 90
    }

    private fun runRaycast(frame: Frame, pending: PendingRaycast) {
        val spec = pending.spec
        val hits = try {
            frame.hitTest(spec.origin, 0, spec.direction, 0)
        } catch (e: Exception) {
            pending.onDone(ArOutcome(errorCode = "raycast_failed", errorMessage = "$e"))
            return
        }
        // A miss is reported with a reason rather than as a bare null. "No
        // surface" is not one condition: a room with no detected planes needs a
        // different instruction from a room whose planes the ray simply missed,
        // and the user cannot tell them apart from the word "unanchored".
        if (hits.isEmpty()) {
            pending.onDone(
                ArOutcome(
                    value = mapOf(
                        "hit" to false,
                        "candidates" to 0,
                        "reason" to "no_candidates",
                    ),
                ),
            )
            return
        }
        var best: HitResult? = null
        for (hit in hits) {
            val trackable = hit.trackable
            val acceptable = when (spec.surface) {
                "plane" -> trackable is Plane
                else -> true
            }
            if (!acceptable) continue
            val distance = hit.distance
            if (distance > spec.maxDistanceM) continue
            if (best == null || distance < best.distance) best = hit
        }
        val chosen = best
        if (chosen == null) {
            pending.onDone(
                ArOutcome(
                    value = mapOf(
                        "hit" to false,
                        "candidates" to hits.size,
                        "reason" to "no_plane_within_range",
                    ),
                ),
            )
            return
        }
        pending.onDone(ArOutcome(value = hitMap(chosen)))
    }

    private fun hitMap(hit: HitResult): Map<String, Any?> {
        val pose = hit.hitPose
        val t = FloatArray(3)
        val q = FloatArray(4)
        pose.getTranslation(t, 0)
        pose.getRotationQuaternion(q, 0)
        val trackable = hit.trackable
        val type = when (trackable) {
            is Plane -> "PLANE_" + trackable.type.name
            else -> trackable.javaClass.simpleName.uppercase()
        }
        val map = HashMap<String, Any?>()
        map["px"] = t[0]
        map["py"] = t[1]
        map["pz"] = t[2]
        map["planeQx"] = q[0]
        map["planeQy"] = q[1]
        map["planeQz"] = q[2]
        map["planeQw"] = q[3]
        map["hit"] = true
        map["distanceM"] = hit.distance
        map["planeType"] = type
        map["trackableKey"] = trackableKey(trackable)
        if (trackable is Plane) {
            map["extentX"] = trackable.extentX
            map["extentZ"] = trackable.extentZ
        }
        return map
    }

    private fun trackableKey(trackable: com.google.ar.core.Trackable): String? {
        if (trackable !is Plane) return null
        return "plane-${System.identityHashCode(trackable)}"
    }

    private fun findPlane(key: String): Plane? {
        val planes = session?.getAllTrackables(Plane::class.java) ?: return null
        for (plane in planes) {
            if (trackableKey(plane) == key) return plane
        }
        return null
    }

    private fun anchorMap(id: String, anchor: Anchor): Map<String, Any?> {
        val t = FloatArray(3)
        anchor.pose.getTranslation(t, 0)
        return mapOf(
            "id" to id,
            "sessionId" to sessionId,
            "px" to t[0],
            "py" to t[1],
            "pz" to t[2],
            "tracking" to anchor.trackingState.name,
        )
    }

    /**
     * What the session reports it can do.
     *
     * Intrinsics are deliberately absent here: they only exist once a frame has
     * arrived, and rather than inventing a value they ride along on the first
     * pose event. Everything else is a property of the way this host configures
     * the session, so it can be stated without measuring anything.
     */
    private fun capabilitiesMap(): Map<String, Any?> = mapOf(
        "textureId" to textureId,
        "sessionId" to sessionId,
        "planes" to true,
        "anchors" to true,
        "cameraImage" to true,
        // Stated as false rather than omitted: the capture module shows this, and
        // "the depth API is off" is a fact about this host, not an unknown.
        "depth" to false,
    )

    // ── EGL ──────────────────────────────────────────────────────────────

    private fun setupEgl() {
        val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
        if (display == EGL14.EGL_NO_DISPLAY) throw RuntimeException("eglGetDisplay failed")
        val version = IntArray(2)
        if (!EGL14.eglInitialize(display, version, 0, version, 1)) {
            throw RuntimeException("eglInitialize failed: ${EGL14.eglGetError()}")
        }
        val attribs = intArrayOf(
            EGL14.EGL_RED_SIZE, 8,
            EGL14.EGL_GREEN_SIZE, 8,
            EGL14.EGL_BLUE_SIZE, 8,
            EGL14.EGL_ALPHA_SIZE, 8,
            EGL14.EGL_DEPTH_SIZE, 16,
            EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
            EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
            EGL14.EGL_NONE,
        )
        val configs = arrayOfNulls<EGLConfig>(1)
        val numConfigs = IntArray(1)
        if (!EGL14.eglChooseConfig(display, attribs, 0, configs, 0, 1, numConfigs, 0) ||
            numConfigs[0] == 0
        ) {
            throw RuntimeException("eglChooseConfig failed: ${EGL14.eglGetError()}")
        }
        val config = configs[0]!!
        val contextAttribs = intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE)
        val context = EGL14.eglCreateContext(display, config, EGL14.EGL_NO_CONTEXT, contextAttribs, 0)
        if (context == EGL14.EGL_NO_CONTEXT) {
            throw RuntimeException("eglCreateContext failed: ${EGL14.eglGetError()}")
        }
        eglDisplay = display
        eglConfig = config
        eglContext = context
        // A pbuffer keeps a current context alive before the window surface exists.
        val pbufferAttribs = intArrayOf(EGL14.EGL_WIDTH, 1, EGL14.EGL_HEIGHT, 1, EGL14.EGL_NONE)
        val pbuffer = EGL14.eglCreatePbufferSurface(display, config, pbufferAttribs, 0)
        if (!EGL14.eglMakeCurrent(display, pbuffer, pbuffer, context)) {
            throw RuntimeException("eglMakeCurrent(pbuffer) failed: ${EGL14.eglGetError()}")
        }
        eglSurface = pbuffer
        pendingResize = true
    }

    private fun releaseEgl() {
        val display = eglDisplay
        if (display != EGL14.EGL_NO_DISPLAY) {
            EGL14.eglMakeCurrent(
                display,
                EGL14.EGL_NO_SURFACE,
                EGL14.EGL_NO_SURFACE,
                EGL14.EGL_NO_CONTEXT,
            )
            if (eglSurface != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, eglSurface)
            if (eglContext != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(display, eglContext)
            EGL14.eglTerminate(display)
        }
        eglDisplay = EGL14.EGL_NO_DISPLAY
        eglContext = EGL14.EGL_NO_CONTEXT
        eglSurface = EGL14.EGL_NO_SURFACE
        eglConfig = null
    }

    // ── helpers ──────────────────────────────────────────────────────────

    /** Runs [block] on the AR thread. Every ARCore call goes through here. */
    private fun post(block: () -> Unit) {
        handler.post(block)
    }

    private fun emit(event: Map<String, Any?>) = onEvent(event)

    /** Resets continuity state exactly when a new ARCore world is created. */
    private fun resetTrackingContinuityForNewSession() {
        trackingContinuityEpoch = 0L
        lastPoseEventTimestampNs = Long.MIN_VALUE
        lastObservedTrackingState = null
        lastObservedFrameTimestampNs = Long.MIN_VALUE
        continuityBreakLatched = false
    }

    private fun mapSessionError(e: Exception): String = when (e) {
        is com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException -> "arcore_not_installed"
        is com.google.ar.core.exceptions.UnavailableApkTooOldException -> "arcore_apk_too_old"
        is com.google.ar.core.exceptions.UnavailableSdkTooOldException -> "arcore_sdk_too_old"
        is com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException -> "device_not_compatible"
        is com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException -> "user_declined_install"
        is com.google.ar.core.exceptions.CameraNotAvailableException -> "camera_not_available"
        else -> "session_failed"
    }

    private fun describeSessionError(e: Exception): String = when (e) {
        is com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException ->
            "Google Play services for AR is not installed."
        is com.google.ar.core.exceptions.UnavailableApkTooOldException ->
            "Google Play services for AR is out of date and needs updating."
        is com.google.ar.core.exceptions.UnavailableSdkTooOldException ->
            "This app was built against an ARCore SDK that is too old for the installed service."
        is com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException ->
            "This device does not support ARCore."
        is com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException ->
            "ARCore installation was declined."
        is com.google.ar.core.exceptions.CameraNotAvailableException ->
            "The camera could not be opened. It may be in use by another app."
        else -> e.message ?: "The AR session could not be started."
    }

    /** One mesh instance to draw, as described by Dart. */
    class DrawSpec(
        val meshId: String,
        val anchorId: String?,
        val localRotation: FloatArray,
        val scale: Float,
        val offsetX: Float,
        val offsetY: Float,
        val offsetZ: Float,
        val tintR: Float,
        val tintG: Float,
        val tintB: Float,
        val tintA: Float,
        val useTint: Boolean,
        val opacity: Float,
        val requireTracking: Boolean,
        /**
         * The engine's column-major `[X | Y | Z | worldPosition]` basis for this
         * instance, or `null` when Dart sent none — see
         * [ArGlRenderer.DrawRequest.modelMatrix]. Only its rotation is consumed;
         * the anchor carries the world translation.
         */
        val modelMatrix: FloatArray? = null,
        /** True when [modelMatrix]'s translation is station-anchor-local. */
        val useModelTranslation: Boolean = false,
    )

    /** A world-space ray to hit-test. */
    class RaycastSpec(
        val origin: FloatArray,
        val direction: FloatArray,
        val maxDistanceM: Float,
        val surface: String,
    )

    private class PendingRaycast(
        val spec: RaycastSpec,
        val onDone: (ArOutcome) -> Unit,
    )

    companion object {
        /** One identity per process-created ARCore [Session]. */
        private val NEXT_SESSION_ID = AtomicLong(0L)

        private const val FRAME_PERIOD_MS = 33L

        /** Flutter guidance rate; ARCore rendering and VIO remain at 30 Hz. */
        private const val POSE_EVENT_INTERVAL_NS = 66_666_667L

        /**
         * Roughly seven dropped 15 Hz deliveries. The Dart gate still resets its capture
         * window at 150 ms; this longer threshold is only for escalating an
         * otherwise invisible stream stall into station-reference recovery.
         */
        private const val CONTINUITY_GAP_NS = 500_000_000L

        /** Full-screen quad in OpenGL normalised device coordinates. */
        val NDC_QUAD = floatArrayOf(
            -1f, -1f,
            1f, -1f,
            -1f, 1f,
            1f, 1f,
        )
    }
}
