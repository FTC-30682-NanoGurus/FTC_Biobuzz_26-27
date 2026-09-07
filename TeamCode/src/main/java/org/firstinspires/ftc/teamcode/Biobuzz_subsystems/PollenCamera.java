package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.teamcode.vision.PollenPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

/**
 * Owns the C920: the VisionPortal, its exposure, and the job of turning the pipeline's
 * frame-by-frame guesses into ONE target the drive code can commit to.
 *
 * <h2>Why there is a confirmation stage at all</h2>
 * {@link PollenPipeline} publishes a fresh answer every frame, and any single frame can be wrong -
 * a specular highlight splits a ball in two, a driver's shoe crosses the mask, one blob's ratio
 * lands a hair either side of MERGE_RATIO and the estimated count jumps. Handing a raw
 * per-frame centroid straight to a trajectory builder means committing the whole robot to the
 * worst frame in the sequence.
 *
 * So a target is only {@link #isTargetReady() ready} once {@link #MIN_STREAK} consecutive frames
 * have agreed with each other to within {@link #TRACK_TOL_IN}, and the position handed out is an
 * exponential average over those frames rather than the newest one.
 *
 * <h2>Why the average is kept in FIELD coordinates</h2>
 * The pipeline reports the pile relative to the ROBOT, and the robot moves. Averaging robot-frame
 * readings taken from different poses averages together points that were never the same place, and
 * the harder the robot drives the worse the smoothing gets - exactly backwards. Each reading is
 * therefore rotated into field coordinates with the pose that was current when it was read, and
 * the average is taken there. A stationary pile then has a stationary estimate no matter what the
 * robot does, which is also what makes the estimate still usable a moment later, after the robot
 * has driven far enough that the pile has left the frame.
 *
 * That is why {@link #update(Pose2d)} must be given a pose. Calling the no-argument
 * {@link #update()} inherited from {@link Subsystem} still runs the tracker, but it marks the
 * target NOT ready, because without a pose there is no honest way to place it on the field.
 *
 * <h2>Latency</h2>
 * The pose used is the one at the moment the RESULT IS READ, not the moment the frame was
 * captured. At 20 fps plus processing that gap is roughly 50-100 ms, which is a few inches of
 * error if the robot is moving quickly when it looks. {@link PollenApproach} handles this by
 * refusing to start a run above a speed threshold; be aware of it before reusing this class
 * somewhere that does not.
 */
@Config
public class PollenCamera extends Subsystem {

    /** Camera name in the robot configuration. Defaults to the one the rest of the code uses. */
    public static String WEBCAM_NAME = RobotConstants.camera;

    /** Streaming resolution. 640x480 matches the default calibration in PollenGeometry. */
    public static int STREAM_WIDTH = 640;
    public static int STREAM_HEIGHT = 480;

    /**
     * Manual exposure. Auto exposure re-white-balances the moment a bright robot drives past,
     * which moves every hue in the frame and quietly breaks a mask that was tuned five seconds
     * ago. Short exposure also freezes motion blur, which matters because a blurred ball loses the
     * circularity the pipeline gates on.
     */
    public static boolean MANUAL_EXPOSURE = true;
    public static int EXPOSURE_MS = 6;
    public static int GAIN = 200;

    /** Frames that must agree before a target is offered to the drive code. */
    public static int MIN_STREAK = 3;
    /** How far apart, in inches, two consecutive frames may put the pile and still "agree". */
    public static double TRACK_TOL_IN = 8.0;
    /** Exponential smoothing weight on the OLD estimate. Higher = smoother and laggier. */
    public static double SMOOTH_ALPHA = 0.6;
    /** A target older than this is stale and no longer offered. */
    public static double MAX_AGE_MS = 400;
    /** Consecutive empty frames before the tracker gives up and resets the streak. */
    public static int LOST_FRAMES = 4;

    /** Push the annotated stream to the dashboard. Costs bandwidth; turn off at competition. */
    public static boolean DASHBOARD_STREAM = true;
    public static int DASHBOARD_FPS = 5;

    private final PollenPipeline pipeline = new PollenPipeline();
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private VisionPortal portal;
    private boolean exposureApplied = false;
    private String status = "not initialised";

    private long lastFrameId = -1;
    private int streak = 0;
    private int missStreak = 0;
    private boolean poseless = false;

    private Double fieldX = null, fieldY = null;   // smoothed estimate, field inches
    private int lastCount = 0;
    private double lastSpread = 0;
    private int lastBlobs = 0;
    private final ElapsedTime sinceGood = new ElapsedTime();

    public PollenCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    // ===========================================================================================
    // Lifecycle
    // ===========================================================================================

    @Override
    public void init() {
        try {
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                    .setCameraResolution(new Size(STREAM_WIDTH, STREAM_HEIGHT))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .addProcessor(pipeline)
                    .build();
            status = "opening";
            if (DASHBOARD_STREAM) {
                FtcDashboard.getInstance().startCameraStream(portal, DASHBOARD_FPS);
            }
        } catch (Exception e) {
            // A missing or misnamed camera must not take the whole teleop down with it. The drive
            // half of the opmode is still perfectly usable; only the auto-approach is lost.
            portal = null;
            status = "NO CAMERA: " + e.getMessage();
        }
    }

    public boolean isAvailable() {
        return portal != null;
    }

    public boolean isStreaming() {
        return portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Stops or resumes the stream. Worth having on a button: colour processing is the single most
     * expensive thing in the opmode, and it is pure waste for the 90% of a match when nobody is
     * about to press the approach button.
     */
    public void setStreaming(boolean on) {
        if (portal == null) return;
        if (on) {
            portal.resumeStreaming();
        } else {
            portal.stopStreaming();
            reset();
        }
    }

    public void close() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }

    /**
     * Forces the exposure and gain settings to be pushed to the camera again.
     *
     * applyExposureOnce() is a one-shot by design - it must not re-issue a USB control transfer
     * every loop. That makes EXPOSURE_MS and GAIN look dead once the first apply has happened, so
     * anything that edits them at runtime has to say so here.
     */
    public void reapplyExposure() {
        exposureApplied = false;
    }

    /** Drops the current lock. Call after acting on a target so the next run re-confirms. */
    public void reset() {
        streak = 0;
        missStreak = 0;
        fieldX = null;
        fieldY = null;
    }

    // ===========================================================================================
    // Tracking
    // ===========================================================================================

    /** Subsystem contract. Runs the tracker but cannot place the target - see the class notes. */
    @Override
    public void update() {
        poseless = true;
        ingest(null);
    }

    /** Call once per loop with the CURRENT pose estimate. */
    public void update(Pose2d robotPose) {
        poseless = false;
        ingest(robotPose);
    }

    private void ingest(Pose2d robotPose) {
        if (portal == null) return;

        applyExposureOnce();

        PollenPipeline.Result r = pipeline.getResult();
        if (r.frameId == lastFrameId) return;   // no new frame; the age timer does the rest
        lastFrameId = r.frameId;

        if (r.best == null) {
            missStreak++;
            if (missStreak >= LOST_FRAMES) reset();
            return;
        }
        missStreak = 0;
        lastCount = r.best.count;
        lastSpread = r.best.spread;
        lastBlobs = r.best.blobs;

        if (robotPose == null) return;   // tracked, but not placeable

        Vector2d p = robotToField(robotPose, r.best.x, r.best.y);

        if (fieldX == null || Math.hypot(p.x - fieldX, p.y - fieldY) > TRACK_TOL_IN) {
            // Disagreement is treated as a NEW target rather than as noise to be averaged in.
            // Averaging across a jump would park the estimate in the empty floor between two piles
            // for as long as the pipeline kept flip-flopping between them.
            fieldX = p.x;
            fieldY = p.y;
            streak = 1;
        } else {
            double a = clamp(SMOOTH_ALPHA, 0.0, 0.95);
            fieldX = a * fieldX + (1 - a) * p.x;
            fieldY = a * fieldY + (1 - a) * p.y;
            if (streak < Integer.MAX_VALUE) streak++;
        }
        sinceGood.reset();
    }

    private void applyExposureOnce() {
        if (exposureApplied || !MANUAL_EXPOSURE) return;
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        try {
            ExposureControl ec = portal.getCameraControl(ExposureControl.class);
            if (ec != null) {
                ec.setMode(ExposureControl.Mode.Manual);
                ec.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            }
            GainControl gc = portal.getCameraControl(GainControl.class);
            if (gc != null) gc.setGain(GAIN);
            status = "streaming (manual exposure)";
        } catch (Exception e) {
            status = "streaming (exposure control unavailable)";
        }
        // Set either way: a camera that refuses manual exposure should not be asked every loop.
        exposureApplied = true;
    }

    // ===========================================================================================
    // Target
    // ===========================================================================================

    /** True when a confirmed, fresh pile is available to drive at. */
    public boolean isTargetReady() {
        return !poseless
                && fieldX != null
                && streak >= MIN_STREAK
                && sinceGood.milliseconds() <= MAX_AGE_MS;
    }

    /** The confirmed pile in FIELD coordinates, or null. Safe to hold across loops. */
    public Vector2d getFieldTarget() {
        return fieldX == null ? null : new Vector2d(fieldX, fieldY);
    }

    /** The confirmed pile relative to the robot at {@code pose}: +X forward, +Y left, inches. */
    public Vector2d getRobotTarget(Pose2d pose) {
        if (fieldX == null || pose == null) return null;
        double dx = fieldX - pose.position.x;
        double dy = fieldY - pose.position.y;
        double h = pose.heading.toDouble();
        double c = Math.cos(-h), s = Math.sin(-h);
        return new Vector2d(dx * c - dy * s, dx * s + dy * c);
    }

    /** Straight-line distance from the robot to the confirmed pile, inches. NaN if none. */
    public double getRange(Pose2d pose) {
        Vector2d v = getRobotTarget(pose);
        return v == null ? Double.NaN : Math.hypot(v.x, v.y);
    }

    /** Estimated balls in the confirmed pile, from the most recent frame that saw it. */
    public int getCount() {
        return lastCount;
    }

    public int getStreak() {
        return streak;
    }

    public double getAgeMs() {
        return fieldX == null ? Double.POSITIVE_INFINITY : sinceGood.milliseconds();
    }

    /** Raw, unsmoothed, unconfirmed view of the latest frame. For telemetry and tuning only. */
    public PollenPipeline.Result getRawResult() {
        return pipeline.getResult();
    }

    public PollenPipeline getPipeline() {
        return pipeline;
    }

    /** Rotates a robot-relative offset into field coordinates about the given pose. */
    public static Vector2d robotToField(Pose2d pose, double xRobot, double yRobot) {
        double h = pose.heading.toDouble();
        double c = Math.cos(h), s = Math.sin(h);
        return new Vector2d(
                pose.position.x + xRobot * c - yRobot * s,
                pose.position.y + xRobot * s + yRobot * c);
    }

    @Override
    public void telemetry() {
        PollenPipeline.Result r = getRawResult();
        telemetry.addData("Pollen cam", isAvailable() ? (isStreaming() ? "streaming" : status) : status);
        telemetry.addData("Pollen blobs / clusters", r.detections.size() + " / " + r.clusters.size());
        telemetry.addData("Pollen vision ms", "%.1f", r.processMs);
        if (r.best != null) {
            telemetry.addData("Pollen best (raw)", "n=%d  %.1f in  %.0f deg  spread %.1f",
                    r.best.count, r.best.range, r.best.bearingDeg, r.best.spread);
        } else {
            telemetry.addData("Pollen best (raw)", "none");
        }
        telemetry.addData("Pollen lock", "%s  streak %d  age %.0f ms",
                isTargetReady() ? "READY" : "no", streak, getAgeMs());
        Vector2d f = getFieldTarget();
        if (f != null) telemetry.addData("Pollen field xy", "%.1f, %.1f", f.x, f.y);
        telemetry.addData("Pollen blobs in pile", lastBlobs);
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
