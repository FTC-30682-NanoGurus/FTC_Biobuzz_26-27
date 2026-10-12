package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Watches OUR alliance's two HIVE CELLS during auto and answers one question: which CELL is up?
 *
 * <h2>Evidence, and how strong each kind is</h2>
 * Each CELL carries a 4-tag cluster on its bottom face, facing the tiles. Only the UP CELL's cluster
 * faces the robots, so:
 * <ul>
 *   <li><b>Settled UP</b> (strong): the cluster is seen right-side up in at least
 *       {@link #SETTLE_FRAMES} frames inside {@link #SETTLE_WINDOW_S}, and every tag seen in both the
 *       oldest and newest of those frames has moved less than {@link #SETTLE_SPREAD_IN} in the FIELD
 *       frame. A CELL that is still swinging up after a tip moves inches per frame, so it does not
 *       count until it has come to rest - shooting into a CELL mid-swing can spoil the TIP.</li>
 *   <li><b>Confirmed DOWN</b>: the cluster is seen upside down in {@link #DOWN_CONFIRM_FRAMES}
 *       frames in a row.</li>
 *   <li>Exactly one CELL per HIVE is up, so evidence about one CELL is evidence about the other.</li>
 *   <li><b>Down by absence</b> - NEAR CELL only. An up CELL's cluster faces the robot and is easy
 *       to see from a shooting pose (41-49 in). A down CELL's is not, and the FAR CELL is not
 *       reliably visible either way (the near CELL hangs in front of it). So the near CELL counts
 *       as down once it has been missing for {@link #ABSENCE_DOWN_S} of camera frames during which
 *       the camera was delivering frames, the robot was nearly stopped, and that CELL was inside
 *       the camera's view cone within {@link #ABSENCE_MAX_RANGE_IN}. Missing tags in any other
 *       situation (driving, blur, far away, out of view, no frames) are never evidence.</li>
 * </ul>
 *
 * <h2>Only our tags</h2>
 * The tag library holds only our alliance's eight IDs, and every detection is checked against them,
 * so the other HIVE cannot affect anything here.
 *
 * <h2>Field positions</h2>
 * Tag positions go through the camera mount in {@link TurretConstants} and then through the robot
 * pose AT THE MOMENT THE FRAME WAS CAPTURED (pose history matched on frameAcquisitionNanoTime), so
 * the robot's own motion is not mistaken for CELL motion. The mount must be measured for the
 * settle test to be meaningful while driving; with the robot still it works regardless.
 *
 * <h2>Sides</h2>
 * {@link Side#LEFT} is the CELL at field x &lt; 0 (the side opposite the audience), {@link Side#RIGHT}
 * the one at x &gt; 0 (audience side). For red that is IDs 30-33 and 34-37; for blue 42-45 and 38-41.
 *
 * Everything degrades safely: no camera, no frames or a missing config entry all just mean "no
 * evidence", and the autos then follow their original plan.
 */
@Config
public class HiveWatcher {

    public enum Side {
        LEFT, RIGHT;

        public Side other() {
            return this == LEFT ? RIGHT : LEFT;
        }
    }

    /** BIOBUZZ tag edge length, INCHES (manual 9.9). */
    public static double TAG_SIZE_IN = 3.25;
    /**
     * Manual short exposure (TurretConstants.EXPOSURE_MS / GAIN) reduces blur while driving, but is
     * near-black in dim light. OFF = auto exposure, the safe default; try ON under field lighting.
     */
    public static boolean MANUAL_EXPOSURE = false;
    /** |roll| below this, DEGREES, is right-side up (CELL up). FIRST's guidance is 90. */
    public static double UPRIGHT_ROLL_LIMIT_DEG = 90.0;
    /** Frames inside the window needed before a CELL counts as settled up. */
    public static int SETTLE_FRAMES = 4;
    /** Window those frames must fall in, SECONDS. */
    public static double SETTLE_WINDOW_S = 0.6;
    /** Max movement of any tag across the window, INCHES. */
    public static double SETTLE_SPREAD_IN = 1.5;
    /** Upside-down frames in a row that confirm a CELL is down. */
    public static int DOWN_CONFIRM_FRAMES = 3;
    /** Tags further than this are ignored, INCHES. */
    public static double MAX_RANGE_IN = 110.0;

    /** Near CELL missing for this long, under the conditions above, means it is down, SECONDS. */
    public static double ABSENCE_DOWN_S = 0.5;
    /** Only a CELL closer than this can be judged by absence, INCHES. Shooting poses are 41-49 in. */
    public static double ABSENCE_MAX_RANGE_IN = 56.0;
    /** Half the camera's horizontal view used for the "in view" test, DEGREES (C920 640x480 is ~27). */
    public static double VIEW_HALF_ANGLE_DEG = 22.0;
    /** Robot slower than this counts as stopped for absence, IN/S and DEG/S. */
    public static double STILL_SPEED_IN_S = 6.0;
    public static double STILL_TURN_DEG_S = 25.0;
    /** No frame for this long means the camera is not delivering, SECONDS. */
    public static double CAMERA_STALE_S = 0.4;

    /** One CELL's evidence. */
    private static class Track {
        final String name;
        final int firstId;
        /** Recent right-side-up frames: {tSeconds, x0,y0,...,x3,y3} with NaN for unseen tags. */
        final ArrayDeque<double[]> frames = new ArrayDeque<>();
        long lastSettledNanos = 0, lastDownNanos = 0, lastSeenNanos = 0, lastUprightNanos = 0;
        int invertedStreak = 0, tagsLastFrame = 0;
        /** Field position of the CELL, for the in-view test. */
        double cellX, cellY;
        /** Seconds of qualifying frames without seeing this CELL upright, and when last updated. */
        double absentS = 0;
        long absenceUpdatedNanos = 0;
        /** Last time the absence test concluded this CELL was down. */
        long lastAbsentDownNanos = 0;
        String note = "not seen";

        // per-frame scratch
        int up, inv;
        final double[] f = new double[4], l = new double[4];
        final boolean[] has = new boolean[4];

        Track(String name, int firstId) {
            this.name = name;
            this.firstId = firstId;
        }

        boolean owns(int id) {
            return id >= firstId && id < firstId + 4;
        }
    }

    private final Track left, right;
    private final Telemetry telemetry;
    private VisionPortal portal;
    private AprilTagProcessor processor;
    private boolean controlsApplied = false;
    private String cameraNote = "";
    private long lastFrameNanos = 0;

    private static final int HISTORY = 256;
    private final long[] histT = new long[HISTORY];
    private final double[] histX = new double[HISTORY], histY = new double[HISTORY], histH = new double[HISTORY];
    private int histHead = -1, histCount = 0;

    /**
     * Opens the camera immediately - call it during INIT, the camera takes a second or two to start.
     *
     * @param red true for the red alliance's tags, false for blue
     */
    public HiveWatcher(HardwareMap hardwareMap, Telemetry telemetry, boolean red) {
        this.telemetry = telemetry;
        left = new Track("LEFT CELL (x<0)", red ? 30 : 42);
        right = new Track("RIGHT CELL (x>0)", red ? 34 : 38);
        left.cellX = red ? TurretConstants.RED_LEFT_X_IN : TurretConstants.BLUE_LEFT_X_IN;
        left.cellY = red ? TurretConstants.RED_LEFT_Y_IN : TurretConstants.BLUE_LEFT_Y_IN;
        right.cellX = red ? TurretConstants.RED_RIGHT_X_IN : TurretConstants.BLUE_RIGHT_X_IN;
        right.cellY = red ? TurretConstants.RED_RIGHT_Y_IN : TurretConstants.BLUE_RIGHT_Y_IN;

        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (Track t : new Track[]{left, right}) {
            for (int id = t.firstId; id < t.firstId + 4; id++) {
                library.addTag(id, t.name + " " + id, TAG_SIZE_IN, DistanceUnit.INCH);
            }
        }
        try {
            processor = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(library.build())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(TurretConstants.CAMERA_FX, TurretConstants.CAMERA_FY,
                            TurretConstants.CAMERA_CX, TurretConstants.CAMERA_CY)
                    .build();
            processor.setDecimation(TurretConstants.APRILTAG_DECIMATION);
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, TurretConstants.WEBCAM_NAME))
                    .setCameraResolution(new Size(TurretConstants.STREAM_WIDTH, TurretConstants.STREAM_HEIGHT))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(false)
                    .addProcessor(processor)
                    .build();
            cameraNote = "opening";
        } catch (Exception e) {
            portal = null;
            processor = null;
            cameraNote = "FAILED: " + e.getMessage();
        }
    }

    /** Call every loop with the current pose estimate. Cheap when no new frame has arrived. */
    public void update(Pose2d pose) {
        long now = System.nanoTime();
        pushPose(now, pose);
        if (portal == null || processor == null) return;

        if (!controlsApplied && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            applyCameraControls();
        }

        List<AprilTagDetection> detections = processor.getFreshDetections();
        if (detections == null) return;
        long lastFrame = lastFrameNanos;
        lastFrameNanos = now;

        for (Track t : new Track[]{left, right}) {
            t.up = t.inv = 0;
            for (int i = 0; i < 4; i++) t.has[i] = false;
        }
        long frameNanos = 0;
        for (AprilTagDetection d : detections) {
            if (d.metadata == null || d.ftcPose == null) continue;
            Track t = left.owns(d.id) ? left : right.owns(d.id) ? right : null;
            if (t == null) continue;
            double[] rb = cameraToRobot(d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
            if (Math.hypot(rb[0], rb[1]) > MAX_RANGE_IN) continue;
            int k = d.id - t.firstId;
            t.has[k] = true;
            t.f[k] = rb[0];
            t.l[k] = rb[1];
            double roll = wrapDeg(d.ftcPose.roll - TurretConstants.CAMERA_MOUNT_ROLL_DEG);
            if (Math.abs(roll) < UPRIGHT_ROLL_LIMIT_DEG) t.up++;
            else t.inv++;
            frameNanos = d.frameAcquisitionNanoTime;
        }
        // An empty list is still a processed frame: it is what absence is measured from.
        long stamp = frameNanos != 0 ? frameNanos : now;
        double[] cap = lookupPose(stamp);
        if (cap == null) cap = new double[]{pose.position.x, pose.position.y, pose.heading.toDouble()};
        if (frameNanos != 0) {
            process(left, frameNanos, cap);
            process(right, frameNanos, cap);
        }
        double frameDt = lastFrame == 0 ? 0 : Math.min(0.1, (now - lastFrame) / 1e9);
        boolean still = robotStill(stamp);
        updateAbsence(left, cap, still, frameDt, now);
        updateAbsence(right, cap, still, frameDt, now);
    }

    /** Accumulates time the CELL should have been visible (if up) but was not seen upright. */
    private void updateAbsence(Track t, double[] cap, boolean still, double frameDt, long now) {
        boolean seenUpright = t.lastUprightNanos != 0 && (now - t.lastUprightNanos) / 1e9 < 0.15;
        if (seenUpright) {
            t.absentS = 0;
            t.absenceUpdatedNanos = now;
            return;
        }
        double dx = t.cellX - cap[0], dy = t.cellY - cap[1];
        double range = Math.hypot(dx, dy);
        double bearing = Math.toDegrees(wrap(Math.atan2(dy, dx) - cap[2]
                - Math.toRadians(TurretConstants.CAMERA_MOUNT_YAW_DEG)));
        boolean inView = range <= ABSENCE_MAX_RANGE_IN && Math.abs(bearing) <= VIEW_HALF_ANGLE_DEG;
        if (inView && still) {
            t.absentS += frameDt;
            t.absenceUpdatedNanos = now;
            if (t.absentS >= ABSENCE_DOWN_S) t.lastAbsentDownNanos = now;
        } else if (!inView) {
            t.absentS = 0;
        }
    }

    private boolean robotStill(long t) {
        double[] a = lookupPose(t), b = lookupPose(t - 150_000_000L);
        if (a == null || b == null) return false;
        double speed = Math.hypot(a[0] - b[0], a[1] - b[1]) / 0.15;
        double turn = Math.toDegrees(Math.abs(wrap(a[2] - b[2]))) / 0.15;
        return speed <= STILL_SPEED_IN_S && turn <= STILL_TURN_DEG_S;
    }

    private void process(Track t, long frameNanos, double[] cap) {
        int n = t.up + t.inv;
        if (n == 0) return;
        t.lastSeenNanos = frameNanos;
        t.tagsLastFrame = n;

        if (t.inv > t.up) {
            t.frames.clear();
            if (++t.invertedStreak >= DOWN_CONFIRM_FRAMES) t.lastDownNanos = frameNanos;
            t.note = "upside down (" + n + " tags)";
            return;
        }
        t.invertedStreak = 0;
        t.lastUprightNanos = frameNanos;

        double c = Math.cos(cap[2]), s = Math.sin(cap[2]);
        double ts = frameNanos / 1e9;
        double[] frame = new double[9];
        frame[0] = ts;
        for (int k = 0; k < 4; k++) {
            if (t.has[k]) {
                frame[1 + 2 * k] = cap[0] + c * t.f[k] - s * t.l[k];
                frame[2 + 2 * k] = cap[1] + s * t.f[k] + c * t.l[k];
            } else {
                frame[1 + 2 * k] = Double.NaN;
                frame[2 + 2 * k] = Double.NaN;
            }
        }
        t.frames.addLast(frame);
        while (!t.frames.isEmpty() && ts - t.frames.peekFirst()[0] > SETTLE_WINDOW_S) t.frames.pollFirst();

        if (t.frames.size() >= SETTLE_FRAMES) {
            double[] oldest = t.frames.peekFirst();
            double worst = -1;
            for (int k = 0; k < 4; k++) {
                double ox = oldest[1 + 2 * k], nx = frame[1 + 2 * k];
                if (Double.isNaN(ox) || Double.isNaN(nx)) continue;
                worst = Math.max(worst, Math.hypot(nx - ox, frame[2 + 2 * k] - oldest[2 + 2 * k]));
            }
            if (worst >= 0 && worst <= SETTLE_SPREAD_IN) {
                t.lastSettledNanos = frameNanos;
                t.note = "UP, settled (" + n + " tags)";
            } else {
                t.note = worst < 0 ? "UP, tag set changing" : String.format("UP, moving %.1f in", worst);
            }
        } else {
            t.note = "UP, confirming (" + n + " tags)";
        }
    }

    // =============================================================================================
    // Queries
    // =============================================================================================

    /** Seen right-side up and at rest within the last maxAgeS seconds. */
    public boolean settledUp(Side side, double maxAgeS) {
        return ageS(track(side).lastSettledNanos) <= maxAgeS;
    }

    /** Seen upside down (confirmed) within the last maxAgeS seconds. */
    public boolean confirmedDown(Side side, double maxAgeS) {
        return ageS(track(side).lastDownNanos) <= maxAgeS;
    }

    /** Seen right-side up at all (settled or not) within the last maxAgeS seconds. */
    public boolean seenUpright(Side side, double maxAgeS) {
        return ageS(track(side).lastUprightNanos) <= maxAgeS;
    }

    /**
     * The NEAR CELL has been missing for at least minS seconds of frames in which it would have
     * been seen if up (see the class comment). Never true for a far CELL or without frames.
     */
    public boolean downByAbsence(Side side, double minS) {
        Track t = track(side);
        return cameraHealthy() && ageS(t.absenceUpdatedNanos) < 0.3 && t.absentS >= minS;
    }

    /** Frames are arriving. */
    public boolean cameraHealthy() {
        return portal != null && ageS(lastFrameNanos) <= CAMERA_STALE_S;
    }

    /**
     * Which CELL is up, from evidence no older than maxAgeS, or null if there is none.
     *
     * Every piece of evidence is an event with a time: "LEFT settled up" says LEFT, "LEFT down"
     * (seen upside down, or missing as the near CELL) says RIGHT, and the same for RIGHT. The most
     * recent event inside the window wins, so a tip is followed correctly: old "LEFT up" evidence
     * is overruled by newer "LEFT missing" evidence and vice versa.
     */
    public Side upCell(double maxAgeS) {
        Side best = null;
        long bestT = 0;
        long[] times = {
                left.lastSettledNanos, Math.max(left.lastDownNanos, left.lastAbsentDownNanos),
                right.lastSettledNanos, Math.max(right.lastDownNanos, right.lastAbsentDownNanos)};
        Side[] says = {Side.LEFT, Side.RIGHT, Side.RIGHT, Side.LEFT};
        for (int i = 0; i < 4; i++) {
            if (ageS(times[i]) <= maxAgeS && times[i] > bestT) {
                bestT = times[i];
                best = says[i];
            }
        }
        return best;
    }

    public String describe() {
        return String.format("cam %s | L: %s, %s, missing %.1fs | R: %s, %s, missing %.1fs", cameraState(),
                left.note, ageText(left.lastSeenNanos), left.absentS,
                right.note, ageText(right.lastSeenNanos), right.absentS);
    }

    public String cameraState() {
        if (portal == null) return cameraNote;
        return portal.getCameraState() + (controlsApplied ? "" : " (settings pending)");
    }

    public void close() {
        if (portal == null) return;
        try {
            portal.close();
        } catch (Exception ignored) {
        }
        portal = null;
    }

    private Track track(Side s) {
        return s == Side.LEFT ? left : right;
    }

    private static double ageS(long nanos) {
        return nanos == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - nanos) / 1e9;
    }

    private static String ageText(long nanos) {
        return nanos == 0 ? "never seen" : String.format("%.1fs ago", ageS(nanos));
    }

    // =============================================================================================
    // Geometry
    // =============================================================================================

    /** Camera frame (ftcPose: x right, y out of lens, z up) to robot frame (x fwd, y left, z up). */
    private static double[] cameraToRobot(double xRight, double yOut, double zUp) {
        double f = yOut, l = -xRight, u = zUp;
        double roll = Math.toRadians(TurretConstants.CAMERA_MOUNT_ROLL_DEG);
        double l1 = l * Math.cos(roll) - u * Math.sin(roll);
        double u1 = l * Math.sin(roll) + u * Math.cos(roll);
        double pitch = Math.toRadians(TurretConstants.CAMERA_MOUNT_PITCH_DEG);
        double f2 = f * Math.cos(pitch) - u1 * Math.sin(pitch);
        double yaw = Math.toRadians(TurretConstants.CAMERA_MOUNT_YAW_DEG);
        double f3 = f2 * Math.cos(yaw) - l1 * Math.sin(yaw);
        double l3 = f2 * Math.sin(yaw) + l1 * Math.cos(yaw);
        return new double[]{f3 + TurretConstants.CAMERA_OFFSET_X_IN, l3 + TurretConstants.CAMERA_OFFSET_Y_IN};
    }

    private void pushPose(long t, Pose2d p) {
        histHead = (histHead + 1) % HISTORY;
        histT[histHead] = t;
        histX[histHead] = p.position.x;
        histY[histHead] = p.position.y;
        histH[histHead] = p.heading.toDouble();
        if (histCount < HISTORY) histCount++;
    }

    private double[] lookupPose(long t) {
        if (histCount == 0) return null;
        int newer = histHead;
        if (t >= histT[newer]) return new double[]{histX[newer], histY[newer], histH[newer]};
        for (int i = 1; i < histCount; i++) {
            int older = (histHead - i + HISTORY) % HISTORY;
            if (histT[older] <= t) {
                double a = (double) (t - histT[older]) / Math.max(1, histT[newer] - histT[older]);
                return new double[]{
                        histX[older] + a * (histX[newer] - histX[older]),
                        histY[older] + a * (histY[newer] - histY[older]),
                        histH[older] + a * wrap(histH[newer] - histH[older])};
            }
            newer = older;
        }
        return null;
    }

    private void applyCameraControls() {
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (MANUAL_EXPOSURE) {
                if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    long lo = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                    long hi = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                    exposure.setExposure(Math.max(lo, Math.min(hi, TurretConstants.EXPOSURE_MS)), TimeUnit.MILLISECONDS);
                }
                GainControl gain = portal.getCameraControl(GainControl.class);
                if (gain != null) {
                    int want = TurretConstants.GAIN < 0 ? gain.getMaxGain() : TurretConstants.GAIN;
                    gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), want)));
                }
            } else if (exposure != null) {
                // Set auto explicitly: a UVC webcam keeps the last manual exposure until unplugged.
                for (ExposureControl.Mode mode : new ExposureControl.Mode[]{ExposureControl.Mode.AperturePriority,
                        ExposureControl.Mode.Auto, ExposureControl.Mode.ContinuousAuto}) {
                    if (exposure.isModeSupported(mode) && exposure.setMode(mode)) break;
                }
            }
        } catch (Exception e) {
            cameraNote = "controls failed: " + e.getMessage();
        }
        controlsApplied = true;
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double wrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }
}
