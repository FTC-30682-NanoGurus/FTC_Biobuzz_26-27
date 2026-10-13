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
import java.util.ArrayList;
import java.util.Arrays;
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
 *       {@link #SETTLE_FRAMES} frames spanning {@link #SETTLE_MIN_SPAN_S} to {@link #SETTLE_WINDOW_S}, and every tag seen at both
 *       ends of that window has moved less than {@link #SETTLE_SPREAD_IN} in 3D (field x, y AND
 *       height). Shooting into a CELL mid-swing can spoil the TIP.</li>
 *   <li><b>TIPPING</b> (strong): a settled-up CELL's tags have moved {@link #TIP_MOVE_IN} or more
 *       (3D) away from where it rested, in {@link #TIP_CONFIRM_FRAMES} frames in a row with the
 *       robot still. That CELL is on its way down, so the other CELL will be the up one.</li>
 *   <li><b>Moving</b>: any of a CELL's tags (upright or not) moved {@link #MOVE_SPREAD_IN} or more
 *       across {@link #MOVE_WINDOW_S} with the robot still - the HIVE is part-way through a tip.</li>
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
 * <h2>Why motion, not just visibility</h2>
 * A TIP is SLOW. BIOBUZZ HIVEs have a damper, and the TIP only counts once the HIVE has swung to
 * its other stable state and the damper touches the frame (manual 10.5.1). Teams have measured
 * 2.5-2.8 s for a TIP with NECTAR in the CELL and 4.8 s for 8 POLLEN, from the ball landing to the
 * scored TIP. For the first part of that swing the falling CELL's tags stay visible and still read
 * as right-side up: they sink (other teams saw the tag row fall from ~50 in by about 7 in before
 * losing it; the full swing moves them ~15 in). So "tags visible and upright" alone would call a
 * CELL that has already started to tip "up", and a slowly creeping CELL could pass a horizontal-
 * only settle test. Tracking each tag's 3D position against where the CELL rested catches the tip
 * within a few inches of movement, long before the tags disappear.
 *
 * <h2>Only our tags</h2>
 * Every detection is checked against our alliance's eight IDs, so the other HIVE cannot affect any
 * decision. (The library holds all sixteen so the alliance can be switched in INIT without
 * rebuilding the camera, and so seeing the OTHER alliance's tags can be reported as a warning.)
 *
 * <h2>Field positions</h2>
 * Tag positions go through the camera mount in {@link TurretConstants} and then through the robot
 * pose AT THE MOMENT THE FRAME WAS CAPTURED (pose history matched on frameAcquisitionNanoTime), so
 * the robot's own motion is not mistaken for CELL motion. The mount must be measured for the
 * settle test to be meaningful while driving; with the robot still it works regardless.
 *
 * <h2>Sides and the alliance frame</h2>
 * Poses passed in are in the ALLIANCE FRAME: the field as seen from our own drivers, written the
 * red way (our side is y &lt; 0, our drivers' right is +x). The field is rotationally symmetric, so
 * for blue this is the real field turned 180 degrees and every auto path works unchanged.
 * {@link Side#LEFT} is the CELL on our drivers' left, {@link Side#RIGHT} the one on their right
 * (the one that starts UP for both alliances). Red: LEFT = IDs 30-33 (side opposite the audience),
 * RIGHT = 34-37 (audience side). Blue: LEFT = 38-41 (audience side), RIGHT = 42-45.
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
    /**
     * The frames must also SPAN at least this long, SECONDS. Without it, 4 quick frames of a CELL
     * that has only just come into view mid-swing look still.
     */
    public static double SETTLE_MIN_SPAN_S = 0.5;
    /** Max 3D movement of any tag across the window, INCHES. */
    public static double SETTLE_SPREAD_IN = 1.5;
    /**
     * A settled-up CELL whose tags (median over the tags seen) have moved this far in 3D from where
     * it rested is TIPPING, INCHES. A full tip moves them about 15 in; a ball landing in the CELL
     * jostles it well under an inch.
     */
    public static double TIP_MOVE_IN = 2.5;
    /** ...in this many frames in a row, with the robot still. */
    public static int TIP_CONFIRM_FRAMES = 3;
    /** "HIVE moving" test: a tip creeps at first, so the window is long, SECONDS / INCHES. */
    public static double MOVE_WINDOW_S = 1.0;
    public static double MOVE_SPREAD_IN = 2.5;
    /** Longest a TIP takes from the ball landing to the HIVE at rest, SECONDS (measured 2.5-4.8 s). */
    public static double TIP_MAX_S = 5.0;
    /** A CELL unseen this long forgets where it rested, SECONDS. */
    public static double REST_FORGET_S = 1.0;
    /**
     * The rest position is only compared from (about) the pose it was learned at, INCHES / DEG, so
     * camera-calibration error seen from a different spot cannot look like the CELL moving.
     */
    public static double REST_POSE_TOL_IN = 2.0;
    public static double REST_POSE_TOL_DEG = 4.0;
    /**
     * Optional extra check: height of an UP CELL's tag row above the tiles, INCHES (other teams
     * measured ~50). NaN = not used. Only set it once TurretConstants.CAMERA_OFFSET_Z_IN and
     * CAMERA_MOUNT_PITCH_DEG are measured - read the value off "BioBuzz HIVE Tip Recorder".
     */
    public static double UP_ROW_HEIGHT_IN = Double.NaN;
    public static double ROW_HEIGHT_TOL_IN = 3.0;
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
        /** Recent right-side-up frames: {tSeconds, x0,y0,z0,...,x3,y3,z3} with NaN for unseen tags. */
        final ArrayDeque<double[]> frames = new ArrayDeque<>();
        /** Every recent frame of this CELL (upright or not) while the robot was still. */
        final ArrayDeque<double[]> motion = new ArrayDeque<>();
        long lastSettledNanos = 0, lastDownNanos = 0, lastSeenNanos = 0, lastUprightNanos = 0;
        long lastTippingNanos = 0, lastMovingNanos = 0;
        /** Where the CELL rested when last settled up (same layout as a frame, no time), or null. */
        double[] rest = null;
        /** Robot pose {x, y, heading} when {@link #rest} was learned. */
        double[] restPose = null;
        /** The rest it tipped away from; it cannot count as settled up again until it has gone. */
        double[] tippedFrom = null;
        int tipFrames = 0;
        double rowHeight = Double.NaN, restDev = Double.NaN;
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
        final double[] f = new double[4], l = new double[4], u = new double[4];
        final boolean[] has = new boolean[4];

        Track(String name, int firstId) {
            this.name = name;
            this.firstId = firstId;
        }

        boolean owns(int id) {
            return id >= firstId && id < firstId + 4;
        }
    }

    private Track left, right;
    private boolean red;
    private long lastOtherAllianceNanos = 0;
    private final Telemetry telemetry;
    private VisionPortal portal;
    private AprilTagProcessor processor;
    private boolean controlsApplied = false;
    private String cameraNote = "";
    private long lastFrameNanos = 0;
    private final List<String> events = new ArrayList<>();

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
        setAlliance(red);

        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id = 30; id <= 45; id++) {
            library.addTag(id, (id < 38 ? "RED " : "BLUE ") + id, TAG_SIZE_IN, DistanceUnit.INCH);
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

    /**
     * Picks whose tags count, and clears all evidence. Cheap - call it whenever the alliance changes
     * in INIT. CELL positions are converted into the alliance frame (blue: turned 180 degrees).
     */
    public void setAlliance(boolean red) {
        this.red = red;
        left = new Track(red ? "LEFT CELL (30-33)" : "LEFT CELL (38-41)", red ? 30 : 38);
        right = new Track(red ? "RIGHT CELL (34-37)" : "RIGHT CELL (42-45)", red ? 34 : 42);
        left.cellX = red ? TurretConstants.RED_LEFT_X_IN : -TurretConstants.BLUE_RIGHT_X_IN;
        left.cellY = red ? TurretConstants.RED_LEFT_Y_IN : -TurretConstants.BLUE_RIGHT_Y_IN;
        right.cellX = red ? TurretConstants.RED_RIGHT_X_IN : -TurretConstants.BLUE_LEFT_X_IN;
        right.cellY = red ? TurretConstants.RED_RIGHT_Y_IN : -TurretConstants.BLUE_LEFT_Y_IN;
        lastOtherAllianceNanos = 0;
        events.clear();
    }

    public boolean isRed() {
        return red;
    }

    /** Any of OUR alliance's tags seen within maxAgeS - a quick INIT check that the alliance is right. */
    public boolean ownTagsSeen(double maxAgeS) {
        return ageS(Math.max(left.lastSeenNanos, right.lastSeenNanos)) <= maxAgeS;
    }

    /** The OTHER alliance's tags seen within maxAgeS (never used for decisions). */
    public boolean otherAllianceTagsSeen(double maxAgeS) {
        return ageS(lastOtherAllianceNanos) <= maxAgeS;
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
            if (t == null) {
                if (d.id >= 30 && d.id <= 45) lastOtherAllianceNanos = now;
                continue;
            }
            double[] rb = cameraToRobot(d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
            if (Math.hypot(rb[0], rb[1]) > MAX_RANGE_IN) continue;
            int k = d.id - t.firstId;
            t.has[k] = true;
            t.f[k] = rb[0];
            t.l[k] = rb[1];
            t.u[k] = rb[2];
            double roll = wrapDeg(d.ftcPose.roll - TurretConstants.CAMERA_MOUNT_ROLL_DEG);
            if (Math.abs(roll) < UPRIGHT_ROLL_LIMIT_DEG) t.up++;
            else t.inv++;
            frameNanos = d.frameAcquisitionNanoTime;
        }
        // An empty list is still a processed frame: it is what absence is measured from.
        long stamp = frameNanos != 0 ? frameNanos : now;
        double[] cap = lookupPose(stamp);
        if (cap == null) cap = new double[]{pose.position.x, pose.position.y, pose.heading.toDouble()};
        boolean still = robotStill(stamp);
        if (frameNanos != 0) {
            process(left, frameNanos, cap, still);
            process(right, frameNanos, cap, still);
        }
        for (Track t : new Track[]{left, right}) {
            double unseen = ageS(t.lastUprightNanos);
            if (t.rest != null && unseen > REST_FORGET_S) t.rest = null;
            if (t.tippedFrom != null && unseen > ABSENCE_DOWN_S) t.tippedFrom = null;
            if (!still) t.motion.clear();
        }
        double frameDt = lastFrame == 0 ? 0 : Math.min(0.1, (now - lastFrame) / 1e9);
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

    private void process(Track t, long frameNanos, double[] cap, boolean still) {
        int n = t.up + t.inv;
        if (n == 0) return;
        t.lastSeenNanos = frameNanos;
        t.tagsLastFrame = n;

        double ts = frameNanos / 1e9;
        double[] frame = new double[13];
        frame[0] = ts;
        double c = Math.cos(cap[2]), s = Math.sin(cap[2]);
        double heightSum = 0;
        for (int k = 0; k < 4; k++) {
            if (t.has[k]) {
                frame[1 + 3 * k] = cap[0] + c * t.f[k] - s * t.l[k];
                frame[2 + 3 * k] = cap[1] + s * t.f[k] + c * t.l[k];
                frame[3 + 3 * k] = t.u[k];
                heightSum += t.u[k];
            } else {
                frame[1 + 3 * k] = frame[2 + 3 * k] = frame[3 + 3 * k] = Double.NaN;
            }
        }
        trackMotion(t, frame, still);

        if (t.inv > t.up) {
            t.frames.clear();
            t.rest = null;
            t.tippedFrom = null;
            t.tipFrames = 0;
            if (++t.invertedStreak >= DOWN_CONFIRM_FRAMES) t.lastDownNanos = frameNanos;
            t.note = "upside down (" + n + " tags)";
            return;
        }
        t.invertedStreak = 0;
        t.lastUprightNanos = frameNanos;
        t.rowHeight = heightSum / n;

        if (t.rest != null && (Math.hypot(cap[0] - t.restPose[0], cap[1] - t.restPose[1]) > REST_POSE_TOL_IN
                || Math.abs(Math.toDegrees(wrap(cap[2] - t.restPose[2]))) > REST_POSE_TOL_DEG)) {
            t.rest = null;
        }
        if (t.rest != null) {
            t.restDev = medianMove(t.rest, frame);
            if (still && t.restDev >= TIP_MOVE_IN) {
                if (++t.tipFrames >= TIP_CONFIRM_FRAMES) {
                    t.lastTippingNanos = frameNanos;
                    t.lastMovingNanos = frameNanos;
                    t.tippedFrom = t.rest;
                    t.rest = null;
                    t.frames.clear();
                    t.tipFrames = 0;
                    event(String.format("%s TIPPING (moved %.1f in from rest)", t.name, t.restDev));
                    t.note = "TIPPING";
                    return;
                }
            } else if (t.restDev < TIP_MOVE_IN) {
                t.tipFrames = 0;
            }
        } else {
            t.restDev = Double.NaN;
        }

        t.frames.addLast(frame);
        while (!t.frames.isEmpty() && ts - t.frames.peekFirst()[0] > SETTLE_WINDOW_S) t.frames.pollFirst();
        if (t.frames.size() < SETTLE_FRAMES || ts - t.frames.peekFirst()[0] < SETTLE_MIN_SPAN_S) {
            t.note = "UP, confirming (" + n + " tags)";
            return;
        }
        double spread = endsMove(t.frames);
        boolean heightOk = Double.isNaN(UP_ROW_HEIGHT_IN)
                || Math.abs(t.rowHeight + TurretConstants.CAMERA_OFFSET_Z_IN - UP_ROW_HEIGHT_IN) <= ROW_HEIGHT_TOL_IN;
        if (spread < 0 || spread > SETTLE_SPREAD_IN || !heightOk) {
            t.note = spread < 0 ? "UP, tag set changing"
                    : !heightOk ? "UP?, row at the wrong height" : String.format("UP, moving %.1f in", spread);
            return;
        }
        if (t.tippedFrom != null) {
            // It started to tip. Only a return to where it was (an aborted tip) counts as up again
            // before it has gone out of sight.
            if (medianMove(t.tippedFrom, frame) > SETTLE_SPREAD_IN) {
                t.note = "tipping - not up";
                return;
            }
            t.tippedFrom = null;
            event(t.name + " back at rest - the tip did not happen");
        }
        boolean wasSettled = ageS(t.lastSettledNanos) < 0.5 && t.lastSettledNanos > t.lastTippingNanos;
        t.lastSettledNanos = frameNanos;
        if (t.rest == null && still) {
            t.rest = average(t.frames);
            t.restPose = cap.clone();
        }
        if (!wasSettled) event(t.name + " settled UP");
        t.note = "UP, settled (" + n + " tags)";
    }

    /** Records "the HIVE is moving" from every sighting of the CELL while the robot is still. */
    private void trackMotion(Track t, double[] frame, boolean still) {
        if (!still) {
            t.motion.clear();
            return;
        }
        t.motion.addLast(frame);
        while (!t.motion.isEmpty() && frame[0] - t.motion.peekFirst()[0] > MOVE_WINDOW_S) t.motion.pollFirst();
        if (t.motion.size() < 4) return;
        double move = endsMove(t.motion);
        if (move >= MOVE_SPREAD_IN) {
            if (ageS(t.lastMovingNanos) > 1.0) event(String.format("%s moving (%.1f in)", t.name, move));
            t.lastMovingNanos = System.nanoTime();
        }
    }

    /** Median, over the tags seen in both, of how far each tag is from {@code from}, INCHES; NaN if none. */
    private static double medianMove(double[] from, double[] frame) {
        double[] d = new double[4];
        int m = 0;
        for (int k = 0; k < 4; k++) {
            double ox = from[1 + 3 * k], nx = frame[1 + 3 * k];
            if (Double.isNaN(ox) || Double.isNaN(nx)) continue;
            d[m++] = Math.sqrt(sq(nx - ox) + sq(frame[2 + 3 * k] - from[2 + 3 * k]) + sq(frame[3 + 3 * k] - from[3 + 3 * k]));
        }
        if (m == 0) return Double.NaN;
        Arrays.sort(d, 0, m);
        return m % 2 == 1 ? d[m / 2] : 0.5 * (d[m / 2 - 1] + d[m / 2]);
    }

    /**
     * Largest per-tag movement between the average of the first two and last two frames (averaging
     * pairs halves single-frame noise), INCHES; -1 if no tag is in both.
     */
    private static double endsMove(ArrayDeque<double[]> frames) {
        double[][] all = frames.toArray(new double[0][]);
        int n = all.length;
        double[] first = averageOf(all[0], all[Math.min(1, n - 1)]);
        double[] last = averageOf(all[Math.max(0, n - 2)], all[n - 1]);
        double worst = -1;
        for (int k = 0; k < 4; k++) {
            if (Double.isNaN(first[1 + 3 * k]) || Double.isNaN(last[1 + 3 * k])) continue;
            worst = Math.max(worst, Math.sqrt(sq(last[1 + 3 * k] - first[1 + 3 * k])
                    + sq(last[2 + 3 * k] - first[2 + 3 * k]) + sq(last[3 + 3 * k] - first[3 + 3 * k])));
        }
        return worst;
    }

    private static double[] averageOf(double[] a, double[] b) {
        double[] out = new double[13];
        for (int i = 0; i < 13; i++) {
            if (Double.isNaN(a[i])) out[i] = b[i];
            else if (Double.isNaN(b[i])) out[i] = a[i];
            else out[i] = 0.5 * (a[i] + b[i]);
        }
        return out;
    }

    private static double[] average(ArrayDeque<double[]> frames) {
        double[] sum = new double[13];
        int[] cnt = new int[13];
        for (double[] f : frames) {
            for (int i = 1; i < 13; i++) {
                if (!Double.isNaN(f[i])) {
                    sum[i] += f[i];
                    cnt[i]++;
                }
            }
        }
        for (int i = 1; i < 13; i++) sum[i] = cnt[i] == 0 ? Double.NaN : sum[i] / cnt[i];
        return sum;
    }

    private static double sq(double v) {
        return v * v;
    }

    private void event(String msg) {
        events.add(msg);
        if (events.size() > 50) events.remove(0);
    }

    // =============================================================================================
    // Queries
    // =============================================================================================

    /** Seen right-side up and at rest within the last maxAgeS seconds, and not seen tipping since. */
    public boolean settledUp(Side side, double maxAgeS) {
        Track t = track(side);
        return ageS(t.lastSettledNanos) <= maxAgeS && t.lastSettledNanos > t.lastTippingNanos;
    }

    /** Seen leaving its rest (starting to tip DOWN) within the last maxAgeS seconds. */
    public boolean tipping(Side side, double maxAgeS) {
        return ageS(track(side).lastTippingNanos) <= maxAgeS;
    }

    /** That CELL was seen moving (or starting to tip) within the last maxAgeS seconds. */
    public boolean moving(Side side, double maxAgeS) {
        Track t = track(side);
        return ageS(Math.max(t.lastMovingNanos, t.lastTippingNanos)) <= maxAgeS;
    }

    /** Either of our CELLS was seen moving within the last maxAgeS seconds. */
    public boolean hiveMoving(double maxAgeS) {
        return moving(Side.LEFT, maxAgeS) || moving(Side.RIGHT, maxAgeS);
    }

    /**
     * Signs that a tip involving this CELL is under way: it was seen moving, or seen upright but
     * not (yet) at rest, within the last maxAgeS seconds.
     */
    public boolean tipActivity(Side side, double maxAgeS) {
        return moving(side, maxAgeS) || (seenUpright(side, maxAgeS) && !settledUp(side, maxAgeS));
    }

    /** New event lines ("LEFT CELL TIPPING", "RIGHT CELL settled UP", ...) since the last call. */
    public List<String> drainEvents() {
        List<String> out = new ArrayList<>(events);
        events.clear();
        return out;
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
     * (seen upside down, missing as the near CELL, or seen starting to tip) says RIGHT, and the
     * same for RIGHT. A CELL seen tipping says the OTHER one - it is where the HIVE is going, so it
     * may still be swinging; callers that are about to shoot also check {@link #hiveMoving}. The most
     * recent event inside the window wins, so a tip is followed correctly: old "LEFT up" evidence
     * is overruled by newer "LEFT missing" evidence and vice versa.
     */
    public Side upCell(double maxAgeS) {
        Side best = null;
        long bestT = 0;
        long[] times = {
                left.lastSettledNanos > left.lastTippingNanos ? left.lastSettledNanos : 0,
                Math.max(Math.max(left.lastDownNanos, left.lastAbsentDownNanos), left.lastTippingNanos),
                right.lastSettledNanos > right.lastTippingNanos ? right.lastSettledNanos : 0,
                Math.max(Math.max(right.lastDownNanos, right.lastAbsentDownNanos), right.lastTippingNanos)};
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
        return String.format("cam %s | L: %s | R: %s", cameraState(), describe(left), describe(right));
    }

    private String describe(Track t) {
        return String.format("%s, %s, row %s in, off rest %s in, missing %.1fs", t.note, ageText(t.lastSeenNanos),
                Double.isNaN(t.rowHeight) ? "-" : String.format("%.1f", t.rowHeight + TurretConstants.CAMERA_OFFSET_Z_IN),
                Double.isNaN(t.restDev) ? "-" : String.format("%.1f", t.restDev), t.absentS);
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

    /**
     * Camera frame (ftcPose: x right, y out of lens, z up) to robot frame (x fwd, y left, z up),
     * z measured from the camera's height (add CAMERA_OFFSET_Z_IN for height above the tiles).
     */
    private static double[] cameraToRobot(double xRight, double yOut, double zUp) {
        double f = yOut, l = -xRight, u = zUp;
        double roll = Math.toRadians(TurretConstants.CAMERA_MOUNT_ROLL_DEG);
        double l1 = l * Math.cos(roll) - u * Math.sin(roll);
        double u1 = l * Math.sin(roll) + u * Math.cos(roll);
        double pitch = Math.toRadians(TurretConstants.CAMERA_MOUNT_PITCH_DEG);
        double f2 = f * Math.cos(pitch) - u1 * Math.sin(pitch);
        double u2 = f * Math.sin(pitch) + u1 * Math.cos(pitch);
        double yaw = Math.toRadians(TurretConstants.CAMERA_MOUNT_YAW_DEG);
        double f3 = f2 * Math.cos(yaw) - l1 * Math.sin(yaw);
        double l3 = f2 * Math.sin(yaw) + l1 * Math.cos(yaw);
        return new double[]{f3 + TurretConstants.CAMERA_OFFSET_X_IN, l3 + TurretConstants.CAMERA_OFFSET_Y_IN, u2};
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
