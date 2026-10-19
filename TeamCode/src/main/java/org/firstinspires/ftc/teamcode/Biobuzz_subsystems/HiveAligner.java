package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Chassis auto-align onto OUR alliance's up HIVE CELL, for TeleOp. The fusion is the one
 * BiobuzzHiveAutoAlignTest proved out, packaged so any TeleOp can use it:
 * <ul>
 *   <li><b>Odometry runs the heading loop.</b> The heading to aim at is recomputed every loop from
 *       the dead-wheel pose, so it is smooth, has no camera latency and keeps working when the
 *       tags are blocked.</li>
 *   <li><b>The camera only measures where each CELL is</b>, in the odometry frame, through the pose
 *       the robot had when the frame was captured (pose history), into a per-CELL 2-D Kalman filter
 *       with a long range / thin bearing uncertainty and a chi-squared outlier gate.</li>
 *   <li><b>Vision never touches odometry.</b> Only the target estimate moves, so camera noise cannot
 *       reach the pose, and the pose (which is noise-free) is what averages the camera noise out.</li>
 * </ul>
 * Added here so the aim can never jump:
 * <ul>
 *   <li>The controller does not aim at the Kalman estimate directly but at an aim point that
 *       follows it at no more than {@link #AIM_POINT_SLEW_IN_S}, so a vision correction (or a
 *       filter re-acquire) blends in over a few hundred ms instead of stepping the setpoint.</li>
 *   <li>The turn-rate command is acceleration-limited ({@link #MAX_TURN_ACCEL}) and starts from the
 *       robot's measured yaw rate on engage, so engaging, retargeting or a large error never kicks.</li>
 *   <li>The target CELL is picked once on engage and only changes on a confirmed tip, and never
 *       while a shot is feeding.</li>
 * </ul>
 * Only our alliance's eight tag IDs are ever used (RED 30-37, BLUE 38-45); the other alliance's are
 * dropped before any maths. Camera mount, intrinsics and surveyed CELL coordinates come from
 * {@link TurretConstants}, like BiobuzzHiveAutoAlignTest, so one calibration serves both.
 */
@Config
public class HiveAligner {

    // ---- shooter geometry ------------------------------------------------------------------
    /** Launcher exit point from robot centre, INCHES, + = LEFT. A miss that grows when close. */
    public static double LAUNCHER_OFFSET_Y_IN = 0.0;
    /** Launcher exit point from robot centre, INCHES, + = forward. Only matters with a yaw. */
    public static double LAUNCHER_OFFSET_X_IN = 0.0;
    /** Shot direction relative to robot forward, DEGREES, + = left. */
    public static double LAUNCHER_YAW_DEG = 0.0;
    /** Aim point minus tag-cluster centre, INCHES, away from the HIVE centre / toward our wall. */
    public static double AIM_OFFSET_OUTWARD_IN = 0.0;
    public static double AIM_OFFSET_TOWARD_ALLIANCE_IN = 0.0;

    // ---- heading control -------------------------------------------------------------------
    /** rad/s of turn per rad of error (the autos' Road Runner headingGain). */
    public static double HEADING_KP = 6.0;
    public static double HEADING_DEADBAND_DEG = 0.2;
    public static double MAX_TURN_RATE = 4.0;
    /** Turn-rate change limit, rad/s^2: no kick on engage, retarget or a big error. */
    public static double MAX_TURN_ACCEL = 14.0;
    /** How fast the aim point may follow the target estimate, in/s: vision corrections blend in. */
    public static double AIM_POINT_SLEW_IN_S = 6.0;
    public static boolean TURN_FEEDFORWARD = true;
    /** |error| below this counts as ON TARGET, DEGREES. */
    public static double ON_TARGET_DEG = 1.5;
    /** ...and the turn rate below this, rad/s, so it is settled rather than swinging through. */
    public static double ON_TARGET_MAX_RATE = 0.6;

    // ---- target choice ---------------------------------------------------------------------
    public static boolean AUTO_RETARGET_ON_TIP = true;

    // ---- vision and estimate (as BiobuzzHiveAutoAlignTest) ------------------------------------
    public static double TAG_SIZE_IN = 3.25;
    public static int TIP_CONFIRM_FRAMES = 3;
    public static double UPRIGHT_ROLL_LIMIT_DEG = 90.0;
    public static boolean MANUAL_EXPOSURE = false;
    public static int DASHBOARD_FPS = 10;
    public static double RANGE_SIGMA_FRAC = 0.03;
    public static double RANGE_SIGMA_MIN_IN = 0.5;
    public static double BEARING_SIGMA_DEG = 0.4;
    public static double LATERAL_SIGMA_MIN_IN = 0.25;
    public static double PARTIAL_CLUSTER_SIGMA_IN = 1.5;
    public static double PROCESS_NOISE_IN = 0.3;
    public static double PRIOR_SIGMA_IN = 3.0;
    public static double GATE_CHI2 = 13.8;
    public static int RESET_AFTER_REJECTS = 15;
    public static double MAX_MEASUREMENT_RANGE_IN = 144.0;
    public static double MAX_TURN_RATE_FOR_VISION_DEG_S = 150.0;
    public static int OFFSET_LEARN_SAMPLES = 60;
    public static int OFFSET_MIN_SAMPLES = 5;

    // ---- velocity drive --------------------------------------------------------------------
    public static double KS_RAMP_IN_S = 2.0;
    public static double WHEEL_SPEED_HEADROOM = 0.9;

    // =========================================================================================

    public enum CellState { UNKNOWN, UP, DOWN }

    /** One CELL: tag IDs, tip state, learned tag layout and the Kalman estimate of its aim point. */
    public static class Cell {
        public final String label;
        final int firstId;
        final double surveyX, surveyY, sideSign;

        CellState state = CellState.UNKNOWN, pending = CellState.UNKNOWN;
        int pendingFrames = 0;
        long lastSeenNanos = 0;

        boolean hasEstimate = false;
        double ex, ey, pxx, pxy, pyy;
        int rejects = 0;
        String lastResult = "never seen";

        final double[] offX = new double[4], offY = new double[4];
        final int[] offN = new int[4];

        int n, upright, inverted;
        final int[] ids = new int[4];
        final double[] fwd = new double[4], left = new double[4];

        Cell(String label, int firstId, double surveyX, double surveyY) {
            this.label = label;
            this.firstId = firstId;
            this.surveyX = surveyX;
            this.surveyY = surveyY;
            this.sideSign = surveyX >= 0 ? 1.0 : -1.0;
        }

        boolean owns(int id) {
            return id >= firstId && id < firstId + 4;
        }

        double sigmaIn() {
            return Math.sqrt(Math.max(0.0, Math.max(pxx, pyy)));
        }
    }

    private final MecaTank mecaTank;
    private boolean red;
    private Cell[] cells;
    private double allianceWallSign;
    private boolean fieldReferenced = false;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean cameraControlsApplied = false;
    private long lastFrameNanos = 0;

    private static final int HISTORY = 128;
    private final long[] histT = new long[HISTORY];
    private final double[] histX = new double[HISTORY], histY = new double[HISTORY];
    private final double[] histH = new double[HISTORY], histW = new double[HISTORY];
    private int histHead = -1, histCount = 0;

    private boolean engaged = false;
    private Cell target = null;
    private boolean aimPointSet = false;
    private double aimX, aimY;
    private double turnCmd = 0.0, headingErrDeg = 0.0, distanceIn = 0.0;
    private boolean onTarget = false;
    private String lastEvent = "";

    private static double cachedVoltage = 12.0;
    private static final ElapsedTime voltageTimer = new ElapsedTime();

    /**
     * Opens the camera with BOTH alliances' tags in the library (so the alliance can still change
     * in INIT without reopening it); only the selected alliance's IDs are ever used.
     */
    public HiveAligner(HardwareMap hardwareMap, MecaTank mecaTank, boolean red) {
        this.mecaTank = mecaTank;
        setAlliance(red);
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id = 30; id <= 45; id++) library.addTag(id, "HIVE tag " + id, TAG_SIZE_IN, DistanceUnit.INCH);
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(library.build())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(TurretConstants.CAMERA_FX, TurretConstants.CAMERA_FY,
                            TurretConstants.CAMERA_CX, TurretConstants.CAMERA_CY)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();
            aprilTag.setDecimation(TurretConstants.APRILTAG_DECIMATION);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, TurretConstants.WEBCAM_NAME))
                    .setCameraResolution(new Size(TurretConstants.STREAM_WIDTH, TurretConstants.STREAM_HEIGHT))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build();
            FtcDashboard.getInstance().startCameraStream(visionPortal, DASHBOARD_FPS);
        } catch (Exception e) {
            visionPortal = null;
            aprilTag = null;
        }
    }

    /** Picks which alliance's two CELLS to track. Forgets anything learned about the other set. */
    public void setAlliance(boolean red) {
        if (cells != null && red == this.red) return;
        this.red = red;
        allianceWallSign = red ? -1.0 : 1.0;
        cells = red
                ? new Cell[]{
                        new Cell("RED SCORING-SIDE CELL", 30, TurretConstants.RED_LEFT_X_IN, TurretConstants.RED_LEFT_Y_IN),
                        new Cell("RED AUDIENCE-SIDE CELL", 34, TurretConstants.RED_RIGHT_X_IN, TurretConstants.RED_RIGHT_Y_IN)}
                : new Cell[]{
                        new Cell("BLUE AUDIENCE-SIDE CELL", 38, TurretConstants.BLUE_RIGHT_X_IN, TurretConstants.BLUE_RIGHT_Y_IN),
                        new Cell("BLUE SCORING-SIDE CELL", 42, TurretConstants.BLUE_LEFT_X_IN, TurretConstants.BLUE_LEFT_Y_IN)};
        if (fieldReferenced) seedSurveyedPrior();
        release();
    }

    /**
     * Call once the pose is in FIELD coordinates (e.g. restored from an auto): the CELLS start at
     * their surveyed positions, so the robot can aim before the camera has seen them.
     */
    public void setFieldReferenced() {
        fieldReferenced = true;
        seedSurveyedPrior();
    }

    private void seedSurveyedPrior() {
        double p = PRIOR_SIGMA_IN * PRIOR_SIGMA_IN;
        for (Cell c : cells) {
            if (c.hasEstimate) continue;
            initEstimate(c, c.surveyX, c.surveyY, p, 0.0, p);
            c.lastResult = "surveyed prior";
        }
    }

    /** Every loop, with the pose updated this loop. */
    public void update(long nowNanos, Pose2d pose, PoseVelocity2d robotVel, double dt) {
        if (visionPortal != null && !cameraControlsApplied
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            applyCameraControls();
        }
        pushPose(nowNanos, pose.position.x, pose.position.y, pose.heading.toDouble(), yawRate(robotVel));
        double q = PROCESS_NOISE_IN * PROCESS_NOISE_IN * dt;
        for (Cell c : cells) {
            if (c.hasEstimate) {
                c.pxx += q;
                c.pyy += q;
            }
        }
        processVision();
    }

    /** Start aiming: picks the target and starts the turn command from the measured yaw rate. */
    public void engage(Pose2d pose, PoseVelocity2d robotVel) {
        engaged = true;
        target = pickTarget(pose);
        aimPointSet = false;
        turnCmd = yawRate(robotVel);
        onTarget = false;
        if (target != null) lastEvent = "LOCKED ON " + target.label;
    }

    public void release() {
        engaged = false;
        target = null;
        aimPointSet = false;
        onTarget = false;
    }

    public boolean engaged() {
        return engaged;
    }

    public boolean hasTarget() {
        return engaged && target != null;
    }

    public boolean onTarget() {
        return hasTarget() && onTarget;
    }

    /**
     * Turn rate to aim the shot line through the target, rad/s CCW, for this loop.
     * @param cmdVx,cmdVy   the FIELD velocity being commanded this loop, in/s (turn feedforward)
     * @param allowRetarget false while a shot is feeding: the target is held
     * @return NaN while nothing has been seen to aim at
     */
    public double turnRate(Pose2d pose, double cmdVx, double cmdVy, double dt, boolean allowRetarget) {
        if (!engaged) return Double.NaN;
        if (target == null) {
            target = pickTarget(pose);
            if (target == null) return Double.NaN;
            lastEvent = "LOCKED ON " + target.label;
            aimPointSet = false;
        }
        if (allowRetarget) retargetIfTipped();

        if (!aimPointSet) {
            aimX = target.ex;
            aimY = target.ey;
            aimPointSet = true;
        } else {
            double ddx = target.ex - aimX, ddy = target.ey - aimY;
            double d = Math.hypot(ddx, ddy), step = AIM_POINT_SLEW_IN_S * dt;
            if (d > step) {
                ddx *= step / d;
                ddy *= step / d;
            }
            aimX += ddx;
            aimY += ddy;
        }

        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        double dx = aimX - x, dy = aimY - y;
        double r = Math.max(1e-6, Math.hypot(dx, dy));
        distanceIn = r;

        // A line leaving (lx, ly) at angle psi (robot frame) passes through a point at range r and
        // bearing phi iff r*sin(phi - psi) = ly*cos(psi) - lx*sin(psi).
        double psi = Math.toRadians(LAUNCHER_YAW_DEG);
        double lateral = LAUNCHER_OFFSET_Y_IN * Math.cos(psi) - LAUNCHER_OFFSET_X_IN * Math.sin(psi);
        double phi = psi + (Math.abs(lateral) < r ? Math.asin(lateral / r) : 0.0);
        double err = wrap(Math.atan2(dy, dx) - phi - h);
        headingErrDeg = Math.toDegrees(err);

        double ff = TURN_FEEDFORWARD ? (dy * cmdVx - dx * cmdVy) / (r * r) : 0.0;
        double fb = Math.abs(headingErrDeg) < HEADING_DEADBAND_DEG ? 0.0 : HEADING_KP * err;
        double want = Range.clip(ff + fb, -MAX_TURN_RATE, MAX_TURN_RATE);
        double step = MAX_TURN_ACCEL * dt;
        turnCmd += Range.clip(want - turnCmd, -step, step);

        onTarget = Math.abs(headingErrDeg) < ON_TARGET_DEG && Math.abs(turnCmd) < ON_TARGET_MAX_RATE;
        return turnCmd;
    }

    // ---- target choice ---------------------------------------------------------------------

    /** Our CELL that is UP; if neither is known, the closest one not known to be down. */
    private Cell pickTarget(Pose2d pose) {
        Cell up = null, best = null, any = null;
        double bestD = Double.MAX_VALUE, anyD = Double.MAX_VALUE;
        for (int i = 0; i < 2; i++) {
            Cell c = cells[i];
            if (!c.hasEstimate) continue;
            CellState s = effectiveState(c, cells[1 - i]);
            double d = Math.hypot(c.ex - pose.position.x, c.ey - pose.position.y);
            if (s == CellState.UP) up = c;
            if (s != CellState.DOWN && d < bestD) {
                bestD = d;
                best = c;
            }
            if (d < anyD) {
                anyD = d;
                any = c;
            }
        }
        return up != null ? up : best != null ? best : any;
    }

    private void retargetIfTipped() {
        if (!AUTO_RETARGET_ON_TIP || target == null) return;
        Cell other = target == cells[0] ? cells[1] : cells[0];
        if (other.hasEstimate && effectiveState(target, other) == CellState.DOWN) {
            target = other;
            aimPointSet = false;
            lastEvent = "CELL TIPPED - SWITCHED TO " + target.label;
        }
    }

    private static CellState effectiveState(Cell c, Cell other) {
        boolean ck = c.state != CellState.UNKNOWN, ok = other.state != CellState.UNKNOWN;
        if (ck && ok && c.state == other.state) {
            return c.lastSeenNanos >= other.lastSeenNanos ? c.state : opposite(other.state);
        }
        if (ck) return c.state;
        if (ok) return opposite(other.state);
        return CellState.UNKNOWN;
    }

    private static CellState opposite(CellState s) {
        return s == CellState.UP ? CellState.DOWN : s == CellState.DOWN ? CellState.UP : CellState.UNKNOWN;
    }

    // ---- vision -> estimate ------------------------------------------------------------------

    private void processVision() {
        if (aprilTag == null) return;
        // FRESH detections only: the same frame fed twice would make the filter over-confident.
        List<AprilTagDetection> dets = aprilTag.getFreshDetections();
        if (dets == null) return;
        for (Cell c : cells) {
            c.n = 0;
            c.upright = c.inverted = 0;
        }
        long frameNanos = 0;
        for (AprilTagDetection det : dets) {
            if (det.metadata == null || det.ftcPose == null) continue;
            Cell owner = null;
            for (Cell c : cells) if (c.owns(det.id)) owner = c;
            if (owner == null || owner.n >= 4) continue;   // not our alliance: dropped here
            double[] rb = cameraToRobot(det.ftcPose.x, det.ftcPose.y, det.ftcPose.z);
            owner.ids[owner.n] = det.id - owner.firstId;
            owner.fwd[owner.n] = rb[0];
            owner.left[owner.n] = rb[1];
            owner.n++;
            double roll = wrapDeg(det.ftcPose.roll - TurretConstants.CAMERA_MOUNT_ROLL_DEG);
            if (Math.abs(roll) < UPRIGHT_ROLL_LIMIT_DEG) owner.upright++;
            else owner.inverted++;
            frameNanos = det.frameAcquisitionNanoTime;
        }
        if (frameNanos == 0) return;
        lastFrameNanos = frameNanos;

        double[] cap = lookupPose(frameNanos);
        for (Cell c : cells) {
            if (c.n == 0) continue;
            c.lastSeenNanos = frameNanos;
            debounce(c, c.upright >= c.inverted ? CellState.UP : CellState.DOWN);
            if (c.inverted > c.upright) {
                c.lastResult = "upside down - not measured";
                continue;
            }
            if (cap == null) {
                c.lastResult = "frame older than pose history";
                continue;
            }
            if (Math.abs(Math.toDegrees(cap[3])) > MAX_TURN_RATE_FOR_VISION_DEG_S) {
                c.lastResult = "turning too fast (blur)";
                continue;
            }
            measure(c, cap);
        }
    }

    private void measure(Cell c, double[] cap) {
        double cx = cap[0], cy = cap[1], ch = Math.cos(cap[2]), sh = Math.sin(cap[2]);
        double[] fx = new double[4], fy = new double[4];
        for (int k = 0; k < c.n; k++) {
            fx[k] = cx + ch * c.fwd[k] - sh * c.left[k];
            fy[k] = cy + sh * c.fwd[k] + ch * c.left[k];
        }
        if (c.n == 4) {
            double mx = (fx[0] + fx[1] + fx[2] + fx[3]) / 4.0;
            double my = (fy[0] + fy[1] + fy[2] + fy[3]) / 4.0;
            for (int k = 0; k < 4; k++) {
                int id = c.ids[k];
                if (c.offN[id] < OFFSET_LEARN_SAMPLES) c.offN[id]++;
                double a = 1.0 / c.offN[id];
                c.offX[id] += a * ((fx[k] - mx) - c.offX[id]);
                c.offY[id] += a * ((fy[k] - my) - c.offY[id]);
            }
        }
        boolean allLearned = true;
        double zx = 0, zy = 0;
        for (int k = 0; k < c.n; k++) {
            int id = c.ids[k];
            boolean learned = c.offN[id] >= OFFSET_MIN_SAMPLES;
            allLearned &= learned;
            zx += fx[k] - (learned ? c.offX[id] : 0.0);
            zy += fy[k] - (learned ? c.offY[id] : 0.0);
        }
        zx = zx / c.n + c.sideSign * AIM_OFFSET_OUTWARD_IN;
        zy = zy / c.n + allianceWallSign * AIM_OFFSET_TOWARD_ALLIANCE_IN;

        double r = Math.hypot(zx - cx, zy - cy);
        if (r > MAX_MEASUREMENT_RANGE_IN || r < 1.0) {
            c.lastResult = "out of range";
            return;
        }
        double ux = (zx - cx) / r, uy = (zy - cy) / r;
        double sr = Math.max(RANGE_SIGMA_MIN_IN, RANGE_SIGMA_FRAC * r);
        double st = Math.max(LATERAL_SIGMA_MIN_IN, r * Math.toRadians(BEARING_SIGMA_DEG));
        double extra = (c.n < 4 && !allLearned) ? PARTIAL_CLUSTER_SIGMA_IN : 0.0;
        double sr2 = sr * sr, st2 = st * st, e2 = extra * extra;
        kalmanUpdate(c, zx, zy,
                sr2 * ux * ux + st2 * uy * uy + e2,
                (sr2 - st2) * ux * uy,
                sr2 * uy * uy + st2 * ux * ux + e2);
    }

    private void kalmanUpdate(Cell c, double zx, double zy, double rxx, double rxy, double ryy) {
        if (!c.hasEstimate) {
            initEstimate(c, zx, zy, rxx, rxy, ryy);
            c.lastResult = "acquired";
            return;
        }
        double sxx = c.pxx + rxx, sxy = c.pxy + rxy, syy = c.pyy + ryy;
        double det = sxx * syy - sxy * sxy;
        if (det <= 1e-12) return;
        double ixx = syy / det, ixy = -sxy / det, iyy = sxx / det;
        double vx = zx - c.ex, vy = zy - c.ey;
        double nis = vx * (ixx * vx + ixy * vy) + vy * (ixy * vx + iyy * vy);
        if (nis > GATE_CHI2) {
            c.rejects++;
            c.lastResult = "gated out (" + c.rejects + ")";
            if (c.rejects >= RESET_AFTER_REJECTS) {
                initEstimate(c, zx, zy, rxx, rxy, ryy);
                c.lastResult = "re-acquired";
            }
            return;
        }
        c.rejects = 0;
        c.lastResult = "tracking";
        double kxx = c.pxx * ixx + c.pxy * ixy, kxy = c.pxx * ixy + c.pxy * iyy;
        double kyx = c.pxy * ixx + c.pyy * ixy, kyy = c.pxy * ixy + c.pyy * iyy;
        c.ex += kxx * vx + kxy * vy;
        c.ey += kyx * vx + kyy * vy;
        double nxx = (1 - kxx) * c.pxx - kxy * c.pxy;
        double nxy = 0.5 * (((1 - kxx) * c.pxy - kxy * c.pyy) + (-kyx * c.pxx + (1 - kyy) * c.pxy));
        double nyy = -kyx * c.pxy + (1 - kyy) * c.pyy;
        c.pxx = nxx;
        c.pxy = nxy;
        c.pyy = nyy;
    }

    private static void initEstimate(Cell c, double x, double y, double pxx, double pxy, double pyy) {
        c.hasEstimate = true;
        c.ex = x;
        c.ey = y;
        c.pxx = pxx;
        c.pxy = pxy;
        c.pyy = pyy;
        c.rejects = 0;
    }

    /** SDK ftcPose (x right, y out, z up) to robot frame (x forward, y left, z up) through the mount. */
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

    private static void debounce(Cell cell, CellState seen) {
        if (seen == cell.state) {
            cell.pending = seen;
            cell.pendingFrames = 0;
            return;
        }
        if (seen == cell.pending) {
            cell.pendingFrames++;
        } else {
            cell.pending = seen;
            cell.pendingFrames = 1;
        }
        if (cell.pendingFrames >= TIP_CONFIRM_FRAMES) {
            cell.state = seen;
            cell.pendingFrames = 0;
        }
    }

    // ---- pose history ------------------------------------------------------------------------

    private void pushPose(long t, double x, double y, double h, double w) {
        histHead = (histHead + 1) % HISTORY;
        histT[histHead] = t;
        histX[histHead] = x;
        histY[histHead] = y;
        histH[histHead] = h;
        histW[histHead] = w;
        if (histCount < HISTORY) histCount++;
    }

    private double[] lookupPose(long t) {
        if (histCount == 0) return null;
        int newer = histHead;
        if (t >= histT[newer]) return new double[]{histX[newer], histY[newer], histH[newer], histW[newer]};
        for (int i = 1; i < histCount; i++) {
            int older = (histHead - i + HISTORY) % HISTORY;
            if (histT[older] <= t) {
                double a = (double) (t - histT[older]) / Math.max(1, histT[newer] - histT[older]);
                return new double[]{
                        histX[older] + a * (histX[newer] - histX[older]),
                        histY[older] + a * (histY[newer] - histY[older]),
                        histH[older] + a * wrap(histH[newer] - histH[older]),
                        histW[older] + a * (histW[newer] - histW[older])};
            }
            newer = older;
        }
        return null;
    }

    private static double yawRate(PoseVelocity2d v) {
        if (v == null || !Double.isFinite(v.angVel)) return 0.0;
        return wrap(v.angVel);
    }

    // ---- driving in physical units -----------------------------------------------------------

    /**
     * Robot-frame velocity (x forward, y left, in/s; w CCW rad/s) to wheel powers through Road
     * Runner's kinematics and kS/kV feedforward. Rotation is served first, translation gets the rest.
     */
    public static void driveRobotVelocity(MecaTank mecaTank, double vx, double vy, double w) {
        MecanumKinematics kin = mecaTank.drive.kinematics;
        double[] rot = wheels(kin, 0, 0, w);
        double[] tr = wheels(kin, vx, vy, 0);
        if (voltageTimer.milliseconds() > 250 || cachedVoltage <= 6.0) {
            double v = mecaTank.drive.voltageSensor.getVoltage();
            if (v > 6.0) cachedVoltage = v;
            voltageTimer.reset();
        }
        double voltage = cachedVoltage;
        double kVin = PARAMS.kV / PARAMS.inPerTick;
        double vMax = Math.max(1.0, WHEEL_SPEED_HEADROOM * (voltage - PARAMS.kS) / kVin);
        double rMax = 0;
        for (double v : rot) rMax = Math.max(rMax, Math.abs(v));
        if (rMax > vMax) for (int i = 0; i < 4; i++) rot[i] *= vMax / rMax;
        double k = 1.0;
        for (int i = 0; i < 4; i++) {
            if (tr[i] > 1e-9) k = Math.min(k, (vMax - rot[i]) / tr[i]);
            else if (tr[i] < -1e-9) k = Math.min(k, (-vMax - rot[i]) / tr[i]);
        }
        k = Range.clip(k, 0.0, 1.0);
        double[] p = new double[4];
        for (int i = 0; i < 4; i++) {
            double v = rot[i] + k * tr[i];
            double ks = Math.signum(v) * PARAMS.kS * Math.min(1.0, Math.abs(v) / Math.max(1e-9, KS_RAMP_IN_S));
            p[i] = (ks + kVin * v) / voltage;
        }
        // p is by WHEEL {lf, lb, rf, rb}; setWheelPowersGuarded takes motor PORTS {fl, bl, fr, br}.
        if (MecanumDrive.DRIVE_SIDES_SWAPPED) {
            mecaTank.setWheelPowersGuarded(-p[2], -p[3], -p[0], -p[1]);
        } else {
            mecaTank.setWheelPowersGuarded(p[0], p[1], p[2], p[3]);
        }
    }

    private static double[] wheels(MecanumKinematics kin, double vx, double vy, double w) {
        MecanumKinematics.WheelVelocities<Time> wv = kin.inverse(
                PoseVelocity2dDual.<Time>constant(new PoseVelocity2d(new Vector2d(vx, vy), w), 1));
        return new double[]{wv.leftFront.value(), wv.leftBack.value(),
                wv.rightFront.value(), wv.rightBack.value()};
    }

    // ---- camera ------------------------------------------------------------------------------

    private void applyCameraControls() {
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (MANUAL_EXPOSURE) {
                if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    long lo = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                    long hi = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                    exposure.setExposure(Math.max(lo, Math.min(hi, TurretConstants.EXPOSURE_MS)), TimeUnit.MILLISECONDS);
                }
                GainControl gain = visionPortal.getCameraControl(GainControl.class);
                if (gain != null) {
                    int want = TurretConstants.GAIN < 0 ? gain.getMaxGain() : TurretConstants.GAIN;
                    gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), want)));
                }
            } else if (exposure != null) {
                // Set auto EXPLICITLY - a UVC webcam keeps the last manual exposure until unplugged.
                for (ExposureControl.Mode mode : new ExposureControl.Mode[]{ExposureControl.Mode.AperturePriority,
                        ExposureControl.Mode.Auto, ExposureControl.Mode.ContinuousAuto}) {
                    if (exposure.isModeSupported(mode) && exposure.setMode(mode)) break;
                }
            }
        } catch (Exception ignored) {
        }
        cameraControlsApplied = true;
    }

    public String cameraState() {
        return visionPortal == null ? "NOT OPEN - check TurretConstants.WEBCAM_NAME" : String.valueOf(visionPortal.getCameraState());
    }

    /** Waits for a mid-open camera to settle before closing it. */
    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        if (visionPortal == null) return;
        long deadline = System.currentTimeMillis() + 1500;
        while (System.currentTimeMillis() < deadline) {
            VisionPortal.CameraState state = visionPortal.getCameraState();
            if (state == VisionPortal.CameraState.STREAMING || state == VisionPortal.CameraState.ERROR
                    || state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) break;
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        try {
            visionPortal.close();
        } catch (Exception ignored) {
        }
        visionPortal = null;
    }

    // ---- telemetry ---------------------------------------------------------------------------

    public void telemetry(Telemetry t) {
        if (hasTarget()) {
            t.addData("AIM", "%s  %s  err %.2f deg  %.0f in", onTarget ? "ON TARGET" : "aligning",
                    target.label, headingErrDeg, distanceIn);
        } else if (engaged) {
            t.addData("AIM", "SEARCHING - no %s CELL seen yet", red ? "RED" : "BLUE");
        }
        if (!lastEvent.isEmpty()) t.addData("   last", lastEvent);
        t.addData("   camera", "%s, last frame %s", cameraState(), lastFrameNanos == 0 ? "none"
                : String.format("%.0f ms ago", (System.nanoTime() - lastFrameNanos) / 1e6));
        for (int i = 0; i < 2; i++) {
            Cell c = cells[i];
            t.addData("   " + c.label, "%s  %s  %s", effectiveState(c, cells[1 - i]),
                    c.hasEstimate ? String.format("(%.1f, %.1f) +/-%.1f", c.ex, c.ey, c.sigmaIn()) : "no estimate",
                    c.lastResult);
        }
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
