package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ HIVE auto-align test: drive exactly like {@link driveTesting} (field centric), click the
 * RIGHT STICK to lock the chassis heading onto the nearest HIVE CELL, click it again to drive normally.
 *
 * <h2>Controls (gamepad1)</h2>
 * <pre>
 * INIT:   B = RED alliance, X = BLUE alliance (locks in, opens the camera)
 *
 * NORMAL DRIVE - identical to driveTesting:
 *   Left stick ........ translate (field centric)    Right stick X ... turn
 *   LB (hold) ......... precision                    RB (hold) ....... override
 *   Y field centric   X heading hold   A translation hold   Back/Options re-zero "forward"
 *
 * RIGHT STICK CLICK ... toggle AUTO-ALIGN on/off
 *
 * AUTO-ALIGN - heading is fully automatic, the RIGHT stick does all the moving:
 *   Right stick UP/DOWN ...... toward / away from the target (radius)
 *   Right stick LEFT/RIGHT ... orbit around the target (an arc - the robot keeps pointing at it)
 *   Left stick ............... ignored
 *   LB (hold) precision, RB (hold) skip the acceleration limit
 *   Lock-on, ON TARGET and CELL switches after a tip are shown at the TOP of telemetry
 *   (no rumble - it works on gamepads without rumble motors).
 * </pre>
 *
 * <h2>The other alliance's tags are never used</h2>
 * Exactly as in {@link BiobuzzHiveTagTest}: the alliance picked in INIT decides which eight IDs go
 * into the tag library (RED 30-37, BLUE 38-45), so the other alliance's tags get no metadata and no
 * pose, and every detection is additionally checked against our eight IDs and dropped otherwise -
 * before it can reach the estimate, the tip state or the target choice. Nothing is detected, and
 * auto-align cannot engage, until an alliance has been picked.
 *
 * <h2>How the aiming works, and why it is built this way</h2>
 * The obvious design - turn until the tag is centred in the image - is the one that cannot be
 * "exact and completely consistent". The camera gives 30 noisy frames a second, each ~50-100 ms
 * old, and none at all while the tags are out of view, motion-blurred or blocked by another robot.
 * A heading loop closed on that jitters, lags behind the motion, and stops working whenever vision
 * does. So the camera is not in the heading loop at all:
 * <ol>
 *   <li><b>Odometry runs the heading loop.</b> The two goBILDA dead wheels plus the IMU give a pose
 *       every loop (~100+ Hz) with essentially no noise. The required heading is recomputed every
 *       loop from {@code atan2(target - robot)}, so it is smooth, has no camera latency, and keeps
 *       working with the camera covered.</li>
 *   <li><b>The camera only measures WHERE THE TARGET IS</b>, in the same odometry frame. Each frame's
 *       tags are pushed through the camera mount into the robot frame, then through the robot pose
 *       AT THE MOMENT THE FRAME WAS CAPTURED (a pose history buffer, matched on
 *       {@code frameAcquisitionNanoTime}) into a field point. Using the current pose instead would
 *       add the robot's motion during the camera latency straight into the target position.</li>
 *   <li><b>Each CELL's position is a 2-D Kalman filter.</b> A CELL does not move while it is up, so
 *       every frame from every viewpoint adds information. Monocular AprilTag range is far noisier
 *       than bearing, so each measurement is given a stretched uncertainty ellipse along the line of
 *       sight; measurements from different angles triangulate, which a plain low-pass cannot do. A
 *       chi-squared gate rejects outliers (a bad decode, a CELL mid-tip). Small process noise lets the
 *       estimate follow slow odometry drift, which is how vision keeps the dead wheels honest.</li>
 *   <li><b>Arc driving.</b> The right stick is mapped to radial and tangential speed around the target.
 *       Releasing the radial axis latches the current radius and holds it, so a pure sideways push is
 *       a true circle instead of a slow outward spiral. The heading loop gets the rate the bearing
 *       is about to change at (from the commanded velocity) as a feedforward, so it does not trail
 *       behind while orbiting.</li>
 *   <li><b>Actuation in physical units.</b> Aim mode commands in/s and rad/s through Road Runner's
 *       mecanum kinematics and its tuned kS/kV feedforward, with live voltage - the same model the
 *       autos run on, so its signs and scaling are already validated. Rotation gets priority over
 *       translation for wheel speed, so the aim never saturates away. Wheel powers still go through
 *       MecaTank's belt-drive backstop.</li>
 * </ol>
 *
 * <h2>Dead wheels in TeleOp: research conclusion</h2>
 * They were already running in TeleOp. {@link MecaTank} builds the Road Runner {@code MecanumDrive},
 * whose localizer is {@code TwoDeadWheelLocalizer}, and driveTesting's field centric reads its pose
 * every loop. This opmode simply depends on it, and it lowers noise rather than adding it:
 * <ul>
 *   <li>Encoder noise is ~0.003 in/tick (inPerTick) - unmeasurably small. Dead wheels are unpowered,
 *       so drive-wheel slip, pushing and spin-outs do not corrupt them.</li>
 *   <li>The real weaknesses are slow: IMU heading drift and lateral pod scrub over a whole match.
 *       Inches over minutes, never jitter. That is exactly what the vision filter removes, because
 *       the target estimate absorbs any drift between "where odometry says the CELL is" and where
 *       it really is.</li>
 *   <li>Vision alone, by contrast, is 30 Hz, laggy and intermittent. The two are complementary,
 *       which is why the fusion goes odometry-for-rate, vision-for-truth.</li>
 *   <li>Odometry is deliberately NOT relocalized from the tags. The tags ride on the CELLs, which
 *       MOVE when they tip; pulling the robot pose toward them would inject that motion into the
 *       pose every auto path relies on. Correcting the target estimate instead leaves odometry
 *       untouched, and for aiming only the target-minus-robot vector matters anyway.</li>
 * </ul>
 *
 * <h2>Where each camera constant goes</h2>
 * Everything about the CAMERA lives in {@link TurretConstants} - the same constants
 * {@link BiobuzzHiveTagTest} uses, so one measurement fixes both opmodes. All are dashboard-editable
 * under TurretConstants. The camera must be fixed to the CHASSIS (not on a turret) for this test.
 * <pre>
 * TurretConstants.WEBCAM_NAME .............. Driver Station config name of the webcam
 * TurretConstants.STREAM_WIDTH / HEIGHT .... 640 x 480 (the resolution the intrinsics are for)
 * TurretConstants.CAMERA_FX, FY, CX, CY .... lens intrinsics, pixels, at that resolution
 * TurretConstants.APRILTAG_DECIMATION ...... 2 (lower = longer range, slower)
 * TurretConstants.CAMERA_OFFSET_X_IN ....... lens position from ROBOT CENTRE, + = forward
 * TurretConstants.CAMERA_OFFSET_Y_IN ....... + = LEFT
 * TurretConstants.CAMERA_OFFSET_Z_IN ....... + = up (only affects the height readout)
 * TurretConstants.CAMERA_MOUNT_YAW_DEG ..... + = camera turned LEFT        (MOST critical)
 * TurretConstants.CAMERA_MOUNT_PITCH_DEG ... + = camera tilted UP          (the tags face DOWN from
 *                                            under the CELL, so expect a sizeable positive pitch)
 * TurretConstants.CAMERA_MOUNT_ROLL_DEG .... + = clockwise seen from behind the camera
 * TurretConstants.EXPOSURE_MS, GAIN ........ only used when MANUAL_EXPOSURE (below) is true
 * </pre>
 * "Robot centre" is the point the Road Runner pose tracks (the centre of rotation the dead-wheel
 * offsets parYTicks/perpXTicks were tuned against). Tag size is {@link #TAG_SIZE_IN} below.
 * The SHOOTER's position is separate: {@link #LAUNCHER_OFFSET_Y_IN} and friends, below.
 *
 * <h2>Calibration, in this order</h2>
 * <ol>
 *   <li>Measure the camera mount above. Then park the robot still, facing a CELL, and drive in a
 *       slow full orbit: the "target estimate" should stay put to within about an inch. If it
 *       swings with the robot's angle, the mount YAW or offset is wrong.</li>
 *   <li>Set {@link #LAUNCHER_OFFSET_Y_IN} / {@link #LAUNCHER_YAW_DEG} for where the shooter really
 *       points, then shoot from a few distances. A constant left/right miss at every distance is
 *       LAUNCHER_YAW_DEG; a miss that grows as you get closer is LAUNCHER_OFFSET_Y_IN.</li>
 *   <li>If balls land consistently off the opening centre even with a perfect heading, the tag
 *       cluster is not directly under the opening: use {@link #AIM_OFFSET_OUTWARD_IN} and
 *       {@link #AIM_OFFSET_TOWARD_ALLIANCE_IN}.</li>
 * </ol>
 *
 * <h2>Choosing the target</h2>
 * On engage, the closest of OUR alliance's two CELLS - but a CELL known to be tipped down is skipped
 * while {@link #PREFER_SCORABLE} is on, since it cannot be scored. The choice is then held for the
 * whole engagement so the aim never wanders between CELLS on noise. The only automatic switch is a
 * confirmed tip of the locked CELL ({@link #AUTO_RETARGET_ON_TIP}). Click twice to re-pick.
 *
 * <h2>Field frame</h2>
 * If an auto stored its end pose ({@link PoseStorage}) the pose is field-referenced and the CELL
 * estimates start at the surveyed opening coordinates in TurretConstants, so the robot can aim
 * before the camera has seen anything. Otherwise the pose starts at (0,0,0), the surveyed points
 * mean nothing in that frame, and each CELL is acquired from its first good sighting instead -
 * still exact, it just needs the camera to see the CELL once.
 */
@Config
@TeleOp(name = "BIOBUZZ Hive Auto Align Test", group = "testing")
public class BiobuzzHiveAutoAlignTest extends LinearOpMode {

    // ============================================================================================
    // SHOOTER GEOMETRY - where the ball actually leaves the robot
    // ============================================================================================

    /** Launcher exit point from robot centre, INCHES, + = LEFT. A miss that grows when close. */
    public static double LAUNCHER_OFFSET_Y_IN = 0.0;
    /** Launcher exit point from robot centre, INCHES, + = forward. Only matters with a yaw. */
    public static double LAUNCHER_OFFSET_X_IN = 0.0;
    /** Shot direction relative to robot forward, DEGREES, + = left. 180 for a rear shooter. */
    public static double LAUNCHER_YAW_DEG = 0.0;

    // ============================================================================================
    // TARGET - the aim point relative to the tag cluster
    // ============================================================================================

    /**
     * Aim point minus tag-cluster centre, INCHES, away from the HIVE's centre plane (toward the
     * CELL's own side of the field). Mirrored automatically for the other CELL and alliance.
     */
    public static double AIM_OFFSET_OUTWARD_IN = 0.0;
    /** Same, toward our own alliance wall. */
    public static double AIM_OFFSET_TOWARD_ALLIANCE_IN = 0.0;

    /** Skip a CELL known to be tipped down when picking the closest. */
    public static boolean PREFER_SCORABLE = true;
    /** Switch to the other CELL when the locked one is confirmed tipped down. */
    public static boolean AUTO_RETARGET_ON_TIP = true;

    // ============================================================================================
    // HEADING CONTROL
    // ============================================================================================

    /** rad/s of turn per rad of error. 6 is the Road Runner headingGain the autos already use. */
    public static double HEADING_KP = 6.0;
    /** Errors below this are left alone, DEGREES - stops the drive buzzing on sensor LSBs. */
    public static double HEADING_DEADBAND_DEG = 0.2;
    /** Turn-rate cap, rad/s. */
    public static double MAX_TURN_RATE = 4.0;
    /** Feed the bearing's rate of change forward so the aim does not trail while orbiting. */
    public static boolean TURN_FEEDFORWARD = true;
    /** |error| below this reads ON TARGET, DEGREES. */
    public static double ON_TARGET_DEG = 1.0;

    // ============================================================================================
    // DRIVING WHILE ALIGNED
    // ============================================================================================

    /** Full-stick speed, in/s. */
    public static double ALIGN_MAX_SPEED_IN_S = 40.0;
    /** Acceleration limit on the commanded velocity, in/s^2. RB skips it. */
    public static double ALIGN_ACCEL_IN_S2 = 70.0;
    /** Stick deadband and cubic blend (0 = linear, 1 = pure cubic). */
    public static double STICK_DEADBAND = 0.04;
    public static double STICK_CURVE = 0.4;
    /** Hold the radius when the stick's radial axis is released, so orbits are true circles. */
    public static boolean RADIUS_HOLD = true;
    /** in/s of radial correction per inch of radius error. */
    public static double RADIUS_HOLD_KP = 2.5;
    public static double RADIUS_HOLD_MAX_IN_S = 15.0;
    /** No driving toward the target closer than this, INCHES (keeps the robot off the HIVE). */
    public static double MIN_RADIUS_IN = 30.0;
    /** kS is ramped in over this wheel speed, in/s, so tiny corrections do not chatter. */
    public static double KS_RAMP_IN_S = 2.0;
    /** Fraction of the theoretical wheel speed actually used, leaving room for the feedback. */
    public static double WHEEL_SPEED_HEADROOM = 0.9;

    // ============================================================================================
    // VISION AND TARGET ESTIMATE
    // ============================================================================================

    /** BIOBUZZ HIVE tag edge length, INCHES (manual 9.9). */
    public static double TAG_SIZE_IN = 3.25;
    /** Frames in a row that must agree before a CELL's up/down state changes. */
    public static int TIP_CONFIRM_FRAMES = 3;
    /** |roll| below this, DEGREES, is right-side up (CELL up). FIRST's guidance is 90. */
    public static double UPRIGHT_ROLL_LIMIT_DEG = 90.0;
    /**
     * Manual exposure. OFF by default for the same reason as in BiobuzzHiveTagTest: 3 ms is
     * near-black in an ordinary room. Turn it ON under field lighting - short exposure is what keeps
     * tags decoding while the robot turns.
     */
    public static boolean MANUAL_EXPOSURE = false;
    public static int DASHBOARD_FPS = 10;

    /** Range noise, 1 sigma, as a fraction of range plus a floor. Monocular range is the weak axis. */
    public static double RANGE_SIGMA_FRAC = 0.03;
    public static double RANGE_SIGMA_MIN_IN = 0.5;
    /** Bearing noise, 1 sigma, DEGREES. Bearing is the strong axis. */
    public static double BEARING_SIGMA_DEG = 0.4;
    public static double LATERAL_SIGMA_MIN_IN = 0.25;
    /** Extra 1-sigma noise for a partial cluster whose tag offsets have not been learned yet, INCHES. */
    public static double PARTIAL_CLUSTER_SIGMA_IN = 1.5;
    /** Random walk allowed for the estimate, in per sqrt(s) - lets vision absorb odometry drift. */
    public static double PROCESS_NOISE_IN = 0.3;
    /** Uncertainty of the surveyed opening coordinates, 1 sigma, INCHES. */
    public static double PRIOR_SIGMA_IN = 3.0;
    /** Start from the surveyed coordinates when the pose is field-referenced. */
    public static boolean USE_SURVEYED_PRIOR = true;
    /** Chi-squared gate, 2 DOF. 13.8 = 99.9%. */
    public static double GATE_CHI2 = 13.8;
    /** This many gated-out frames IN A ROW means the estimate is wrong, not the camera: re-acquire. */
    public static int RESET_AFTER_REJECTS = 15;
    /** Tags further than this are ignored, INCHES. */
    public static double MAX_MEASUREMENT_RANGE_IN = 144.0;
    /** Frames captured while turning faster than this are ignored (blur), DEG/S. */
    public static double MAX_TURN_RATE_FOR_VISION_DEG_S = 150.0;
    /** Full-cluster sightings averaged into each tag's learned offset. */
    public static int OFFSET_LEARN_SAMPLES = 60;
    /** Sightings a tag's offset needs before it is trusted. */
    public static int OFFSET_MIN_SAMPLES = 5;

    // ============================================================================================
    // START POSE
    // ============================================================================================

    /** Use the pose the last auto stored in PoseStorage (field-referenced aiming from the start). */
    public static boolean USE_AUTO_END_POSE = true;
    /** Or place the robot at a known spot and set it here (used only if no auto pose is used). */
    public static boolean USE_MANUAL_START_POSE = false;
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_DEG = 90.0;

    public static double TELEMETRY_INTERVAL_MS = 200.0;

    // ============================================================================================

    private enum Alliance { NONE, RED, BLUE }

    private enum CellState { UNKNOWN, UP, DOWN }

    /** One CELL: its tag IDs, tip state, learned tag layout and the Kalman estimate of its aim point. */
    private static class Cell {
        final String label;
        final int firstId;
        final double surveyX, surveyY;
        /** +1 when the CELL sits at x > 0 (audience side), -1 otherwise. */
        final double sideSign;

        CellState state = CellState.UNKNOWN;
        CellState pending = CellState.UNKNOWN;
        int pendingFrames = 0;
        long lastSeenNanos = 0;

        // Aim-point estimate in the odometry frame, inches, and its covariance.
        boolean hasEstimate = false;
        double ex, ey, pxx, pxy, pyy;
        int rejects = 0, resets = 0, accepted = 0;
        double lastNis = 0;
        String lastResult = "never seen";

        // Each tag's offset from the cluster centre (odometry frame), learned from full sightings.
        final double[] offX = new double[4], offY = new double[4];
        final int[] offN = new int[4];

        // Per-frame scratch
        int n;
        final int[] ids = new int[4];
        final double[] fwd = new double[4], left = new double[4], up = new double[4];
        int upright, inverted;
        double lastTagHeightIn, lastRangeIn;
        int lastTagsSeen;

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

    private MecaTank mecaTank;
    private BulkRead bulkRead;
    private Alliance alliance = Alliance.NONE;
    private Cell[] cells;
    private double allianceWallSign;   // -1 red (y < 0), +1 blue
    private boolean fieldReferenced = false;
    private String poseSource = "none";

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean cameraControlsApplied = false;
    private String cameraReport = "not applied yet";
    private long lastFrameNanos = 0;
    private int framesUsed = 0;
    private String lastVisionNote = "";

    // Pose history for latency compensation: time, x, y, heading, yaw rate.
    private static final int HISTORY = 128;
    private final long[] histT = new long[HISTORY];
    private final double[] histX = new double[HISTORY], histY = new double[HISTORY];
    private final double[] histH = new double[HISTORY], histW = new double[HISTORY];
    private int histHead = -1, histCount = 0;

    // Aim mode state
    private boolean aligning = false;
    private boolean directDriving = false;   // aim controller (true) or MecaTank searching path
    private Cell target = null;
    private double cmdVx = 0, cmdVy = 0;     // commanded FIELD velocity, in/s
    private boolean radiusLatched = false;
    private double radiusHold = 0;
    private double headingErrDeg = 0, distanceIn = 0, turnCmd = 0;
    private boolean onTarget = false;

    /** Latest lock-on / retarget message, shown at the top of telemetry for EVENT_SHOW_S. */
    private String lastEvent = "";
    private final ElapsedTime eventTimer = new ElapsedTime();
    private static final double EVENT_SHOW_S = 3.0;

    private double cachedVoltage = 12.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevA, prevOptions, prevRsb;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mecaTank = new MecaTank(hardwareMap, telemetry, new Pose2d(0, 0, 0));
        // Same bulk-read arrangement as driveTesting: one clearCache() per loop.
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();
        mecaTank.setAutoPoseUpdate(false);
        MecaTank.FIELD_CENTRIC = true;

        // ---- INIT: alliance pick opens the camera (the DS preview only works during INIT) --------
        while (!isStarted() && !isStopRequested()) {
            if (alliance == Alliance.NONE) {
                if (gamepad1.b) selectAlliance(Alliance.RED);
                if (gamepad1.x) selectAlliance(Alliance.BLUE);
            }
            telemetry.addLine("BIOBUZZ HIVE AUTO ALIGN TEST");
            if (alliance == Alliance.NONE) {
                telemetry.addLine("Which alliance?   gamepad1 B = RED   X = BLUE");
            } else {
                telemetry.addData("Alliance", "%s (restart the opmode to change)", alliance);
                telemetry.addData("Camera", visionPortal == null ? "FAILED TO OPEN - check the config name"
                        : String.valueOf(visionPortal.getCameraState()));
                telemetry.addData("Pose", "%s%s", poseSource,
                        fieldReferenced ? "  (field-referenced)" : "  (NOT field-referenced: CELLS acquired by camera)");
                telemetry.addLine("Right stick click = toggle auto-align after START");
            }
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            shutdownVision();
            return;
        }
        while (alliance == Alliance.NONE && opModeIsActive()) {
            if (gamepad1.b) selectAlliance(Alliance.RED);
            if (gamepad1.x) selectAlliance(Alliance.BLUE);
            telemetry.addLine("NO ALLIANCE SELECTED - gamepad1 B = RED, X = BLUE");
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) {
            shutdownVision();
            return;
        }

        // "Forward" for the driver = wherever the robot points at START, exactly like driveTesting.
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();

        telemetryTimer.reset();
        long lastNanos = System.nanoTime();
        double loopMsMax = 0;

        while (opModeIsActive()) {
            bulkRead.clearCache();
            long now = System.nanoTime();
            double dt = Range.clip((now - lastNanos) / 1e9, 1e-4, 0.1);
            lastNanos = now;
            if (dt * 1000.0 > loopMsMax) loopMsMax = dt * 1000.0;

            if (visionPortal != null && !cameraControlsApplied
                    && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                applyCameraControls();
            }

            // Every loop, unconditionally: the aim, the pose history and field centric all need it.
            mecaTank.updatePoseEstimate();
            Pose2d pose = mecaTank.getPoseEstimate();
            PoseVelocity2d velRobot = mecaTank.getMeasuredRobotVelocity();
            double heading = pose.heading.toDouble();
            pushPose(now, pose.position.x, pose.position.y, heading, yawRate(velRobot));

            // Kalman predict: the CELL is still, but the odometry frame drifts slowly under it.
            double q = PROCESS_NOISE_IN * PROCESS_NOISE_IN * dt;
            for (Cell c : cells) {
                if (c.hasEstimate) {
                    c.pxx += q;
                    c.pyy += q;
                }
            }
            processVision();

            // ---- buttons ---------------------------------------------------------------------
            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            if (gamepad1.a && !prevA) MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();
            if (gamepad1.options && !prevOptions) mecaTank.resetDriveHeading();
            boolean rsbPressed = gamepad1.right_stick_button && !prevRsb;
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevA = gamepad1.a;
            prevBack = gamepad1.back;
            prevOptions = gamepad1.options;
            prevRsb = gamepad1.right_stick_button;

            if (rsbPressed) {
                if (aligning) {
                    aligning = false;
                    directDriving = false;
                    target = null;
                    // Hand back to the smooth drive from the robot's real motion - no lurch, and
                    // heading hold re-latches where the robot now points instead of snapping back.
                    mecaTank.resetSmoothDriveState(velRobot);
                } else {
                    aligning = true;
                    directDriving = false;
                    target = pickTarget(pose);
                    onTarget = false;
                }
            }

            // ---- drive -----------------------------------------------------------------------
            if (!aligning) {
                // Exactly driveTesting.
                mecaTank.setDrivePowersSmoothArcade(
                        gamepad1.left_stick_x, gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_bumper, gamepad1.right_bumper);
            } else {
                if (target == null) target = pickTarget(pose);   // keep looking until one is seen
                if (target == null) {
                    // SEARCHING: nothing to aim at yet. Right stick translates through the normal
                    // field-centric path, no rotation, until a CELL is acquired.
                    directDriving = false;
                    mecaTank.setDrivePowersSmoothArcade(
                            gamepad1.right_stick_x, gamepad1.right_stick_y, 0.0,
                            gamepad1.left_bumper, gamepad1.right_bumper);
                } else {
                    if (!directDriving) {
                        // Start the velocity command from what the robot is really doing.
                        double c = Math.cos(heading), s = Math.sin(heading);
                        double vx = velRobot == null ? 0 : velRobot.linearVel.x;
                        double vy = velRobot == null ? 0 : velRobot.linearVel.y;
                        cmdVx = c * vx - s * vy;
                        cmdVy = s * vx + c * vy;
                        radiusLatched = false;
                        directDriving = true;
                        showEvent("LOCKED ON " + target.label);
                    }
                    retargetIfTipped();
                    runAlign(pose, dt);
                }
            }

            // ---- telemetry -------------------------------------------------------------------
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                sendTelemetry(pose, loopMsMax);
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }

        mecaTank.setWheelPowersGuarded(0, 0, 0, 0);
        shutdownVision();
    }

    // ============================================================================================
    // AIM CONTROLLER
    // ============================================================================================

    private void runAlign(Pose2d pose, double dt) {
        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        double dx = target.ex - x, dy = target.ey - y;
        double r = Math.max(1e-6, Math.hypot(dx, dy));
        distanceIn = r;
        double ux = dx / r, uy = dy / r;   // toward the target
        double tx = uy, ty = -ux;          // stick RIGHT: clockwise orbit seen from above

        // ---- translation: radial + tangential, radius held when the radial axis is released ----
        double speed = ALIGN_MAX_SPEED_IN_S * (gamepad1.left_bumper ? MecaTank.PRECISION_SCALE : 1.0);
        double radialIn = shape(-gamepad1.right_stick_y);
        double tangentIn = shape(gamepad1.right_stick_x);
        double vr = radialIn * speed;
        double vt = tangentIn * speed;

        if (RADIUS_HOLD && radialIn == 0.0) {
            if (!radiusLatched) {
                radiusHold = r;
                radiusLatched = true;
            }
            vr += Range.clip(RADIUS_HOLD_KP * (r - radiusHold), -RADIUS_HOLD_MAX_IN_S, RADIUS_HOLD_MAX_IN_S);
        } else {
            radiusLatched = false;
        }
        if (r < MIN_RADIUS_IN && vr > 0) vr = 0;

        double wantVx = vr * ux + vt * tx;
        double wantVy = vr * uy + vt * ty;
        double mag = Math.hypot(wantVx, wantVy);
        if (mag > speed && mag > 1e-9) {
            wantVx *= speed / mag;
            wantVy *= speed / mag;
        }

        // Vector slew, so the direction of travel is not distorted while accelerating.
        if (gamepad1.right_bumper) {
            cmdVx = wantVx;
            cmdVy = wantVy;
        } else {
            double ddx = wantVx - cmdVx, ddy = wantVy - cmdVy;
            double d = Math.hypot(ddx, ddy), step = ALIGN_ACCEL_IN_S2 * dt;
            if (d > step) {
                ddx *= step / d;
                ddy *= step / d;
            }
            cmdVx += ddx;
            cmdVy += ddy;
        }

        // ---- heading: point the SHOT LINE (not the robot centre) through the target ----------
        // In the robot frame, a line leaving (lx, ly) at angle psi passes through a point at range
        // r and relative bearing phi iff r*sin(phi - psi) = ly*cos(psi) - lx*sin(psi).
        double psi = Math.toRadians(LAUNCHER_YAW_DEG);
        double lateral = LAUNCHER_OFFSET_Y_IN * Math.cos(psi) - LAUNCHER_OFFSET_X_IN * Math.sin(psi);
        double phi = psi + (Math.abs(lateral) < r ? Math.asin(lateral / r) : 0.0);
        double desired = Math.atan2(dy, dx) - phi;
        double err = wrap(desired - h);
        headingErrDeg = Math.toDegrees(err);

        // d(bearing)/dt from the commanded velocity: the turn the motion is about to require.
        double ff = TURN_FEEDFORWARD ? (dy * cmdVx - dx * cmdVy) / (r * r) : 0.0;
        double fb = Math.abs(headingErrDeg) < HEADING_DEADBAND_DEG ? 0.0 : HEADING_KP * err;
        turnCmd = Range.clip(ff + fb, -MAX_TURN_RATE, MAX_TURN_RATE);

        boolean nowOn = Math.abs(headingErrDeg) < ON_TARGET_DEG;
        onTarget = nowOn;

        double c = Math.cos(h), s = Math.sin(h);
        driveRobotVelocity(c * cmdVx + s * cmdVy, -s * cmdVx + c * cmdVy, turnCmd);
    }

    /**
     * Robot-frame velocity (x forward, y left, in/s; w CCW, rad/s) to motor powers through Road
     * Runner's kinematics and feedforward. Rotation is served first; translation gets what is left.
     */
    private void driveRobotVelocity(double vx, double vy, double w) {
        MecanumKinematics kin = mecaTank.drive.kinematics;
        double[] rot = wheels(kin, 0, 0, w);
        double[] tr = wheels(kin, vx, vy, 0);

        double voltage = voltage();
        double kVin = PARAMS.kV / PARAMS.inPerTick;   // volts per in/s
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

    /** Wheel speeds, in/s, ordered {frontLeft, backLeft, frontRight, backRight} like MecaTank. */
    private static double[] wheels(MecanumKinematics kin, double vx, double vy, double w) {
        MecanumKinematics.WheelVelocities<Time> wv = kin.inverse(
                PoseVelocity2dDual.<Time>constant(new PoseVelocity2d(new Vector2d(vx, vy), w), 1));
        return new double[]{wv.leftFront.value(), wv.leftBack.value(),
                wv.rightFront.value(), wv.rightBack.value()};
    }

    private double voltage() {
        if (voltageTimer.milliseconds() > 250 || cachedVoltage <= 6.0) {
            double v = mecaTank.drive.voltageSensor.getVoltage();
            if (v > 6.0) cachedVoltage = v;
            voltageTimer.reset();
        }
        return cachedVoltage;
    }

    private static double shape(double x) {
        double mag = Math.abs(x);
        if (mag < STICK_DEADBAND) return 0.0;
        double s = Math.min(1.0, (mag - STICK_DEADBAND) / (1.0 - STICK_DEADBAND));
        return Math.copySign(STICK_CURVE * s * s * s + (1.0 - STICK_CURVE) * s, x);
    }

    // ============================================================================================
    // TARGET CHOICE
    // ============================================================================================

    /** Closest CELL with an estimate, skipping one known to be tipped down if PREFER_SCORABLE. */
    private Cell pickTarget(Pose2d pose) {
        Cell best = null, bestAny = null;
        double bestD = Double.MAX_VALUE, bestAnyD = Double.MAX_VALUE;
        for (int i = 0; i < 2; i++) {
            Cell c = cells[i];
            if (!c.hasEstimate) continue;
            double d = Math.hypot(c.ex - pose.position.x, c.ey - pose.position.y);
            if (d < bestAnyD) {
                bestAnyD = d;
                bestAny = c;
            }
            if (effectiveState(c, cells[1 - i]) != CellState.DOWN && d < bestD) {
                bestD = d;
                best = c;
            }
        }
        return PREFER_SCORABLE && best != null ? best : bestAny;
    }

    private void retargetIfTipped() {
        if (!AUTO_RETARGET_ON_TIP || target == null) return;
        Cell other = target == cells[0] ? cells[1] : cells[0];
        if (other.hasEstimate && effectiveState(target, other) == CellState.DOWN) {
            target = other;
            radiusLatched = false;
            showEvent("CELL TIPPED - SWITCHED TO " + target.label);
        }
    }

    /**
     * Exactly one CELL per HIVE is up. A CELL's own sighting wins; otherwise it is inferred from the
     * other one. If both claim the same state, the more recent sighting is believed - a CELL that
     * tips away usually just stops being visible, so its stale "UP" would otherwise linger.
     */
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

    // ============================================================================================
    // VISION -> TARGET ESTIMATE
    // ============================================================================================

    private void processVision() {
        if (aprilTag == null) return;
        // FRESH detections only. getDetections() returns the same frame on every loop until the
        // next one arrives, and feeding one frame to the filter ten times would make it ten times
        // over-confident in it.
        List<AprilTagDetection> dets = aprilTag.getFreshDetections();
        if (dets == null) return;

        for (Cell c : cells) {
            c.n = 0;
            c.upright = c.inverted = 0;
        }
        long frameNanos = 0;
        for (AprilTagDetection det : dets) {
            // Only our alliance's eight IDs; everything else is dropped before any maths.
            if (det.metadata == null || det.ftcPose == null) continue;
            Cell owner = null;
            for (Cell c : cells) if (c.owns(det.id)) owner = c;
            if (owner == null || owner.n >= 4) continue;

            double[] rb = cameraToRobot(det.ftcPose.x, det.ftcPose.y, det.ftcPose.z);
            owner.ids[owner.n] = det.id - owner.firstId;
            owner.fwd[owner.n] = rb[0];
            owner.left[owner.n] = rb[1];
            owner.up[owner.n] = rb[2];
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
            c.lastTagsSeen = c.n;
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

    /** One CELL's tags in this frame -> one aim-point measurement -> Kalman update. */
    private void measure(Cell c, double[] cap) {
        double cx = cap[0], cy = cap[1], ch = Math.cos(cap[2]), sh = Math.sin(cap[2]);
        double[] fx = new double[4], fy = new double[4];
        double heightSum = 0;
        for (int k = 0; k < c.n; k++) {
            fx[k] = cx + ch * c.fwd[k] - sh * c.left[k];
            fy[k] = cy + sh * c.fwd[k] + ch * c.left[k];
            heightSum += c.up[k];
        }
        c.lastTagHeightIn = heightSum / c.n;

        // Learn the cluster layout from full sightings, so a partly visible cluster can still be
        // centred exactly instead of shifting by inches as tags come and go.
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
        c.lastRangeIn = r;
        if (r > MAX_MEASUREMENT_RANGE_IN || r < 1.0) {
            c.lastResult = "out of range";
            return;
        }

        // Measurement covariance: long along the line of sight (range), thin across it (bearing).
        double ux = (zx - cx) / r, uy = (zy - cy) / r;
        double sr = Math.max(RANGE_SIGMA_MIN_IN, RANGE_SIGMA_FRAC * r);
        double st = Math.max(LATERAL_SIGMA_MIN_IN, r * Math.toRadians(BEARING_SIGMA_DEG));
        double extra = (c.n < 4 && !allLearned) ? PARTIAL_CLUSTER_SIGMA_IN : 0.0;
        double sr2 = sr * sr, st2 = st * st, e2 = extra * extra;
        double rxx = sr2 * ux * ux + st2 * uy * uy + e2;
        double rxy = (sr2 - st2) * ux * uy;
        double ryy = sr2 * uy * uy + st2 * ux * ux + e2;

        kalmanUpdate(c, zx, zy, rxx, rxy, ryy);
        framesUsed++;
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
        c.lastNis = nis;

        if (nis > GATE_CHI2) {
            c.rejects++;
            c.lastResult = "gated out (" + c.rejects + ")";
            if (c.rejects >= RESET_AFTER_REJECTS) {
                // Consistently disagreeing for this long: the estimate is what is wrong.
                initEstimate(c, zx, zy, rxx, rxy, ryy);
                c.resets++;
                c.lastResult = "re-acquired";
            }
            return;
        }
        c.rejects = 0;
        c.accepted++;
        c.lastResult = "accepted";

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

    /**
     * Camera frame (SDK ftcPose: x right, y out of the lens, z up) to robot frame (x forward,
     * y left, z up), through the mount roll, pitch, yaw and offset. Same maths as BiobuzzHiveTagTest.
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

        return new double[]{
                f3 + TurretConstants.CAMERA_OFFSET_X_IN,
                l3 + TurretConstants.CAMERA_OFFSET_Y_IN,
                u2 + TurretConstants.CAMERA_OFFSET_Z_IN
        };
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

    // ============================================================================================
    // POSE HISTORY (latency compensation)
    // ============================================================================================

    private void pushPose(long t, double x, double y, double h, double w) {
        histHead = (histHead + 1) % HISTORY;
        histT[histHead] = t;
        histX[histHead] = x;
        histY[histHead] = y;
        histH[histHead] = h;
        histW[histHead] = w;
        if (histCount < HISTORY) histCount++;
    }

    /** {x, y, heading, yawRate} at time t, interpolated; null if t is older than the history. */
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

    /** Gyro yaw rate with the localizer's issue-#617 multiple-of-2pi offset folded away. */
    private static double yawRate(PoseVelocity2d v) {
        if (v == null || !Double.isFinite(v.angVel)) return 0.0;
        return wrap(v.angVel);
    }

    // ============================================================================================
    // SETUP
    // ============================================================================================

    private void selectAlliance(Alliance a) {
        alliance = a;
        boolean red = a == Alliance.RED;
        allianceWallSign = red ? -1.0 : 1.0;
        // Audience is the x = +72 wall: the audience-side CELL is the x > 0 one.
        cells = red
                ? new Cell[]{
                        new Cell("RED SCORING-SIDE CELL", 30, TurretConstants.RED_LEFT_X_IN, TurretConstants.RED_LEFT_Y_IN),
                        new Cell("RED AUDIENCE-SIDE CELL", 34, TurretConstants.RED_RIGHT_X_IN, TurretConstants.RED_RIGHT_Y_IN)}
                : new Cell[]{
                        new Cell("BLUE AUDIENCE-SIDE CELL", 38, TurretConstants.BLUE_RIGHT_X_IN, TurretConstants.BLUE_RIGHT_Y_IN),
                        new Cell("BLUE SCORING-SIDE CELL", 42, TurretConstants.BLUE_LEFT_X_IN, TurretConstants.BLUE_LEFT_Y_IN)};

        Pose2d stored = red ? PoseStorage.getLastRedPose() : PoseStorage.getLastBluePose();
        if (USE_AUTO_END_POSE && stored != null) {
            mecaTank.setPoseEstimate(stored);
            fieldReferenced = true;
            poseSource = String.format("auto end pose (%.1f, %.1f, %.1f deg)",
                    stored.position.x, stored.position.y, Math.toDegrees(stored.heading.toDouble()));
        } else if (USE_MANUAL_START_POSE) {
            mecaTank.setPoseEstimate(new Pose2d(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
            fieldReferenced = true;
            poseSource = "manual start pose";
        } else {
            poseSource = "(0, 0, 0)";
        }
        if (fieldReferenced && USE_SURVEYED_PRIOR) {
            double p = PRIOR_SIGMA_IN * PRIOR_SIGMA_IN;
            for (Cell c : cells) {
                initEstimate(c, c.surveyX, c.surveyY, p, 0.0, p);
                c.lastResult = "surveyed prior";
            }
        }
        startVision();
    }

    private void startVision() {
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (Cell cell : cells) {
            for (int id = cell.firstId; id < cell.firstId + 4; id++) {
                library.addTag(id, cell.label + " tag " + id, TAG_SIZE_IN, DistanceUnit.INCH);
            }
        }
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

    /** Same as BiobuzzHiveTagTest: wait for a mid-open camera to settle before closing it. */
    private void shutdownVision() {
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

    /** Same as BiobuzzHiveTagTest. */
    private void applyCameraControls() {
        StringBuilder report = new StringBuilder();
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (MANUAL_EXPOSURE) {
                if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    long lo = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                    long hi = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                    long use = Math.max(lo, Math.min(hi, TurretConstants.EXPOSURE_MS));
                    exposure.setExposure(use, TimeUnit.MILLISECONDS);
                    report.append("manual exposure ").append(use).append(" ms");
                } else {
                    report.append("manual exposure unsupported");
                }
                GainControl gain = visionPortal.getCameraControl(GainControl.class);
                if (gain != null) {
                    int want = TurretConstants.GAIN < 0 ? gain.getMaxGain() : TurretConstants.GAIN;
                    int use = Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), want));
                    gain.setGain(use);
                    report.append(", gain ").append(use);
                }
            } else {
                // Set auto EXPLICITLY - a UVC webcam keeps the last manual exposure until unplugged.
                ExposureControl.Mode[] autoModes = {ExposureControl.Mode.AperturePriority,
                        ExposureControl.Mode.Auto, ExposureControl.Mode.ContinuousAuto};
                String applied = "auto exposure unsupported";
                if (exposure != null) {
                    for (ExposureControl.Mode mode : autoModes) {
                        if (exposure.isModeSupported(mode) && exposure.setMode(mode)) {
                            applied = "auto exposure (" + mode + ")";
                            break;
                        }
                    }
                }
                report.append(applied);
            }
        } catch (Exception e) {
            report.append(" (camera controls failed: ").append(e.getMessage()).append(")");
        }
        cameraReport = report.toString();
        cameraControlsApplied = true;
    }

    // ============================================================================================
    // TELEMETRY
    // ============================================================================================

    private void sendTelemetry(Pose2d pose, double loopMsMax) {
        String mode = !aligning ? "DRIVER (right stick click = auto-align)"
                : target == null ? "AUTO-ALIGN: SEARCHING - point the camera at a CELL"
                : "AUTO-ALIGN locked on " + target.label;
        telemetry.addData("MODE", mode);
        if (aligning && target != null) {
            telemetry.addLine(onTarget ? ">>>>>> ON TARGET <<<<<<" : "...aligning...");
        }
        if (!lastEvent.isEmpty() && eventTimer.seconds() < EVENT_SHOW_S) {
            telemetry.addLine("*** " + lastEvent + " ***");
        }
        if (aligning && target != null) {
            telemetry.addData(">>> HEADING ERROR", "%.2f deg  %s", headingErrDeg, onTarget ? "ON TARGET" : "");
            telemetry.addData("Distance to target", "%.1f in   radius hold %s", distanceIn,
                    radiusLatched ? String.format("%.1f in", radiusHold) : "off (moving in/out)");
            telemetry.addData("Turn cmd", "%.2f rad/s", turnCmd);
        }
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg  %s", pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()), fieldReferenced ? "field-ref" : "NOT field-ref");
        telemetry.addData("Camera", "%s | %s",
                visionPortal == null ? "NOT OPEN" : visionPortal.getCameraState(), cameraReport);
        telemetry.addData("Last frame age", lastFrameNanos == 0 ? "none"
                : String.format("%.0f ms (frames used %d)", (System.nanoTime() - lastFrameNanos) / 1e6, framesUsed));

        for (int i = 0; i < 2; i++) {
            Cell c = cells[i];
            telemetry.addLine("--- " + c.label + " (IDs " + c.firstId + "-" + (c.firstId + 3) + ")"
                    + (c == target ? "  <== TARGET" : ""));
            telemetry.addData("State", effectiveState(c, cells[1 - i]));
            if (c.hasEstimate) {
                telemetry.addData("Estimate", "(%.2f, %.2f) +/- %.2f in", c.ex, c.ey, c.sigmaIn());
            } else {
                telemetry.addLine("Estimate: none yet");
            }
            telemetry.addData("Vision", "%d tags | %s | NIS %.1f | ok %d reset %d",
                    c.lastTagsSeen, c.lastResult, c.lastNis, c.accepted, c.resets);
            telemetry.addData("Range / tag height", "%.1f in / %.1f in", c.lastRangeIn, c.lastTagHeightIn);
            telemetry.addData("Tag offsets learned", "%d %d %d %d", c.offN[0], c.offN[1], c.offN[2], c.offN[3]);
        }
        telemetry.addData("Field centric / heading hold", "%s / %s", MecaTank.FIELD_CENTRIC, MecaTank.HEADING_HOLD);
        telemetry.addData("Loop max (ms)", "%.1f", loopMsMax);
        telemetry.update();
    }

    private void showEvent(String event) {
        lastEvent = event;
        eventTimer.reset();
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
