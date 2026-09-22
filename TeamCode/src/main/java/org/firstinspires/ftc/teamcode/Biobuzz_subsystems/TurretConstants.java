package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Every tunable number for the BioBuzz auto-aiming turret, in one place.
 *
 * <h2>How to read this file</h2>
 * Each constant carries its UNITS and a one-line "what happens if you change this". Nothing in the
 * turret code may contain a bare number that belongs here - if you find yourself editing a literal
 * inside {@link TurretAimController}, {@link TurretVision} or {@link Turret}, it belongs in this
 * file instead.
 *
 * <h2>Everything marked TODO-MEASURE is currently 0.0 and the turret WILL NOT aim correctly</h2>
 * A 0.0 camera mount pose means "the camera is exactly at robot centre, pointing straight
 * forward, perfectly level". That is never true. Because {@link TurretAimController} back-projects
 * each detection through the camera mount pose into a FIELD coordinate, any error in the mount
 * pose maps one-for-one into target-position error, and then into aim error that grows with
 * range. Measure these carefully - they are the single largest accuracy lever in the whole
 * subsystem.
 *
 * <h2>Coordinate conventions used everywhere in this subsystem</h2>
 * <ul>
 *   <li>ROBOT frame: +x forward, +y LEFT, +z up. Angles CCW-positive, 0 = robot forward.</li>
 *   <li>FIELD frame: whatever RoadRunner's localizer is seeded with. Inches, radians.</li>
 *   <li>CAMERA frame (as the SDK reports it in {@code ftcPose}): +x RIGHT, +y FORWARD,
 *       +z UP. This is NOT the OpenCV optical convention; the conversion happens in exactly two
 *       places, {@link TurretAimController} and {@link TurretAimOverlay}, and is commented there.</li>
 *   <li>TURRET angle: degrees, CCW-positive, 0 = turret pointing along its own mechanical zero
 *       (which is {@link #TURRET_MOUNT_OFFSET_DEG} away from robot forward).</li>
 * </ul>
 *
 * Annotated {@code @Config} so every value here is live-editable from FTC Dashboard while tuning.
 */
@Config
public class TurretConstants {

    private TurretConstants() { } // constants holder - never instantiated

    // =============================================================================================
    // ALLIANCE
    // =============================================================================================

    /**
     * Which alliance we are scoring for this match. Selected by the driver during init and passed
     * into {@link TurretVision#getBestTarget(Alliance)}; it is NOT stored here, because a stale
     * static across opmode runs is exactly how a robot ends up shooting at the wrong hive.
     *
     * The mapping to AprilTag metadata is by NAME PREFIX: BioBuzz cluster names begin with 'R' for
     * red and 'B' for blue. See {@link TurretVision#allianceOf(String)}.
     */
    public enum Alliance {
        RED,
        BLUE
    }

    // =============================================================================================
    // TURRET GEOMETRY AND SERVO MAPPING
    // =============================================================================================

    /**
     * Servo gearing reduction, unitless. Two REV 270-degree servos ganged through a 1.5:1
     * reduction give 270 / 1.5 = 180 degrees at the turret output.
     * CHANGE THIS and every angle-to-servo-position mapping rescales.
     * TODO-CONFIRM: verify the physical reduction on the real turret before trusting any aim.
     */
    public static double SERVO_REDUCTION = 1.5;

    /** Raw travel of a REV smart servo, degrees. Only used to derive {@link #OUTPUT_RANGE_DEG}. */
    public static double SERVO_RAW_RANGE_DEG = 270.0;

    /**
     * Total turret output travel, degrees: 270 / 1.5 = 180.
     * CHANGE THIS and the degrees-per-servo-unit scale changes, so every commanded angle moves the
     * turret by a different physical amount.
     */
    public static double OUTPUT_RANGE_DEG = SERVO_RAW_RANGE_DEG / SERVO_REDUCTION;

    /**
     * Soft travel limit, degrees from turret centre. +-90 for a 180-degree output range.
     * CHANGE THIS to shrink travel if the turret fouls the robot before reaching the hard stop.
     * Commands beyond this are clamped and raise {@link TurretAimController#needsRobotRotation()}.
     */
    public static double TURRET_MAX_ANGLE_DEG = 90.0;
    public static double TURRET_MIN_ANGLE_DEG = -90.0;

    /**
     * Robot-configuration names of the two ganged turret servos.
     * TODO-FILL: real config names from the Driver Station configuration.
     * CHANGE THESE and the turret grabs different (or no) hardware - a typo here shows up as
     * "turret servos not found" on telemetry and a turret that never moves.
     */
    public static String TURRET_SERVO_A_NAME = "turret_servo_a";
    public static String TURRET_SERVO_B_NAME = "turret_servo_b";

    /**
     * Servo position commanded when the turret is at its centre (0 degrees output), unitless 0..1.
     * CHANGE THIS to shift the mechanical centre without re-cutting parts.
     */
    public static double SERVO_CENTER_POSITION = 0.5;

    /**
     * Per-servo {@code scaleRange} calibration, unitless 0..1. These clip the usable band of each
     * servo so that the two ganged servos travel together instead of fighting.
     * TODO-MEASURE: jog each servo to both physical limits with {@link org.firstinspires.ftc.teamcode.opmodes.testing_opmodes.TurretTuningOpMode}
     * and record the positions here. Leaving them at 0..1 is safe but wastes resolution and lets
     * the servos drive into the hard stops.
     * CHANGE THIS and the effective degrees-per-unit changes - re-check the centre afterwards.
     */
    public static double SERVO_A_SCALE_MIN = 0.0;
    public static double SERVO_A_SCALE_MAX = 1.0;
    public static double SERVO_B_SCALE_MIN = 0.0;
    public static double SERVO_B_SCALE_MAX = 1.0;

    /**
     * Whether servo B is mounted mirrored relative to servo A and must be reversed.
     * TODO-CONFIRM: if the two servos fight each other on the first power-up, flip this.
     * CHANGE THIS and servo B rotates the other way - getting it wrong stalls both servos.
     */
    public static boolean SERVO_B_REVERSED = true;

    /**
     * Angle from ROBOT FORWARD to the turret's mechanical zero, degrees, CCW-positive.
     * 0.0 means the turret points straight forward when commanded to 0.
     * TODO-MEASURE: sight down the turret barrel at mechanical zero and measure the angle to robot
     * forward. This is a pure bias on every aim command - a 5-degree error here is a 5-degree miss
     * at every range.
     */
    public static double TURRET_MOUNT_OFFSET_DEG = 0.0;

    /**
     * How close to a soft limit counts as "at the hard stop", degrees.
     * CHANGE THIS and {@link Turret#isAtHardStop()} trips earlier or later, which gates
     * {@link TurretAimController#readyToFire()}.
     */
    public static double TURRET_HARD_STOP_MARGIN_DEG = 1.0;

    // ---- Optional absolute output encoder -------------------------------------------------------

    /**
     * Whether a turret output encoder is physically installed.
     * false = open-loop aiming (servo commands trusted blindly, camera bearing trims them).
     * true  = closed-loop aiming (true output angle measured, PID drives the error to zero).
     * TODO-CONFIRM: set true only once the encoder is wired AND its config name below is right.
     * CHANGE THIS to true with no encoder present and {@link Turret} falls back to open-loop
     * automatically, logging the failure rather than crashing.
     */
    public static boolean TURRET_ENCODER_PRESENT = false;

    /**
     * Robot-configuration name of the turret output encoder, read through a motor port.
     * TODO-FILL: real config name from the Driver Station configuration.
     */
    public static String TURRET_ENCODER_NAME = "turret_encoder";

    /**
     * Encoder counts per full revolution OF THE TURRET OUTPUT, ticks/rev.
     * TODO-MEASURE: rotate the turret output exactly one full turn by hand and record the delta.
     * CHANGE THIS and the measured angle scales - wrong value means the closed loop chases a
     * moving target and can oscillate.
     */
    public static double TURRET_ENCODER_TICKS_PER_REV = 8192.0;

    /**
     * Encoder reading, ticks, when the turret is at its mechanical zero.
     * TODO-MEASURE: park the turret at centre and record the raw count.
     * CHANGE THIS and the measured angle shifts by a constant bias.
     */
    public static double TURRET_ENCODER_ZERO_OFFSET_TICKS = 0.0;

    /** Set true if the encoder counts the opposite way from the turret's CCW-positive convention. */
    public static boolean TURRET_ENCODER_REVERSED = false;

    // =============================================================================================
    // CAMERA - HARDWARE AND STREAM
    // =============================================================================================

    /**
     * Robot-configuration name of the Logitech C920.
     *
     * RESOLVED from existing code, not invented: every webcam lookup in this repository resolves to
     * "Webcam 1" - {@code Camera}, {@code YellowSampleCamera} and {@code PollenCamera} all go
     * through {@code RobotConstants.camera}, and {@code DecodeCAM} and {@code AprilTagLocalization}
     * hard-code the same literal. Referencing the shared field rather than copying the literal
     * means renaming the webcam in one place renames it for the turret too.
     *
     * Only change this if the turret gets its OWN second webcam, which would need a distinct name
     * in the Driver Station configuration.
     */
    public static String WEBCAM_NAME = RobotConstants.camera;

    /**
     * Stream resolution, pixels. 640x480 is deliberate: the SDK ships a built-in C920 calibration
     * at exactly this resolution, so the pose solver is accurate out of the box.
     * CHANGE THIS and you lose the built-in calibration - the intrinsics below would then be wrong
     * and every range/bearing would be biased.
     */
    public static int STREAM_WIDTH = 640;
    public static int STREAM_HEIGHT = 480;

    /** Target stream rate, frames/second. 30 is the C920's native rate at 640x480. */
    public static int STREAM_FPS = 30;

    // ---- Lens intrinsics -------------------------------------------------------------------------

    /**
     * Focal lengths and principal point, PIXELS, for the C920 at 640x480.
     * These are the SDK's own built-in C920 values, set explicitly as documentation and as
     * insurance against the built-in calibration being missed (for example if the camera enumerates
     * under a name the SDK does not recognise).
     * CHANGE THESE and every range and bearing the tag processor reports changes - only replace
     * them with values from a real calibration of YOUR camera at THIS resolution.
     */
    public static double CAMERA_FX = 622.001;
    public static double CAMERA_FY = 622.001;
    public static double CAMERA_CX = 319.803;
    public static double CAMERA_CY = 241.251;

    // ---- AprilTag decode ------------------------------------------------------------------------

    /** AprilTag physical edge length, METRES. 3.25 in = 0.08255 m. Wrong size = wrong range. */
    public static double TAG_SIZE_METERS = 0.08255;

    /** Same tag size in INCHES, for telemetry and sanity checks only. */
    public static double TAG_SIZE_INCHES = 3.25;

    /**
     * Detector decimation, unitless. 2 = decode on a half-scale image.
     * Higher = faster and longer range is lost; lower = slower but detects smaller/further tags.
     * CHANGE THIS and both frame rate and maximum detection range move together.
     */
    public static int APRILTAG_DECIMATION = 2;

    // ---- Exposure and gain ----------------------------------------------------------------------

    /**
     * Whether to take manual control of exposure/gain. Auto-exposure re-hunts whenever a bright
     * robot drives past, which blurs tags mid-match.
     * CHANGE THIS to false and the camera will drift under match lighting.
     */
    public static boolean MANUAL_EXPOSURE = true;

    /**
     * Manual exposure, MILLISECONDS. Start at 3; the useful sweep is 1-6.
     * Short exposure freezes motion blur so tags still decode while the robot drives; the light it
     * gives up is bought back with maximum gain below. This is the standard FTC
     * minimize-exposure / maximize-gain technique.
     * TODO-TUNE: run {@link org.firstinspires.ftc.teamcode.opmodes.testing_opmodes.TurretTuningOpMode}
     * under real field lighting and keep the SHORTEST exposure that still decodes reliably.
     * CHANGE THIS lower and tags stop decoding in dim light; higher and they smear while moving.
     */
    public static int EXPOSURE_MS = 3;

    /** Lower / upper bound of the sweep the tuning opmode walks, milliseconds. */
    public static int EXPOSURE_SWEEP_MIN_MS = 1;
    public static int EXPOSURE_SWEEP_MAX_MS = 6;

    /**
     * Sensor gain, raw sensor units. -1 means "ask the camera for {@code getMaxGain()} and use
     * that", which is what pairs with the very short exposure above.
     * CHANGE THIS to a fixed number only if maximum gain proves too noisy for the decoder.
     */
    public static int GAIN = -1;

    /**
     * Lock focus and white balance to fixed values when the camera supports those controls.
     * Both hunt mid-match if left automatic, and a re-focus event drops several frames.
     * CHANGE THIS to false only if a control is unsupported and the log is noisy about it.
     */
    public static boolean LOCK_FOCUS = true;
    public static boolean LOCK_WHITE_BALANCE = true;

    /** Fixed white-balance colour temperature, KELVIN, used when {@link #LOCK_WHITE_BALANCE}. */
    public static int WHITE_BALANCE_TEMPERATURE_K = 4000;

    // ---- Camera mount pose (ACCURACY CRITICAL) ---------------------------------------------------

    /**
     * Camera position relative to ROBOT CENTRE, INCHES, in the robot frame
     * (+x forward, +y LEFT, +z up).
     *
     * TODO-MEASURE - ACCURACY CRITICAL. {@link TurretAimController} transforms every detection
     * from the camera frame through this mount pose into a field coordinate, so an error here is
     * added directly to the estimated target position. Measure to the LENS, not the housing, with
     * a tape measure and a square. An inch of error here is an inch of error in the target
     * estimate at every range.
     */
    public static double CAMERA_OFFSET_X_IN = 0.0;
    public static double CAMERA_OFFSET_Y_IN = 0.0;
    public static double CAMERA_OFFSET_Z_IN = 0.0;

    /**
     * Camera mount orientation relative to robot forward, DEGREES.
     * YAW is CCW-positive about the robot's up axis (camera turned to the LEFT is positive).
     * PITCH is positive when the camera is tilted UP.
     * ROLL is positive when the camera is rotated clockwise as seen from behind it.
     *
     * TODO-MEASURE - ACCURACY CRITICAL, and yaw most of all. Yaw error rotates the entire target
     * estimate about the robot, so at 96 in a 2-degree yaw error is roughly 3.4 in of lateral
     * target error and the shot misses. Pitch matters because it tilts the forward axis into the
     * vertical one, shortening the computed ground range.
     */
    public static double CAMERA_MOUNT_YAW_DEG = 0.0;
    public static double CAMERA_MOUNT_PITCH_DEG = 0.0;
    public static double CAMERA_MOUNT_ROLL_DEG = 0.0;

    // =============================================================================================
    // TARGET SELECTION GATES
    // =============================================================================================

    /**
     * Minimum fraction of a cluster's four tags that must be visible before the cluster is
     * accepted, PERCENT (0-100).
     * A cluster seen at 25% is one tag - its pose solution is noisy and its roll is unreliable.
     * CHANGE THIS higher for fewer, better detections; lower to keep aiming through occlusion at
     * the cost of accepting worse pose data.
     */
    public static double MIN_PERCENT_CLUSTER_FOUND = 40.0;

    /**
     * A CELL is scorable (its opening faces up) iff {@code abs(ftcPose.roll) <} this, DEGREES.
     * 90 is the geometric definition of "opening still faces up" and should not need changing.
     * CHANGE THIS and the robot will start accepting flipped, unscorable cells.
     */
    public static double SCORABLE_MAX_ROLL_DEG = 90.0;

    /**
     * How close to the +-90-degree flip point counts as TRANSITIONING (mid-tip), DEGREES.
     * While transitioning, the target estimate is FROZEN and firing is blocked, because the hive
     * is rotating and both its roll and its pose solution are momentarily meaningless.
     * CHANGE THIS wider to hold fire more conservatively around a tip; narrower to resume
     * shooting sooner at the risk of firing into a closing cell.
     */
    public static double TRANSITION_ROLL_MARGIN_DEG = 20.0;

    /**
     * Drop in cluster coverage between consecutive frames that also signals a tip, PERCENT points.
     * A hive rotating away sheds visible tags fast; a simple occlusion usually does not.
     * CHANGE THIS lower and ordinary occlusions will be misread as tips (the robot holds fire more).
     */
    public static double TRANSITION_COVERAGE_DROP_PCT = 25.0;

    // =============================================================================================
    // TARGET FIELD-COORDINATE ESTIMATE AND FILTERING
    // =============================================================================================

    /**
     * Low-pass factor for the target field-coordinate estimate, unitless 0..1.
     * new_estimate = ALPHA * measurement + (1 - ALPHA) * old_estimate.
     * The CELL opening is quasi-static between tips, so heavy smoothing is free accuracy: it
     * averages down the noisy monocular RANGE axis, which is where nearly all the error lives.
     * CHANGE THIS higher to react faster to a genuinely moved target; lower for a steadier
     * estimate that lags a real move.
     */
    public static double TARGET_ESTIMATE_ALPHA = 0.15;

    /**
     * Innovation gate, INCHES. A new measurement further than this from the current estimate is
     * NOT smoothed in - the estimate is reset to it and marked fresh.
     * This is what makes a hive tip (or a switch to a different scorable CELL) snap instead of
     * sliding the estimate across the field through empty space.
     * CHANGE THIS lower and ordinary range noise will keep resetting the filter; higher and a real
     * target change will be smoothed through, aiming at a point between two cells.
     */
    public static double TARGET_INNOVATION_GATE_IN = 18.0;

    /**
     * How long a target estimate stays usable after its last update, SECONDS.
     * Past this the estimate is stale: aiming continues on it but {@link TurretAimController}
     * reports the age so telemetry and firing decisions can see it.
     * CHANGE THIS longer to coast further through occlusion; shorter to distrust old data sooner.
     */
    public static double TARGET_ESTIMATE_MAX_AGE_S = 3.0;

    /**
     * Pose history ring buffer, SAMPLES. At a ~50 Hz loop, 64 samples is ~1.3 s of history.
     * This buffer exists so a detection can be paired with the robot pose at the frame's CAPTURE
     * time rather than the current pose. Without it, robot motion during camera latency smears the
     * computed target coordinate: at 30 in/s and 80 ms of latency that is 2.4 in of pure error.
     * CHANGE THIS larger only if the loop runs fast enough that 64 samples no longer covers the
     * camera latency.
     */
    public static int POSE_HISTORY_SIZE = 64;

    /**
     * Oldest pose sample the history will match against, SECONDS. A detection whose capture
     * timestamp is older than this is treated as un-matchable and the current pose is used, with
     * the sample flagged in telemetry.
     */
    public static double POSE_HISTORY_MAX_LOOKBACK_S = 0.5;

    // ---- HIVE TARGET COORDINATES (CONFIRMED - each alliance has TWO hives) ----------------------

    /**
     * Which of an alliance's two hives a target is. Named from the RED ALLIANCE PERSPECTIVE, the
     * same convention the coordinates below are given in, so LEFT and RIGHT mean the same physical
     * direction regardless of which alliance is being aimed at.
     */
    public enum HiveSide {
        LEFT,
        RIGHT
    }

    /**
     * Field coordinates of the four HIVE CELL opening centres, INCHES, in the standard
     * RoadRunner/MeepMeep field frame, given from the red-alliance perspective.
     *
     * These are CONFIRMED opening-centre positions, not rough seeds. That changes how the aim
     * controller treats them: the blind feedforward is now trustworthy on its own, so the turret
     * points accurately at a hive from odometry alone before the camera has ever seen it, and it
     * keeps pointing accurately straight through a long occlusion. Vision's job is reduced from
     * "find the target" to "correct the last degree or two", which is what it is actually good at.
     *
     * The two hives of one alliance sit roughly 24 in apart, both near the field centreline. That
     * is close enough that they are easy to confuse and far enough that shooting at the wrong one
     * misses completely, which is why the controller keeps a SEPARATE estimate for each and picks
     * between them explicitly rather than averaging.
     */
    public static double RED_RIGHT_X_IN  =  12.232;
    public static double RED_RIGHT_Y_IN  = -19.277;
    public static double RED_LEFT_X_IN   = -10.998;
    public static double RED_LEFT_Y_IN   = -19.297;
    public static double BLUE_LEFT_X_IN  = -12.248;
    public static double BLUE_LEFT_Y_IN  =  19.277;
    public static double BLUE_RIGHT_X_IN =  13.268;
    public static double BLUE_RIGHT_Y_IN =  19.277;

    /**
     * Height of the CELL opening above the field, INCHES.
     *
     * Used for the LAUNCH ARC only, never for azimuth. This turret rotates in one axis, so the
     * height cannot change where it points; it changes how hard the ball has to be thrown, which
     * is what {@link LaunchVelocityTable} encodes. It is here so the arc maths has a single source
     * when someone adds a hood or a variable-angle shooter.
     */
    public static double TARGET_OPENING_HEIGHT_IN = 21.5;

    // ---- Cluster name -> hive mapping (TO FILL) --------------------------------------------------

    /**
     * Maps the AprilTag cluster name the SDK reports onto which hive it is.
     *
     * TODO-FILL - ALL FOUR ARE EMPTY. Run
     * {@link org.firstinspires.ftc.teamcode.opmodes.testing_opmodes.TurretTuningOpMode}, point the
     * camera at each hive in turn, read the cluster name it prints, and paste the exact strings
     * here. Expect something like "RED SCORING" / "RED AUDIENCE", but do NOT guess - the SDK's
     * spelling is what has to match, and it is compared case-insensitively but otherwise exactly.
     *
     * While these are empty the controller still works: it falls back to associating each detection
     * with whichever hive estimate it lands nearest to, which is reliable because the two hives are
     * about 24 in apart and the estimates start seeded at the confirmed coordinates above. Filling
     * these in makes the association exact instead of geometric, which matters most when the robot
     * is far away and the range noise is comparable to the hive spacing.
     */
    public static String RED_RIGHT_CLUSTER_NAME  = "";   // TODO-FILL e.g. "RED SCORING"
    public static String RED_LEFT_CLUSTER_NAME   = "";   // TODO-FILL e.g. "RED AUDIENCE"
    public static String BLUE_LEFT_CLUSTER_NAME  = "";   // TODO-FILL e.g. "BLUE AUDIENCE"
    public static String BLUE_RIGHT_CLUSTER_NAME = "";   // TODO-FILL e.g. "BLUE SCORING"

    // ---- Choosing between an alliance's two hives -------------------------------------------------

    /**
     * How much closer the other hive has to be before the turret will switch to it, INCHES.
     *
     * This is hysteresis, and without it the robot parked equidistant between two hives 24 in apart
     * would flip the turret back and forth every time range noise moved the estimate by an inch.
     * Each switch costs a servo sweep and a re-spin-up, so the cost of dithering is far higher than
     * the cost of shooting at the slightly-further hive.
     *
     * CHANGE THIS larger to commit harder to the current hive; smaller to follow the nearest one
     * more eagerly at the risk of dithering near the midpoint.
     */
    public static double HIVE_SWITCH_HYSTERESIS_IN = 8.0;

    /**
     * How long a hive must have been the better choice before the switch actually happens, SECONDS.
     * Works with the distance hysteresis above: the challenger has to be both clearly closer AND
     * consistently closer, so a single bad frame cannot move the turret.
     */
    public static double HIVE_SWITCH_DWELL_S = 0.35;

    // ---- Coarse field-half fallback --------------------------------------------------------------

    /**
     * Field-half boundary, INCHES, compared against the robot's odometry X.
     *
     * COARSE LAST RESORT ONLY, and now less useful than it ever was. Both of an alliance's hives
     * sit near the field centreline about 24 in apart, so which half of the field the ROBOT is
     * standing in says almost nothing about which hive to shoot at. It survives only to give a
     * mis-configured opmode - one that reached start() with no alliance selected at all - something
     * visible to do rather than nothing.
     *
     * The real selectors, in order, are: the alliance prefix on the cluster metadata name, then the
     * name-to-hive mapping above, then nearest-scorable-hive with hysteresis. See
     * {@link TurretAimController#chooseActiveHive}.
     *
     * Left at 0.0, which is the field centerline in the RoadRunner field frame.
     * TODO-CONFIRM: confirm which axis (x or y) the field-half split uses for this field. The
     * comparison in {@link TurretAimController} currently tests the robot's X against this value;
     * if BioBuzz splits the field along Y instead, that comparison has to change with this comment.
     */
    public static double FIELD_HALF_BOUNDARY_IN = 0.0;

    /**
     * When the robot's X is GREATER than {@link #FIELD_HALF_BOUNDARY_IN}, which alliance's target
     * does that half hold? TODO-CONFIRM against the real field layout.
     */
    public static Alliance FIELD_HALF_POSITIVE_SIDE_ALLIANCE = Alliance.RED;

    // =============================================================================================
    // AIM CONTROL GAINS
    // =============================================================================================

    /**
     * Aim deadband, DEGREES. Inside this the turret is not re-commanded at all.
     * Servos buzz and draw current when fed a stream of sub-degree corrections; this stops that.
     * CHANGE THIS larger for a quieter, cooler turret that aims slightly less precisely.
     */
    public static double AIM_DEADBAND_DEG = 0.5;

    /**
     * Aim error below which {@link TurretAimController#readyToFire()} may return true, DEGREES.
     * This is the accuracy gate on the shot. It should be tighter than the angular half-width of
     * the CELL opening at your maximum shooting range.
     * CHANGE THIS tighter for better shots that take longer to line up; looser to fire sooner and
     * miss more.
     */
    public static double AIM_READY_TOLERANCE_DEG = 2.0;

    /**
     * Open-loop bearing trim gain, unitless. Fraction of the live camera bearing error folded into
     * the command each loop when there is NO encoder.
     * CHANGE THIS higher for faster convergence onto the live bearing and more overshoot/hunting;
     * lower for a lazier, steadier trim.
     */
    public static double BEARING_TRIM_GAIN = 0.35;

    /**
     * Maximum correction the live-bearing trim may add on top of the odometry-geometry command,
     * DEGREES. This cap is what stops one bad frame from swinging the turret across the field.
     * CHANGE THIS larger to let vision override geometry more aggressively.
     */
    public static double BEARING_TRIM_MAX_DEG = 15.0;

    /**
     * Low-pass factor on the trim itself, unitless 0..1. Smooths frame-to-frame bearing noise.
     * CHANGE THIS higher to follow the camera more closely and jitter more.
     */
    public static double BEARING_TRIM_ALPHA = 0.3;

    /**
     * Closed-loop (encoder present) PID gains on turret angle error.
     * Units: output DEGREES of correction per degree of error (P), per degree-second (I), and per
     * degree/second (D).
     * TODO-TUNE on the real turret, and only once the encoder is installed and zeroed.
     * CHANGE P too high and the turret oscillates around the target; too low and it never closes
     * the last degree.
     */
    public static double TURRET_KP = 0.6;
    public static double TURRET_KI = 0.0;
    public static double TURRET_KD = 0.02;

    /** Clamp on the integral term's contribution, DEGREES. Prevents wind-up against a hard stop. */
    public static double TURRET_MAX_I_DEG = 5.0;

    /**
     * Maximum age of a detection before its live bearing is no longer trusted for trimming,
     * SECONDS. Past this the controller flies on the field-coordinate estimate alone.
     * CHANGE THIS longer and stale bearings will be applied as if fresh.
     */
    public static double BEARING_MAX_LATENCY_S = 0.25;

    /**
     * Projectile flight time used to LEAD the shot while the robot is moving, SECONDS.
     * 0.0 disables lead entirely, which is correct until it is measured.
     * TODO-MEASURE (optional/advanced): time a shot from release to arrival at a mid-range target.
     * CHANGE THIS non-zero and the turret aims where the target will be relative to the moving
     * robot - only helpful once it is measured, actively harmful if guessed.
     */
    public static double PROJECTILE_FLIGHT_TIME_S = 0.0;

    // =============================================================================================
    // INTAKE
    // =============================================================================================

    /**
     * Robot-configuration name of the intake roller motor.
     *
     * RESOLVED from existing code, not invented: this season's own {@code Biobuzz_subsystems.Intake2_0}
     * already builds its roller motor from {@code BiobuzzRobotConstants.rollers}, which is
     * "rollers". Referencing the shared field keeps the turret subsystem and Intake2_0 pointed at
     * the same physical motor.
     *
     * Note this is NOT the old DECODE/Into-the-Deep {@code RobotConstants.intakeMotor}
     * ("intake_motor"), which belongs to a different season's mechanism.
     */
    public static String INTAKE_MOTOR_NAME = BiobuzzRobotConstants.rollers;

    /** Set true if the intake motor spins the wrong way when {@link Intake#start()} is called. */
    public static boolean INTAKE_MOTOR_REVERSED = false;

    /**
     * Intake power while collecting, unitless -1..1.
     * CHANGE THIS higher to grab harder and jam more; lower to be gentler and miss pickups.
     */
    public static double INTAKE_POWER = 0.9;

    /**
     * Intake power while ejecting, unitless -1..1. Negative runs the rollers backwards.
     * CHANGE THIS more negative to clear jams faster.
     */
    public static double INTAKE_REVERSE_POWER = -0.7;

    // =============================================================================================
    // LAUNCHER / FLYWHEEL
    // =============================================================================================

    /**
     * Robot-configuration name of the flywheel motor, driven through the existing
     * {@code NGMotor} wrapper.
     * TODO-FILL: real config name.
     */
    public static String LAUNCHER_MOTOR_NAME = "flywheel_motor";

    /** Set true if the flywheel spins backwards on the first spin-up. */
    public static boolean LAUNCHER_MOTOR_REVERSED = false;

    /**
     * Velocity band inside which {@link Launcher#isAtVelocity()} reports true, TICKS/SECOND.
     * This is the "spun up" gate - firing below it throws short.
     * CHANGE THIS tighter for more consistent shots that take longer to green-light; looser to
     * fire sooner with more velocity scatter.
     */
    public static double LAUNCHER_VELOCITY_TOLERANCE_TPS = 50.0;

    /**
     * Flywheel PIDF coefficients. LIVE - these are pushed into NGMotor by
     * {@link Launcher#applyFlywheelPIDF()}, which runs at {@link Launcher#init()} and hits BOTH of
     * NGMotor's entry points: {@code setPIDF(P, I, D, F)} for its position loop and
     * {@code setCustomVelocityPID(target, P, I, D, F)} for the flywheel velocity loop the launcher
     * actually runs. Never set on a raw DcMotorEx and never re-implemented - NGMotor owns the loop.
     *
     * Units are NGMotor's, whose velocity loop produces a motor POWER in -1..1 from an error in
     * TICKS/SECOND: P is power per (tick/second) of error, I is power per (tick-second), D is power
     * per (tick/second-squared), and F multiplies the TARGET velocity as a feedforward, so it is
     * also power per (tick/second).
     *
     * CAVEAT ON P: {@code NGMotor.updateFlywheels()} gain-schedules its proportional term between
     * its own internal {@code kP_Recovery} and {@code kP_Stable} fields rather than reading the P
     * passed in. I, D and F are read normally. P is still wired through both entry points, but
     * changing it will not move the flywheel's proportional response until NGMotor is edited. See
     * the class javadoc on {@link Launcher} before changing that.
     *
     * TODO-TUNE on the real shooter. A useful starting point already exists in this repository:
     * {@code Biobuzz_subsystems/Intake2_0.java:88} runs its flywheels at
     * {@code setCustomVelocityPID(vel, 0.0085, 0.015, 0.0001, 0.000426)}. If the turret shoots
     * through that same flywheel, those four numbers are a far better start than zeros.
     */
    public static double FLYWHEEL_P = 0.0;
    public static double FLYWHEEL_I = 0.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 0.0;

    // ---- Feeder / indexer ------------------------------------------------------------------------

    /**
     * Whether a feeder/indexer actuator exists to push one ball into the flywheel.
     * TODO-CONFIRM: this whole feeder block is an ASSUMPTION. If there is no feeder servo, leave
     * this false and {@link Launcher#feedOne()} becomes a safe no-op rather than a crash.
     */
    public static boolean FEEDER_PRESENT = false;

    /** Robot-configuration name of the feeder servo. TODO-FILL if a feeder exists. */
    public static String FEEDER_SERVO_NAME = "feeder_servo";

    /** Feeder servo position while holding a ball back, unitless 0..1. TODO-MEASURE. */
    public static double FEEDER_HOLD_POSITION = 0.0;

    /** Feeder servo position at the end of a feed stroke, unitless 0..1. TODO-MEASURE. */
    public static double FEEDER_FEED_POSITION = 0.5;

    /**
     * How long the feeder stays at the feed position before returning, SECONDS.
     * CHANGE THIS shorter and the ball may not clear; longer and the feeder blocks the next shot.
     */
    public static double FEEDER_STROKE_TIME_S = 0.25;
}
