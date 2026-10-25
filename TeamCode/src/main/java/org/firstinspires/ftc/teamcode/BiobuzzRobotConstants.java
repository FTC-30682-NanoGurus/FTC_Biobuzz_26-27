package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;

@Config
public class BiobuzzRobotConstants {
    public static String fr = "frontRight";
    public static String br = "backRight";
    public static String fl = "frontLeft";
    public static String bl = "backLeft";
    public static LazyImu imu;
    public static boolean imu_init = false;

    public static Pose2d pose = new Pose2d(0,0,0);

    /**
     * Alliance for the BIOBUZZ autos, picked with the gamepad during INIT (B = red, X = blue) by
     * AllianceSelector. The field is rotationally symmetric, so every auto path is written once in
     * the "alliance frame" (our own side is y < 0, our drivers' right is +x - the red view) and runs
     * unchanged for blue; only the HIVE AprilTag IDs and CELL positions depend on this. Static, so
     * the last choice is remembered until the Robot Controller app restarts.
     */
    public static boolean allianceRed = true;
    public static String rollers = "rollers";
    public static String flicker = "flicker";
    public static String gate = "gate";
    public static String transferRollers = "transferRollers";
    public static double
        gateClosedPos = 0, // TODO: edit after testing gate positions
        gateOpenPos = 0.65; // TODO: edit after testing gate positions
    //public static String transferRollers = "transferRollers";
    //public static String interTransfer = "interTransfer";
    public static String flywheels = "flywheels";
    public static double
        pollenFixedShootingVel = 1280,
        nectarFixedShootingVel = 1400;

    /**
     * Flywheel motor directions for the autos (Intake2_0). Pollen is REVERSE because that is how
     * the pollen wheel was set up when its PIDF was tuned (FlywheelIntakeTestingOpMode). Nectar is
     * untested: if it reads a negative velocity while spinning, flip it.
     */
    public static boolean
        pollenFlywheelReversed = true,
        nectarFlywheelReversed = false;

    /**
     * Servo-driven ramp that drops onto a FLOWER to intake from it. Starts at the stowed position and
     * swings to the intake position only while collectFromFlower() runs.
     * PLACEHOLDERS: stowed 0.0 per the mechanism; 0.4 is a best guess - jog the servo and measure.
     */
    /**
     * The intake ("rollers") and the transfer into the shooter ("transferRollers") are SEPARATE
     * motors. transferReversed matches the direction Intake2_0 has always given the transfer motor.
     * transferAlsoRunsIntake: also spin the intake while transferring, so artifacts still sitting in
     * the intake get pushed through to the transfer within the same feed window. Set false if the
     * transfer pulls everything on its own.
     */
    public static boolean
        transferReversed = true,
        transferAlsoRunsIntake = true;

    public static String flowerRamp = "flowerRamp";
    public static double
        flowerRampStowedPos = 0.86,
        flowerRampIntakePos = 0.46;

    /**
     * Every FLOWER intake in the autos ends with a straight forward drive of this many INCHES, at the
     * intake heading, onto the FLOWER intake pose (HiveAutoKit.approachFlower / flowerRoute), so the
     * lowered ramp always meets the FLOWER head-on. The ramp is already lowered before this straight
     * starts. Set it longer than the lowered ramp reaches past where it first touches the FLOWER.
     */
    public static double flowerApproachIn = 6.0;

    /**
     * After a FLOWER intake the ramp stays down until the robot has moved this many INCHES from the
     * FLOWER intake pose, so it never swings up into the FLOWER (HiveAutoKit.leaveFlower). Keep it
     * above flowerApproachIn: the ramp reaches up to flowerApproachIn into the FLOWER.
     */
    public static double flowerRampClearIn = 8.0;

    /**
     * Robot footprint, INCHES: width is side to side, length is front to back (the heading
     * direction). Used by HiveAutoKit to keep every auto route clear of the HIVE.
     */
    public static double
        robotWidthIn = 15.0,
        robotLengthIn = 18.0;

    //public static String hoodAdjuster = "hoodAdjuster";

    // =============================================================================================
    // BIOBUZZ INTAKE + DUAL SHOOTER (Biobuzz_TeleOp)
    //
    // Everything below was added for the two-flywheel shooter. The single-flywheel names above
    // (flywheels, gate) are left exactly as they were, because Intake2_0 and every auto still use
    // them - do NOT repoint those at the new hardware without updating those files too.
    // =============================================================================================

    /** Servo that raises and lowers the intake. Was a bare literal in driveTesting.java. */
    public static String intakeLifter = "intakeLifter";

    /**
     * Intake lifter travel, unitless 0..1. Values carried over from driveTesting.java.
     * The teleop drops the lifter to LOWERED at init and raises it automatically once the colour
     * sensor has counted {@code artifactLiftCount} artifacts.
     */
    public static double
        intakeLifterLiftedPos = 0.6,
        intakeLifterLoweredPos = 0.02;

    /**
     * REV Color Sensor V3 that counts pollen and nectar through the intake.
     * The Driver Station configuration type MUST be "REV Color Sensor V3", not the V2's
     * "REV Color/Range Sensor" - see colorSensorTesting.java, which explains why the two are not
     * interchangeable and why the hue bands have to be re-measured on the V3.
     */
    public static String colorSensor = "colorSensor";

    /** How many artifacts must be counted before the intake lifter raises itself. */
    public static int artifactLiftCount = 4;

    /**
     * The two shooter flywheels: one dedicated to POLLEN, one to NECTAR. Separate motors, but both
     * run the same PIDF gains - only their distance/velocity tables differ.
     */
    public static String pollenFlywheel = "pollenFlywheel";   // the motor the pollen PIDF was tuned on
    public static String nectarFlywheel = "nectarFlywheel";

    /**
     * The two gate servos, REV Smart Robot Servos in 180-DEGREE mode, one feeding each flywheel.
     * Each holds its artifact back until its flywheel is up to speed, then opens so the rollers can
     * transfer it in.
     */
    public static String pollenGate = "pollenGate";
    public static String nectarGate = "nectarGate";

    /**
     * Gate travel, unitless 0..1 over the servo's full 180 degrees, so 0.5 is a 90-degree swing.
     * Kept per-gate rather than shared, because the two gates are mirrored and will not trim to the
     * same numbers.
     * TODO: edit after testing gate positions on the real shooter.
     */
    public static double
        pollenGateClosedPos = 1.0,
        pollenGateOpenPos = 0.52,
        nectarGateClosedPos = 0,
        nectarGateOpenPos = 0.45;

    // =============================================================================================
    // FLYWHEEL PIDF - the single source for the autos (Intake2_0) and both teleops.
    //
    // NGMotor ignores the P passed to setCustomVelocityPID; the P that acts is scheduled between
    // KpRecovery (wheel more than RecoveryThreshold BELOW target) and KpStable (everything else).
    // Both are per-motor fields, pinned by each opmode through NGMotorFlywheelTuner.
    // =============================================================================================

    /** Pollen: tuned in FlywheelIntakeTestingOpMode. */
    public static double
        pollenKF = 0.000408,
        pollenKI = 0,
        pollenKD = 0,
        pollenKpStable = 0.0007,
        pollenKpRecovery = 0.0075, //O.012
        pollenAlpha = 0.7,
        pollenRecoveryThreshold = 90.0;

    /** Nectar: tuned in NectarFlywheelIntakeTestingOpMode. */
    public static double
        nectarKF = 0.000498, //0.000426
        nectarKI = 0,
        nectarKD = 0,
        nectarKpStable = 0.0006, //0.0007
        nectarKpRecovery = 0, //0.0075
        nectarAlpha = 0.7,
        nectarRecoveryThreshold = 0; //90

    /**
     * Load boost while artifacts go through a flywheel: kLoad + kRamp * seconds feeding. STATIC on
     * NGMotor, so one pair serves both wheels; every opmode writes these into NGMotor at init.
     */
    public static double
        flywheelKLoad = 0, //0.1
        flywheelKRamp = 0; //0.05

    /** Velocity band counted as "at speed", TICKS/SECOND - the feed interlock. */
    public static double flywheelAtSpeedTolerance = 50.0;

    // =============================================================================================
    // INTAKE / TRANSFER POWERS (teleops)
    // =============================================================================================
    public static double
        intakePower = 0.76,
        intakeReversePower = -0.7,
        transferPower = 0.7;
}
