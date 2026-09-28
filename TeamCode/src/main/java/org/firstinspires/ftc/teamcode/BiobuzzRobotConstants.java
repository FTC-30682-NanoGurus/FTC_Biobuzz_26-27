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
        fixedShootingVel = 1500; // TODO: edit after testing shooting and tuning PIDF
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
    public static String pollenFlywheel = "pollenFlywheel";
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
        pollenGateClosedPos = 0,
        pollenGateOpenPos = 0.45,
        nectarGateClosedPos = 0,
        nectarGateOpenPos = 0.45;
}
