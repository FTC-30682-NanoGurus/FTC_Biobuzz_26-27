package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.LaunchVelocityTable;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;
import org.firstinspires.ftc.teamcode.library.NGServo;

// ===== BEGIN CAMERA CODE - DISABLED =====
// Uncomment these three imports together with every other CAMERA CODE block in this file.
// import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
// import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretTarget;
// import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretVision;
// ===== END CAMERA CODE - DISABLED =====

/**
 * BIOBUZZ competition teleop: driveTesting's drivetrain, a counting intake, and a two-flywheel
 * shooter that meters pollen and nectar through separate gates.
 *
 * <h2>Where each half came from</h2>
 * The drive half - every field, every line of setup, and every gamepad1 binding - is copied
 * VERBATIM from {@code opmodes/testing_opmodes/driveTesting.java}, including the parts that file
 * warns about itself: the un-negated stick values, the sign-sensitive features, and the manual
 * bulk-read cycle. If this drives differently from driveTesting, that is a bug in this file. The
 * same was done for {@code TurretDriveShootTeleOp}, so all three drive identically.
 *
 * ONE DRIVE BINDING MOVED. driveTesting toggles {@code MecaTank.TRANSLATION_HOLD} with gamepad1 A,
 * and A is now the intake toggle on both pads. Translation hold moved to gamepad1 DPAD LEFT, which
 * was free. Nothing else about the drive changed - not the mapping, not the defaults, not the code.
 *
 * The counting logic is copied from {@code colorSensorTesting.java} (its hue bands, its gain, its
 * proximity gate, its CONFIRM_MS/CLEAR_MS debounce) and the flywheel gains from
 * {@code FlywheelIntakeTestingOpMode.java}. The gamepad PIDF editor from that tuning opmode is
 * deliberately NOT here: the gains are set once at init and left alone.
 *
 * <h2>The shooter</h2>
 * Two independent sides, {@link ShooterSide}, one per artifact type:
 * <ul>
 *   <li><b>POLLEN</b> - {@code pollenFlywheel} + {@code pollenGate}, velocities from the pollen
 *       {@link LaunchVelocityTable}.</li>
 *   <li><b>NECTAR</b> - {@code nectarFlywheel} + {@code nectarGate}, velocities from the nectar
 *       table.</li>
 * </ul>
 * Both sides run the SAME PIDF gains - a flywheel's gains describe the wheel and the motor, not
 * what is being thrown - but they get SEPARATE distance/velocity tables, because a pollen ball and
 * a nectar ball differ in mass and drag and will not score from the same velocity at the same
 * range.
 *
 * THE GATE IS NOT A TRIGGER, IT IS AN INTERLOCK. Pulling a shoot trigger spins the flywheel up and
 * nothing else. The gate only opens, and the rollers only transfer, once that flywheel is within
 * {@link #AT_SPEED_TOLERANCE} of its commanded velocity. Feeding an artifact into a wheel that is
 * still spinning up throws it short and drags the wheel down further, which is the whole reason
 * {@code NGMotor}'s loop has a kLoad/kRamp feeder term at all.
 *
 * THE TRANSFER MOTOR IS THE INTAKE MOTOR. There is one {@code rollers} motor and it does both jobs,
 * so a shot always wins the arbitration over the intake toggle - see the power resolution in the
 * loop. Releasing the trigger hands the rollers straight back to whatever the toggle last asked
 * for.
 *
 * <h2>Distance and the C920 - CURRENTLY DISABLED</h2>
 * Distance-based shooting is wired end to end, but the camera that measures the distance is
 * commented out. Today {@link #getShotDistanceInches()} returns {@link #MANUAL_DISTANCE_IN}, a
 * dashboard-editable number, so the tables and both gates can be tested on a bench right now.
 *
 * Every piece of camera code needed is written and sits in blocks marked
 * {@code ===== BEGIN CAMERA CODE - DISABLED =====} (the same convention {@code TurretVision}
 * already uses for the SDK 12 cluster code). Uncommenting all of them - imports, field, init,
 * per-loop update, the distance maths, telemetry, and close - is the whole job; nothing else has to
 * be written. {@code TurretVision} already owns the C920's VisionPortal, its AprilTag processor and
 * its manual exposure, so this file only converts one detection into a ground range.
 *
 * <h2>CONTROLS</h2>
 *
 * <pre>
 * GAMEPAD 1 - DRIVING (copied from driveTesting)
 *   Left stick X ........ strafe   (translation)
 *   Left stick Y ........ forward  (translation)
 *   Right stick X ....... turn
 *   Left bumper (hold) .. precision mode, 35%
 *   Right bumper (hold).. override - bypasses the accel limiter and traction control
 *   Y ................... toggle field centric (ON by default)
 *   X ................... toggle heading hold
 *   Dpad left ........... toggle translation hold   (was A in driveTesting - A is now the intake)
 *   Back / Options ...... reset the heading reference
 *
 * GAMEPAD 1 - DURING INIT ONLY
 *   B ................... select RED alliance      (which nectar is ours)
 *   X ................... select BLUE alliance
 *
 * INTAKE - EITHER PAD, same buttons on both
 *   A ................... toggle the intake:  1.0 -> 0.0 -> 1.0 ...
 *   B (hold) ............ reverse at -1.0 to eject; releasing returns to the toggle state
 *   Dpad up ............. intake lifter to LIFTED   (manual override)
 *   Dpad down ........... intake lifter to LOWERED, and reset the artifact count
 *
 * GAMEPAD 2 - SHOOTER
 *   Right trigger (hold)  POLLEN: spin up, then gate + transfer once at speed
 *   Left trigger (hold) . NECTAR: spin up, then gate + transfer once at speed
 *   Right bumper (hold).. POLLEN: spin up ONLY, gate stays shut (pre-spin before a shot)
 *   Left bumper (hold) .. NECTAR: spin up ONLY, gate stays shut
 *   X ................... reset the artifact count without moving the lifter
 *   Y ................... toggle the colour sensor's LED (diagnostic - see colorSensorTesting)
 * </pre>
 *
 * <h2>BEFORE THE FIRST DRIVE</h2>
 * Read driveTesting's header sign warnings - field centric, heading hold, traction control and the
 * arcade strafe/turn signs are all still unvalidated, and this file inherits every one of them.
 * Then re-measure the colour bands on the V3 and fill in the two velocity tables; both ship flat
 * and untuned on purpose, so telemetry shows plainly that nothing has been measured yet.
 */
@Config
@TeleOp(name = "BioBuzz TeleOp", group = "competition")
public class Biobuzz_TeleOp extends LinearOpMode {

    // =============================================================================================
    // DRIVE - fields copied verbatim from driveTesting.java
    // =============================================================================================
    MecaTank mecaTank;
    BulkRead bulkRead;

    public static double TELEMETRY_INTERVAL_MS = 200.0;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevOptions, prevDpadLeft;

    // =============================================================================================
    // INTAKE
    // =============================================================================================
    private DcMotorEx intake;
    private NGServo intakeLifter;

    private double liftedPos = BiobuzzRobotConstants.intakeLifterLiftedPos;
    private double loweredPos = BiobuzzRobotConstants.intakeLifterLoweredPos;

    /** Intake power when the A toggle is ON. */
    public static double INTAKE_POWER = 1.0;
    /** Intake power while B is held. */
    public static double INTAKE_REVERSE_POWER = -1.0;
    /** Rollers power while an artifact is being transferred into a flywheel. */
    public static double TRANSFER_POWER = 1.0;

    /** A-toggle state. Starts OFF so nothing spins the moment the opmode starts. */
    private boolean intakeOn = false;
    /** Last power actually written to the hub, so an unchanged command costs nothing. */
    private double lastIntakePower = Double.NaN;

    /** True once the lifter has been raised, either automatically at count or by dpad up. */
    private boolean lifterLifted = false;

    private boolean prevA1, prevA2, prevUp, prevDown, g2PrevUp, g2PrevDown;

    // =============================================================================================
    // COLOUR SENSOR - bands, gain, gate and debounce copied from colorSensorTesting.java
    //
    // TODO-REMEASURE: these were measured on the V2's AMS TCS34725. The V3's Broadcom APDS-9151
    // reports different hues for the same artifacts. Nothing crashes if they are wrong - the counts
    // are simply wrong, which looks exactly like a logic bug. Re-measure on the init screen.
    // =============================================================================================

    /** Red wraps through 0, so it needs two bounds instead of a low/high pair. */
    public static double RED_H_MAX = 20;
    public static double RED_H_MIN = 340;

    public static double BLUE_H_MIN = 170;
    public static double BLUE_H_MAX = 250;

    public static double YELLOW_H_MIN = 44;
    public static double YELLOW_H_MAX = 62;

    /** Shared by all three: below these the hue is not trustworthy enough to classify. */
    public static double S_MIN = 0.3;
    public static double V_MIN = 0.2;

    /** Colour gain. A SOFTWARE multiplier on R, G and B - tune it on the init screen. */
    public static float GAIN = 4.0f;

    /** Reflectance-based, so dark nectar reads further away than bright pollen. Tune against red. */
    public static boolean USE_DISTANCE_GATE = true;
    public static double MAX_DISTANCE_MM = 100.0;

    /** How long one colour must hold steady before it is counted as an artifact. */
    public static double CONFIRM_MS = 60.0;
    /** How long the sensor must read empty afterwards before the next artifact can be counted. */
    public static double CLEAR_MS = 120.0;

    /** True makes the lift count include the opposing alliance's nectar as well. */
    public static boolean COUNT_OPPONENT_NECTAR = false;

    /** The three artifacts, plus the absence of one. */
    private enum Element { NONE, RED_NECTAR, BLUE_NECTAR, YELLOW_POLLEN }

    private enum Alliance { RED, BLUE }

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;   // null if this sensor has no range half
    private RevColorSensorV3 revV3;          // null if what is plugged in is not a V3
    private boolean ledOn = true;

    private Alliance alliance = Alliance.RED;

    /** Pollen + our nectar. This is the count the lifter reacts to. */
    private int artifactCount = 0;
    private int pollenCount = 0;
    private int allianceNectarCount = 0;
    private int opponentNectarCount = 0;

    private Element candidate = Element.NONE;
    private Element lastCounted = Element.NONE;
    private boolean armed = true;
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime clearTimer = new ElapsedTime();

    /** Scratch for Color.colorToHSV, reused every loop rather than reallocated. */
    private final float[] hsv = new float[3];

    /** Cached reading, so telemetry prints the exact sample the classifier acted on. */
    private NormalizedRGBA lastColors;
    private double lastDistanceMm = Double.MAX_VALUE;

    private boolean prevInitB, prevInitX, g2PrevX, g2PrevY;

    // =============================================================================================
    // SHOOTER
    // =============================================================================================

    /**
     * Flywheel PIDF, from FlywheelIntakeTestingOpMode's starting constants. Both flywheels get
     * these same numbers.
     *
     * Three of them cannot be reached through {@code setCustomVelocityPID}, whose P argument
     * NGMotor stores and never reads. The gain that actually acts is scheduled between
     * {@code kP_Recovery} (while the wheel is more than RECOVERY_THRESHOLD BELOW target, i.e. just
     * after an artifact went through) and {@code kP_Stable} (everything else, including all
     * overshoot). Both are package-private in NGMotor, so they are pinned here through
     * {@link NGMotorFlywheelTuner}.
     *
     * Pinning rather than inheriting is deliberate: NGMotor's field initialisers currently hold
     * exactly these values, but they are the scratchpad a tuning session writes to, and a
     * competition teleop should not quietly shoot with whatever was left there.
     */
    public static double KF = 0.0007;
    public static double KP_STABLE = 0.005;
    public static double KP_RECOVERY = 0.0320;
    public static double KI = 0;
    public static double KD = 0;
    public static double ALPHA = 0.7;
    public static double RECOVERY_THRESHOLD = 90.0;

    /** Feeder load compensation. STATIC on NGMotor, so this is shared by both flywheels. */
    public static double KLOAD = 0.8;
    public static double KRAMP = 1.0;

    /** Velocity band counted as "at speed", TICKS/SECOND. This is the gate interlock. */
    public static double AT_SPEED_TOLERANCE = 30.0;

    /** Trigger pull that counts as a request. */
    public static double TRIGGER_THRESHOLD = 0.5;

    // ---- the two velocity tables -----------------------------------------------------------------
    // EVERY ROW IS A PLACEHOLDER AND MUST BE MEASURED. They are all fixedShootingVel on purpose:
    // a flat table shows plainly on telemetry that nothing has been tuned, whereas a plausible
    // made-up curve would hide that until the robot missed at competition. Park the robot at each
    // distance, sweep the velocity until the artifact consistently scores, record it here.
    public static double TABLE_D1_IN = 24.0;
    public static double TABLE_D2_IN = 48.0;
    public static double TABLE_D3_IN = 72.0;
    public static double TABLE_D4_IN = 96.0;

    public static double POLLEN_VEL_D1 = BiobuzzRobotConstants.fixedShootingVel;
    public static double POLLEN_VEL_D2 = BiobuzzRobotConstants.fixedShootingVel;
    public static double POLLEN_VEL_D3 = BiobuzzRobotConstants.fixedShootingVel;
    public static double POLLEN_VEL_D4 = BiobuzzRobotConstants.fixedShootingVel;

    public static double NECTAR_VEL_D1 = BiobuzzRobotConstants.fixedShootingVel;
    public static double NECTAR_VEL_D2 = BiobuzzRobotConstants.fixedShootingVel;
    public static double NECTAR_VEL_D3 = BiobuzzRobotConstants.fixedShootingVel;
    public static double NECTAR_VEL_D4 = BiobuzzRobotConstants.fixedShootingVel;

    private ShooterSide pollenShooter;
    private ShooterSide nectarShooter;

    // =============================================================================================
    // DISTANCE
    // =============================================================================================

    /**
     * Distance used for shooting while the camera is disabled, INCHES. Dashboard-editable, so both
     * tables and the whole feed cycle can be exercised on a bench with no camera and no field.
     * Once the CAMERA CODE blocks are uncommented this becomes the fallback for "no tag in view".
     */
    public static double MANUAL_DISTANCE_IN = 48.0;

    // ===== BEGIN CAMERA CODE - DISABLED =====
    // private TurretVision vision;
    // /** Last ground range the camera measured, INCHES, or NaN when no tag has been seen. */
    // private double lastCameraDistanceIn = Double.NaN;
    // /** How stale a camera distance may be before falling back to MANUAL_DISTANCE_IN, SECONDS. */
    // public static double CAMERA_DISTANCE_MAX_AGE_SEC = 0.5;
    // private final ElapsedTime cameraDistanceAge = new ElapsedTime();
    // ===== END CAMERA CODE - DISABLED =====

    // =============================================================================================
    // One shooter side: a flywheel, the gate that feeds it, and the table it takes velocity from
    // =============================================================================================

    /**
     * Everything one artifact type needs to be shot, so the pollen and nectar paths cannot drift
     * apart. Non-static inner class purely so it can reach {@code telemetry}.
     *
     * The flywheel's own per-loop telemetry is switched OFF here: {@code updateFlywheels()} writes
     * fixed keys ("Current Velocity", "Error"), so two flywheels would overwrite each other's rows
     * and neither number could be trusted. This class prints both sides under its own labels
     * instead.
     */
    private class ShooterSide {
        private final String label;
        private final NGMotor flywheel;
        private final NGServo gate;
        private final LaunchVelocityTable table;
        private final double gateClosedPos;
        private final double gateOpenPos;

        /** Velocity asked of the wheel this loop, TICKS/SECOND. 0 when not spinning up. */
        private double commandedVelocity = 0.0;
        /** Raw velocity read once per loop, and reused by the interlock and telemetry. */
        private double velocity = 0.0;
        private boolean spinningUp = false;
        /** True while the gate is open and the rollers are transferring into this wheel. */
        private boolean feeding = false;
        /** True once power has been cut for this idle period - see the note in update(). */
        private boolean powerCut = false;

        ShooterSide(String label, String motorName, String gateName,
                    double gateClosedPos, double gateOpenPos, LaunchVelocityTable table) {
            this.label = label;
            this.flywheel = new NGMotor(hardwareMap, telemetry, motorName);
            this.gate = new NGServo(hardwareMap, telemetry, gateName);
            this.table = table;
            this.gateClosedPos = gateClosedPos;
            this.gateOpenPos = gateOpenPos;
        }

        void init() {
            // RUN_WITHOUT_ENCODER with BRAKE. updateFlywheels() computes the power itself rather
            // than handing a velocity to the controller's built-in PIDF, but getVelocity() still
            // needs the encoder plugged in - a flat zero velocity while the wheel spins is that
            // cable, not these gains.
            flywheel.init();
            flywheel.setZeroPowerBehavior_Brake();
            flywheel.setTelemetryEnabled(false);

            // The gains that setCustomVelocityPID can carry. Target 0: nothing spins at init.
            flywheel.setCustomVelocityPID(0.0, KP_STABLE, KI, KD, KF);
            powerCut = true;   // nothing has been commanded yet, so there is nothing to cut

            // The gains it cannot - see the KP_RECOVERY comment above. Per-motor instance fields,
            // so each side is pinned separately.
            NGMotorFlywheelTuner.setStableKp(flywheel, KP_STABLE);
            NGMotorFlywheelTuner.setRecoveryKp(flywheel, KP_RECOVERY);
            NGMotorFlywheelTuner.setAlpha(flywheel, ALPHA);
            NGMotorFlywheelTuner.setRecoveryThreshold(flywheel, RECOVERY_THRESHOLD);

            gate.setPosition(gateClosedPos);
        }

        /**
         * One loop of this side.
         *
         * @param distanceIn   range to the target, INCHES, straight into this side's own table
         * @param wantSpinUp   driver is asking for the wheel to come up to speed
         * @param wantFeed     driver is asking to actually shoot
         */
        void update(double distanceIn, boolean wantSpinUp, boolean wantFeed) {
            velocity = flywheel.getVelocity();

            // ---- SPIN-UP EDGE: re-arm the controller, then stand down for one loop --------------
            // Re-arming on every spin-up means each run starts from a clean integrator. This is the
            // ONLY place setCustomVelocityPID is called during play, because it wipes the integral,
            // the last error AND the loop timer - calling it every loop would keep the derivative
            // dividing by a near-zero dt and stop the integral ever accumulating.
            //
            // The return is the point of this branch. It hands the controller a FULL loop period
            // before its first velocity read, instead of letting updateFlywheels() run against the
            // timer it just reset and divide the derivative by a clamped 1 ms dt. KD is 0 today so
            // that spike would be multiplied out, but this stops a future nonzero KD from producing
            // a one-loop power kick on every trigger pull that nothing in telemetry would explain.
            // The cost is one loop of spin-up, a few milliseconds.
            if (wantSpinUp && !spinningUp) {
                commandedVelocity = table.getVelocity(distanceIn);
                flywheel.setCustomVelocityPID(commandedVelocity, KP_STABLE, KI, KD, KF);
                spinningUp = true;
                powerCut = false;
                feeding = false;
                gate.setPosition(gateClosedPos);
                return;
            }
            if (!wantSpinUp && spinningUp) {
                // Clear the filter's memory while the wheel is stopped, so the next spin-up is not
                // dragged by the last run's value.
                NGMotorFlywheelTuner.resetVelocityFilter(flywheel);
            }
            spinningUp = wantSpinUp;

            if (spinningUp) {
                // Plain field write, NOT setCustomVelocityPID: the target moves every loop as the
                // robot drives, and resetting the controller underneath it would make the wheel
                // chase its own reset instead of the new velocity.
                commandedVelocity = table.getVelocity(distanceIn);
                flywheel.targetVelocity = commandedVelocity;

                // THE INTERLOCK. The gate only opens once the wheel is actually at speed.
                feeding = wantFeed && atSpeed();

                // The feeder flag drives NGMotor's kLoad + kRamp*seconds boost, which is what stops
                // the wheel sagging while artifacts are going through.
                flywheel.updateFlywheels(feeding);
            } else {
                commandedVelocity = 0.0;
                feeding = false;
                flywheel.targetVelocity = 0.0;
                // Cut power rather than asking the velocity loop for zero, which would command a
                // large negative power and brake the wheel hard against its own momentum.
                //
                // Once, on the falling edge, not every idle loop: setAbsPower() writes straight to
                // the motor and bypasses NGMotor's power cache, so re-sending the same 0 would be a
                // hub round trip per flywheel per loop to change nothing.
                if (!powerCut) {
                    flywheel.setAbsPower(0.0);
                    powerCut = true;
                }
            }

            gate.setPosition(feeding ? gateOpenPos : gateClosedPos);
        }

        boolean atSpeed() {
            return spinningUp && commandedVelocity > 0
                    && Math.abs(commandedVelocity - velocity) <= AT_SPEED_TOLERANCE;
        }

        boolean isFeeding() {
            return feeding;
        }

        void stop() {
            spinningUp = false;
            feeding = false;
            powerCut = true;
            flywheel.targetVelocity = 0.0;
            flywheel.setAbsPower(0.0);
            gate.setPosition(gateClosedPos);
        }

        void telemetry() {
            telemetry.addData(label, !spinningUp ? "IDLE"
                    : feeding ? "FEEDING" : atSpeed() ? "AT SPEED" : "SPINNING UP");
            telemetry.addData("  target / actual (tps)", "%.0f / %.0f", commandedVelocity, velocity);
            telemetry.addData("  error (tps)", "%.0f", commandedVelocity - velocity);
            telemetry.addData("  power", "%.3f", flywheel.getPower());
            telemetry.addData("  gate", feeding ? "OPEN" : "closed");
            telemetry.addData("  table", table.describe());
        }
    }

    // =============================================================================================
    // OPMODE
    // =============================================================================================

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- DRIVE SETUP - verbatim from driveTesting.java ---------------------------------------
        mecaTank = new MecaTank(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        // MecaTank's internal MecanumDrive forces every hub to AUTO, so take MANUAL back and let
        // one clearCache() per loop define the read cycle.
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();

        // This opmode drives the pose estimate itself, and only when a feature actually needs it.
        mecaTank.setAutoPoseUpdate(false);

        // FIELD CENTRIC ON BY DEFAULT. The rotation is already implemented inside
        // MecaTank.smoothDriveCore(), which reads drive.pose.heading from the tuned localizer every
        // loop - do NOT re-apply it here or the sticks get rotated twice. Switching the flag on
        // also makes smoothDriveNeedsPose() return true, which is what causes updatePoseEstimate()
        // to run each loop below, so the heading the transform uses is never a loop stale.
        MecaTank.FIELD_CENTRIC = true;

        // Define "forward" as wherever the robot is pointing when the opmode starts. Without this
        // the reference is whatever a previous opmode left in the static.
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();
        // ---- end verbatim drive setup -------------------------------------------------------------

        // ---- INTAKE SETUP -------------------------------------------------------------------------
        intake = hardwareMap.get(DcMotorEx.class, BiobuzzRobotConstants.rollers);
        intakeLifter = new NGServo(hardwareMap, telemetry, BiobuzzRobotConstants.intakeLifter);

        // ---- COLOUR SENSOR SETUP ------------------------------------------------------------------
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, BiobuzzRobotConstants.colorSensor);
        colorSensor.setGain(GAIN);

        // Both casts are asked for rather than assumed, so a wrong config entry degrades to "no
        // proximity gate" or "no LED toggle" instead of crashing on the first loop.
        distanceSensor = (colorSensor instanceof DistanceSensor) ? (DistanceSensor) colorSensor : null;
        revV3 = (colorSensor instanceof RevColorSensorV3) ? (RevColorSensorV3) colorSensor : null;
        setLed(ledOn);

        // ---- SHOOTER SETUP ------------------------------------------------------------------------
        // kLoad and kRamp are STATIC on NGMotor, so these two writes cover both flywheels.
        NGMotor.kLoad = KLOAD;
        NGMotor.kRamp = KRAMP;

        pollenShooter = new ShooterSide("POLLEN",
                BiobuzzRobotConstants.pollenFlywheel,
                BiobuzzRobotConstants.pollenGate,
                BiobuzzRobotConstants.pollenGateClosedPos,
                BiobuzzRobotConstants.pollenGateOpenPos,
                buildTable(POLLEN_VEL_D1, POLLEN_VEL_D2, POLLEN_VEL_D3, POLLEN_VEL_D4));

        nectarShooter = new ShooterSide("NECTAR",
                BiobuzzRobotConstants.nectarFlywheel,
                BiobuzzRobotConstants.nectarGate,
                BiobuzzRobotConstants.nectarGateClosedPos,
                BiobuzzRobotConstants.nectarGateOpenPos,
                buildTable(NECTAR_VEL_D1, NECTAR_VEL_D2, NECTAR_VEL_D3, NECTAR_VEL_D4));

        pollenShooter.init();
        nectarShooter.init();

        // ===== BEGIN CAMERA CODE - DISABLED =====
        // TurretVision owns the C920 outright: the VisionPortal, the AprilTagProcessor, the
        // 640x480 stream that matches the SDK's built-in C920 calibration, and the manual
        // exposure/gain/focus lock. Nothing about the camera is re-implemented here.
        //
        // vision = new TurretVision(hardwareMap, telemetry);
        //
        // Mirror the live stream to the dashboard. Null-guarded: a camera that failed to open must
        // not take the opmode down mid-match.
        // if (vision.getCameraStreamSource() != null) {
        //     FtcDashboard.getInstance().startCameraStream(vision.getCameraStreamSource(), 0);
        // }
        // ===== END CAMERA CODE - DISABLED =====

        intakeLifter.setPosition(loweredPos);

        // ---- INIT LOOP: pick the alliance ---------------------------------------------------------
        //
        // A loop rather than a bare waitForStart(), because the answer comes from the gamepad and
        // nothing reads the gamepad unless something is polling it. Live colour readout comes along
        // for free, which is also where GAIN gets tuned: hold an artifact at the working distance
        // and watch V before pressing START.
        //
        // On gamepad1 deliberately. The drive loop does not exist yet, so gamepad1 B and X are free
        // here even though X is heading-hold during play.
        while (opModeInInit()) {
            boolean b = gamepad1.b && !prevInitB;
            boolean x = gamepad1.x && !prevInitX;
            prevInitB = gamepad1.b;
            prevInitX = gamepad1.x;

            if (b) alliance = Alliance.RED;
            if (x) alliance = Alliance.BLUE;

            colorSensor.setGain(GAIN);   // so dashboard edits take effect while aiming
            Element seen = classify(readHsv(), readDistanceMm());

            // ===== BEGIN CAMERA CODE - DISABLED =====
            // vision.update();
            // ===== END CAMERA CODE - DISABLED =====

            telemetry.addLine("BIOBUZZ TELEOP");
            telemetry.addLine("ALLIANCE SELECT - press B for RED, X for BLUE");
            telemetry.addData(">> ALLIANCE", alliance);
            telemetry.addData("Our nectar", alliance == Alliance.RED ? "RED" : "BLUE");
            telemetry.addLine();
            addSensorTelemetry(seen);
            telemetry.addLine();
            telemetry.addLine("Raise GAIN until V reads about 0.4-0.8 on a real artifact,");
            telemetry.addLine("but stop before R, G or B pins at 1.00.");
            telemetry.addLine();
            telemetry.addData("Distance source", "MANUAL %.1f in (camera code disabled)",
                    MANUAL_DISTANCE_IN);
            // ===== BEGIN CAMERA CODE - DISABLED =====
            // telemetry.addData("Camera", vision.getCameraStateString());
            // telemetry.addData("Camera setup", vision.getCameraSetupReport());
            // ===== END CAMERA CODE - DISABLED =====
            telemetry.addLine();
            telemetry.addLine("Flywheels start STOPPED. Clear the shooter before pulling a trigger.");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Seed the edge detectors from the CURRENT state. X selects BLUE on the init screen, so
        // without this a driver still holding X at START would immediately toggle heading hold.
        prevX = gamepad1.x;
        g2PrevX = gamepad2.x;

        stableTimer.reset();
        clearTimer.reset();
        telemetryTimer.reset();
        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();

            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // =========================================================================================
            // DRIVE - gamepad1 ONLY. Verbatim from driveTesting.java, bar the A -> dpad left move.
            // =========================================================================================

            // Heading hold, field centric and traction control all read the pose estimate. Skip the
            // update entirely when none of them are on - it costs two IMU reads.
            if (mecaTank.smoothDriveNeedsPose()) {
                mecaTank.updatePoseEstimate();
            }

            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            // TRANSLATION_HOLD is on DPAD LEFT here, not A. A is the intake toggle on both pads.
            if (gamepad1.dpad_left && !prevDpadLeft) {
                MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            }
            // Heading reset on either button. Re-zeroes the DRIVER's field-centric reference
            // (MecaTank.fieldCentricRef) so the driver can redefine "forward"; odometry x/y and the
            // pose estimate are not touched, so autos and the shooter's geometry stay valid.
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();
            if (gamepad1.options && !prevOptions) mecaTank.resetDriveHeading();
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevDpadLeft = gamepad1.dpad_left;
            prevBack = gamepad1.back;
            prevOptions = gamepad1.options;

            // ARCADE mapping: LEFT stick translates (x = strafe, y = forward), RIGHT stick turns.
            // The four values are passed UN-NEGATED, which is what reverses all four wheels relative
            // to the old tank mapping. Done here rather than with setDirection(REVERSE) on the
            // motors because the two are equivalent for driving but NOT for odometry:
            // RawEncoder.applyDirection() multiplies the encoder count by DcMotorEx.getDirection(),
            // and TwoDeadWheelLocalizer reads the pods through the fl and fr MOTOR ports, so
            // flipping the motors would invert localisation and break every auto.
            //
            // Everything downstream of the sticks is the same code the tank mapping used - see
            // MecaTank.smoothDriveCore() - so the curve, the field-centric rotation, traction
            // control and the accel limiter are unchanged. The triggers no longer strafe.
            mecaTank.setDrivePowersSmoothArcade(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper, gamepad1.right_bumper);

            // =========================================================================================
            // COLOUR SENSOR - count artifacts, and lift the intake once there are enough
            // =========================================================================================
            colorSensor.setGain(GAIN);

            double distanceMm = readDistanceMm();
            Element seen = classify(readHsv(), distanceMm);
            updateCount(seen);

            // THE AUTOMATIC LIFT. Latched on the count rather than re-commanded every loop, so a
            // driver who lowers the lifter by hand at 4 artifacts is not fought by this line.
            if (!lifterLifted && artifactCount >= BiobuzzRobotConstants.artifactLiftCount) {
                intakeLifter.setPosition(liftedPos);
                lifterLifted = true;
            }

            // =========================================================================================
            // INTAKE - A toggles, B reverses, both readable from EITHER pad
            // =========================================================================================
            boolean intakeToggle = (gamepad1.a && !prevA1) || (gamepad2.a && !prevA2);
            prevA1 = gamepad1.a;
            prevA2 = gamepad2.a;
            if (intakeToggle) intakeOn = !intakeOn;

            // Hold to eject. Deliberately NOT a toggle: an intake latched into reverse quietly
            // spits out everything the robot picks up, and nothing on screen looks wrong.
            boolean reverseHeld = gamepad1.b || gamepad2.b;

            // Manual lifter override, on either pad. Dpad down also resets the count, because that
            // is the driver saying "the robot is empty now" - without it the lift would re-latch
            // immediately, the count still being >= 4.
            // Edge-detected on BOTH pads, not just gamepad1: a held dpad-down would otherwise call
            // resetCounts() every loop, which silently stops the sensor ever counting an artifact.
            boolean liftPressed = (gamepad1.dpad_up && !prevUp) || (gamepad2.dpad_up && !g2PrevUp);
            boolean lowerPressed = (gamepad1.dpad_down && !prevDown)
                    || (gamepad2.dpad_down && !g2PrevDown);
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            g2PrevUp = gamepad2.dpad_up;
            g2PrevDown = gamepad2.dpad_down;
            if (liftPressed) {
                intakeLifter.setPosition(liftedPos);
                lifterLifted = true;
            }
            if (lowerPressed) {
                intakeLifter.setPosition(loweredPos);
                lifterLifted = false;
                resetCounts();
            }

            // gamepad2 X resets the count without touching the lifter, for re-arming the counter
            // after a miscount.
            if (gamepad2.x && !g2PrevX) resetCounts();
            g2PrevX = gamepad2.x;

            // Colour sensor LED diagnostic: if the classification barely changes with the LED off,
            // the sensor is reading room light rather than the artifact.
            if (gamepad2.y && !g2PrevY) setLed(!ledOn);
            g2PrevY = gamepad2.y;

            // =========================================================================================
            // SHOOTER - gamepad2 triggers and bumpers
            // =========================================================================================
            double distanceIn = getShotDistanceInches();

            // Bumper = spin up only. Trigger = spin up AND ask to feed. A trigger therefore implies
            // its own spin-up, so pre-spinning on the bumper and then pulling the trigger never
            // drops the wheel.
            boolean pollenFeed = gamepad2.right_trigger > TRIGGER_THRESHOLD;
            boolean nectarFeed = gamepad2.left_trigger > TRIGGER_THRESHOLD;
            boolean pollenSpin = pollenFeed || gamepad2.right_bumper;
            boolean nectarSpin = nectarFeed || gamepad2.left_bumper;

            pollenShooter.update(distanceIn, pollenSpin, pollenFeed);
            nectarShooter.update(distanceIn, nectarSpin, nectarFeed);

            // =========================================================================================
            // ROLLERS - ONE motor, three claims on it, resolved in this order
            //
            // There is a single "rollers" motor: it is the intake AND the transfer. A shot in
            // progress therefore outranks the intake toggle, because an artifact half-way to a
            // flywheel that is already at speed has to keep moving. Releasing the trigger hands the
            // motor straight back to whatever the toggle last asked for - no state to unwind.
            // =========================================================================================
            boolean transferring = pollenShooter.isFeeding() || nectarShooter.isFeeding();

            double intakePower;
            if (transferring) {
                intakePower = TRANSFER_POWER;
            } else if (reverseHeld) {
                intakePower = INTAKE_REVERSE_POWER;
            } else {
                intakePower = intakeOn ? INTAKE_POWER : 0.0;
            }
            // Only write on a change. setPower() is a hub round trip the bulk read does not cover,
            // and this loop runs a few hundred times a second.
            if (intakePower != lastIntakePower) {
                intake.setPower(intakePower);
                lastIntakePower = intakePower;
            }

            // =========================================================================================
            // TELEMETRY
            // =========================================================================================
            double now = System.nanoTime();
            double loopMs = (now - lastLoopTime) / 1e6;
            lastLoopTime = now;
            if (loopMs > loopMsMax) loopMsMax = loopMs;

            if (sendTelemetry) {
                telemetry.addData("ALLIANCE", alliance);

                telemetry.addLine("--- DRIVE ---");
                telemetry.addData("Precision", gamepad1.left_bumper);
                telemetry.addData("Override", gamepad1.right_bumper);
                telemetry.addData("Field centric", MecaTank.FIELD_CENTRIC);
                telemetry.addData("Heading hold", MecaTank.HEADING_HOLD);
                telemetry.addData("Translation hold", MecaTank.TRANSLATION_HOLD);
                telemetry.addData("Traction control", MecaTank.TRACTION_CONTROL);
                mecaTank.smoothDriveTelemetry();

                telemetry.addLine("--- INTAKE ---");
                telemetry.addData("Rollers", "%.2f  %s", intakePower,
                        transferring ? "TRANSFERRING (shot in progress)"
                                : reverseHeld ? "REVERSE (B held)"
                                : intakeOn ? "IN (A toggle)" : "off");
                telemetry.addData("Lifter", "%s (%.2f)",
                        lifterLifted ? "LIFTED" : "lowered",
                        lifterLifted ? liftedPos : loweredPos);

                telemetry.addLine("--- ARTIFACT COUNT ---");
                telemetry.addData(">>> COUNT", "%d / %d%s", artifactCount,
                        BiobuzzRobotConstants.artifactLiftCount,
                        artifactCount >= BiobuzzRobotConstants.artifactLiftCount ? "  LIFT" : "");
                telemetry.addData("    pollen (yellow)", pollenCount);
                telemetry.addData("    our nectar (" + alliance + ")", allianceNectarCount);
                telemetry.addData("    opposing nectar", "%d%s", opponentNectarCount,
                        COUNT_OPPONENT_NECTAR ? " (counted)" : " (NOT counted)");
                telemetry.addData("    last counted", lastCounted);
                addSensorTelemetry(seen);
                telemetry.addData("Armed", armed ? "yes - ready to count"
                        : "no - waiting for the sensor to clear");

                telemetry.addLine("--- SHOOTER ---");
                telemetry.addData("Distance (in)", "%.1f  [%s]", distanceIn, distanceSourceName());
                pollenShooter.telemetry();
                nectarShooter.telemetry();
                // ===== BEGIN CAMERA CODE - DISABLED =====
                // telemetry.addLine("--- VISION ---");
                // telemetry.addData("Camera", vision.getCameraStateString());
                // telemetry.addData("Camera setup", vision.getCameraSetupReport());
                // telemetry.addData("Exposure/gain/WB applied", "%d ms / %d / %d K",
                //         vision.getAppliedExposureMs(), vision.getAppliedGain(),
                //         vision.getAppliedWhiteBalanceK());
                // telemetry.addData("Detections", vision.getLastDetectionCount());
                // telemetry.addData("Reject reason", vision.getLastRejectReason());
                // ===== END CAMERA CODE - DISABLED =====

                telemetry.addData("Loop Time (ms)", loopMs);
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }

        // Leave nothing spinning or open when the opmode ends.
        pollenShooter.stop();
        nectarShooter.stop();
        intake.setPower(0);
        // ===== BEGIN CAMERA CODE - DISABLED =====
        // vision.close();
        // ===== END CAMERA CODE - DISABLED =====
    }

    // =============================================================================================
    // Distance
    // =============================================================================================

    /**
     * Range to the target for the velocity tables, INCHES.
     *
     * CAMERA DISABLED TODAY - this returns {@link #MANUAL_DISTANCE_IN}, which is dashboard-editable,
     * so the tables, the interlock and both gates can all be exercised without a camera or a field.
     * Uncommenting the block below (and the other CAMERA CODE blocks) switches it to the C920 with
     * the manual value as the no-tag fallback.
     */
    private double getShotDistanceInches() {
        // ===== BEGIN CAMERA CODE - DISABLED =====
        // vision.update();
        //
        // TurretVision.getBestTarget() already applies every selection gate - alliance match,
        // coverage, scorable roll, mid-tip rejection - and returns AT MOST one target.
        // TurretConstants.Alliance visionAlliance = (alliance == Alliance.RED)
        //         ? TurretConstants.Alliance.RED : TurretConstants.Alliance.BLUE;
        // TurretTarget target = vision.getBestTarget(visionAlliance);
        //
        // if (target != null && target.usableForEstimate()) {
        //     lastCameraDistanceIn = groundRangeInches(target);
        //     cameraDistanceAge.reset();
        // }
        //
        // A tag drops out of frame constantly while driving, so the last good range is held for
        // CAMERA_DISTANCE_MAX_AGE_SEC rather than collapsing to the fallback the moment one frame
        // misses - that would make the commanded velocity jump mid-shot. Past that age the reading
        // is too stale to shoot on, because the robot has moved since.
        // if (!Double.isNaN(lastCameraDistanceIn)
        //         && cameraDistanceAge.seconds() <= CAMERA_DISTANCE_MAX_AGE_SEC) {
        //     return lastCameraDistanceIn;
        // }
        // ===== END CAMERA CODE - DISABLED =====

        return MANUAL_DISTANCE_IN;
    }

    /** What the distance readout on telemetry is actually coming from. */
    private String distanceSourceName() {
        // ===== BEGIN CAMERA CODE - DISABLED =====
        // if (!Double.isNaN(lastCameraDistanceIn)
        //         && cameraDistanceAge.seconds() <= CAMERA_DISTANCE_MAX_AGE_SEC) {
        //     return String.format("C920, %.2fs old", cameraDistanceAge.seconds());
        // }
        // return "MANUAL - no fresh tag";
        // ===== END CAMERA CODE - DISABLED =====
        return "MANUAL - camera code disabled";
    }

    // ===== BEGIN CAMERA CODE - DISABLED =====
    /*
     * HORIZONTAL ground range from robot centre to the target, INCHES, from one C920 detection.
     *
     * WHY NOT JUST USE target.rangeIn. That field is the STRAIGHT-LINE distance from the LENS to
     * the tag, and neither of those is what a shooter needs:
     *   1. it is a slant range, so it is longer than the ground distance by however much the tag
     *      sits above the camera - at close range, where the angle is steepest, that error is
     *      biggest and the velocity table is steepest too;
     *   2. it is measured from the lens, not from robot centre, so it carries the camera's mount
     *      offset as a fixed bias;
     *   3. a pitched-up camera tilts its own forward axis into the vertical, which shortens the
     *      forward component - ignoring pitch reads the target as nearer than it is.
     *
     * So the detection is transformed properly: roll, then pitch, then yaw, then the mount offset,
     * and only then flattened to a ground range. Every constant comes from TurretConstants, which
     * is where the camera mount pose already lives - do NOT duplicate those numbers here.
     *
     * ftcPose convention, which is NOT the OpenCV optical one: +x RIGHT, +y FORWARD (out of the
     * lens), +z UP. The robot frame is +x FORWARD, +y LEFT, +z UP, hence the sign flip on x.
     *
     * ACCURACY IS BOUNDED BY TurretConstants.CAMERA_* - all of which are still 0.0 TODO-MEASURE.
     * A 0.0 mount pose claims the lens is exactly at robot centre, level and pointing straight
     * forward, which is never true. Measure to the LENS with a tape and a square before trusting a
     * distance from this, because every inch and every degree of mount error lands directly in the
     * range, and then in the commanded velocity.
     *
    private double groundRangeInches(TurretTarget t) {
        double roll  = Math.toRadians(TurretConstants.CAMERA_MOUNT_ROLL_DEG);
        double pitch = Math.toRadians(TurretConstants.CAMERA_MOUNT_PITCH_DEG);
        double yaw   = Math.toRadians(TurretConstants.CAMERA_MOUNT_YAW_DEG);

        // 1. Undo ROLL, a rotation about the camera's own forward axis. It mixes right and up and
        //    leaves forward alone, so it barely moves the range for a centred target - but it does
        //    move the lateral term, and that term is squared into the answer below.
        double camRight = t.camX * Math.cos(roll) + t.camZ * Math.sin(roll);
        double camUp    = -t.camX * Math.sin(roll) + t.camZ * Math.cos(roll);
        double camFwd   = t.camY;

        // 2. Undo PITCH, a rotation about the camera's right axis. This is the one that matters
        //    most: it is what separates "up" from "forward" for a camera aimed above the horizon.
        double forward = camFwd * Math.cos(pitch) - camUp * Math.sin(pitch);
        double left    = -camRight;                 // ftcPose +x is RIGHT; robot +y is LEFT

        // 3. Undo YAW, a rotation about the robot's up axis, CCW-positive.
        double robotFwd  = forward * Math.cos(yaw) - left * Math.sin(yaw);
        double robotLeft = forward * Math.sin(yaw) + left * Math.cos(yaw);

        // 4. Shift from the LENS to ROBOT CENTRE.
        robotFwd  += TurretConstants.CAMERA_OFFSET_X_IN;
        robotLeft += TurretConstants.CAMERA_OFFSET_Y_IN;

        // 5. Flatten. The vertical component is dropped on purpose: the velocity table is indexed
        //    by HORIZONTAL distance, because the target's height above the floor is a constant of
        //    the field and is already baked into every measured row.
        return Math.hypot(robotFwd, robotLeft);
    }
    */
    // ===== END CAMERA CODE - DISABLED =====

    // =============================================================================================
    // Velocity tables
    // =============================================================================================

    /**
     * One {@link LaunchVelocityTable} seeded from four dashboard-editable rows.
     *
     * The table's own constructor pre-seeds placeholders, so it is cleared first - otherwise those
     * defaults would survive at any distance these four rows do not cover. Add more rows wherever
     * the curve bends; near-range rows matter more than far ones, because velocity changes fastest
     * there. Outside the measured range the table CLAMPS rather than extrapolating, so shots fall
     * short beyond the last row instead of flying over the field.
     */
    private LaunchVelocityTable buildTable(double v1, double v2, double v3, double v4) {
        LaunchVelocityTable t = new LaunchVelocityTable();
        t.clear();
        t.add(TABLE_D1_IN, v1);
        t.add(TABLE_D2_IN, v2);
        t.add(TABLE_D3_IN, v3);
        t.add(TABLE_D4_IN, v4);
        return t;
    }

    // =============================================================================================
    // Sensing - copied from colorSensorTesting.java
    // =============================================================================================

    /**
     * One colour reading, converted to H (0-360), S (0-1), V (0-1).
     *
     * toColor() clips anything above 1.0 on the way, which is exactly why GAIN must not be pushed
     * until a channel saturates: two different colours that both clip red land on the same int and
     * become the same hue, and no threshold downstream can tell them apart again.
     */
    private float[] readHsv() {
        lastColors = colorSensor.getNormalizedColors();
        Color.colorToHSV(lastColors.toColor(), hsv);
        return hsv;
    }

    /**
     * Distance in mm, or {@link Double#MAX_VALUE} when there is no usable reading.
     *
     * Both NaN and "no sensor" have to collapse to a large number rather than being passed through,
     * because every comparison against NaN is false, so a raw NaN would slip through the gate's
     * {@code >} test and be treated as close enough to classify.
     */
    private double readDistanceMm() {
        if (distanceSensor == null) return Double.MAX_VALUE;
        double d = distanceSensor.getDistance(DistanceUnit.MM);
        lastDistanceMm = (Double.isNaN(d) || Double.isInfinite(d)) ? Double.MAX_VALUE : d;
        return lastDistanceMm;
    }

    /** Turns one HSV reading into an artifact, or NONE. The bands do not overlap, so first match wins. */
    private Element classify(float[] c, double distanceMm) {
        if (USE_DISTANCE_GATE && distanceMm > MAX_DISTANCE_MM) return Element.NONE;

        double h = c[0], s = c[1], v = c[2];
        if (s < S_MIN || v < V_MIN) return Element.NONE;

        // Red is the wrap-around case: its band is split across the 0/360 seam, so it is the one
        // colour that cannot be written as a single low <= h <= high test.
        if (h <= RED_H_MAX || h >= RED_H_MIN) return Element.RED_NECTAR;
        if (h >= BLUE_H_MIN && h <= BLUE_H_MAX) return Element.BLUE_NECTAR;
        if (h >= YELLOW_H_MIN && h <= YELLOW_H_MAX) return Element.YELLOW_POLLEN;

        return Element.NONE;
    }

    // =============================================================================================
    // Counting
    // =============================================================================================

    /**
     * Debounced edge counter, called once per loop with the current classification.
     *
     * A detection is one ARTIFACT, not one loop iteration. This loop runs a few hundred times a
     * second, so counting every loop that sees yellow would turn one ball sitting still in front of
     * the sensor into thousands of detections - and the lifter would fly up on the first one.
     *
     * Two rules, both in milliseconds rather than loop counts, because loop rate changes with
     * telemetry, with the dashboard connected, and with a drivetrain and two flywheels in the same
     * loop - a debounce measured in loops would silently change length:
     *   1. a colour must classify the same way for CONFIRM_MS before it counts, which throws away
     *      the mixed readings taken while an artifact is half across the sensor's field;
     *   2. afterwards the sensor must read NONE for CLEAR_MS before anything else can count, so an
     *      artifact that momentarily reads as nothing - a highlight, a seam - is not counted twice.
     *
     * The cost of rule 2 is that two touching artifacts count as one, even in different colours.
     * That is deliberate: re-arming on a colour change instead double-counts every artifact whose
     * hue drifts across a band edge on the way past, which is far more common.
     */
    private void updateCount(Element seen) {
        if (seen != candidate) {
            candidate = seen;
            stableTimer.reset();
        }

        if (seen == Element.NONE) {
            // Only a sustained gap re-arms. A single empty loop between two readings of the same
            // artifact must not look like the artifact leaving.
            if (!armed && clearTimer.milliseconds() >= CLEAR_MS) {
                armed = true;
            }
            return;
        }

        // Something is there, so the clear timer restarts from now. Whenever this line stops running
        // for CLEAR_MS straight, the branch above re-arms.
        clearTimer.reset();

        if (armed && stableTimer.milliseconds() >= CONFIRM_MS) {
            count(seen);
            armed = false;
        }
    }

    private void count(Element e) {
        lastCounted = e;

        boolean ours = (e == Element.YELLOW_POLLEN)
                || (e == Element.RED_NECTAR && alliance == Alliance.RED)
                || (e == Element.BLUE_NECTAR && alliance == Alliance.BLUE);

        if (e == Element.YELLOW_POLLEN) {
            pollenCount++;
        } else if (ours) {
            allianceNectarCount++;
        } else {
            opponentNectarCount++;
        }

        // The opposing alliance's nectar is classified and shown, but kept out of the lift count by
        // default: it is worth rejecting, not storing, so it should not fill the robot up.
        if (ours || COUNT_OPPONENT_NECTAR) artifactCount++;
    }

    private void resetCounts() {
        artifactCount = 0;
        pollenCount = 0;
        allianceNectarCount = 0;
        opponentNectarCount = 0;
        lastCounted = Element.NONE;
        candidate = Element.NONE;
        armed = true;
        stableTimer.reset();
        clearTimer.reset();
    }

    // =============================================================================================
    // Plumbing
    // =============================================================================================

    /**
     * Turns the sensor's white illumination LED on or off.
     *
     * Goes through the concrete {@link RevColorSensorV3}, because enableLed() is not on any generic
     * interface and neither REV sensor implements SwitchableLight. Falls back to SwitchableLight for
     * any other device that does, and is a silent no-op when neither applies, so nothing here can
     * fail on a swapped sensor.
     */
    private void setLed(boolean on) {
        ledOn = on;
        if (revV3 != null) {
            revV3.enableLed(on);
        } else if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }

    /** The raw readout, shared by the init loop and the run loop so both show the same thing. */
    private void addSensorTelemetry(Element seen) {
        telemetry.addData("SEEING", seen);
        telemetry.addData("H / S / V", "%.0f deg   %.3f   %.3f", hsv[0], hsv[1], hsv[2]);
        if (lastColors != null) {
            telemetry.addData("R / G / B", "%.3f  %.3f  %.3f",
                    lastColors.red, lastColors.green, lastColors.blue);
        }
        telemetry.addData("Gain", "%.1f (software)", colorSensor.getGain());

        if (distanceSensor == null) {
            telemetry.addData("Sensor distance", "no range sensor under '"
                    + BiobuzzRobotConstants.colorSensor + "'");
        } else if (lastDistanceMm == Double.MAX_VALUE) {
            telemetry.addData("Sensor distance", "out of range");
        } else {
            telemetry.addData("Sensor distance", "%.1f mm (IR reflectance - darker reads further)",
                    lastDistanceMm);
        }
        telemetry.addData("Proximity gate", USE_DISTANCE_GATE
                ? String.format("ON, <= %.0f mm", MAX_DISTANCE_MM) : "OFF");

        // Read back from the device rather than echoed from our own flag, so a driver that silently
        // refused the write shows up here instead of being believed.
        String ledState = (colorSensor instanceof Light)
                ? (((Light) colorSensor).isLightOn() ? "on" : "OFF")
                : "unknown";
        telemetry.addData("LED", revV3 != null || colorSensor instanceof SwitchableLight
                ? ledState + "  (gp2 Y toggles)" : ledState + "  (no control on this device)");

        // Says outright whether a V3 is really what is plugged in. Getting this wrong is the most
        // likely single cause of bad readings, and it is invisible otherwise - a V2 under a V3
        // config still returns numbers, just wrong ones.
        telemetry.addData("Driver", revV3 != null
                ? "RevColorSensorV3 - correct"
                : "NOT a V3: " + colorSensor.getClass().getSimpleName()
                  + " - check the config type is 'REV Color Sensor V3'");
    }
}
