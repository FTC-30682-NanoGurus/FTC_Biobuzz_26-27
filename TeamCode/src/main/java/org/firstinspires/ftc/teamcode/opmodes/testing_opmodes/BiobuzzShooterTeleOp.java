package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;

/**
 * Drive + intake + both flywheels + both gates, no velocity table.
 *
 * <ul>
 *   <li><b>Driving</b> is driveTesting's code, including FULL_SPEED. One binding moved, the same
 *       way Biobuzz_TeleOp moved it: translation hold is on DPAD LEFT because A is the intake.</li>
 *   <li><b>Pollen flywheel</b> runs the gains tuned in "Flywheel + Intake Testing (no drive)",
 *       from BiobuzzRobotConstants, on the motor that file tuned ({@code pollenFlywheel}).</li>
 *   <li><b>Nectar flywheel</b> runs the gains tuned in {@link NectarFlywheelIntakeTestingOpMode}.</li>
 *   <li><b>Intake</b> uses Biobuzz_TeleOp's controls: A toggles, B (hold) reverses.</li>
 * </ul>
 *
 * <h2>Gates, and why the opmode does not depend on them</h2>
 * Each flywheel has its own gate servo ({@code BiobuzzRobotConstants.pollenGate} / {@code nectarGate},
 * positions {@code pollenGateOpenPos} etc.). A gate swings OPEN while its shoot trigger is held and
 * back to CLOSED the moment it is released.
 *
 * The feed interlock is still in software: the rollers only push an artifact in once the wheel is
 * within {@code BiobuzzRobotConstants.flywheelAtSpeedTolerance} of its target. Nothing ever waits on, or reads back from, a
 * gate - a servo cannot report its position anyway - so shooting behaves the same with the gates
 * configured but unplugged (setPosition goes nowhere) or not in the configuration at all (reported
 * on telemetry, then skipped).
 *
 * The intake ({@code rollers}) and the transfer into the shooter ({@code transferRollers}) are
 * separate motors. The transfer only runs while a wheel is feeding. While it does, the intake is
 * also driven forward if {@code BiobuzzRobotConstants.transferAlsoRunsIntake}, so artifacts still
 * in the intake follow through - that outranks the intake toggle and reverse for the shot.
 *
 * <h2>Fixed target velocities</h2>
 * Each wheel has ONE target, starting at {@code BiobuzzRobotConstants.pollenFixedShootingVel} /
 * {@code nectarFixedShootingVel},
 * changed live with the dpad. Dpad up/down edits whichever wheel is SELECTED (telemetry shows it):
 * the last one you shot or pre-spun. Changes take effect immediately, even mid spin-up.
 *
 * <h2>CONTROLS - gamepad 1 can do everything alone</h2>
 * <pre>
 * GAMEPAD 1
 *   Left stick .......... drive (field centric)       Right stick X ... turn
 *   LB (hold) precision   RB (hold) override          Y field centric   X heading hold
 *   Dpad left ........... translation hold            Back / Options .. re-zero "forward"
 *   A ................... intake on/off                B (hold) ........ intake reverse
 *   Right trigger (hold)  SHOOT POLLEN  (gate opens, spins up, feeds once at speed; release
 *                         closes the gate. Also selects pollen)
 *   Left trigger (hold) . SHOOT NECTAR  (same, nectar gate and wheel)
 *   Dpad right .......... both flywheels on/off (keep them spinning between shots)
 *   Dpad up / down ...... selected flywheel's target velocity +/- VELOCITY_STEP
 *
 * GAMEPAD 2 (optional - same shooter and intake buttons, plus)
 *   A / B / triggers / dpad right / dpad up+down ... same as gamepad 1
 *   Right bumper (hold) . pre-spin POLLEN only and select it
 *   Left bumper (hold) .. pre-spin NECTAR only and select it
 *   X ................... switch the selected flywheel
 * </pre>
 */
@Config
@TeleOp(name = "BioBuzz Drive + Shooter Test", group = "testing")
public class BiobuzzShooterTeleOp extends LinearOpMode {

    // =============================================================================================
    // DRIVE - as driveTesting
    // =============================================================================================
    public static double TELEMETRY_INTERVAL_MS = 200.0;

    /** TEMPORARY: no acceleration limits, linear sticks. See MecaTank.setFullSpeed. */
    public static boolean FULL_SPEED = true;

    // Flywheel motors, directions, fixed velocities, PIDF, kLoad/kRamp, at-speed tolerance and the
    // intake/transfer powers all come from BiobuzzRobotConstants - the same values the autos use.

    /** One dpad press, TICKS/SECOND. */
    public static double VELOCITY_STEP = 25.0;
    public static double TRIGGER_THRESHOLD = 0.5;

    private MecaTank mecaTank;
    private BulkRead bulkRead;
    private DcMotorEx rollers;
    /** Null when "transferRollers" is not configured - the wheels still spin, nothing feeds. */
    private DcMotorEx transfer;
    private double lastTransferPower = Double.NaN;
    private Shooter pollen, nectar, selected;

    private boolean intakeOn = false;
    private boolean flywheelsOn = false;
    private double lastRollerPower = Double.NaN;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    // gamepad1 drive toggles
    private boolean prevY, prevX, prevBack, prevOptions, prevLeft1;
    // shooter / intake edges, per pad
    private boolean prevA1, prevA2, prevRight1, prevRight2, prevUp1, prevUp2, prevDown1, prevDown2, prevX2;

    // =============================================================================================
    // One flywheel. Same spin-up / interlock logic as Biobuzz_TeleOp's ShooterSide, minus the gate
    // and the velocity table.
    // =============================================================================================
    private class Shooter {
        final String label;
        final NGMotor motor;
        String fault = "";
        final boolean pollenGains;

        /** Null when the gate is not in the configuration. Unplugged-but-configured is not null. */
        final Servo gate;
        String gateFault = "";
        boolean gateOpen = false;
        /** Last position written, so an unchanged command costs no hub write. */
        double lastGatePos = Double.NaN;

        double target;
        double velocity = 0.0;
        boolean spinningUp = false;
        boolean feeding = false;
        boolean powerCut = true;

        Shooter(String label, String motorName, String gateName, boolean reversed, double target,
                boolean pollenGains) {
            this.label = label;
            this.target = target;
            this.pollenGains = pollenGains;
            NGMotor m = null;
            try {
                m = new NGMotor(hardwareMap, telemetry, motorName);
                m.init();   // RUN_WITHOUT_ENCODER + BRAKE; getVelocity() still needs the encoder cable
                m.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                // updateFlywheels() writes fixed telemetry keys, so two wheels would overwrite each
                // other's rows. Each Shooter prints under its own label instead.
                m.setTelemetryEnabled(false);
            } catch (Exception e) {
                // A missing motor costs that wheel, not the whole opmode.
                m = null;
                fault = "'" + motorName + "' not in the configuration";
            }
            motor = m;
            if (motor != null) applyGains(0.0);

            Servo g = null;
            try {
                g = hardwareMap.get(Servo.class, gateName);
            } catch (Exception e) {
                gateFault = "'" + gateName + "' not in the configuration";
            }
            gate = g;
            setGate(false);
        }

        double gateOpenPos() {
            return pollenGains ? BiobuzzRobotConstants.pollenGateOpenPos : BiobuzzRobotConstants.nectarGateOpenPos;
        }

        double gateClosedPos() {
            return pollenGains ? BiobuzzRobotConstants.pollenGateClosedPos : BiobuzzRobotConstants.nectarGateClosedPos;
        }

        /** Safe with no gate configured; with a gate configured but unplugged the write just goes nowhere. */
        void setGate(boolean open) {
            gateOpen = open;
            if (gate == null) return;
            double pos = open ? gateOpenPos() : gateClosedPos();
            if (pos != lastGatePos) {
                gate.setPosition(pos);
                lastGatePos = pos;
            }
        }

        /** setCustomVelocityPID resets the integral and the loop timer - only call on spin-up. */
        private void applyGains(double velocityTarget) {
            double kf = pollenGains ? BiobuzzRobotConstants.pollenKF : BiobuzzRobotConstants.nectarKF;
            double kps = pollenGains ? BiobuzzRobotConstants.pollenKpStable : BiobuzzRobotConstants.nectarKpStable;
            double kpr = pollenGains ? BiobuzzRobotConstants.pollenKpRecovery : BiobuzzRobotConstants.nectarKpRecovery;
            double ki = pollenGains ? BiobuzzRobotConstants.pollenKI : BiobuzzRobotConstants.nectarKI;
            double kd = pollenGains ? BiobuzzRobotConstants.pollenKD : BiobuzzRobotConstants.nectarKD;
            motor.setCustomVelocityPID(velocityTarget, kps, ki, kd, kf);
            // The gains setCustomVelocityPID cannot carry: NGMotor ignores the P passed to it and
            // schedules between these two instead.
            NGMotorFlywheelTuner.setStableKp(motor, kps);
            NGMotorFlywheelTuner.setRecoveryKp(motor, kpr);
            NGMotorFlywheelTuner.setAlpha(motor, pollenGains ? BiobuzzRobotConstants.pollenAlpha : BiobuzzRobotConstants.nectarAlpha);
            NGMotorFlywheelTuner.setRecoveryThreshold(motor,
                    pollenGains ? BiobuzzRobotConstants.pollenRecoveryThreshold : BiobuzzRobotConstants.nectarRecoveryThreshold);
        }

        /**
         * @param wantSpin  spin up (toggle, pre-spin bumper or trigger)
         * @param wantFeed  shoot trigger held: opens the gate, and feeds once at speed
         */
        void update(boolean wantSpin, boolean wantFeed) {
            // The gate follows the trigger, independent of the flywheel - so it still opens and
            // closes on cue even if this wheel's motor is missing.
            setGate(wantFeed);
            if (motor == null) return;
            velocity = motor.getVelocity();

            // Spin-up edge: re-arm the controller, then stand down for one loop so its first
            // velocity read gets a full loop period of dt (see Biobuzz_TeleOp.ShooterSide).
            if (wantSpin && !spinningUp) {
                applyGains(target);
                spinningUp = true;
                powerCut = false;
                feeding = false;
                return;
            }
            if (!wantSpin && spinningUp) NGMotorFlywheelTuner.resetVelocityFilter(motor);
            spinningUp = wantSpin;

            if (spinningUp) {
                // Plain field write: a dpad change must not reset the controller underneath it.
                motor.targetVelocity = target;
                feeding = wantFeed && atSpeed();   // THE INTERLOCK
                motor.updateFlywheels(feeding);    // feeding drives the kLoad + kRamp boost
            } else {
                feeding = false;
                motor.targetVelocity = 0.0;
                // Cut power once rather than asking the loop for 0, which would brake the wheel
                // hard against its own momentum.
                if (!powerCut) {
                    motor.setAbsPower(0.0);
                    powerCut = true;
                }
            }
        }

        boolean atSpeed() {
            return spinningUp && target > 0 && Math.abs(target - velocity) <= BiobuzzRobotConstants.flywheelAtSpeedTolerance;
        }

        void stop() {
            setGate(false);
            if (motor == null) return;
            spinningUp = false;
            feeding = false;
            powerCut = true;
            motor.targetVelocity = 0.0;
            motor.setAbsPower(0.0);
        }

        void telemetry() {
            String name = (this == selected ? ">> " : "   ") + label;
            if (motor == null) {
                telemetry.addData(name, "MISSING - " + fault);
            } else {
                telemetry.addData(name, "%s   target %.0f / actual %.0f tps   power %.2f",
                        !spinningUp ? "idle" : feeding ? "FEEDING" : atSpeed() ? "AT SPEED" : "spinning up",
                        target, velocity, motor.getPower());
            }
            telemetry.addData("   " + label + " gate", gate == null ? "not configured - skipped ("
                    + gateFault + ")" : (gateOpen ? "OPEN" : "closed") + String.format(" (%.2f)", lastGatePos));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- DRIVE SETUP - as driveTesting --------------------------------------------------------
        mecaTank = new MecaTank(hardwareMap, telemetry, new Pose2d(0, 0, 0));
        // MecaTank's internal MecanumDrive forces every hub to AUTO, so take MANUAL back and let
        // one clearCache() per loop define the read cycle.
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();
        mecaTank.setAutoPoseUpdate(false);
        MecaTank.FIELD_CENTRIC = true;
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();

        // ---- INTAKE + SHOOTER SETUP ---------------------------------------------------------------
        rollers = hardwareMap.get(DcMotorEx.class, BiobuzzRobotConstants.rollers);
        try {
            transfer = hardwareMap.get(DcMotorEx.class, BiobuzzRobotConstants.transferRollers);
            transfer.setDirection(BiobuzzRobotConstants.transferReversed
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            transfer = null;
        }
        NGMotor.kLoad = BiobuzzRobotConstants.flywheelKLoad;
        NGMotor.kRamp = BiobuzzRobotConstants.flywheelKRamp;
        pollen = new Shooter("POLLEN", BiobuzzRobotConstants.pollenFlywheel, BiobuzzRobotConstants.pollenGate,
                BiobuzzRobotConstants.pollenFlywheelReversed, BiobuzzRobotConstants.pollenFixedShootingVel, true);
        nectar = new Shooter("NECTAR", BiobuzzRobotConstants.nectarFlywheel, BiobuzzRobotConstants.nectarGate,
                BiobuzzRobotConstants.nectarFlywheelReversed, BiobuzzRobotConstants.nectarFixedShootingVel, false);
        selected = pollen;

        telemetry.addLine("BIOBUZZ DRIVE + SHOOTER TEST");
        telemetry.addLine("RT = shoot pollen   LT = shoot nectar   dpad right = flywheels on/off");
        telemetry.addLine("A = intake   B = reverse   dpad up/down = selected target velocity");
        telemetry.addLine("Flywheels start STOPPED.");
        if (pollen.motor == null) telemetry.addLine("POLLEN FLYWHEEL MISSING: " + pollen.fault);
        if (nectar.motor == null) telemetry.addLine("NECTAR FLYWHEEL MISSING: " + nectar.fault);
        if (transfer == null) telemetry.addLine("TRANSFER MOTOR MISSING: '"
                + BiobuzzRobotConstants.transferRollers + "' - nothing can feed the flywheels");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetryTimer.reset();
        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();
            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // =====================================================================================
            // DRIVE - gamepad1, as driveTesting (translation hold moved from A to dpad left)
            // =====================================================================================
            mecaTank.setFullSpeed(FULL_SPEED);
            if (mecaTank.smoothDriveNeedsPose()) {
                mecaTank.updatePoseEstimate();
            }
            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            if (gamepad1.dpad_left && !prevLeft1) MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();
            if (gamepad1.options && !prevOptions) mecaTank.resetDriveHeading();
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevLeft1 = gamepad1.dpad_left;
            prevBack = gamepad1.back;
            prevOptions = gamepad1.options;

            mecaTank.setDrivePowersSmoothArcade(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper, gamepad1.right_bumper);

            // =====================================================================================
            // INTAKE - A toggles, B (hold) reverses, either pad
            // =====================================================================================
            if ((gamepad1.a && !prevA1) || (gamepad2.a && !prevA2)) intakeOn = !intakeOn;
            prevA1 = gamepad1.a;
            prevA2 = gamepad2.a;
            boolean reverseHeld = gamepad1.b || gamepad2.b;

            // =====================================================================================
            // SHOOTER
            // =====================================================================================
            if ((gamepad1.dpad_right && !prevRight1) || (gamepad2.dpad_right && !prevRight2)) {
                flywheelsOn = !flywheelsOn;
            }
            prevRight1 = gamepad1.dpad_right;
            prevRight2 = gamepad2.dpad_right;

            boolean pollenFeed = gamepad1.right_trigger > TRIGGER_THRESHOLD
                    || gamepad2.right_trigger > TRIGGER_THRESHOLD;
            boolean nectarFeed = gamepad1.left_trigger > TRIGGER_THRESHOLD
                    || gamepad2.left_trigger > TRIGGER_THRESHOLD;

            // Whatever you last reached for is what the dpad edits.
            if (pollenFeed || gamepad2.right_bumper) selected = pollen;
            if (nectarFeed || gamepad2.left_bumper) selected = nectar;
            if (gamepad2.x && !prevX2) selected = selected == pollen ? nectar : pollen;
            prevX2 = gamepad2.x;

            boolean up = (gamepad1.dpad_up && !prevUp1) || (gamepad2.dpad_up && !prevUp2);
            boolean down = (gamepad1.dpad_down && !prevDown1) || (gamepad2.dpad_down && !prevDown2);
            prevUp1 = gamepad1.dpad_up;
            prevUp2 = gamepad2.dpad_up;
            prevDown1 = gamepad1.dpad_down;
            prevDown2 = gamepad2.dpad_down;
            if (up) selected.target += VELOCITY_STEP;
            if (down) selected.target = Math.max(0.0, selected.target - VELOCITY_STEP);

            pollen.update(flywheelsOn || pollenFeed || gamepad2.right_bumper, pollenFeed);
            nectar.update(flywheelsOn || nectarFeed || gamepad2.left_bumper, nectarFeed);

            // =====================================================================================
            // INTAKE + TRANSFER - separate motors. The transfer runs only while a wheel feeds; the
            // intake follows its toggle/reverse, except that a shot can drive it forward too.
            // =====================================================================================
            boolean transferring = pollen.feeding || nectar.feeding;
            double transferPower = transferring ? BiobuzzRobotConstants.transferPower : 0.0;
            double rollerPower = transferring && BiobuzzRobotConstants.transferAlsoRunsIntake ? BiobuzzRobotConstants.transferPower
                    : reverseHeld ? BiobuzzRobotConstants.intakeReversePower
                    : intakeOn ? BiobuzzRobotConstants.intakePower : 0.0;
            if (rollerPower != lastRollerPower) {   // setPower is a hub round trip; skip repeats
                rollers.setPower(rollerPower);
                lastRollerPower = rollerPower;
            }
            if (transfer != null && transferPower != lastTransferPower) {
                transfer.setPower(transferPower);
                lastTransferPower = transferPower;
            }

            // =====================================================================================
            // TELEMETRY
            // =====================================================================================
            double now = System.nanoTime();
            double loopMs = (now - lastLoopTime) / 1e6;
            lastLoopTime = now;
            if (loopMs > loopMsMax) loopMsMax = loopMs;

            if (sendTelemetry) {
                telemetry.addLine("--- SHOOTER (>> = dpad up/down edits this one) ---");
                pollen.telemetry();
                nectar.telemetry();
                telemetry.addData("Flywheels held on", flywheelsOn);
                telemetry.addData("Intake", "%.2f  %s", rollerPower,
                        transferring && BiobuzzRobotConstants.transferAlsoRunsIntake ? "pushing to transfer"
                                : reverseHeld ? "REVERSE (B held)" : intakeOn ? "IN (A toggle)" : "off");
                telemetry.addData("Transfer", transfer == null ? "MISSING"
                        : String.format("%.2f  %s", transferPower, transferring ? "FEEDING" : "off"));

                telemetry.addLine("--- DRIVE ---");
                telemetry.addData("FULL SPEED (temporary)", FULL_SPEED);
                telemetry.addData("Precision", gamepad1.left_bumper);
                telemetry.addData("Field centric", MecaTank.FIELD_CENTRIC);
                telemetry.addData("Heading hold", MecaTank.HEADING_HOLD);
                telemetry.addData("Translation hold", MecaTank.TRANSLATION_HOLD);
                mecaTank.smoothDriveTelemetry();
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }

        pollen.stop();
        nectar.stop();
        rollers.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
    }
}
