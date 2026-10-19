package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAligner;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

/**
 * BiobuzzShooterTeleOp with POLLEN shooting only: same drive, intake, flower ramp and pollen
 * shooter controls, no nectar wheel. The left trigger and gamepad 2's X do nothing here.
 *
 * <h2>Shared transfer</h2>
 * The transfer ({@code transferRollers}) is one motor feeding both shooters, so while it runs it
 * also pushes on the nectar side. The nectar gate ({@code BiobuzzRobotConstants.nectarGate}) is
 * therefore held CLOSED for the whole opmode if it is configured, and nectar just spins against it.
 *
 * The pollen gate is the feed interlock: it opens only once the right trigger is held AND the
 * wheel is within {@code BiobuzzRobotConstants.flywheelAtSpeedTolerance} of target, then stays
 * open until the trigger is released. The transfer then stays stopped (even with the intake on)
 * until the gate has had time to reach its open position - see {@link #gateTravelMs()} - and
 * then runs until the trigger is released. While it runs the intake is also driven forward if
 * {@code BiobuzzRobotConstants.transferAlsoRunsIntake}.
 *
 * <h2>Flower ramp</h2>
 * {@code BiobuzzRobotConstants.flowerRamp} starts at {@code flowerRampStowedPos}. Right bumper on
 * either pad toggles it between stowed and {@code flowerRampIntakePos}; dropping it to the intake
 * position also switches the intake on if it was off. Stowing it leaves the intake as it was.
 *
 * <h2>CONTROLS - gamepad 1 can do everything alone</h2>
 * <pre>
 * GAMEPAD 1
 *   Left stick .......... drive (field centric)       Right stick X ... turn
 *   LB (hold) precision                               Y field centric   X heading hold
 *   Dpad left ........... translation hold            Back / Options .. re-zero "forward"
 *   A ................... intake + transfer on/off     B (hold) ........ both reverse
 *   RB .................. flower ramp down/up (down also turns the intake on)
 *   Right trigger (hold)  SHOOT POLLEN  (spins up, gate opens once at speed, transfer feeds;
 *                         release closes the gate). With WHEEL_LOCK_ENABLED the robot also holds
 *                         its spot against pushes; with AUTO_ALIGN_ENABLED it turns onto our up
 *                         HIVE CELL and the gate waits for ON TARGET.
 * INIT: B = RED alliance, X = BLUE (either pad) - picks which HIVE tags auto-align uses.
 *   Dpad right .......... flywheel on/off (keep it spinning between shots)
 *   Dpad up / down ...... target velocity +/- VELOCITY_STEP
 *
 * GAMEPAD 2 (optional)
 *   A / B / RB / right trigger / dpad right / dpad up+down ... same as gamepad 1
 * </pre>
 */
@Config
@TeleOp(name = "BioBuzz Drive + Pollen Shooter Test", group = "testing")
public class BiobuzzPollenTeleOp extends LinearOpMode {

    // =============================================================================================
    // DRIVE - as driveTesting
    // =============================================================================================
    public static double TELEMETRY_INTERVAL_MS = 200.0;

    /** TEMPORARY: no acceleration limits, linear sticks. See MecaTank.setFullSpeed. */
    public static boolean FULL_SPEED = true;

    // Flywheel motor, direction, fixed velocity, PIDF, kLoad/kRamp, at-speed tolerance and the
    // intake/transfer powers all come from BiobuzzRobotConstants - the same values the autos use.

    /** One dpad press, TICKS/SECOND. */
    public static double VELOCITY_STEP = 25.0;
    public static double TRIGGER_THRESHOLD = 0.5;
    /**
     * The gate servo has no position feedback, so "at its open position" is timed from its speed:
     * REV Smart Robot Servo, 0.14 s per 60 degrees at 6 V, about 0.17 s on the hub's 5 V servo
     * port. Travel is |open - closed| of its 180-degree range. Plus a margin, MILLISECONDS.
     */
    public static double GATE_SEC_PER_60_DEG = 0.17;
    public static double GATE_RANGE_DEG = 180.0;
    public static double GATE_SETTLE_MARGIN_MS = 40.0;
    /** Transfer power while it runs with the intake (A / B) and while it feeds a shot. */
    public static double
        TRANSFER_INTAKE_POWER = 0.25,
        TRANSFER_FEED_POWER = 0.85;

    /**
     * Flip each motor relative to BiobuzzRobotConstants, for this opmode only (the autos and the
     * other teleops keep the constants as they are). Read once at init.
     */
    public static boolean
        REVERSE_INTAKE = true,
        REVERSE_TRANSFER = false,
        REVERSE_FLYWHEEL = true;

    /**
     * POLLEN counter: counts current spikes on the intake rollers (one per POLLEN pulled in) while
     * the intake runs forward from A. Reaching POLLEN_CAPACITY switches the intake and transfer off
     * once; A turns them back on as usual. A finished shot resets the count.
     * false = the opmode behaves exactly as without it (no current reads at all).
     */
    public static boolean POLLEN_COUNTER_ENABLED = true;
    public static int POLLEN_CAPACITY = 4;
    /** A spike starts this many AMPS above the running baseline and ends below RELEASE above it. */
    public static double SPIKE_RISE_A = 0.5; //0.8
    public static double SPIKE_RELEASE_A = 0.34;
    /** Spikes closer together than this count once, MILLISECONDS. */
    public static double MIN_SPIKE_GAP_MS = 160; //150.0
    /** Ignore the motor's start-up surge for this long after the intake turns on, MILLISECONDS. */
    public static double STARTUP_IGNORE_MS = 300.0;
    /** How fast the baseline follows the current between spikes, 0..1 per loop. */
    public static double BASELINE_ALPHA = 0.05;

    /**
     * WHEEL LOCK while the right trigger is held: the robot actively holds the spot (and heading)
     * where the trigger was pressed on odometry, pushing back against defense instead of just
     * braking. Sticks are ignored while it is locked. false = RT only shoots, as before.
     */
    public static boolean WHEEL_LOCK_ENABLED = true;
    /** in/s of push-back per inch the robot is moved off its spot, and the cap. */
    public static double LOCK_KP = 6.0;
    public static double LOCK_MAX_IN_S = 30.0;
    /** Errors below these are left alone, so the wheels do not buzz on sensor LSBs. */
    public static double LOCK_DEADBAND_IN = 0.15;
    public static double LOCK_HEADING_KP = 6.0;
    public static double LOCK_HEADING_DEADBAND_DEG = 0.3;
    public static double LOCK_MAX_TURN_RATE = 4.0;

    /**
     * AUTO-ALIGN while the right trigger is held: turns to aim at OUR alliance's up HIVE CELL
     * (HiveAligner - odometry runs the heading loop, the camera only locates the CELL), and the gate
     * waits for ON TARGET before the first artifact goes. With the wheel lock it aims in place; without
     * it the left stick still translates. Read at INIT (it opens the camera). false = no camera at all.
     */
    public static boolean AUTO_ALIGN_ENABLED = false;
    /** Start from the pose the last auto stored (field coordinates: can aim before seeing a tag). */
    public static boolean USE_AUTO_END_POSE = true;
    /** Translation while aligning without the wheel lock: full-stick in/s, accel in/s^2, turn rad/s. */
    public static double ALIGN_DRIVE_MAX_IN_S = 40.0;
    public static double ALIGN_DRIVE_ACCEL_IN_S2 = 70.0;
    public static double DRIVER_MAX_TURN_RATE = 3.0;

    private HiveAligner aligner;
    private boolean shotHoldActive = false;
    private Pose2d holdPose;
    private double cmdVx = 0, cmdVy = 0, driverForwardRef = 0;
    private String holdNote = "";

    private MecaTank mecaTank;
    private BulkRead bulkRead;
    private DcMotorEx rollers;
    /** Null when "transferRollers" is not configured - the wheels still spin, nothing feeds. */
    private DcMotorEx transfer;
    private double lastTransferPower = Double.NaN;
    private Shooter pollen;
    /** Held closed all opmode; null when not configured. */
    private Servo nectarGate;

    private boolean intakeOn = false;
    private final PollenCounter counter = new PollenCounter();
    private boolean wasTransferring = false;
    private boolean flywheelOn = false;
    private double lastRollerPower = Double.NaN;
    /** Null when "flowerRamp" is not configured - the toggle then only switches the intake on. */
    private Servo flowerRamp;
    private boolean flowerRampDown = false;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    // gamepad1 drive toggles
    private boolean prevY, prevX, prevBack, prevOptions, prevLeft1;
    // shooter / intake edges, per pad
    private boolean prevA1, prevA2, prevRight1, prevRight2, prevUp1, prevUp2, prevDown1, prevDown2, prevRB1, prevRB2;

    // =============================================================================================
    // The pollen flywheel and its gate - BiobuzzShooterTeleOp's Shooter, pollen gains only.
    // =============================================================================================
    private class Shooter {
        final String label;
        final NGMotor motor;
        String fault = "";

        /** Null when the gate is not in the configuration. Unplugged-but-configured is not null. */
        final Servo gate;
        String gateFault = "";
        boolean gateOpen = false;
        /** Last position written, so an unchanged command costs no hub write. */
        double lastGatePos = Double.NaN;
        final ElapsedTime gateTimer = new ElapsedTime();

        double target;
        double velocity = 0.0;
        boolean spinningUp = false;
        /** Gate open, swung clear and wheel reached speed; held until the gate closes. */
        boolean ready = false;
        /** The transfer is actually pushing into this side this loop (drives the kLoad boost). */
        boolean feeding = false;
        boolean powerCut = true;
        /** Set on the spin-up edge so drive() stands down for that one loop. */
        private boolean armedThisLoop = false;

        Shooter(String label, String motorName, String gateName, boolean reversed, double target) {
            this.label = label;
            this.target = target;
            NGMotor m = null;
            try {
                m = new NGMotor(hardwareMap, telemetry, motorName);
                m.init();   // RUN_WITHOUT_ENCODER + BRAKE; getVelocity() still needs the encoder cable
                m.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                // Printed under its own label instead of updateFlywheels()'s fixed keys.
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


        /** Never opens with no gate configured; with a gate configured but unplugged the write just goes nowhere. */
        void setGate(boolean open) {
            if (gate == null) open = false;
            if (open && !gateOpen) gateTimer.reset();
            gateOpen = open;
            if (gate == null) return;
            double pos = open ? BiobuzzRobotConstants.pollenGateOpenPos : BiobuzzRobotConstants.pollenGateClosedPos;
            if (pos != lastGatePos) {
                gate.setPosition(pos);
                lastGatePos = pos;
            }
        }

        /** setCustomVelocityPID resets the integral and the loop timer - only call on spin-up. */
        private void applyGains(double velocityTarget) {
            double kps = BiobuzzRobotConstants.pollenKpStable;
            motor.setCustomVelocityPID(velocityTarget, kps, BiobuzzRobotConstants.pollenKI,
                    BiobuzzRobotConstants.pollenKD, BiobuzzRobotConstants.pollenKF);
            // The gains setCustomVelocityPID cannot carry: NGMotor ignores the P passed to it and
            // schedules between these two instead.
            NGMotorFlywheelTuner.setStableKp(motor, kps);
            NGMotorFlywheelTuner.setRecoveryKp(motor, BiobuzzRobotConstants.pollenKpRecovery);
            NGMotorFlywheelTuner.setAlpha(motor, BiobuzzRobotConstants.pollenAlpha);
            NGMotorFlywheelTuner.setRecoveryThreshold(motor, BiobuzzRobotConstants.pollenRecoveryThreshold);
        }

        /**
         * First half of the loop: spin state and gate. Call drive() once the shared transfer has
         * been decided.
         * @param wantSpin  spin up (toggle or shoot trigger)
         * @param wantFeed  shoot trigger is held
         */
        void update(boolean wantSpin, boolean wantFeed) {
            update(wantSpin, wantFeed, true);
        }

        /** @param aimReady the aim allows the gate to open (always true without auto-align) */
        void update(boolean wantSpin, boolean wantFeed, boolean aimReady) {
            armedThisLoop = false;
            if (motor == null) {
                setGate(false);
                ready = false;
                return;
            }
            velocity = motor.getVelocity();

            // Spin-up edge: re-arm the controller, then stand down for one loop so its first
            // velocity read gets a full loop period of dt (see Biobuzz_TeleOp.ShooterSide).
            if (wantSpin && !spinningUp) {
                applyGains(target);
                spinningUp = true;
                powerCut = false;
                armedThisLoop = true;
                setGate(false);
                ready = false;
                return;
            }
            if (!wantSpin && spinningUp) NGMotorFlywheelTuner.resetVelocityFilter(motor);
            spinningUp = wantSpin;

            // Open once at speed, then hold open for the rest of the trigger press.
            if (!spinningUp || !wantFeed) setGate(false);
            else if (!gateOpen && atSpeed() && aimReady) setGate(true);
            // Latched like the gate: each artifact dips the wheel below tolerance, and dropping
            // ready on that dip is what made the transfer pulse.
            ready = gateOpen && (ready || (gateTimer.milliseconds() >= gateTravelMs() && atSpeed()));
        }

        /** Second half of the loop. @param loading the transfer is pushing into the wheel */
        void drive(boolean loading) {
            feeding = loading;
            if (motor == null || armedThisLoop) return;
            if (spinningUp) {
                // Plain field write: a dpad change must not reset the controller underneath it.
                motor.targetVelocity = target;
                motor.updateFlywheels(feeding);    // feeding drives the kLoad + kRamp boost
            } else {
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
            ready = false;
            feeding = false;
            powerCut = true;
            motor.targetVelocity = 0.0;
            motor.setAbsPower(0.0);
        }

        void telemetry() {
            String name = label;
            if (motor == null) {
                telemetry.addData(name, "MISSING - " + fault);
            } else {
                telemetry.addData(name, "%s   target %.0f / actual %.0f tps   power %.2f",
                        !spinningUp ? "idle" : feeding ? "FEEDING" : gateOpen && !ready ? "gate opening"
                                : atSpeed() ? "AT SPEED" : "spinning up",
                        target, velocity, motor.getPower());
            }
            telemetry.addData("   " + label + " gate", gate == null ? "not configured - skipped ("
                    + gateFault + ")" : (gateOpen ? "OPEN" : "closed") + String.format(" (%.2f)", lastGatePos));
        }
    }

    /** Time for the gate to swing from closed to fully open, MILLISECONDS. */
    static double gateTravelMs() {
        double deg = Math.abs(BiobuzzRobotConstants.pollenGateOpenPos - BiobuzzRobotConstants.pollenGateClosedPos) * GATE_RANGE_DEG;
        return deg / 60.0 * GATE_SEC_PER_60_DEG * 1000.0 + GATE_SETTLE_MARGIN_MS;
    }

    // =============================================================================================
    // POLLEN counter - intake roller current spikes
    // =============================================================================================
    private class PollenCounter {
        int count = 0;
        /** Set on the spike that reaches POLLEN_CAPACITY; the opmode clears it once acted on. */
        boolean justFilled = false;
        double amps = 0.0, baseline = Double.NaN, lastPeakA = 0.0, peakA = 0.0;
        private boolean running = false, inSpike = false;
        private final ElapsedTime runTimer = new ElapsedTime();
        private final ElapsedTime spikeTimer = new ElapsedTime();

        /** @param counting the intake is running forward from A (not shooting, not reversing) */
        void update(boolean counting) {
            if (!counting) {
                running = false;
                inSpike = false;
                return;
            }
            if (!running) {
                running = true;
                runTimer.reset();
                baseline = Double.NaN;
            }
            amps = rollers.getCurrent(CurrentUnit.AMPS);   // a hub round trip; only read while counting
            if (runTimer.milliseconds() < STARTUP_IGNORE_MS) return;
            if (Double.isNaN(baseline)) baseline = amps;
            double rise = amps - baseline;
            if (!inSpike) {
                if (rise >= SPIKE_RISE_A && (count == 0 || spikeTimer.milliseconds() >= MIN_SPIKE_GAP_MS)) {
                    inSpike = true;
                    peakA = rise;
                    spikeTimer.reset();
                    count++;
                    if (count == POLLEN_CAPACITY) justFilled = true;
                } else {
                    baseline += BASELINE_ALPHA * rise;
                }
            } else {
                peakA = Math.max(peakA, rise);
                if (rise <= SPIKE_RELEASE_A) {
                    inSpike = false;
                    lastPeakA = peakA;
                }
            }
        }

        void reset() {
            count = 0;
            justFilled = false;
            inSpike = false;
            running = false;
        }
    }

    /**
     * Drives while RT is held: wheel lock (hold the spot on odometry), auto-align (heading from
     * HiveAligner), or both. Everything is a velocity through Road Runner's kinematics.
     */
    private void driveShotHold(Pose2d pose, boolean lockOn, boolean alignOn, double dt) {
        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        double wantVx, wantVy;
        if (lockOn) {
            double ex = holdPose.position.x - x, ey = holdPose.position.y - y;
            double e = Math.hypot(ex, ey);
            double v = e < LOCK_DEADBAND_IN ? 0.0 : Math.min(LOCK_MAX_IN_S, LOCK_KP * e);
            wantVx = e > 1e-9 ? ex / e * v : 0.0;
            wantVy = e > 1e-9 ? ey / e * v : 0.0;
            cmdVx = wantVx;
            cmdVy = wantVy;
        } else {
            double speed = ALIGN_DRIVE_MAX_IN_S * (gamepad1.left_bumper ? MecaTank.PRECISION_SCALE : 1.0);
            double fwd = -gamepad1.left_stick_y * speed, left = -gamepad1.left_stick_x * speed;
            double ref = MecaTank.FIELD_CENTRIC ? driverForwardRef : h;
            wantVx = Math.cos(ref) * fwd - Math.sin(ref) * left;
            wantVy = Math.sin(ref) * fwd + Math.cos(ref) * left;
            double ddx = wantVx - cmdVx, ddy = wantVy - cmdVy;
            double d = Math.hypot(ddx, ddy), step = ALIGN_DRIVE_ACCEL_IN_S2 * dt;
            if (d > step) {
                ddx *= step / d;
                ddy *= step / d;
            }
            cmdVx += ddx;
            cmdVy += ddy;
        }

        double turn = alignOn ? aligner.turnRate(pose, cmdVx, cmdVy, dt, !pollen.ready) : Double.NaN;
        if (Double.isNaN(turn)) {
            if (lockOn) {
                double err = holdPose.heading.toDouble() - h;
                while (err > Math.PI) err -= 2 * Math.PI;
                while (err <= -Math.PI) err += 2 * Math.PI;
                turn = Math.abs(Math.toDegrees(err)) < LOCK_HEADING_DEADBAND_DEG ? 0.0
                        : Range.clip(LOCK_HEADING_KP * err, -LOCK_MAX_TURN_RATE, LOCK_MAX_TURN_RATE);
            } else {
                turn = -gamepad1.right_stick_x * DRIVER_MAX_TURN_RATE;
            }
        } else if (lockOn) {
            // Aiming turns the robot about its centre, so the held spot is unchanged; only the
            // heading it holds follows the aim.
            holdPose = new Pose2d(holdPose.position, h);
        }
        holdNote = (lockOn ? String.format("WHEELS LOCKED, %.2f in off the spot", Math.hypot(holdPose.position.x - x, holdPose.position.y - y))
                : "driving with the left stick")
                + (alignOn ? (aligner.hasTarget() ? (aligner.onTarget() ? ", ON TARGET" : ", aligning") : ", no CELL seen - heading held") : "");

        double c = Math.cos(h), sn = Math.sin(h);
        HiveAligner.driveRobotVelocity(mecaTank, c * cmdVx + sn * cmdVy, -sn * cmdVx + c * cmdVy, turn);
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
        rollers.setDirection(REVERSE_INTAKE ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        try {
            transfer = hardwareMap.get(DcMotorEx.class, BiobuzzRobotConstants.transferRollers);
            transfer.setDirection(BiobuzzRobotConstants.transferReversed != REVERSE_TRANSFER
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            transfer = null;
        }

        NGMotor.kLoad = BiobuzzRobotConstants.flywheelKLoad;
        NGMotor.kRamp = BiobuzzRobotConstants.flywheelKRamp;
        pollen = new Shooter("POLLEN", BiobuzzRobotConstants.pollenFlywheel, BiobuzzRobotConstants.pollenGate,
                BiobuzzRobotConstants.pollenFlywheelReversed != REVERSE_FLYWHEEL, BiobuzzRobotConstants.pollenFixedShootingVel);
        try {
            nectarGate = hardwareMap.get(Servo.class, BiobuzzRobotConstants.nectarGate);
            nectarGate.setPosition(BiobuzzRobotConstants.nectarGateClosedPos);
        } catch (Exception e) {
            nectarGate = null;
        }
        try {
            flowerRamp = hardwareMap.get(Servo.class, BiobuzzRobotConstants.flowerRamp);
            flowerRamp.setPosition(BiobuzzRobotConstants.flowerRampStowedPos);
        } catch (Exception e) {
            flowerRamp = null;
        }

        telemetry.addLine("BIOBUZZ DRIVE + POLLEN SHOOTER TEST");
        telemetry.addLine("RT = shoot pollen   dpad right = flywheel on/off");
        telemetry.addLine("A = intake   B = reverse   RB = flower ramp   dpad up/down = target velocity");
        telemetry.addLine("Flywheel starts STOPPED.");
        if (pollen.motor == null) telemetry.addLine("POLLEN FLYWHEEL MISSING: " + pollen.fault);
        if (transfer == null) telemetry.addLine("TRANSFER MOTOR MISSING: '"
                + BiobuzzRobotConstants.transferRollers + "' - nothing can feed the flywheel");
        if (flowerRamp == null) telemetry.addLine("FLOWER RAMP MISSING: '" + BiobuzzRobotConstants.flowerRamp + "'");
        telemetry.update();

        boolean alignAtInit = AUTO_ALIGN_ENABLED;
        if (alignAtInit) aligner = new HiveAligner(hardwareMap, mecaTank, AllianceSelector.red());
        while (!isStarted() && !isStopRequested()) {
            AllianceSelector.poll(gamepad1, gamepad2);
            if (aligner != null) aligner.setAlliance(AllianceSelector.red());
            telemetry.addLine("BIOBUZZ DRIVE + POLLEN SHOOTER TEST");
            AllianceSelector.show(telemetry);
            telemetry.addData("Wheel lock (RT)", WHEEL_LOCK_ENABLED ? "ON" : "off");
            telemetry.addData("Auto-align (RT)", aligner == null ? "off"
                    : "ON - aims at the up " + (AllianceSelector.red() ? "RED" : "BLUE") + " CELL, camera " + aligner.cameraState());
            if (pollen.motor == null) telemetry.addLine("POLLEN FLYWHEEL MISSING: " + pollen.fault);
            if (transfer == null) telemetry.addLine("TRANSFER MOTOR MISSING");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            if (aligner != null) aligner.close();
            return;
        }
        boolean red = AllianceSelector.red();
        if (aligner != null) aligner.setAlliance(red);
        Pose2d stored = red ? PoseStorage.getLastRedPose() : PoseStorage.getLastBluePose();
        if (USE_AUTO_END_POSE && stored != null) {
            mecaTank.setPoseEstimate(stored);
            if (aligner != null) aligner.setFieldReferenced();
        }
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();
        driverForwardRef = mecaTank.getPoseEstimate().heading.toDouble();

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
            boolean shootHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD
                    || gamepad2.right_trigger > TRIGGER_THRESHOLD;
            boolean lockOn = WHEEL_LOCK_ENABLED && shootHeld;
            boolean alignOn = AUTO_ALIGN_ENABLED && aligner != null && shootHeld;
            long nowNanos = System.nanoTime();
            double dt = Range.clip((nowNanos - lastLoopTime) / 1e9, 1e-4, 0.1);
            if (aligner != null || WHEEL_LOCK_ENABLED || mecaTank.smoothDriveNeedsPose()) {
                mecaTank.updatePoseEstimate();
            }
            Pose2d pose = mecaTank.getPoseEstimate();
            PoseVelocity2d velRobot = mecaTank.getMeasuredRobotVelocity();
            if (aligner != null) aligner.update(nowNanos, pose, velRobot, dt);
            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            if (gamepad1.dpad_left && !prevLeft1) MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            if ((gamepad1.back && !prevBack) || (gamepad1.options && !prevOptions)) {
                mecaTank.resetDriveHeading();
                driverForwardRef = pose.heading.toDouble();
            }
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevLeft1 = gamepad1.dpad_left;
            prevBack = gamepad1.back;
            prevOptions = gamepad1.options;

            if (lockOn || alignOn) {
                if (!shotHoldActive) {
                    shotHoldActive = true;
                    holdPose = pose;
                    double c = Math.cos(pose.heading.toDouble()), sn = Math.sin(pose.heading.toDouble());
                    double vx = velRobot == null ? 0 : velRobot.linearVel.x, vy = velRobot == null ? 0 : velRobot.linearVel.y;
                    cmdVx = c * vx - sn * vy;
                    cmdVy = sn * vx + c * vy;
                    if (alignOn) aligner.engage(pose, velRobot);
                }
                driveShotHold(pose, lockOn, alignOn, dt);
            } else {
                if (shotHoldActive) {
                    shotHoldActive = false;
                    if (aligner != null) aligner.release();
                    // Hand back from the robot's real motion: no lurch, heading hold re-latches here.
                    mecaTank.resetSmoothDriveState(velRobot);
                }
                mecaTank.setDrivePowersSmoothArcade(
                        gamepad1.left_stick_x, gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_bumper, false);
            }

            // =====================================================================================
            // INTAKE - A toggles, B (hold) reverses, either pad
            // =====================================================================================
            if ((gamepad1.a && !prevA1) || (gamepad2.a && !prevA2)) intakeOn = !intakeOn;
            prevA1 = gamepad1.a;
            prevA2 = gamepad2.a;
            boolean reverseHeld = gamepad1.b || gamepad2.b;

            // =====================================================================================
            // FLOWER RAMP - RB toggles, either pad. Dropping it turns the intake on.
            // =====================================================================================
            if ((gamepad1.right_bumper && !prevRB1) || (gamepad2.right_bumper && !prevRB2)) {
                flowerRampDown = !flowerRampDown;
                if (flowerRampDown) intakeOn = true;
                if (flowerRamp != null) flowerRamp.setPosition(flowerRampDown
                        ? BiobuzzRobotConstants.flowerRampIntakePos : BiobuzzRobotConstants.flowerRampStowedPos);
            }
            prevRB1 = gamepad1.right_bumper;
            prevRB2 = gamepad2.right_bumper;

            // =====================================================================================
            // SHOOTER
            // =====================================================================================
            if ((gamepad1.dpad_right && !prevRight1) || (gamepad2.dpad_right && !prevRight2)) {
                flywheelOn = !flywheelOn;
            }
            prevRight1 = gamepad1.dpad_right;
            prevRight2 = gamepad2.dpad_right;

            boolean pollenFeed = gamepad1.right_trigger > TRIGGER_THRESHOLD
                    || gamepad2.right_trigger > TRIGGER_THRESHOLD;

            boolean up = (gamepad1.dpad_up && !prevUp1) || (gamepad2.dpad_up && !prevUp2);
            boolean down = (gamepad1.dpad_down && !prevDown1) || (gamepad2.dpad_down && !prevDown2);
            prevUp1 = gamepad1.dpad_up;
            prevUp2 = gamepad2.dpad_up;
            prevDown1 = gamepad1.dpad_down;
            prevDown2 = gamepad2.dpad_down;
            if (up) pollen.target += VELOCITY_STEP;
            if (down) pollen.target = Math.max(0.0, pollen.target - VELOCITY_STEP);

            boolean aimReady = aligner == null || !aligner.engaged() || !aligner.hasTarget() || aligner.onTarget();
            pollen.update(flywheelOn || pollenFeed, pollenFeed, aimReady);

            // =====================================================================================
            // INTAKE + TRANSFER - one motor each. The transfer feeds the wheel once the pollen gate
            // is open and the wheel ready, and otherwise follows the intake (in with A, out with B)
            // so artifacts are carried up to the closed gate. A shot can drive the intake forward too.
            // =====================================================================================
            boolean transferring = pollen.ready;
            pollen.drive(transferring);
            if (POLLEN_COUNTER_ENABLED) {
                // A finished shot empties the robot: the feed ran and the trigger is released.
                if (wasTransferring && !pollenFeed) counter.reset();
                counter.update(intakeOn && !reverseHeld && !pollenFeed && !transferring);
                if (counter.justFilled) {
                    intakeOn = false;
                    counter.justFilled = false;
                }
            } else if (counter.count > 0) {
                counter.reset();
            }
            wasTransferring = transferring || (wasTransferring && pollenFeed);
            double transferPower = transferring ? TRANSFER_FEED_POWER
                    : pollenFeed ? 0.0
                    : reverseHeld ? -TRANSFER_INTAKE_POWER
                    : intakeOn ? TRANSFER_INTAKE_POWER : 0.0;
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
                telemetry.addLine("--- SHOOTER ---");
                pollen.telemetry();
                telemetry.addData("Flywheel held on", flywheelOn);
                telemetry.addData("Nectar gate", nectarGate == null ? "not configured" : "held closed");
                telemetry.addData("Intake", "%.2f  %s", rollerPower,
                        transferring && BiobuzzRobotConstants.transferAlsoRunsIntake ? "pushing to transfer"
                                : reverseHeld ? "REVERSE (B held)" : intakeOn ? "IN (A toggle)" : "off");
                telemetry.addData("Transfer", transfer == null ? "MISSING"
                        : String.format("%.2f  %s", transferPower, transferring ? "FEEDING"
                                : pollenFeed ? "waiting for gate / wheel"
                                : reverseHeld ? "REVERSE (B held)" : intakeOn ? "with intake" : "off"));
                if (POLLEN_COUNTER_ENABLED) {
                    telemetry.addData("POLLEN count", "%d / %d%s", counter.count, POLLEN_CAPACITY,
                            counter.count >= POLLEN_CAPACITY ? "  FULL" : "");
                    telemetry.addData("   intake current", "%.2f A  baseline %.2f A  last spike +%.2f A",
                            counter.amps, Double.isNaN(counter.baseline) ? 0.0 : counter.baseline, counter.lastPeakA);
                }
                telemetry.addData("Flower ramp", (flowerRampDown ? "DOWN (intake)" : "stowed")
                        + (flowerRamp == null ? " - not configured" : ""));

                if (shotHoldActive) {
                    telemetry.addData("SHOT HOLD", holdNote);
                }
                if (aligner != null && (shotHoldActive || AUTO_ALIGN_ENABLED)) aligner.telemetry(telemetry);
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
        if (aligner != null) aligner.close();
        rollers.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
    }
}
