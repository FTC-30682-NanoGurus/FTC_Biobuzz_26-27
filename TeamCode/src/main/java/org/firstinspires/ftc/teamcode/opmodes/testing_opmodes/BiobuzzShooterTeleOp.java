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

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAligner;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.LaunchVelocityTable;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

/**
 * Drive + intake + flower ramp + BOTH shooters (pollen and nectar), with every feature of
 * BiobuzzPollenTeleOp except the pollen counter: velocity table, gate-travel timing, transfer
 * reverse on release, wheel lock, auto-align, drive while aiming, auto end pose. Each feature has
 * its own boolean below, as in BiobuzzPollenTeleOp.
 *
 * <h2>The robot</h2>
 * Both shooters face out of the robot's RIGHT side and sit on its front-back centre line, NECTAR in
 * front of POLLEN. The camera faces the same way. Their geometry lives in HiveAligner
 * (LAUNCHER_YAW_DEG, POLLEN_/NECTAR_LAUNCHER_OFFSET_X_IN) and TurretConstants (camera mount).
 *
 * <h2>One transfer, two gates</h2>
 * The intake ({@code rollers}) and the transfer ({@code transferRollers}) are one motor each, and
 * the transfer feeds BOTH shooters, so any shoot request (or the flywheel toggle) spins up both
 * wheels and the transfer pushes on both sides whenever it runs. The gates release them
 * separately: holding only RT opens only the pollen gate (nectar spins against its closed gate),
 * holding only LT opens only the nectar gate, holding both opens both.
 *
 * A gate opens once its trigger is held AND its wheel is within
 * {@code BiobuzzRobotConstants.flywheelAtSpeedTolerance} of target (and, with auto-align, the aim is
 * ON TARGET), then stays open until that trigger is released. The transfer stays stopped while a
 * trigger is held until every open gate has had time to swing open (timed from the servo speed)
 * and its wheel is ready, then feeds until the triggers are released.
 *
 * <h2>Aiming (AUTO_ALIGN_ENABLED)</h2>
 * RT aims the POLLEN shooter's exit at the CELL, LT the NECTAR shooter's, both triggers the point
 * midway between them. The CELL is OUR CELL on the robot's own half of the field (same sign of x as
 * the robot), up or down - shots across the HIVE are impossible, so cross over for the other CELL.
 * The gate is NOT held for a down CELL: that is the driver's call.
 *
 * <h2>Start pose</h2>
 * With USE_AUTO_END_POSE the pose the last auto stored (PoseStorage, field coordinates) is used;
 * without one, BiobuzzRobotConstants.teleopDefaultStart* (in our LOADING ZONE, facing away from our
 * drivers) - put the robot there. With auto-align on, the camera then corrects x/y the first time
 * it tracks one of our CELLS, and again whenever odometry has drifted more than
 * HiveAligner.RELOCALIZE_THRESHOLD_IN (never while aiming).
 *
 * <h2>CONTROLS - gamepad 1 can do everything alone</h2>
 * <pre>
 * GAMEPAD 1
 *   Left stick .......... drive (field centric)       Right stick X ... turn
 *   LB (hold) precision                               Y field centric   X heading hold
 *   Dpad left ........... translation hold            Back / Options .. re-zero "forward"
 *   A ................... intake + transfer on/off     B (hold) ........ both reverse
 *   RB .................. flower ramp down/up (down also turns the intake on)
 *   Right trigger (hold)  SHOOT POLLEN   } wheel lock / auto-align while either is held
 *   Left trigger (hold) . SHOOT NECTAR   } (both held = both gates, aim between the shooters)
 *   Dpad right .......... both flywheels on/off (keep them spinning between shots)
 *   Dpad up / down ...... selected flywheel's target +/- VELOCITY_STEP (a trim with the table)
 *   
 * GAMEPAD 2 (optional)
 *   A / B / RB / triggers / dpad right / dpad up+down ... same as gamepad 1
 *   X ................... switch the selected flywheel (the last one shot is selected anyway)
 *   BACK ................ auto-align off / back on (off: the triggers shoot without aiming)
 * INIT: B = RED alliance, X = BLUE (either pad).
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

    // Flywheel motors, directions, fixed velocities, PIDF, kLoad/kRamp, at-speed tolerance, gate
    // positions and the intake/transfer powers all come from BiobuzzRobotConstants.

    /** One dpad press, TICKS/SECOND. */
    public static double VELOCITY_STEP = 25.0;
    /**
     * Each flywheel's velocity from its distance table by the HIVE distance (HiveAligner, so it
     * needs AUTO_ALIGN_ENABLED), interpolated between rows and clamped to the end rows. A target
     * only changes while its gate is closed, so a volley is shot at one speed. Dpad up/down trims
     * the selected wheel on top of its table. With no distance it falls back to the fixed velocity.
     * Pollen rows: LaunchVelocityTable's constructor. Nectar rows: NECTAR_TABLE_* below.
     * false = the fixed velocities with dpad steps.
     */
    public static boolean USE_VELOCITY_TABLE = false;
    /** NECTAR distance table, INCHES -> TICKS/SECOND. PLACEHOLDERS (flat) - measure on the robot. */
    public static double[] NECTAR_TABLE_DISTANCES_IN = {24.0, 48.0, 72.0, 96.0};
    public static double[] NECTAR_TABLE_VELOCITIES_TPS = {1400.0, 1400.0, 1400.0, 1400.0};
    public static double TRIGGER_THRESHOLD = 0.5;
    /**
     * The gate servos have no position feedback, so "open" is timed from their speed: REV Smart
     * Robot Servo, ~0.17 s per 60 degrees on the hub's 5 V port, over a 180-degree range, plus a
     * margin, from each gate's own closed/open positions.
     */
    public static double GATE_SEC_PER_60_DEG = 0.17;
    public static double GATE_RANGE_DEG = 180.0;
    public static double GATE_SETTLE_MARGIN_MS = 40.0;
    /**
     * When the triggers are released, run the transfer in REVERSE for TRANSFER_RELEASE_REVERSE_MS
     * from the moment a gate is commanded shut, pulling the next artifact back so it cannot jam
     * under the closing gate. false = the transfer just stops on release.
     */
    public static boolean TRANSFER_REVERSE_ON_RELEASE = true;
    public static double TRANSFER_RELEASE_REVERSE_MS = 1400.0;
    public static double TRANSFER_RELEASE_REVERSE_POWER = 1.0;

    /** Flip each motor relative to BiobuzzRobotConstants, for this opmode only. Read once at init. */
    public static boolean
        REVERSE_INTAKE = true,
        REVERSE_TRANSFER = false,
        REVERSE_POLLEN_FLYWHEEL = false,
        REVERSE_NECTAR_FLYWHEEL = false;

    /**
     * WHEEL LOCK while either trigger is held: the robot actively holds the spot (and heading)
     * where the trigger was pressed on odometry, pushing back against defense. Sticks are ignored
     * while it is locked. false = the triggers only shoot.
     */
    public static boolean WHEEL_LOCK_ENABLED = true;
    /** in/s of push-back per inch the robot is moved off its spot, and the cap. */
    public static double LOCK_KP = 6.5;
    public static double LOCK_MAX_IN_S = 30.0;
    /** Errors below these are left alone, so the wheels do not buzz on sensor LSBs. */
    public static double LOCK_DEADBAND_IN = 0.15;
    public static double LOCK_HEADING_KP = 6.0;
    public static double LOCK_HEADING_DEADBAND_DEG = 0.3;
    public static double LOCK_MAX_TURN_RATE = 4.0;

    /**
     * AUTO-ALIGN while a trigger is held: turns the firing shooter onto OUR CELL on the robot's half
     * (HiveAligner - odometry runs the heading loop, the camera locates the CELL and corrects the
     * pose), and the gates wait for ON TARGET. Read at INIT (it opens the camera). false = no camera.
     */
    public static boolean AUTO_ALIGN_ENABLED = false;

    /**
     * Start from the pose the last auto stored. false (or none stored) = the default start pose
     * BiobuzzRobotConstants.teleopDefaultStart* (alliance frame), corrected by the camera.
     */
    public static boolean USE_AUTO_END_POSE = true;
    /** Translation while aligning without the wheel lock: full-stick in/s, accel in/s^2, turn rad/s. */
    public static double ALIGN_DRIVE_MAX_IN_S = 40.0;
    public static double ALIGN_DRIVE_ACCEL_IN_S2 = 70.0;
    public static double DRIVER_MAX_TURN_RATE = 3.0;

    /**
     * DRIVE WHILE AIMING: while the wheel lock AND auto-align are both active, the left stick moves
     * the locked spot, so the robot can be repositioned while it aims (and is still pushed back to
     * that moving spot). The stick tops out at AIMING_DRIVE_MAX_IN_S, and artifacts only start
     * feeding while the robot is slower than AIMING_FEED_MAX_IN_S. With the shooters facing RIGHT,
     * driving forward/back is what carries a shot sideways off the CELL.
     * false = the lock holds a fixed spot and ignores the sticks.
     */
    public static boolean DRIVE_WHILE_AIMING_ENABLED = false;
    public static double AIMING_DRIVE_MAX_IN_S = 14.0;
    /** The first artifact of a volley waits until the speed is below this, in/s (resumes below 80%). */
    public static double AIMING_FEED_MAX_IN_S = 11.0;
    /** true = also pause the transfer mid-volley whenever the speed goes over AIMING_FEED_MAX_IN_S. */
    public static boolean PAUSE_FEED_WHEN_TOO_FAST = false;
    /** The moving spot never runs more than this ahead of the robot, in. */
    public static double AIMING_LEASH_IN = 3.0;
    public static double AIMING_DRIVE_ACCEL_IN_S2 = 40.0;

    private HiveAligner aligner;
    private final LaunchVelocityTable pollenTable = new LaunchVelocityTable();
    private final LaunchVelocityTable nectarTable = new LaunchVelocityTable();
    private boolean shotHoldActive = false;
    private Pose2d holdPose;
    private double cmdVx = 0, cmdVy = 0, driverForwardRef = 0;
    private double stickVx = 0, stickVy = 0;
    private boolean slowEnoughToFeed = true;
    private String holdNote = "";

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
    /** Null when "flowerRamp" is not configured - the toggle then only switches the intake on. */
    private Servo flowerRamp;
    private boolean flowerRampDown = false;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevOptions, prevLeft1;
    private boolean alignOffByDriver = false, prevBack2 = false;
    private boolean prevA1, prevA2, prevRight1, prevRight2, prevUp1, prevUp2, prevDown1, prevDown2, prevX2, prevRB1, prevRB2;

    // =============================================================================================
    // One flywheel and its gate. The gate is the interlock because the transfer is shared.
    // =============================================================================================
    private class Shooter {
        final String label;
        final NGMotor motor;
        String fault = "";
        final boolean pollenGains;
        final LaunchVelocityTable table;
        final double fixedVel;
        double trim = 0.0;
        String velocityNote = "fixed";

        /** Null when the gate is not in the configuration. Unplugged-but-configured is not null. */
        final Servo gate;
        String gateFault = "";
        boolean gateOpen = false;
        double lastGatePos = Double.NaN;
        final ElapsedTime gateTimer = new ElapsedTime();
        /** Since the gate was last commanded shut after being open; starts far in the past. */
        final ElapsedTime closeTimer = new ElapsedTime(0);
        private boolean everClosedAfterOpen = false;

        double target;
        double velocity = 0.0;
        boolean spinningUp = false;
        /** Gate open, swung clear and wheel reached speed; held until the gate closes. */
        boolean ready = false;
        /** The transfer is actually pushing into this side this loop (drives the kLoad boost). */
        boolean feeding = false;
        boolean powerCut = true;
        private boolean armedThisLoop = false;

        Shooter(String label, String motorName, String gateName, boolean reversed, double target,
                boolean pollenGains, LaunchVelocityTable table) {
            this.label = label;
            this.target = target;
            this.fixedVel = target;
            this.pollenGains = pollenGains;
            this.table = table;
            NGMotor m = null;
            try {
                m = new NGMotor(hardwareMap, telemetry, motorName);
                m.init();   // RUN_WITHOUT_ENCODER + BRAKE; getVelocity() still needs the encoder cable
                m.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                // updateFlywheels() writes fixed telemetry keys, so two wheels would overwrite each
                // other's rows. Each Shooter prints under its own label instead.
                m.setTelemetryEnabled(false);
            } catch (Exception e) {
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

        /** Time for this gate to swing from closed to fully open, MILLISECONDS. */
        double gateTravelMs() {
            double deg = Math.abs(gateOpenPos() - gateClosedPos()) * GATE_RANGE_DEG;
            return deg / 60.0 * GATE_SEC_PER_60_DEG * 1000.0 + GATE_SETTLE_MARGIN_MS;
        }

        /** True for TRANSFER_RELEASE_REVERSE_MS after the gate was commanded shut following a shot. */
        boolean closing() {
            return everClosedAfterOpen && !gateOpen && closeTimer.milliseconds() < TRANSFER_RELEASE_REVERSE_MS;
        }

        void setGate(boolean open) {
            if (gate == null) open = false;
            if (open && !gateOpen) gateTimer.reset();
            if (!open && gateOpen) {
                closeTimer.reset();
                everClosedAfterOpen = true;
            }
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
            NGMotorFlywheelTuner.setStableKp(motor, kps);
            NGMotorFlywheelTuner.setRecoveryKp(motor, kpr);
            NGMotorFlywheelTuner.setAlpha(motor, pollenGains ? BiobuzzRobotConstants.pollenAlpha : BiobuzzRobotConstants.nectarAlpha);
            NGMotorFlywheelTuner.setRecoveryThreshold(motor,
                    pollenGains ? BiobuzzRobotConstants.pollenRecoveryThreshold : BiobuzzRobotConstants.nectarRecoveryThreshold);
        }

        /** Velocity table (or fixed) target, only while the gate is closed: one speed per volley. */
        void updateTarget(double hiveIn) {
            if (!USE_VELOCITY_TABLE || gateOpen) return;
            double base;
            if (Double.isNaN(hiveIn)) {
                base = fixedVel;
                velocityNote = aligner == null ? "fixed - no HIVE distance (auto-align off)" : "fixed - no HIVE distance yet";
            } else {
                base = table.getVelocity(hiveIn);
                velocityNote = String.format("table %.0f tps at %.1f in", base, hiveIn);
            }
            target = Math.max(0.0, base + trim);
        }

        /**
         * First half of the loop: spin state and gate. Call drive() once the shared transfer has
         * been decided.
         * @param wantSpin  spin up (toggle or either trigger - the wheels spin up together)
         * @param wantFeed  this side's trigger is held
         * @param aimReady  the aim allows the gate to open (always true without auto-align)
         */
        void update(boolean wantSpin, boolean wantFeed, boolean aimReady) {
            armedThisLoop = false;
            if (motor == null) {
                setGate(false);
                ready = false;
                return;
            }
            velocity = motor.getVelocity();

            // Spin-up edge: re-arm the controller, then stand down for one loop so its first
            // velocity read gets a full loop period of dt.
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

            // Open once at speed (and aimed), then hold open for the rest of the trigger press.
            if (!spinningUp || !wantFeed) setGate(false);
            else if (!gateOpen && atSpeed() && aimReady) setGate(true);
            // Latched: each artifact dips the wheel below tolerance, and dropping ready on that
            // dip would pulse the transfer.
            ready = gateOpen && (ready || (gateTimer.milliseconds() >= gateTravelMs() && atSpeed()));
        }

        /** Second half of the loop. @param loading the transfer is pushing into this side */
        void drive(boolean loading) {
            feeding = loading;
            if (motor == null || armedThisLoop) return;
            if (spinningUp) {
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
            String name = (this == selected ? ">> " : "   ") + label;
            if (motor == null) {
                telemetry.addData(name, "MISSING - " + fault);
            } else {
                telemetry.addData(name, "%s   target %.0f / actual %.0f tps   power %.2f",
                        !spinningUp ? "idle" : feeding ? "FEEDING" : gateOpen && !ready ? "gate opening"
                                : atSpeed() ? "AT SPEED" : "spinning up",
                        target, velocity, motor.getPower());
                telemetry.addData("   " + label + " velocity", USE_VELOCITY_TABLE
                        ? velocityNote + (trim != 0.0 ? String.format(", trim %+.0f", trim) : "") : "fixed (dpad steps)");
            }
            telemetry.addData("   " + label + " gate", gate == null ? "not configured - skipped ("
                    + gateFault + ")" : (gateOpen ? "OPEN" : "closed") + String.format(" (%.2f)", lastGatePos));
        }
    }

    /**
     * Drives while a trigger is held: wheel lock (hold the spot on odometry), auto-align (heading
     * from HiveAligner), or both. Everything is a velocity through Road Runner's kinematics.
     */
    private void driveShotHold(Pose2d pose, boolean lockOn, boolean alignOn, boolean moveWhileAiming,
                               boolean anyReady, double dt) {
        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        double wantVx, wantVy;
        if (lockOn && moveWhileAiming) {
            // The stick moves the locked spot; the lock then tracks it with the stick velocity as
            // feedforward, so driving makes no error and only an outside push gets corrected.
            double speed = AIMING_DRIVE_MAX_IN_S * (gamepad1.left_bumper ? MecaTank.PRECISION_SCALE : 1.0);
            double fwd = -gamepad1.left_stick_y * speed, left = -gamepad1.left_stick_x * speed;
            double ref = MecaTank.FIELD_CENTRIC ? driverForwardRef : h;
            wantVx = Math.cos(ref) * fwd - Math.sin(ref) * left;
            wantVy = Math.sin(ref) * fwd + Math.cos(ref) * left;
            double ddx = wantVx - stickVx, ddy = wantVy - stickVy;
            double d = Math.hypot(ddx, ddy), step = AIMING_DRIVE_ACCEL_IN_S2 * dt;
            if (d > step) {
                ddx *= step / d;
                ddy *= step / d;
            }
            stickVx += ddx;
            stickVy += ddy;
            double hx = holdPose.position.x + stickVx * dt, hy = holdPose.position.y + stickVy * dt;
            double lx = hx - x, ly = hy - y, lead = Math.hypot(lx, ly);
            if (lead > AIMING_LEASH_IN) {
                hx = x + lx * AIMING_LEASH_IN / lead;
                hy = y + ly * AIMING_LEASH_IN / lead;
            }
            holdPose = new Pose2d(hx, hy, holdPose.heading.toDouble());
            double ex = hx - x, ey = hy - y;
            double e = Math.hypot(ex, ey);
            double v = e < LOCK_DEADBAND_IN ? 0.0 : Math.min(LOCK_MAX_IN_S, LOCK_KP * e);
            cmdVx = stickVx + (e > 1e-9 ? ex / e * v : 0.0);
            cmdVy = stickVy + (e > 1e-9 ? ey / e * v : 0.0);
        } else if (lockOn) {
            double ex = holdPose.position.x - x, ey = holdPose.position.y - y;
            double e = Math.hypot(ex, ey);
            double v = e < LOCK_DEADBAND_IN ? 0.0 : Math.min(LOCK_MAX_IN_S, LOCK_KP * e);
            cmdVx = e > 1e-9 ? ex / e * v : 0.0;
            cmdVy = e > 1e-9 ? ey / e * v : 0.0;
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

        double turn = alignOn ? aligner.turnRate(pose, cmdVx, cmdVy, dt, !anyReady) : Double.NaN;
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
        holdNote = (lockOn ? String.format(moveWhileAiming ? "LOCKED, stick moves the spot (%.2f in off)%s"
                        : "WHEELS LOCKED, %.2f in off the spot%s", Math.hypot(holdPose.position.x - x, holdPose.position.y - y),
                        moveWhileAiming && !slowEnoughToFeed && (!anyReady || PAUSE_FEED_WHEN_TOO_FAST)
                                ? ", TOO FAST TO START FEEDING" : "")
                : "driving with the left stick")
                + (alignOn ? (aligner.hasTarget() ? (aligner.onTarget() ? ", ON TARGET" : ", aligning") : ", no CELL known - heading held") : "");

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
        rollers.setDirection(BiobuzzRobotConstants.intakeReversed != REVERSE_INTAKE
                ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        try {
            transfer = hardwareMap.get(DcMotorEx.class, BiobuzzRobotConstants.transferRollers);
            transfer.setDirection(BiobuzzRobotConstants.transferReversed != REVERSE_TRANSFER
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            transfer = null;
        }

        nectarTable.clear();
        for (int i = 0; i < Math.min(NECTAR_TABLE_DISTANCES_IN.length, NECTAR_TABLE_VELOCITIES_TPS.length); i++) {
            nectarTable.add(NECTAR_TABLE_DISTANCES_IN[i], NECTAR_TABLE_VELOCITIES_TPS[i]);
        }
        NGMotor.kLoad = BiobuzzRobotConstants.flywheelKLoad;
        NGMotor.kRamp = BiobuzzRobotConstants.flywheelKRamp;
        pollen = new Shooter("POLLEN", BiobuzzRobotConstants.pollenFlywheel, BiobuzzRobotConstants.pollenGate,
                BiobuzzRobotConstants.pollenFlywheelReversed != REVERSE_POLLEN_FLYWHEEL,
                BiobuzzRobotConstants.pollenFixedShootingVel, true, pollenTable);
        nectar = new Shooter("NECTAR", BiobuzzRobotConstants.nectarFlywheel, BiobuzzRobotConstants.nectarGate,
                BiobuzzRobotConstants.nectarFlywheelReversed != REVERSE_NECTAR_FLYWHEEL,
                BiobuzzRobotConstants.nectarFixedShootingVel, false, nectarTable);
        selected = pollen;
        try {
            flowerRamp = hardwareMap.get(Servo.class, BiobuzzRobotConstants.flowerRamp);
            flowerRamp.setPosition(BiobuzzRobotConstants.flowerRampStowedPos);
        } catch (Exception e) {
            flowerRamp = null;
        }

        if (AUTO_ALIGN_ENABLED) aligner = new HiveAligner(hardwareMap, mecaTank, AllianceSelector.red());
        while (!isStarted() && !isStopRequested()) {
            AllianceSelector.poll(gamepad1, gamepad2);
            if (aligner != null) aligner.setAlliance(AllianceSelector.red());
            boolean red = AllianceSelector.red();
            Pose2d stored = red ? PoseStorage.getLastRedPose() : PoseStorage.getLastBluePose();
            telemetry.addLine("BIOBUZZ DRIVE + SHOOTER TEST");
            telemetry.addLine("RT = pollen   LT = nectar   dpad right = flywheels on/off");
            telemetry.addLine("A = intake   B = reverse   RB = flower ramp   dpad up/down = selected velocity");
            AllianceSelector.show(telemetry);
            telemetry.addData("Start pose", USE_AUTO_END_POSE && stored != null
                    ? String.format("from the last auto: (%.1f, %.1f) %.0f deg", stored.position.x, stored.position.y,
                    Math.toDegrees(stored.heading.toDouble()))
                    : "DEFAULT - put the robot in our LOADING ZONE, facing away from our drivers"
                    + (aligner == null ? "" : " (the camera corrects x/y)"));
            telemetry.addData("Wheel lock (triggers)", WHEEL_LOCK_ENABLED ? "ON" : "off");
            telemetry.addData("Auto-align (triggers)", aligner == null ? "off"
                    : "ON - our CELL on the robot's half, camera " + aligner.cameraState());
            if (pollen.motor == null) telemetry.addLine("POLLEN FLYWHEEL MISSING: " + pollen.fault);
            if (nectar.motor == null) telemetry.addLine("NECTAR FLYWHEEL MISSING: " + nectar.fault);
            if (transfer == null) telemetry.addLine("TRANSFER MOTOR MISSING: '"
                    + BiobuzzRobotConstants.transferRollers + "' - nothing can feed the flywheels");
            if (flowerRamp == null) telemetry.addLine("FLOWER RAMP MISSING: '" + BiobuzzRobotConstants.flowerRamp + "'");
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
        boolean guessedPose = !(USE_AUTO_END_POSE && stored != null);
        Pose2d start = guessedPose ? PoseStorage.allianceToField(red, new Pose2d(
                BiobuzzRobotConstants.teleopDefaultStartX, BiobuzzRobotConstants.teleopDefaultStartY,
                Math.toRadians(BiobuzzRobotConstants.teleopDefaultStartHeadingDeg))) : stored;
        mecaTank.setPoseEstimate(start);
        if (aligner != null) aligner.setFieldReferenced(guessedPose);
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();
        driverForwardRef = mecaTank.getPoseEstimate().heading.toDouble();

        telemetryTimer.reset();
        double lastLoopTime = System.nanoTime();
        long lastControlNanos = System.nanoTime();
        double loopMsMax = 0;

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();
            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // =====================================================================================
            // DRIVE - gamepad1, as driveTesting (translation hold moved from A to dpad left)
            // =====================================================================================
            mecaTank.setFullSpeed(FULL_SPEED);
            boolean pollenFeed = gamepad1.right_trigger > TRIGGER_THRESHOLD
                    || gamepad2.right_trigger > TRIGGER_THRESHOLD;
            boolean nectarFeed = gamepad1.left_trigger > TRIGGER_THRESHOLD
                    || gamepad2.left_trigger > TRIGGER_THRESHOLD;
            boolean shootHeld = pollenFeed || nectarFeed;
            boolean lockOn = WHEEL_LOCK_ENABLED && shootHeld;
            if (gamepad2.back && !prevBack2) alignOffByDriver = !alignOffByDriver;
            prevBack2 = gamepad2.back;
            boolean alignOn = AUTO_ALIGN_ENABLED && aligner != null && shootHeld && !alignOffByDriver;
            boolean moveWhileAiming = DRIVE_WHILE_AIMING_ENABLED && lockOn && alignOn;
            long nowNanos = System.nanoTime();
            double dt = Range.clip((nowNanos - lastControlNanos) / 1e9, 1e-4, 0.1);
            lastControlNanos = nowNanos;
            if (aligner != null || WHEEL_LOCK_ENABLED || mecaTank.smoothDriveNeedsPose()) {
                mecaTank.updatePoseEstimate();
            }
            Pose2d pose = mecaTank.getPoseEstimate();
            PoseVelocity2d velRobot = mecaTank.getMeasuredRobotVelocity();
            if (aligner != null) {
                aligner.setLauncher(pollenFeed && nectarFeed ? HiveAligner.Launcher.BOTH
                        : nectarFeed ? HiveAligner.Launcher.NECTAR : HiveAligner.Launcher.POLLEN);
                aligner.update(nowNanos, pose, velRobot, dt);
                // Camera fix of x/y: the aligner has already moved its CELL estimates, so apply it now.
                double[] fix = aligner.takePoseCorrection();
                if (fix != null) {
                    pose = new Pose2d(pose.position.x + fix[0], pose.position.y + fix[1], pose.heading.toDouble());
                    mecaTank.setPoseEstimate(pose);
                    if (shotHoldActive && holdPose != null) {
                        holdPose = new Pose2d(holdPose.position.x + fix[0], holdPose.position.y + fix[1],
                                holdPose.heading.toDouble());
                    } else {
                        mecaTank.resetSmoothDriveState(velRobot);   // re-latch translation hold here
                    }
                }
            }
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

            boolean anyReady = pollen.ready || nectar.ready;
            if (lockOn || alignOn) {
                if (!shotHoldActive) {
                    shotHoldActive = true;
                    holdPose = pose;
                    double c = Math.cos(pose.heading.toDouble()), sn = Math.sin(pose.heading.toDouble());
                    double vx = velRobot == null ? 0 : velRobot.linearVel.x, vy = velRobot == null ? 0 : velRobot.linearVel.y;
                    cmdVx = c * vx - sn * vy;
                    cmdVy = sn * vx + c * vy;
                    if (alignOn) aligner.engage(pose, velRobot);
                    stickVx = 0;
                    stickVy = 0;
                    slowEnoughToFeed = true;
                }
                // gamepad2 BACK can switch auto-align off or on while a trigger is still held.
                if (aligner != null) {
                    if (alignOn && !aligner.engaged()) aligner.engage(pose, velRobot);
                    else if (!alignOn && aligner.engaged()) aligner.release();
                }
                driveShotHold(pose, lockOn, alignOn, moveWhileAiming, anyReady, dt);
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
            // SHOOTERS
            // =====================================================================================
            if ((gamepad1.dpad_right && !prevRight1) || (gamepad2.dpad_right && !prevRight2)) {
                flywheelsOn = !flywheelsOn;
            }
            prevRight1 = gamepad1.dpad_right;
            prevRight2 = gamepad2.dpad_right;

            // Whatever you last reached for is what the dpad edits.
            if (pollenFeed) selected = pollen;
            if (nectarFeed) selected = nectar;
            if (gamepad2.x && !prevX2) selected = selected == pollen ? nectar : pollen;
            prevX2 = gamepad2.x;

            boolean up = (gamepad1.dpad_up && !prevUp1) || (gamepad2.dpad_up && !prevUp2);
            boolean down = (gamepad1.dpad_down && !prevDown1) || (gamepad2.dpad_down && !prevDown2);
            prevUp1 = gamepad1.dpad_up;
            prevUp2 = gamepad2.dpad_up;
            prevDown1 = gamepad1.dpad_down;
            prevDown2 = gamepad2.dpad_down;
            if (USE_VELOCITY_TABLE) {
                if (up) selected.trim += VELOCITY_STEP;
                if (down) selected.trim -= VELOCITY_STEP;
                double hiveIn = aligner == null ? Double.NaN : aligner.distanceToHive(pose);
                pollen.updateTarget(hiveIn);
                nectar.updateTarget(hiveIn);
            } else {
                if (up) selected.target += VELOCITY_STEP;
                if (down) selected.target = Math.max(0.0, selected.target - VELOCITY_STEP);
            }

            boolean aimReady = aligner == null || !aligner.engaged() || !aligner.hasTarget() || aligner.onTarget();
            if (moveWhileAiming) {
                double speed = velRobot == null ? 0.0 : Math.hypot(velRobot.linearVel.x, velRobot.linearVel.y);
                if (speed > AIMING_FEED_MAX_IN_S) slowEnoughToFeed = false;
                else if (speed < 0.8 * AIMING_FEED_MAX_IN_S) slowEnoughToFeed = true;
                aimReady &= slowEnoughToFeed;
            }
            // One transfer feeds both wheels, so they spin up together.
            boolean spin = flywheelsOn || shootHeld;
            pollen.update(spin, pollenFeed, aimReady);
            nectar.update(spin, nectarFeed, aimReady);

            // =====================================================================================
            // INTAKE + TRANSFER - one motor each. The transfer feeds while at least one gate is
            // open and every open gate's wheel is ready (the closed side spins against its gate),
            // and otherwise follows the intake (in with A, out with B) so artifacts are carried up
            // to the closed gates. A shot can drive the intake forward too.
            // =====================================================================================
            boolean gatesReady = (pollen.gateOpen || nectar.gateOpen)
                    && (!pollen.gateOpen || pollen.ready) && (!nectar.gateOpen || nectar.ready);
            boolean transferring = gatesReady
                    && (!moveWhileAiming || !PAUSE_FEED_WHEN_TOO_FAST || slowEnoughToFeed);
            pollen.drive(transferring && pollen.gateOpen);
            nectar.drive(transferring && nectar.gateOpen);
            boolean unjamming = TRANSFER_REVERSE_ON_RELEASE && !shootHeld && (pollen.closing() || nectar.closing());
            double transferPower = transferring ? BiobuzzRobotConstants.TRANSFER_FEED_POWER
                    : shootHeld ? 0.0
                    : unjamming ? -TRANSFER_RELEASE_REVERSE_POWER
                    : reverseHeld ? -BiobuzzRobotConstants.TRANSFER_INTAKE_POWER
                    : intakeOn ? BiobuzzRobotConstants.TRANSFER_INTAKE_POWER : 0.0;
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
                telemetry.addLine("--- SHOOTERS (>> = dpad up/down edits this one) ---");
                pollen.telemetry();
                nectar.telemetry();
                telemetry.addData("Flywheels held on", flywheelsOn);
                telemetry.addData("Intake", "%.2f  %s", rollerPower,
                        transferring && BiobuzzRobotConstants.transferAlsoRunsIntake ? "pushing to transfer"
                                : reverseHeld ? "REVERSE (B held)" : intakeOn ? "IN (A toggle)" : "off");
                telemetry.addData("Transfer", transfer == null ? "MISSING"
                        : String.format("%.2f  %s", transferPower, transferring ? "FEEDING"
                                : shootHeld ? "waiting for gate / wheel"
                                : unjamming ? "REVERSE after release (unjam)"
                                : reverseHeld ? "REVERSE (B held)" : intakeOn ? "with intake" : "off"));
                telemetry.addData("Flower ramp", (flowerRampDown ? "DOWN (intake)" : "stowed")
                        + (flowerRamp == null ? " - not configured" : ""));

                if (shotHoldActive) telemetry.addData("SHOT HOLD", holdNote);
                if (aligner != null) {
                    telemetry.addData("Auto-align", alignOffByDriver ? "OFF (gamepad2 BACK) - triggers shoot without aiming"
                            : "on (gamepad2 BACK turns it off)");
                    double hiveIn = aligner.distanceToHive(pose);
                    telemetry.addData("HIVE distance", Double.isNaN(hiveIn) ? "unknown - no CELL known yet"
                            : String.format("%.1f in (%.2f ft) to %s", hiveIn, hiveIn / 12.0, aligner.distanceCellLabel(pose)));
                    aligner.telemetry(telemetry);
                } else {
                    telemetry.addData("HIVE distance", "unknown - auto-align is off (no camera)");
                }
                telemetry.addLine("--- DRIVE ---");
                telemetry.addData("Pose", "(%.1f, %.1f) %.0f deg%s", pose.position.x, pose.position.y,
                        Math.toDegrees(pose.heading.toDouble()), guessedPose ? "  (started from the DEFAULT pose)" : "");
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
        if (aligner != null) aligner.close();
        rollers.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
    }
}
