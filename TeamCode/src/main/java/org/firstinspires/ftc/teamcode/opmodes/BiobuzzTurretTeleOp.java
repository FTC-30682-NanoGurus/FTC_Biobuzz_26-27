package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAligner;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.LaunchVelocityTable;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

/**
 * THE TURRET ROBOT'S TELEOP: one POLLEN flywheel on a motor turret, one intake motor, one transfer
 * motor, one gate servo, the camera fixed to the chassis facing forward. Everything
 * BiobuzzPollenTeleOp / BiobuzzShooterTeleOp have (drive, intake, flower ramp, POLLEN counter,
 * velocity table, gate-travel timing, transfer reverse on release, wheel lock, drive while aiming,
 * start pose from the last auto, camera pose fixes, our CELL on the robot's half) with the AIM done
 * by the TURRET instead of by turning the robot.
 *
 * <h2>Turret</h2>
 * The turret TRACKS ALL THE TIME (not only while shooting): it points the shot at our CELL on the
 * robot's half of the field (HiveAligner: odometry pose, CELLS seeded at their surveyed positions,
 * corrected by the camera whenever it sees their tags), compensating for the robot driving and
 * turning. A target outside the turret's travel (+-TurretConstants.TURRET_LIMIT_DEG) parks the
 * turret at the limit; while RT is held the chassis then turns just enough to bring it in range.
 * gamepad2 BACK switches tracking off (turret holds forward, gamepad2 right stick X jogs it).
 *
 * <h2>Shooting</h2>
 * RT spins up (if not already spinning), and the gate opens once the wheel is at speed AND the
 * turret is ON TARGET (tracking on), then stays open until RT is released. The transfer waits for
 * the gate to swing open, then feeds until release; on release it runs backwards for
 * TRANSFER_RELEASE_REVERSE_MS so nothing jams under the closing gate. Camera pose fixes wait while
 * RT is held, so the aim cannot jump mid-volley.
 *
 * <h2>CONTROLS - gamepad 1 can do everything alone</h2>
 * <pre>
 * GAMEPAD 1
 *   Left stick .......... drive (field centric)       Right stick X ... turn
 *   LB (hold) precision                               Y field centric   X heading hold
 *   Dpad left ........... translation hold            Back / Options .. re-zero "forward"
 *   A ................... intake + transfer on/off     B (hold) ........ both reverse
 *   RB .................. flower ramp down/up (down also turns the intake on)
 *   Right trigger (hold)  SHOOT (wheel lock holds the spot while held)
 *   Dpad right .......... flywheel on/off (keep it spinning between shots)
 *   Dpad up / down ...... flywheel target +/- VELOCITY_STEP (a trim on the table)
 *
 * GAMEPAD 2 (optional)
 *   A / B / RB / RT / dpad right / dpad up+down ... same as gamepad 1
 *   BACK ................ turret tracking off / back on
 *   Right stick X ....... jog the turret while tracking is off
 *
 * INIT: B = RED alliance, X = BLUE (either pad).  gamepad2 Y = re-zero the turret (face it FORWARD).
 * </pre>
 */
@Config
@TeleOp(name = "BioBuzz Turret TeleOp", group = "biobuzz")
public class BiobuzzTurretTeleOp extends LinearOpMode {

    public static double TELEMETRY_INTERVAL_MS = 200.0;
    /** TEMPORARY: no acceleration limits, linear sticks. See MecaTank.setFullSpeed. */
    public static boolean FULL_SPEED = true;

    // ---- flywheel ----------------------------------------------------------------------------
    /** One dpad press, TICKS/SECOND. */
    public static double VELOCITY_STEP = 25.0;
    /**
     * Flywheel velocity from LaunchVelocityTable by the distance from the shooter EXIT to the CELL
     * (Turret.exitDistanceIn), only changed while the gate is closed (one speed per volley). Dpad
     * trims on top. false = BiobuzzRobotConstants.pollenFixedShootingVel with dpad steps.
     */
    public static boolean USE_VELOCITY_TABLE = false;
    public static double TRIGGER_THRESHOLD = 0.5;

    // ---- gate / transfer -----------------------------------------------------------------------
    /** Gate swing timing (no position feedback): REV Smart Robot Servo ~0.17 s per 60 deg on 5 V. */
    public static double GATE_SEC_PER_60_DEG = 0.17;
    public static double GATE_RANGE_DEG = 180.0;
    public static double GATE_SETTLE_MARGIN_MS = 40.0;
    public static boolean TRANSFER_REVERSE_ON_RELEASE = true;
    public static double TRANSFER_RELEASE_REVERSE_MS = 300;
    public static double TRANSFER_RELEASE_REVERSE_POWER = 1.0;

    /** Flip each motor relative to BiobuzzRobotConstants, for this opmode only. Read at INIT. */
    public static boolean
        REVERSE_INTAKE = true,
        REVERSE_TRANSFER = false,
        REVERSE_FLYWHEEL = false;

    // ---- POLLEN counter (proximity of BiobuzzRobotConstants.colorSensor) -------------------------
    public static boolean POLLEN_COUNTER_ENABLED = true;
    public static int POLLEN_FULL_COUNT = 4;
    public static double POLLEN_DIP_MM = 10.0;
    public static double POLLEN_RELEASE_FRACTION = 0.5;
    public static double POLLEN_BASELINE_TAU_S = 0.5;
    /** After FULL, run the intake backwards this long, SECONDS (0 = no eject). */
    public static double FULL_EJECT_S = 1.5;

    // ---- turret ---------------------------------------------------------------------------------
    /** Turret tracks our CELL. false = it holds forward (gamepad2 right stick jogs it). Read live. */
    public static boolean TURRET_TRACKING_ENABLED = false;
    /** Camera on: corrects the CELL positions and the robot pose. false = odometry + surveyed CELLS only. Read at INIT. */
    public static boolean CAMERA_ENABLED = false;
    /** The gate waits for the turret to be ON TARGET (TurretConstants.AIM_READY_TOLERANCE_DEG). */
    public static boolean GATE_WAITS_FOR_TURRET = false;
    /** Turret jog speed at full gamepad2 right stick (tracking off), DEGREES/SECOND. */
    public static double TURRET_JOG_DEG_S = 120.0;

    // ---- wheel lock / drive while aiming ---------------------------------------------------------
    /** While RT is held the robot holds the spot where RT was pressed against pushes. */
    public static boolean WHEEL_LOCK_ENABLED = true;
    public static double LOCK_KP = 6.5;
    public static double LOCK_MAX_IN_S = 30.0;
    public static double LOCK_DEADBAND_IN = 0.15;
    public static double LOCK_HEADING_KP = 6.0;
    public static double LOCK_HEADING_DEADBAND_DEG = 0.3;
    public static double LOCK_MAX_TURN_RATE = 4.0;
    /**
     * While RT is held the left stick moves the locked spot (the turret keeps aiming), so the robot
     * can be repositioned mid-volley. Artifacts start feeding only below AIMING_FEED_MAX_IN_S.
     */
    public static boolean DRIVE_WHILE_AIMING_ENABLED = false;
    public static double AIMING_DRIVE_MAX_IN_S = 14.0;
    public static double AIMING_FEED_MAX_IN_S = 11.0;
    public static boolean PAUSE_FEED_WHEN_TOO_FAST = false;
    public static double AIMING_LEASH_IN = 3.0;
    public static double AIMING_DRIVE_ACCEL_IN_S2 = 40.0;
    /** Without the wheel lock, RT held: left stick translation (in/s, in/s^2) and right stick turn (rad/s). */
    public static double SHOOT_DRIVE_MAX_IN_S = 40.0;
    public static double SHOOT_DRIVE_ACCEL_IN_S2 = 70.0;
    public static double DRIVER_MAX_TURN_RATE = 3.0;

    /** Start from the pose the last auto stored; else BiobuzzRobotConstants.teleopDefaultStart*. */
    public static boolean USE_AUTO_END_POSE = true;

    // ---------------------------------------------------------------------------------------------

    private MecaTank mecaTank;
    private BulkRead bulkRead;
    private DcMotorEx rollers, transfer;
    private Turret turret;
    private HiveAligner aligner;
    private final LaunchVelocityTable velocityTable = new LaunchVelocityTable();
    private Shooter pollen;
    private Servo flowerRamp;
    private final PollenCounter counter = new PollenCounter();

    private double lastTransferPower = Double.NaN, lastRollerPower = Double.NaN;
    private double velocityTrim = 0.0;
    private String velocityNote = "fixed";
    private boolean intakeOn = false, flywheelOn = false, flowerRampDown = false;
    private boolean wasTransferring = false, ejecting = false;
    private final ElapsedTime ejectTimer = new ElapsedTime();

    private boolean trackingOffByDriver = false;
    private double manualTurretDeg = 0.0;
    /** This loop's turret aim: unclamped wanted angle (NaN = no target) and the chassis excess. */
    private double aimWantDeg = Double.NaN, aimExcessDeg = 0.0, hiveDistanceIn = Double.NaN;
    private String aimNote = "";

    private boolean shotHoldActive = false;
    private Pose2d holdPose;
    private double cmdVx = 0, cmdVy = 0, stickVx = 0, stickVy = 0, driverForwardRef = 0;
    private boolean slowEnoughToFeed = true;
    private String holdNote = "";

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevOptions, prevLeft1, prevBack2;
    private boolean prevA1, prevA2, prevRight1, prevRight2, prevUp1, prevUp2, prevDown1, prevDown2, prevRB1, prevRB2;

    // =============================================================================================
    // Flywheel + gate (as BiobuzzPollenTeleOp's Shooter)
    // =============================================================================================
    private class Shooter {
        final NGMotor motor;
        String fault = "";
        final Servo gate;
        String gateFault = "";
        boolean gateOpen = false;
        double lastGatePos = Double.NaN;
        final ElapsedTime gateTimer = new ElapsedTime();
        final ElapsedTime closeTimer = new ElapsedTime(0);
        private boolean everClosedAfterOpen = false;
        double target;
        double velocity = 0.0;
        boolean spinningUp = false, ready = false, feeding = false, powerCut = true;
        private boolean armedThisLoop = false;

        Shooter(boolean reversed, double target) {
            this.target = target;
            NGMotor m = null;
            try {
                m = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.pollenFlywheel);
                m.init();
                m.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                m.setTelemetryEnabled(false);
            } catch (Exception e) {
                m = null;
                fault = "'" + BiobuzzRobotConstants.pollenFlywheel + "' not in the configuration";
            }
            motor = m;
            if (motor != null) applyGains(0.0);
            Servo g = null;
            try {
                g = hardwareMap.get(Servo.class, BiobuzzRobotConstants.pollenGate);
            } catch (Exception e) {
                gateFault = "'" + BiobuzzRobotConstants.pollenGate + "' not in the configuration";
            }
            gate = g;
            setGate(false);
        }

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
            double pos = open ? BiobuzzRobotConstants.pollenGateOpenPos : BiobuzzRobotConstants.pollenGateClosedPos;
            if (pos != lastGatePos) {
                gate.setPosition(pos);
                lastGatePos = pos;
            }
        }

        private void applyGains(double velocityTarget) {
            double kps = BiobuzzRobotConstants.pollenKpStable;
            motor.setCustomVelocityPID(velocityTarget, kps, BiobuzzRobotConstants.pollenKI,
                    BiobuzzRobotConstants.pollenKD, BiobuzzRobotConstants.pollenKF);
            NGMotorFlywheelTuner.setStableKp(motor, kps);
            NGMotorFlywheelTuner.setRecoveryKp(motor, BiobuzzRobotConstants.pollenKpRecovery);
            NGMotorFlywheelTuner.setAlpha(motor, BiobuzzRobotConstants.pollenAlpha);
            NGMotorFlywheelTuner.setRecoveryThreshold(motor, BiobuzzRobotConstants.pollenRecoveryThreshold);
        }

        void update(boolean wantSpin, boolean wantFeed, boolean aimReady) {
            armedThisLoop = false;
            if (motor == null) {
                setGate(false);
                ready = false;
                return;
            }
            velocity = motor.getVelocity();
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
            if (!spinningUp || !wantFeed) setGate(false);
            else if (!gateOpen && atSpeed() && aimReady) setGate(true);
            ready = gateOpen && (ready || (gateTimer.milliseconds() >= gateTravelMs() && atSpeed()));
        }

        void drive(boolean loading) {
            feeding = loading;
            if (motor == null || armedThisLoop) return;
            if (spinningUp) {
                motor.targetVelocity = target;
                motor.updateFlywheels(feeding);
            } else {
                motor.targetVelocity = 0.0;
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
            if (motor == null) {
                telemetry.addData("POLLEN", "MISSING - " + fault);
            } else {
                telemetry.addData("POLLEN", "%s   target %.0f / actual %.0f tps   power %.2f",
                        !spinningUp ? "idle" : feeding ? "FEEDING" : gateOpen && !ready ? "gate opening"
                                : atSpeed() ? "AT SPEED" : "spinning up",
                        target, velocity, motor.getPower());
            }
            telemetry.addData("   gate", gate == null ? "not configured - skipped (" + gateFault + ")"
                    : (gateOpen ? "OPEN" : "closed") + String.format(" (%.2f)", lastGatePos));
        }
    }

    static double gateTravelMs() {
        double deg = Math.abs(BiobuzzRobotConstants.pollenGateOpenPos - BiobuzzRobotConstants.pollenGateClosedPos) * GATE_RANGE_DEG;
        return deg / 60.0 * GATE_SEC_PER_60_DEG * 1000.0 + GATE_SETTLE_MARGIN_MS;
    }

    // =============================================================================================
    // POLLEN counter (as BiobuzzPollenTeleOp)
    // =============================================================================================
    private class PollenCounter {
        boolean justFilled = false;
        DistanceSensor sensor;
        String fault = "";
        double distanceMm = Double.NaN, baselineMm = Double.NaN;
        boolean seeing = false;
        int count = 0;
        private final ElapsedTime loopTimer = new ElapsedTime();

        void init() {
            try {
                sensor = hardwareMap.get(DistanceSensor.class, BiobuzzRobotConstants.colorSensor);
            } catch (Exception e) {
                sensor = null;
                fault = "'" + BiobuzzRobotConstants.colorSensor + "' not in the configuration";
            }
        }

        void update(boolean counting) {
            if (sensor == null) return;
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (!counting) return;
            double d = sensor.getDistance(DistanceUnit.MM);
            if (Double.isNaN(d) || Double.isInfinite(d)) return;
            distanceMm = d;
            if (Double.isNaN(baselineMm)) {
                baselineMm = d;
                return;
            }
            if (!seeing && d <= baselineMm - POLLEN_DIP_MM) {
                seeing = true;
                count++;
                if (count == POLLEN_FULL_COUNT) justFilled = true;
            } else if (seeing && d >= baselineMm - POLLEN_DIP_MM * POLLEN_RELEASE_FRACTION) {
                seeing = false;
            }
            if (!seeing) {
                double a = POLLEN_BASELINE_TAU_S <= 0 ? 1.0 : Math.min(1.0, dt / POLLEN_BASELINE_TAU_S);
                baselineMm += a * (d - baselineMm);
            }
        }

        void reset() {
            justFilled = false;
            count = 0;
        }
    }

    // =============================================================================================
    // Turret aim - every loop
    // =============================================================================================
    private void updateTurret(Pose2d pose, PoseVelocity2d velRobot, boolean shooting, boolean feeding, double dt) {
        boolean tracking = TURRET_TRACKING_ENABLED && !trackingOffByDriver && aligner != null;
        aimWantDeg = Double.NaN;
        aimExcessDeg = 0.0;
        hiveDistanceIn = Double.NaN;
        if (!tracking) {
            manualTurretDeg = Turret.clampAngle(manualTurretDeg - gamepad2.right_stick_x * TURRET_JOG_DEG_S * dt);
            turret.setTarget(manualTurretDeg, 0.0);
            turret.update();
            aimNote = trackingOffByDriver ? "tracking OFF (gamepad2 BACK) - right stick jogs" : "tracking disabled";
            return;
        }
        double[] aim = aligner.turretAimPoint(pose, !feeding);
        if (aim == null) {
            turret.setTarget(turret.getTargetDeg(), 0.0);
            turret.update();
            aimNote = "no CELL known yet - holding";
            return;
        }
        double h = pose.heading.toDouble(), c = Math.cos(h), s = Math.sin(h);
        double vx = 0, vy = 0;
        if (velRobot != null) {
            vx = c * velRobot.linearVel.x - s * velRobot.linearVel.y;
            vy = s * velRobot.linearVel.x + c * velRobot.linearVel.y;
        }
        double ax = aim[0] - vx * TurretConstants.PROJECTILE_FLIGHT_TIME_S;
        double ay = aim[1] - vy * TurretConstants.PROJECTILE_FLIGHT_TIME_S;
        double want = Turret.chooseTargetDeg(Turret.aimAngleDeg(pose, ax, ay), turret.getTargetDeg());
        double ff = Turret.aimRateDegS(pose, ax, ay, vx, vy, aligner.yawRate());
        turret.setTarget(want, ff);
        turret.update();
        aimWantDeg = want;
        aimExcessDeg = Turret.chassisExcessDeg(want);
        hiveDistanceIn = Turret.exitDistanceIn(pose, aim[0], aim[1]);
        aimNote = String.format("%s  want %.1f deg%s", aligner.turretCellLabel(), want,
                turret.wasLastCommandClamped() ? (shooting ? "  OUT OF TRAVEL - chassis turning" : "  OUT OF TRAVEL (RT turns the chassis)")
                        : turret.onTarget() ? "  ON TARGET" : "  aiming");
    }

    /** Turret aim allows the gate to open. */
    private boolean aimReady() {
        if (!GATE_WAITS_FOR_TURRET) return true;
        boolean tracking = TURRET_TRACKING_ENABLED && !trackingOffByDriver && aligner != null;
        if (!tracking || Double.isNaN(aimWantDeg)) return true;
        return turret.onTarget() && !turret.wasLastCommandClamped();
    }

    // =============================================================================================
    // Drive while RT is held: wheel lock and/or chassis assist for an out-of-travel aim
    // =============================================================================================
    private void driveShotHold(Pose2d pose, boolean lockOn, boolean moveWhileAiming, double dt) {
        double x = pose.position.x, y = pose.position.y, h = pose.heading.toDouble();
        if (lockOn && moveWhileAiming) {
            double speed = AIMING_DRIVE_MAX_IN_S * (gamepad1.left_bumper ? MecaTank.PRECISION_SCALE : 1.0);
            double fwd = -gamepad1.left_stick_y * speed, left = -gamepad1.left_stick_x * speed;
            double ref = MecaTank.FIELD_CENTRIC ? driverForwardRef : h;
            double wantVx = Math.cos(ref) * fwd - Math.sin(ref) * left;
            double wantVy = Math.sin(ref) * fwd + Math.cos(ref) * left;
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
            double ex = hx - x, ey = hy - y, e = Math.hypot(ex, ey);
            double v = e < LOCK_DEADBAND_IN ? 0.0 : Math.min(LOCK_MAX_IN_S, LOCK_KP * e);
            cmdVx = stickVx + (e > 1e-9 ? ex / e * v : 0.0);
            cmdVy = stickVy + (e > 1e-9 ? ey / e * v : 0.0);
        } else if (lockOn) {
            double ex = holdPose.position.x - x, ey = holdPose.position.y - y, e = Math.hypot(ex, ey);
            double v = e < LOCK_DEADBAND_IN ? 0.0 : Math.min(LOCK_MAX_IN_S, LOCK_KP * e);
            cmdVx = e > 1e-9 ? ex / e * v : 0.0;
            cmdVy = e > 1e-9 ? ey / e * v : 0.0;
        } else {
            double speed = SHOOT_DRIVE_MAX_IN_S * (gamepad1.left_bumper ? MecaTank.PRECISION_SCALE : 1.0);
            double fwd = -gamepad1.left_stick_y * speed, left = -gamepad1.left_stick_x * speed;
            double ref = MecaTank.FIELD_CENTRIC ? driverForwardRef : h;
            double wantVx = Math.cos(ref) * fwd - Math.sin(ref) * left;
            double wantVy = Math.sin(ref) * fwd + Math.cos(ref) * left;
            double ddx = wantVx - cmdVx, ddy = wantVy - cmdVy;
            double d = Math.hypot(ddx, ddy), step = SHOOT_DRIVE_ACCEL_IN_S2 * dt;
            if (d > step) {
                ddx *= step / d;
                ddy *= step / d;
            }
            cmdVx += ddx;
            cmdVy += ddy;
        }

        double turn;
        boolean assist = Math.abs(aimExcessDeg) > 1e-6;
        if (assist) {
            turn = Range.clip(TurretConstants.TURRET_CHASSIS_ASSIST_KP * Math.toRadians(aimExcessDeg),
                    -TurretConstants.TURRET_CHASSIS_ASSIST_MAX, TurretConstants.TURRET_CHASSIS_ASSIST_MAX);
            if (lockOn) holdPose = new Pose2d(holdPose.position, h);
        } else if (lockOn) {
            double err = holdPose.heading.toDouble() - h;
            while (err > Math.PI) err -= 2 * Math.PI;
            while (err <= -Math.PI) err += 2 * Math.PI;
            turn = Math.abs(Math.toDegrees(err)) < LOCK_HEADING_DEADBAND_DEG ? 0.0
                    : Range.clip(LOCK_HEADING_KP * err, -LOCK_MAX_TURN_RATE, LOCK_MAX_TURN_RATE);
        } else {
            turn = -gamepad1.right_stick_x * DRIVER_MAX_TURN_RATE;
        }
        holdNote = (lockOn ? String.format(moveWhileAiming ? "LOCKED, stick moves the spot (%.2f in off)%s"
                        : "WHEELS LOCKED, %.2f in off the spot%s", Math.hypot(holdPose.position.x - x, holdPose.position.y - y),
                moveWhileAiming && !slowEnoughToFeed && (!pollen.ready || PAUSE_FEED_WHEN_TOO_FAST) ? ", TOO FAST TO START FEEDING" : "")
                : "driving with the left stick")
                + (assist ? String.format(", chassis turning %.0f deg for the turret", aimExcessDeg) : "");

        double c = Math.cos(h), sn = Math.sin(h);
        HiveAligner.driveRobotVelocity(mecaTank, c * cmdVx + sn * cmdVy, -sn * cmdVx + c * cmdVy, turn);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mecaTank = new MecaTank(hardwareMap, telemetry, new Pose2d(0, 0, 0));
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();
        mecaTank.setAutoPoseUpdate(false);
        MecaTank.FIELD_CENTRIC = true;
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();

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
        NGMotor.kLoad = BiobuzzRobotConstants.flywheelKLoad;
        NGMotor.kRamp = BiobuzzRobotConstants.flywheelKRamp;
        pollen = new Shooter(BiobuzzRobotConstants.pollenFlywheelReversed != REVERSE_FLYWHEEL,
                BiobuzzRobotConstants.pollenFixedShootingVel);
        counter.init();
        try {
            flowerRamp = hardwareMap.get(Servo.class, BiobuzzRobotConstants.flowerRamp);
            flowerRamp.setPosition(BiobuzzRobotConstants.flowerRampStowedPos);
        } catch (Exception e) {
            flowerRamp = null;
        }
        turret = new Turret(hardwareMap, telemetry, false);
        aligner = new HiveAligner(hardwareMap, mecaTank, AllianceSelector.red(), CAMERA_ENABLED);

        boolean prevY2 = false;
        while (!isStarted() && !isStopRequested()) {
            bulkRead.clearCache();
            AllianceSelector.poll(gamepad1, gamepad2);
            aligner.setAlliance(AllianceSelector.red());
            if (gamepad2.y && !prevY2) turret.rezero();
            prevY2 = gamepad2.y;
            turret.refresh();
            boolean red = AllianceSelector.red();
            Pose2d stored = red ? PoseStorage.getLastRedPose() : PoseStorage.getLastBluePose();
            telemetry.addLine("BIOBUZZ TURRET TELEOP");
            AllianceSelector.show(telemetry);
            telemetry.addData("Start pose", USE_AUTO_END_POSE && stored != null
                    ? String.format("from the last auto: (%.1f, %.1f) %.0f deg", stored.position.x, stored.position.y,
                    Math.toDegrees(stored.heading.toDouble()))
                    : "DEFAULT - put the robot in our LOADING ZONE, facing away from our drivers");
            telemetry.addData("Turret zero", turret.keptAutoZero()
                    ? "kept from the auto (gamepad2 Y re-zeroes - turret must face FORWARD)"
                    : "set at INIT - turret must face FORWARD (gamepad2 Y re-zeroes)");
            telemetry.addData("Turret angle", "%.1f deg (travel +-%.0f)", turret.getAngleDeg(), Turret.maxAngleDeg());
            telemetry.addData("Camera", CAMERA_ENABLED ? aligner.cameraState() : "off (odometry + surveyed CELLS)");
            if (!turret.isHardwareOk()) telemetry.addLine("TURRET MISSING: " + turret.getHardwareFault());
            if (pollen.motor == null) telemetry.addLine("POLLEN FLYWHEEL MISSING: " + pollen.fault);
            if (transfer == null) telemetry.addLine("TRANSFER MOTOR MISSING");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            aligner.close();
            turret.stop();
            return;
        }
        boolean red = AllianceSelector.red();
        aligner.setAlliance(red);
        Pose2d stored = red ? PoseStorage.getLastRedPose() : PoseStorage.getLastBluePose();
        boolean guessedPose = !(USE_AUTO_END_POSE && stored != null);
        mecaTank.setPoseEstimate(guessedPose ? PoseStorage.allianceToField(red, new Pose2d(
                BiobuzzRobotConstants.teleopDefaultStartX, BiobuzzRobotConstants.teleopDefaultStartY,
                Math.toRadians(BiobuzzRobotConstants.teleopDefaultStartHeadingDeg))) : stored);
        aligner.setFieldReferenced(guessedPose);
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();
        driverForwardRef = mecaTank.getPoseEstimate().heading.toDouble();
        manualTurretDeg = 0.0;

        telemetryTimer.reset();
        long lastNanos = System.nanoTime();
        double loopMsMax = 0;

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();
            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;
            long nowNanos = System.nanoTime();
            double dt = Range.clip((nowNanos - lastNanos) / 1e9, 1e-4, 0.1);
            loopMsMax = Math.max(loopMsMax, (nowNanos - lastNanos) / 1e6);
            lastNanos = nowNanos;

            // ---- pose + camera -------------------------------------------------------------------
            mecaTank.setFullSpeed(FULL_SPEED);
            boolean shootHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD;
            mecaTank.updatePoseEstimate();
            Pose2d pose = mecaTank.getPoseEstimate();
            PoseVelocity2d velRobot = mecaTank.getMeasuredRobotVelocity();
            aligner.setHoldPose(shootHeld);
            aligner.update(nowNanos, pose, velRobot, dt);
            double[] fix = aligner.takePoseCorrection();
            if (fix != null) {
                pose = new Pose2d(pose.position.x + fix[0], pose.position.y + fix[1], pose.heading.toDouble());
                mecaTank.setPoseEstimate(pose);
                if (shotHoldActive && holdPose != null) {
                    holdPose = new Pose2d(holdPose.position.x + fix[0], holdPose.position.y + fix[1], holdPose.heading.toDouble());
                } else {
                    mecaTank.resetSmoothDriveState(velRobot);
                }
            }

            // ---- turret --------------------------------------------------------------------------
            if (gamepad2.back && !prevBack2) {
                trackingOffByDriver = !trackingOffByDriver;
                manualTurretDeg = Turret.clampAngle(turret.getAngleDeg());
            }
            prevBack2 = gamepad2.back;
            updateTurret(pose, velRobot, shootHeld, pollen.feeding, dt);

            // ---- drive ---------------------------------------------------------------------------
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

            boolean lockOn = WHEEL_LOCK_ENABLED && shootHeld;
            boolean moveWhileAiming = DRIVE_WHILE_AIMING_ENABLED && lockOn;
            boolean needAssist = shootHeld && Math.abs(aimExcessDeg) > 1e-6;
            if (lockOn || needAssist) {
                if (!shotHoldActive) {
                    shotHoldActive = true;
                    holdPose = pose;
                    double c = Math.cos(pose.heading.toDouble()), sn = Math.sin(pose.heading.toDouble());
                    double vx = velRobot == null ? 0 : velRobot.linearVel.x, vy = velRobot == null ? 0 : velRobot.linearVel.y;
                    cmdVx = c * vx - sn * vy;
                    cmdVy = sn * vx + c * vy;
                    stickVx = 0;
                    stickVy = 0;
                    slowEnoughToFeed = true;
                }
                driveShotHold(pose, lockOn, moveWhileAiming, dt);
            } else {
                if (shotHoldActive) {
                    shotHoldActive = false;
                    mecaTank.resetSmoothDriveState(velRobot);
                }
                mecaTank.setDrivePowersSmoothArcade(gamepad1.left_stick_x, gamepad1.left_stick_y,
                        gamepad1.right_stick_x, gamepad1.left_bumper, false);
            }

            // ---- intake / flower ramp ------------------------------------------------------------
            if ((gamepad1.a && !prevA1) || (gamepad2.a && !prevA2)) intakeOn = !intakeOn;
            prevA1 = gamepad1.a;
            prevA2 = gamepad2.a;
            boolean reverseHeld = gamepad1.b || gamepad2.b;
            if ((gamepad1.right_bumper && !prevRB1) || (gamepad2.right_bumper && !prevRB2)) {
                flowerRampDown = !flowerRampDown;
                if (flowerRampDown) intakeOn = true;
                if (flowerRamp != null) flowerRamp.setPosition(flowerRampDown
                        ? BiobuzzRobotConstants.flowerRampIntakePos : BiobuzzRobotConstants.flowerRampStowedPos);
            }
            prevRB1 = gamepad1.right_bumper;
            prevRB2 = gamepad2.right_bumper;

            // ---- shooter -------------------------------------------------------------------------
            if ((gamepad1.dpad_right && !prevRight1) || (gamepad2.dpad_right && !prevRight2)) flywheelOn = !flywheelOn;
            prevRight1 = gamepad1.dpad_right;
            prevRight2 = gamepad2.dpad_right;
            boolean up = (gamepad1.dpad_up && !prevUp1) || (gamepad2.dpad_up && !prevUp2);
            boolean down = (gamepad1.dpad_down && !prevDown1) || (gamepad2.dpad_down && !prevDown2);
            prevUp1 = gamepad1.dpad_up;
            prevUp2 = gamepad2.dpad_up;
            prevDown1 = gamepad1.dpad_down;
            prevDown2 = gamepad2.dpad_down;
            if (USE_VELOCITY_TABLE) {
                if (up) velocityTrim += VELOCITY_STEP;
                if (down) velocityTrim -= VELOCITY_STEP;
                if (!pollen.gateOpen) {
                    double base;
                    if (Double.isNaN(hiveDistanceIn)) {
                        base = BiobuzzRobotConstants.pollenFixedShootingVel;
                        velocityNote = "fixed - no HIVE distance (tracking off or no CELL)";
                    } else {
                        base = velocityTable.getVelocity(hiveDistanceIn);
                        velocityNote = String.format("table %.0f tps at %.1f in", base, hiveDistanceIn);
                    }
                    pollen.target = Math.max(0.0, base + velocityTrim);
                }
            } else {
                if (up) pollen.target += VELOCITY_STEP;
                if (down) pollen.target = Math.max(0.0, pollen.target - VELOCITY_STEP);
                velocityNote = "fixed (dpad steps)";
            }

            boolean aimOk = aimReady();
            if (moveWhileAiming) {
                double speed = velRobot == null ? 0.0 : Math.hypot(velRobot.linearVel.x, velRobot.linearVel.y);
                if (speed > AIMING_FEED_MAX_IN_S) slowEnoughToFeed = false;
                else if (speed < 0.8 * AIMING_FEED_MAX_IN_S) slowEnoughToFeed = true;
                aimOk &= slowEnoughToFeed;
            }
            pollen.update(flywheelOn || shootHeld, shootHeld, aimOk);

            // ---- intake + transfer ---------------------------------------------------------------
            boolean transferring = pollen.ready && (!moveWhileAiming || !PAUSE_FEED_WHEN_TOO_FAST || slowEnoughToFeed);
            pollen.drive(transferring);
            if (POLLEN_COUNTER_ENABLED) {
                if (wasTransferring && !shootHeld) counter.reset();
                counter.update(intakeOn && !reverseHeld && !shootHeld && !transferring);
                if (counter.justFilled) {
                    intakeOn = false;
                    counter.justFilled = false;
                    ejecting = FULL_EJECT_S > 0;
                    ejectTimer.reset();
                }
            } else if (counter.justFilled) {
                counter.reset();
            }
            wasTransferring = transferring || (wasTransferring && shootHeld);
            if (ejecting && (ejectTimer.seconds() >= FULL_EJECT_S || intakeOn || reverseHeld || shootHeld)) ejecting = false;
            boolean unjamming = TRANSFER_REVERSE_ON_RELEASE && !shootHeld && pollen.closing();
            double transferPower = transferring ? BiobuzzRobotConstants.TRANSFER_FEED_POWER
                    : shootHeld ? 0.0
                    : unjamming ? -TRANSFER_RELEASE_REVERSE_POWER
                    : reverseHeld ? -BiobuzzRobotConstants.TRANSFER_INTAKE_POWER
                    : intakeOn ? BiobuzzRobotConstants.TRANSFER_INTAKE_POWER : 0.0;
            double rollerPower = transferring && BiobuzzRobotConstants.transferAlsoRunsIntake ? BiobuzzRobotConstants.transferPower
                    : reverseHeld ? BiobuzzRobotConstants.intakeReversePower
                    : ejecting ? BiobuzzRobotConstants.intakeReversePower
                    : intakeOn ? BiobuzzRobotConstants.intakePower : 0.0;
            if (rollerPower != lastRollerPower) {
                rollers.setPower(rollerPower);
                lastRollerPower = rollerPower;
            }
            if (transfer != null && transferPower != lastTransferPower) {
                transfer.setPower(transferPower);
                lastTransferPower = transferPower;
            }

            // ---- telemetry -----------------------------------------------------------------------
            if (sendTelemetry) {
                telemetry.addLine("--- TURRET ---");
                telemetry.addData("Aim", aimNote);
                turret.telemetry();
                telemetry.addData("HIVE distance (exit)", Double.isNaN(hiveDistanceIn) ? "unknown"
                        : String.format("%.1f in (%.2f ft)", hiveDistanceIn, hiveDistanceIn / 12.0));
                telemetry.addLine("--- SHOOTER ---");
                pollen.telemetry();
                telemetry.addData("Flywheel held on", flywheelOn);
                telemetry.addData("Flywheel velocity", "%.0f tps  %s%s", pollen.target, velocityNote,
                        USE_VELOCITY_TABLE && velocityTrim != 0.0 ? String.format(", trim %+.0f", velocityTrim) : "");
                telemetry.addData("Intake", "%.2f  %s", rollerPower,
                        transferring && BiobuzzRobotConstants.transferAlsoRunsIntake ? "pushing to transfer"
                                : reverseHeld ? "REVERSE (B held)"
                                : ejecting ? String.format("EJECTING extra pollen (%.1f / %.1f s)", ejectTimer.seconds(), FULL_EJECT_S)
                                : intakeOn ? "IN (A toggle)" : "off");
                telemetry.addData("Transfer", transfer == null ? "MISSING"
                        : String.format("%.2f  %s", transferPower, transferring ? "FEEDING"
                        : shootHeld ? "waiting for gate / wheel / turret"
                        : unjamming ? "REVERSE after release (unjam)"
                        : reverseHeld ? "REVERSE (B held)" : intakeOn ? "with intake" : "off"));
                if (POLLEN_COUNTER_ENABLED) {
                    telemetry.addData("POLLEN count", counter.sensor == null ? "SENSOR MISSING - " + counter.fault
                            : String.format("%d / %d%s", counter.count, POLLEN_FULL_COUNT, counter.count >= POLLEN_FULL_COUNT ? "  FULL" : ""));
                }
                telemetry.addData("Flower ramp", (flowerRampDown ? "DOWN (intake)" : "stowed")
                        + (flowerRamp == null ? " - not configured" : ""));
                if (shotHoldActive) telemetry.addData("SHOT HOLD", holdNote);
                aligner.telemetry(telemetry);
                telemetry.addLine("--- DRIVE ---");
                telemetry.addData("Pose", "(%.1f, %.1f) %.0f deg%s", pose.position.x, pose.position.y,
                        Math.toDegrees(pose.heading.toDouble()), guessedPose ? "  (started from the DEFAULT pose)" : "");
                telemetry.addData("Field centric", MecaTank.FIELD_CENTRIC);
                telemetry.addData("Heading hold", MecaTank.HEADING_HOLD);
                telemetry.addData("Translation hold", MecaTank.TRANSLATION_HOLD);
                telemetry.addData("Loop Time Max (ms)", "%.1f", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }

        pollen.stop();
        turret.stop();
        aligner.close();
        rollers.setPower(0.0);
        if (transfer != null) transfer.setPower(0.0);
    }
}
