package org.firstinspires.ftc.teamcode.DECODE_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.Subsystem;

/**
 * Single-motor TETRIX channel arm + 180 degree servo claw.
 *
 * WHY THIS IS NOT AN NGMotor
 * --------------------------
 * An arm's load is not constant: the torque gravity applies scales with cos(angle), so a plain
 * position PID has to be tuned hot enough to hold the arm out horizontally, which then makes it
 * slam and oscillate near vertical where there is almost no load. The fix is a gravity
 * feedforward - kG * cos(angle) - that cancels the load directly, leaving the PID with almost
 * nothing to do. NGMotor has no notion of arm angle, and its update() also builds telemetry
 * strings on every call, so this subsystem drives the motor directly.
 *
 * LOOP COST
 * ---------
 * update() performs exactly one encoder read (bulk-cached), zero allocations and zero string
 * building, and only writes motor/servo power when the commanded value actually changed. Call it
 * once per loop after your BulkRead.clearCache(). telemetry() is separate so you can throttle it.
 *
 * ASSUMED HARDWARE (change the constants, not the code)
 * -----------------------------------------------------
 *   Motor : NeveRest 60  -> 28 CPR encoder x 60:1 = 1680 ticks per output revolution
 *           NeveRest 40 would be 1120. Multiply by any external chain/gear ratio on top.
 *   Arm   : mid-size TETRIX channel, ~350-400 g including the claw and the servo hanging off
 *           the end, centre of mass roughly 24 cm from the pivot.
 *   Claw  : FTC 180 degree servo. Open = 0.0, closed = 0.22.
 */
@Config
public class ArmClaw extends Subsystem {

    // ---------------------------------------------------------------------------------------
    // Hardware names
    // ---------------------------------------------------------------------------------------
    public static String ARM_MOTOR_NAME = "arm";
    public static String CLAW_SERVO_NAME = "claw";

    // ---------------------------------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------------------------------
    /** Encoder ticks per FULL REVOLUTION OF THE ARM. NeveRest 60 = 1680, NeveRest 40 = 1120,
     *  then multiply by any external reduction (e.g. a 2:1 chain run makes it 3360). */
    public static double TICKS_PER_ARM_REV = 1680.0;

    /**
     * The arm's real-world angle, in degrees, at the moment init() zeroes the encoder.
     * 0 = perfectly horizontal, negative = pointing down below horizontal.
     *
     * This is the single most important number here: the gravity feedforward is built on it. If
     * your arm rests on a hard stop 30 degrees below horizontal, this is -30. Measure it once with
     * a phone level - a wrong value here shows up as an arm that sags on one half of its travel
     * and overshoots on the other.
     */
    public static double START_ANGLE_DEG = 0.0;

    // ---------------------------------------------------------------------------------------
    // Feedforward
    // ---------------------------------------------------------------------------------------
    /**
     * Motor power needed to hold the arm still when it is straight out horizontal.
     *
     * Derivation for the assumed build: 0.4 kg at 0.24 m gives 0.4 * 9.81 * 0.24 = 0.94 N.m of
     * load. A NeveRest 60 stalls at about 4.19 N.m, and near zero speed torque tracks applied
     * power, so 0.94 / 4.19 = 0.22. Backed off to 0.18 because you never want to sit at stall and
     * the gearbox contributes some holding friction of its own.
     *
     * On a NeveRest 40 (2.47 N.m stall) the same arm would want roughly 0.38.
     *
     * TUNE FIRST, BEFORE ANY PID GAIN: set kP/kI/kD to 0, hold the arm horizontal by hand, and
     * raise kG until it stays put on its own when you let go.
     */
    public static double kG = 0.18;

    /** Static friction / stiction breakaway. Applied outside the tolerance band only, in the
     *  direction of travel. Too high and the arm hunts around the target. */
    public static double kS = 0.035;

    // ---------------------------------------------------------------------------------------
    // PID  (units: motor power per encoder tick / per tick-second)
    // ---------------------------------------------------------------------------------------
    /**
     * With 1680 ticks/rev the arm sees 4.67 ticks per degree, so kP = 0.004 means a 10 degree
     * error asks for 0.19 power and a 30 degree error asks for 0.56. That is firm without being
     * twitchy. Because kG already carries the weight, this only has to fight friction.
     */
    public static double kP = 0.004;
    /** Trims out the last bit of steady-state sag. Kept small and fenced in by I_ZONE + MAX_I. */
    public static double kI = 0.0006;
    /** Damping, on MEASUREMENT not error, so a new setpoint does not produce a derivative kick. */
    public static double kD = 0.0004;

    /** Integral only accumulates inside this many ticks of the target (~21 degrees). */
    public static double I_ZONE_TICKS = 100.0;
    /** Hard ceiling on the integral's contribution to output power. */
    public static double MAX_I_POWER = 0.12;
    /** EMA smoothing on the measured velocity. Encoder quantisation makes raw dPos/dt noisy, and
     *  kD multiplies that noise straight into the motor. 0 = no filtering, 0.9 = very smooth. */
    public static double D_FILTER = 0.7;

    // ---------------------------------------------------------------------------------------
    // Motion + limits
    // ---------------------------------------------------------------------------------------
    /** Setpoint slew rate. A NeveRest 60 free-spins at ~2940 ticks/s; half that keeps the profile
     *  achievable so the arm tracks the ramp instead of lagging behind it. */
    public static double MAX_VEL_TICKS_PER_SEC = 1400.0;
    public static double MAX_POWER = 0.9;
    public static double TOLERANCE_TICKS = 12.0;   // ~2.6 degrees

    /** Soft limits, in ticks from the init position. MEASURE THESE ON YOUR ARM before trusting
     *  them - they are what stops the arm driving itself into the chassis. */
    public static double MIN_TICKS = -50.0;
    public static double MAX_TICKS = 700.0;        // ~150 degrees of travel

    /** Manual-drive stick deadband and how much of MAX_POWER a full stick deflection commands. */
    public static double MANUAL_DEADBAND = 0.05;
    public static double MANUAL_SCALE = 0.6;

    // ---------------------------------------------------------------------------------------
    // Claw
    // ---------------------------------------------------------------------------------------
    public static double CLAW_OPEN = 0.0;
    public static double CLAW_CLOSED = 0.22;
    /** Seconds for a full 0->1 sweep. A 180 degree servo at ~0.15 s/60 degrees needs ~0.45 s, plus
     *  margin. Used only by isClawSettled(); a servo gives no position feedback. */
    public static double CLAW_SWEEP_SEC = 0.60;

    // ---------------------------------------------------------------------------------------
    // Hardware + state
    // ---------------------------------------------------------------------------------------
    private final DcMotorEx armMotor;
    private final Servo claw;
    private final Telemetry telemetry;

    private final ElapsedTime dtTimer = new ElapsedTime();
    private final ElapsedTime clawTimer = new ElapsedTime();

    private double targetTicks = 0.0;    // where we want to end up
    private double profiledTicks = 0.0;  // slew-limited setpoint actually fed to the PID
    private double integralSum = 0.0;
    private double velEstimate = 0.0;    // filtered, ticks/sec
    private int lastPosition = 0;
    private int position = 0;            // this loop's encoder read, taken exactly once

    private boolean manualMode = false;
    private double manualStick = 0.0;
    private boolean enabled = true;

    // Last values actually written to the hardware. NaN forces the first write through.
    // Plain setPower()/setPosition() issue a Lynx command on every call - neither caches.
    private double cachedPower = Double.NaN;
    private double cachedClaw = Double.NaN;
    private double clawTarget = CLAW_OPEN;

    private double lastOutput = 0.0;     // for telemetry only

    public ArmClaw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = hardwareMap.get(DcMotorEx.class, ARM_MOTOR_NAME);
        claw = hardwareMap.get(Servo.class, CLAW_SERVO_NAME);
    }

    /**
     * Zeroes the encoder and parks the arm where it currently sits.
     *
     * RUN_WITHOUT_ENCODER is deliberate: the encoder is still read for our PID, but the hub's own
     * velocity loop is left out of it. Running RUN_TO_POSITION here would put a second controller
     * in series with this one and the two would fight.
     *
     * Call with the arm resting at START_ANGLE_DEG.
     */
    @Override
    public void init() {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        position = 0;
        lastPosition = 0;
        targetTicks = 0.0;
        profiledTicks = 0.0;
        integralSum = 0.0;
        velEstimate = 0.0;
        manualMode = false;

        openClaw();
        dtTimer.reset();
    }

    // ---------------------------------------------------------------------------------------
    // Arm API
    // ---------------------------------------------------------------------------------------

    public void setTargetTicks(double ticks) {
        targetTicks = Range.clip(ticks, MIN_TICKS, MAX_TICKS);
        manualMode = false;
    }

    /** @param degrees arm angle measured from horizontal, the same reference as START_ANGLE_DEG. */
    public void setTargetDegrees(double degrees) {
        setTargetTicks((degrees - START_ANGLE_DEG) * ticksPerDegree());
    }

    /** Shifts the target without disturbing the profile - handy for trim buttons. */
    public void nudgeTicks(double deltaTicks) {
        setTargetTicks(targetTicks + deltaTicks);
    }

    /**
     * Direct driver control. Pass the raw stick value every loop; pass 0 when untouched.
     *
     * While the stick is live the arm is power-controlled but still gravity-compensated, so it
     * feels weightless rather than dropping the moment you take over. When the stick returns to
     * neutral the target snaps to wherever the arm ended up, so it holds there instead of
     * springing back to its old setpoint.
     */
    public void setManualPower(double stick) {
        if (Math.abs(stick) > MANUAL_DEADBAND) {
            manualMode = true;
            manualStick = stick;
        } else if (manualMode) {
            // Just released - capture the current position as the new hold target.
            manualMode = false;
            manualStick = 0.0;
            targetTicks = Range.clip(position, MIN_TICKS, MAX_TICKS);
            profiledTicks = targetTicks;
            integralSum = 0.0;
        }
    }

    /**
     * Kill switch. While disabled the motor is held at zero power but update() keeps reading the
     * encoder, so the arm tracks wherever gravity drags it and re-enabling holds it exactly where
     * it ended up instead of snapping back to a stale setpoint.
     *
     * The arm WILL sag when disabled - BRAKE only resists so much.
     */
    public void setEnabled(boolean on) {
        this.enabled = on;
    }

    public boolean isEnabled() { return enabled; }

    public boolean isAtTarget() {
        return Math.abs(targetTicks - position) < TOLERANCE_TICKS && Math.abs(velEstimate) < 40.0;
    }

    public double getAngleDegrees() {
        return START_ANGLE_DEG + (position / ticksPerDegree());
    }

    public int getPosition() { return position; }
    public double getTargetTicks() { return targetTicks; }
    public boolean isManual() { return manualMode; }

    // ---------------------------------------------------------------------------------------
    // Claw API
    // ---------------------------------------------------------------------------------------

    public void openClaw() { setClawPosition(CLAW_OPEN); }
    public void closeClaw() { setClawPosition(CLAW_CLOSED); }

    public void toggleClaw() {
        setClawPosition(isClawOpen() ? CLAW_CLOSED : CLAW_OPEN);
    }

    public void setClawPosition(double pos) {
        double clipped = Range.clip(pos, Math.min(CLAW_OPEN, CLAW_CLOSED), Math.max(CLAW_OPEN, CLAW_CLOSED));
        if (clipped != clawTarget) {
            clawTarget = clipped;
            clawTimer.reset();
        }
        writeClaw(clipped);
    }

    /** Closer to open than closed. */
    public boolean isClawOpen() {
        return Math.abs(clawTarget - CLAW_OPEN) < Math.abs(clawTarget - CLAW_CLOSED);
    }

    /**
     * Best-effort "has the claw finished moving". A hobby servo reports nothing back, so this is a
     * travel-time estimate from the size of the last commanded move - use it to sequence an
     * auto (close, wait, lift) rather than as proof the game piece is gripped.
     */
    public boolean isClawSettled() {
        double travel = Math.abs(CLAW_CLOSED - CLAW_OPEN);
        return clawTimer.seconds() >= Math.max(0.06, travel * CLAW_SWEEP_SEC);
    }

    // ---------------------------------------------------------------------------------------
    // Control loop - call once per loop, after BulkRead.clearCache()
    // ---------------------------------------------------------------------------------------
    @Override
    public void update() {
        // The one and only encoder read this loop. Bulk-cached, so it costs nothing extra.
        position = armMotor.getCurrentPosition();

        double dt = dtTimer.seconds();
        dtTimer.reset();
        if (dt < 0.0005) dt = 0.0005;   // first loop / clock jitter
        if (dt > 0.1) dt = 0.1;         // a stall or a breakpoint must not blow up I and D

        // Velocity from the encoder, smoothed. Derivative is taken on measurement, never on error.
        double rawVel = (position - lastPosition) / dt;
        lastPosition = position;
        velEstimate = (D_FILTER * velEstimate) + ((1.0 - D_FILTER) * rawVel);

        if (!enabled) {
            // Track the arm as it sags so re-enabling holds it where it actually is.
            targetTicks = Range.clip(position, MIN_TICKS, MAX_TICKS);
            profiledTicks = targetTicks;
            integralSum = 0.0;
            manualMode = false;
            manualStick = 0.0;
            lastOutput = 0.0;
            writePower(0.0);
            return;
        }

        // Gravity feedforward. cos() of the CURRENT angle, so it stays correct mid-swing.
        double angleRad = Math.toRadians(getAngleDegrees());
        double gravity = kG * Math.cos(angleRad);

        double output;

        if (manualMode) {
            // Driver has the stick. Hold the setpoint under the arm so releasing is seamless.
            targetTicks = Range.clip(position, MIN_TICKS, MAX_TICKS);
            profiledTicks = targetTicks;
            integralSum = 0.0;

            output = (manualStick * MANUAL_SCALE) + gravity;

            // Soft limits still apply - refuse to drive further past an end stop.
            if (position <= MIN_TICKS && output < gravity) output = gravity;
            if (position >= MAX_TICKS && output > gravity) output = gravity;

        } else {
            // Slew the setpoint toward the target so a big command becomes a ramp, not a step.
            double maxStep = MAX_VEL_TICKS_PER_SEC * dt;
            if (profiledTicks < targetTicks) {
                profiledTicks = Math.min(targetTicks, profiledTicks + maxStep);
            } else if (profiledTicks > targetTicks) {
                profiledTicks = Math.max(targetTicks, profiledTicks - maxStep);
            }

            double error = profiledTicks - position;

            // Integral: only near the target, only while we are not saturated, always clamped.
            if (Math.abs(error) < I_ZONE_TICKS) {
                integralSum += error * dt;
            } else {
                integralSum = 0.0;
            }
            double iTerm = kI * integralSum;
            if (iTerm > MAX_I_POWER) {
                iTerm = MAX_I_POWER;
                integralSum = (kI != 0) ? MAX_I_POWER / kI : 0.0;
            } else if (iTerm < -MAX_I_POWER) {
                iTerm = -MAX_I_POWER;
                integralSum = (kI != 0) ? -MAX_I_POWER / kI : 0.0;
            }

            output = (kP * error) + iTerm - (kD * velEstimate) + gravity;

            // Break stiction, but only when we are genuinely away from the target.
            if (Math.abs(targetTicks - position) > TOLERANCE_TICKS) {
                output += kS * Math.signum(error);
            }
        }

        output = Range.clip(output, -MAX_POWER, MAX_POWER);
        lastOutput = output;
        writePower(output);
    }

    // ---------------------------------------------------------------------------------------
    // Cached hardware writes - identical values are not re-sent to the hub
    // ---------------------------------------------------------------------------------------
    private void writePower(double power) {
        if (Double.isNaN(cachedPower) || Math.abs(power - cachedPower) >= 0.005
                || (power == 0.0 && cachedPower != 0.0)) {
            armMotor.setPower(power);
            cachedPower = power;
        }
    }

    private void writeClaw(double pos) {
        if (Double.isNaN(cachedClaw) || pos != cachedClaw) {
            claw.setPosition(pos);
            cachedClaw = pos;
        }
    }

    private double ticksPerDegree() {
        return TICKS_PER_ARM_REV / 360.0;
    }

    /** Cuts the motor. The arm will sag unless it is against a stop - BRAKE only resists so much. */
    public void stop() {
        manualMode = false;
        targetTicks = position;
        profiledTicks = position;
        integralSum = 0.0;
        writePower(0.0);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Arm ticks", position);
        telemetry.addData("Arm target", targetTicks);
        telemetry.addData("Arm angle (deg)", getAngleDegrees());
        telemetry.addData("Arm power", lastOutput);
        telemetry.addData("Arm vel (t/s)", velEstimate);
        telemetry.addData("Arm at target", isAtTarget());
        telemetry.addData("Arm manual", manualMode);
        telemetry.addData("Claw target", clawTarget);
        telemetry.addData("Claw open", isClawOpen());
    }
}
