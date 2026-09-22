package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.NGMotor;

/**
 * The flywheel shooter, plus an optional feeder that pushes one ball into it.
 *
 * <h2>All velocity control goes through NGMotor</h2>
 * The flywheel motor is wrapped in the team's existing {@link NGMotor}, and the velocity loop is
 * NGMotor's own: {@code setCustomVelocityPID(...)} installs the gains and
 * {@code updateFlywheels(boolean)} runs one iteration of the loop. Nothing here re-implements a
 * controller, and no PIDF is ever written to a raw {@code DcMotorEx}. That matters because
 * NGMotor's flywheel loop already does three things a naive controller would not: it low-pass
 * filters the measured velocity, it compensates the output for battery voltage, and it adds a
 * feed-forward boost while a ball is actually being fed, which is exactly when the wheel is being
 * dragged down.
 *
 * <h3>WHERE FLYWHEEL PIDF IS SET, exactly</h3>
 * All four of {@link TurretConstants#FLYWHEEL_P}, {@code FLYWHEEL_I}, {@code FLYWHEEL_D} and
 * {@code FLYWHEEL_F} are pushed into NGMotor by {@link #applyFlywheelPIDF()}, which runs at
 * {@link #init()} and again on every {@link #setPIDF(double, double, double, double)} call. It
 * makes TWO calls, because NGMotor has two separate control loops and two separate entry points:
 *
 * <table>
 *   <tr><th>Entry point</th><th>Fields it writes</th><th>Loop that reads them</th></tr>
 *   <tr><td>{@code NGMotor.setPIDF(P, I, D, F)}</td>
 *       <td>{@code NGMotor.P/I/D/F}</td>
 *       <td>{@code NGMotor.update()}, the POSITION loop</td></tr>
 *   <tr><td>{@code NGMotor.setCustomVelocityPID(target, P, I, D, F)}</td>
 *       <td>{@code NGMotor.kP/kI/kD/kF}</td>
 *       <td>{@code NGMotor.updateFlywheels()}, the VELOCITY loop this class runs</td></tr>
 * </table>
 *
 * <h3>Units</h3>
 * NGMotor's velocity loop computes a motor POWER in -1..1 and the error is in TICKS/SECOND, so:
 * P is power per (tick/second) of error, I is power per (tick-second), D is power per
 * (tick/second-squared), and F is a pure feedforward multiplier applied to the TARGET velocity
 * ({@code F_Term = kF * targetVelocity}), so F is also power per (tick/second).
 *
 * <h3>One honest caveat about P, which needs your decision</h3>
 * {@code NGMotor.updateFlywheels()} gain-schedules its own proportional term between two internal
 * fields, {@code kP_Recovery} (when the wheel is more than 60 ticks/s below target, i.e. recovering
 * from a shot) and {@code kP_Stable} (near target), instead of reading the {@code kP} that
 * {@code setCustomVelocityPID} stores. {@code kI}, {@code kD} and {@code kF} ARE read from it.
 * Those two fields are package-private in {@code ...teamcode.library}, so this class cannot reach
 * them without editing NGMotor, which is out of scope here and would change behaviour for every
 * other subsystem that uses NGMotor flywheels. FLYWHEEL_P is therefore WIRED but currently lands
 * only on the position loop and on {@code kP}. See the note in TurretConstants and ask before
 * changing NGMotor.
 *
 * <h2>The feeder is an assumption</h2>
 * {@link TurretConstants#FEEDER_PRESENT} defaults to false. Until someone confirms the feeder
 * hardware exists and says how it actuates, {@link #feedOne()} is a safe no-op that reports why.
 */
public class Launcher {

    private final Telemetry telemetry;

    /** The flywheel, wrapped. Null only if the motor is missing from the configuration. */
    private NGMotor flywheel;

    /** Optional feeder/indexer. Null when absent or not configured. */
    private Servo feeder;

    /** Commanded flywheel velocity, TICKS/SECOND. 0 means "stopped". */
    private double targetVelocityTps = 0.0;

    /** False while stopped, so the velocity loop is not run and cannot drive the wheel backwards. */
    private boolean spinning = false;

    /** True while a feed stroke is in progress - NGMotor uses this to boost through the load. */
    private boolean feeding = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    private String hardwareFault = "";

    /**
     * Grabs the flywheel motor (and feeder, if configured) and applies the PIDF constants.
     * A missing motor is reported rather than thrown.
     */
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            flywheel = new NGMotor(hardwareMap, telemetry, TurretConstants.LAUNCHER_MOTOR_NAME);
            flywheel.setDirection(TurretConstants.LAUNCHER_MOTOR_REVERSED
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            flywheel = null;
            hardwareFault = "flywheel motor not found: " + e.getMessage();
            if (telemetry != null) telemetry.addLine("LAUNCHER: " + hardwareFault);
        }

        if (TurretConstants.FEEDER_PRESENT) {
            try {
                feeder = hardwareMap.get(Servo.class, TurretConstants.FEEDER_SERVO_NAME);
            } catch (Exception e) {
                feeder = null;
                hardwareFault = "feeder servo not found: " + e.getMessage();
                if (telemetry != null) telemetry.addLine("LAUNCHER: " + hardwareFault);
            }
        }
    }

    /**
     * Resets the motor and installs the flywheel PIDF from {@link TurretConstants}.
     * Call once, after construction, before the opmode starts.
     */
    public void init() {
        if (flywheel != null) {
            // NGMotor.init() puts the motor in RUN_WITHOUT_ENCODER with BRAKE, which is what the
            // flywheel loop expects: it computes the power itself rather than handing a velocity
            // to the motor controller's own built-in PIDF.
            flywheel.init();
            applyFlywheelPIDF();
        }
        hold();
    }

    /**
     * Pushes {@link TurretConstants#FLYWHEEL_P}/I/D/F into NGMotor. THIS is the single place
     * flywheel PIDF is set; see the table in the class javadoc for which loop reads which field.
     *
     * Both NGMotor entry points are fed from the same four constants, so the position loop and the
     * velocity loop can never drift out of agreement with each other.
     *
     * Resets NGMotor's integral accumulator and loop timer, so this is called at init and on an
     * explicit re-tune, never per loop.
     */
    public void applyFlywheelPIDF() {
        if (flywheel == null) return;

        // Entry point 1: NGMotor's own setPIDF, writing NGMotor.P/I/D/F (the position loop).
        flywheel.setPIDF(TurretConstants.FLYWHEEL_P,
                         TurretConstants.FLYWHEEL_I,
                         TurretConstants.FLYWHEEL_D,
                         TurretConstants.FLYWHEEL_F);

        // Entry point 2: NGMotor's flywheel velocity loop, writing NGMotor.kP/kI/kD/kF. The target
        // starts at zero; setTargetVelocity() moves it afterwards without touching these gains.
        flywheel.setCustomVelocityPID(targetVelocityTps,
                         TurretConstants.FLYWHEEL_P,
                         TurretConstants.FLYWHEEL_I,
                         TurretConstants.FLYWHEEL_D,
                         TurretConstants.FLYWHEEL_F);
    }

    /**
     * Re-applies PIDF gains at runtime from explicit values rather than from constants.
     * Convenience pass-through for dashboard tuning; hits both NGMotor entry points, exactly as
     * {@link #applyFlywheelPIDF()} does. Resets the integral accumulator, so not per loop.
     *
     * @param p power per (tick/second) of velocity error
     * @param i power per (tick-second)
     * @param d power per (tick/second-squared)
     * @param f feedforward, power per (tick/second) of TARGET velocity
     */
    public void setPIDF(double p, double i, double d, double f) {
        if (flywheel == null) return;
        flywheel.setPIDF(p, i, d, f);
        flywheel.setCustomVelocityPID(targetVelocityTps, p, i, d, f);
    }

    // =============================================================================================
    // Velocity control
    // =============================================================================================

    /**
     * Sets the flywheel velocity target.
     *
     * @param velocityTps target velocity, TICKS/SECOND. Zero or less stops the wheel.
     *
     * The target is written straight to NGMotor's public {@code targetVelocity} field rather than
     * through {@code setCustomVelocityPID}, on purpose: that method also zeroes the integral
     * accumulator and restarts the loop timer. This is called every loop with a velocity that
     * drifts as the robot moves, so routing it through there would reset the integral term dozens
     * of times a second and the wheel would never settle.
     */
    public void setTargetVelocity(double velocityTps) {
        if (velocityTps <= 0.0) {
            stop();
            return;
        }
        targetVelocityTps = velocityTps;
        spinning = true;
        if (flywheel != null) flywheel.targetVelocity = velocityTps;
    }

    /**
     * Runs ONE iteration of NGMotor's flywheel velocity loop. Must be called every loop while
     * spinning, or the wheel coasts.
     *
     * While stopped it commands zero power directly instead of running the loop, because a velocity
     * loop asked for zero would fight the wheel's own momentum and brake it hard.
     */
    public void update() {
        if (flywheel == null) return;

        // End a feed stroke that has run its course.
        if (feeding && feedTimer.seconds() >= TurretConstants.FEEDER_STROKE_TIME_S) {
            hold();
        }

        if (spinning) {
            // The feeding flag is what triggers NGMotor's load-compensation boost, which is the
            // whole reason it takes this argument: the wheel is being dragged down at exactly this
            // moment and the plain PID would not react until after the shot had already gone slow.
            flywheel.updateFlywheels(feeding);
        } else {
            flywheel.targetVelocity = 0.0;
            flywheel.setAbsPower(0.0);
        }
    }

    /** Stops the flywheel and cuts power. */
    public void stop() {
        targetVelocityTps = 0.0;
        spinning = false;
        if (flywheel != null) {
            flywheel.targetVelocity = 0.0;
            flywheel.setAbsPower(0.0);
        }
    }

    /**
     * True when the wheel is spun up to within {@link TurretConstants#LAUNCHER_VELOCITY_TOLERANCE_TPS}
     * of its target. This is the gate on firing: below it the shot throws short.
     *
     * Returns false whenever the target is zero, so "stopped" never reads as "ready".
     */
    public boolean isAtVelocity() {
        return isAtVelocity(TurretConstants.LAUNCHER_VELOCITY_TOLERANCE_TPS);
    }

    /**
     * True when the measured velocity is within {@code toleranceTps} of the target.
     *
     * @param toleranceTps allowed error, TICKS/SECOND
     */
    public boolean isAtVelocity(double toleranceTps) {
        if (flywheel == null || !spinning || targetVelocityTps <= 0.0) return false;
        return Math.abs(targetVelocityTps - flywheel.getVelocity()) <= toleranceTps;
    }

    /** Measured flywheel velocity, TICKS/SECOND, or 0 when the motor is missing. */
    public double getVelocity() {
        return (flywheel == null) ? 0.0 : flywheel.getVelocity();
    }

    /** Commanded flywheel velocity, TICKS/SECOND. */
    public double getTargetVelocity() {
        return targetVelocityTps;
    }

    /** True when the flywheel has been told to spin. */
    public boolean isSpinning() {
        return spinning;
    }

    // =============================================================================================
    // Feeder
    // =============================================================================================

    /**
     * Pushes one ball into the flywheel and starts the stroke timer.
     *
     * <h3>The feed mechanism is NOT yet defined on this robot</h3>
     * {@link TurretConstants#FEEDER_PRESENT} is false and no feeder hardware has been confirmed, so
     * this currently does nothing and says so. It is deliberately a reporting no-op rather than a
     * guessed servo stroke: inventing a feeder would make the fire path LOOK functional on
     * telemetry while nothing moved, which is the worst possible failure mode to debug on a field.
     *
     * Ignored while a stroke is already in progress, so holding the fire button cannot try to jam
     * two balls in at once.
     *
     * @return true if a feed stroke actually started; false if there is no feeder or one is already
     *         running. Callers should surface a false on telemetry rather than ignore it.
     */
    public boolean feedOne() {
        if (feeder == null) return false;
        if (feeding) return false;
        feeder.setPosition(TurretConstants.FEEDER_FEED_POSITION);
        feeding = true;
        feedTimer.reset();
        return true;
    }

    /**
     * Why a fire command cannot be executed, or "" when the launcher itself is ready.
     * Covers only the LAUNCHER's reasons; aiming reasons come from
     * {@link TurretAimController#getNotReadyReason()}.
     */
    public String getFireBlockReason() {
        if (flywheel == null) return "flywheel motor missing";
        if (!hasFeeder()) {
            return TurretConstants.FEEDER_PRESENT
                    ? "feeder servo not found in configuration"
                    : "NO FEEDER CONFIGURED - feed mechanism undefined";
        }
        if (!spinning) return "flywheel not spinning";
        if (!isAtVelocity()) return "flywheel not at velocity";
        if (feeding) return "feed stroke in progress";
        return "";
    }

    /** Returns the feeder to its holding position and ends the stroke. */
    public void hold() {
        if (feeder != null) feeder.setPosition(TurretConstants.FEEDER_HOLD_POSITION);
        feeding = false;
    }

    /** True while a feed stroke is in progress. */
    public boolean isFeeding() {
        return feeding;
    }

    /** True when a feeder is configured and was found. */
    public boolean hasFeeder() {
        return feeder != null;
    }

    /** True when the flywheel motor was found. */
    public boolean isHardwareOk() {
        return flywheel != null;
    }

    /** Last hardware problem, or "". Telemetry only. */
    public String getHardwareFault() {
        return hardwareFault;
    }

    /** Adds launcher state to telemetry. */
    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Launcher target (tps)", "%.0f", targetVelocityTps);
        telemetry.addData("Launcher actual (tps)", "%.0f", getVelocity());
        telemetry.addData("Launcher at velocity", isAtVelocity());
        telemetry.addData("Feeder", hasFeeder() ? (feeding ? "FEEDING" : "HOLD") : "NOT CONFIGURED");
        if (!hardwareFault.isEmpty()) telemetry.addData("Launcher fault", hardwareFault);
    }
}
