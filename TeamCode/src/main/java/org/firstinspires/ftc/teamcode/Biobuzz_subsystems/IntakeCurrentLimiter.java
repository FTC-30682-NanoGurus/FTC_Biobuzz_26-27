package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * INTAKE CURRENT LIMITER - shared by BiobuzzShooterTeleOp and BiobuzzTurretTeleOp (each has its own
 * INTAKE_CURRENT_LIMIT_ENABLED switch). When the robot holds 4 POLLEN the next one cannot go in, the
 * intake rollers stall against it and the motor current jumps. Once the current stays at or above
 * FULL_CURRENT_AMPS for SPIKE_CONFIRM_MS, {@link #update} returns true (the teleop turns the intake
 * off) and the limiter latches FULL.
 *
 * <ul>
 *   <li>Every time the intake starts, the motor's start-up inrush is ignored for INRUSH_IGNORE_MS.</li>
 *   <li>Starting the intake while FULL is latched (driver pressed A) is a RE-CHECK: a shorter ignore
 *       window (RECHECK_INRUSH_IGNORE_MS) and RECHECK_CONFIRM_MS (0 = the first sample over the
 *       threshold) so the intake is cut again as soon as possible if the robot is still full. The
 *       first sample under the threshold clears FULL and normal detection carries on.</li>
 *   <li>After a volley the teleop calls {@link #clearFull} and restarts the intake.</li>
 * </ul>
 *
 * getCurrent() is NOT part of the bulk read - it is one extra hub round trip (~1-2 ms), so it is only
 * read while the intake is actually running in.
 *
 * TUNE FULL_CURRENT_AMPS on the robot: watch the "Intake current" telemetry while the intake runs
 * empty, then with 4 POLLEN loaded, and set it between the two (closer to the full reading).
 */
@Config
public class IntakeCurrentLimiter {
    /** Current that means "4 POLLEN, rollers stalled", AMPS. PLACEHOLDER - tune on the robot. */
    public static double FULL_CURRENT_AMPS = 4.0;
    /** Ignore the start-up inrush for this long after the intake turns on, MILLISECONDS. */
    public static double INRUSH_IGNORE_MS = 250.0;
    /** The current must stay over the threshold this long to count (one POLLEN squeezing through is shorter), MILLISECONDS. */
    public static double SPIKE_CONFIRM_MS = 120.0;
    /** Re-check after A while FULL: inrush ignore, MILLISECONDS. */
    public static double RECHECK_INRUSH_IGNORE_MS = 80.0;
    /** Re-check after A while FULL: confirm time, MILLISECONDS (0 = cut on the first sample over). */
    public static double RECHECK_CONFIRM_MS = 0.0;

    private final DcMotorEx motor;
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime aboveTimer = new ElapsedTime();
    private boolean wasRunning = false, above = false, full = false, recheck = false;
    private double amps = Double.NaN, tripAmps = Double.NaN;

    public IntakeCurrentLimiter(DcMotorEx intakeMotor) {
        motor = intakeMotor;
    }

    /**
     * Once per loop.
     * @param running the intake is running IN on the driver's toggle (not reversing, not feeding a shot)
     * @return true on the loop the limit trips - turn the intake off
     */
    public boolean update(boolean running) {
        if (!running || motor == null) {
            wasRunning = false;
            above = false;
            return false;
        }
        if (!wasRunning) {
            wasRunning = true;
            runTimer.reset();
            above = false;
            recheck = full;
        }
        amps = motor.getCurrent(CurrentUnit.AMPS);
        if (runTimer.milliseconds() < (recheck ? RECHECK_INRUSH_IGNORE_MS : INRUSH_IGNORE_MS)) return false;
        if (amps >= FULL_CURRENT_AMPS) {
            if (!above) {
                above = true;
                aboveTimer.reset();
            }
            if (aboveTimer.milliseconds() >= (recheck ? RECHECK_CONFIRM_MS : SPIKE_CONFIRM_MS)) {
                full = true;
                recheck = false;
                above = false;
                wasRunning = false;
                tripAmps = amps;
                return true;
            }
        } else {
            above = false;
            if (recheck) {   // ran free after A: the robot is no longer full
                recheck = false;
                full = false;
            }
        }
        return false;
    }

    /** POLLEN were shot: no longer full. */
    public void clearFull() {
        full = false;
        recheck = false;
    }

    /** Feature switched off: forget everything. */
    public void reset() {
        clearFull();
        wasRunning = false;
        above = false;
    }

    public boolean isFull() {
        return full;
    }

    public boolean isRechecking() {
        return recheck && wasRunning;
    }

    /** Last reading, AMPS (NaN until the intake has run). */
    public double getAmps() {
        return amps;
    }

    public String status() {
        String a = Double.isNaN(amps) ? "--" : String.format("%.2f", amps);
        if (recheck && wasRunning) return String.format("%s A - RE-CHECKING after A (limit %.2f A)", a, FULL_CURRENT_AMPS);
        if (full) return String.format("FULL (tripped at %.2f A, limit %.2f A) - A re-checks", tripAmps, FULL_CURRENT_AMPS);
        return String.format("%s A (limit %.2f A)", a, FULL_CURRENT_AMPS);
    }
}
