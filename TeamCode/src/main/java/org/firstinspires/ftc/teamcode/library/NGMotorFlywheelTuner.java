package org.firstinspires.ftc.teamcode.library;

/**
 * Read/write access to the flywheel constants that {@link NGMotor} keeps package-private.
 *
 * <h2>Why this class has to exist</h2>
 * {@code NGMotor.updateFlywheels()} does NOT use the {@code P} value handed to
 * {@code setCustomVelocityPID(target, P, I, D, F)}. That value is stored in {@code kP} and then
 * never read. The real proportional gain is gain-scheduled between two other fields:
 *
 * <pre>
 *   if (error &gt; RECOVERY_THRESHOLD)  P_Term = error * kP_Recovery;   // NGMotor.java:171
 *   else                              P_Term = error * kP_Stable;     // NGMotor.java:173
 * </pre>
 *
 * {@code kP_Recovery}, {@code kP_Stable}, {@code ALPHA} and {@code RECOVERY_THRESHOLD} are all
 * declared with default (package-private) access at NGMotor.java lines 86-91, so only a class in
 * {@code org.firstinspires.ftc.teamcode.library} can touch them. A tuning opmode living in
 * {@code opmodes.testing_opmodes} cannot, which would leave the single most important flywheel gain
 * untunable from the very opmode written to tune it.
 *
 * <h2>Why not just edit NGMotor</h2>
 * Because {@code kP_Recovery} and {@code kP_Stable} are shared by every flywheel NGMotor drives,
 * including {@code Intake2_0}'s. Changing how they are read changes behaviour for subsystems
 * outside this tuning work. This class is purely additive: NGMotor is not modified, and nothing
 * here runs unless a tuning opmode calls it.
 *
 * <h2>Scope</h2>
 * TUNING AND DIAGNOSTICS ONLY. Nothing in a competition opmode should call this. Once the values
 * are settled they belong in NGMotor's own field initialisers, which is where the permanent home
 * for them is - see the report that came with this file.
 */
public final class NGMotorFlywheelTuner {

    private NGMotorFlywheelTuner() { } // static utility - never instantiated

    // =============================================================================================
    // Proportional gains - the ones setCustomVelocityPID cannot reach
    // =============================================================================================

    /**
     * Sets the RECOVERY proportional gain, power per (tick/second) of error.
     *
     * Applied only while the wheel is more than {@code RECOVERY_THRESHOLD} BELOW target, which in
     * practice means "just after a ball went through". Note the comparison is one-sided
     * ({@code error > threshold}, not {@code abs(error)}), so this gain never acts on overshoot.
     */
    public static void setRecoveryKp(NGMotor motor, double value) {
        if (motor == null) return;
        motor.kP_Recovery = value;
    }

    /** @return the recovery proportional gain currently in effect. */
    public static double getRecoveryKp(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.kP_Recovery;
    }

    /**
     * Sets the STABLE proportional gain, power per (tick/second) of error.
     *
     * Applied whenever the wheel is within {@code RECOVERY_THRESHOLD} of target AND whenever it is
     * ABOVE target by any amount. It is therefore the only proportional term that ever corrects
     * overshoot, which is why a too-high feedforward takes so long to settle out.
     */
    public static void setStableKp(NGMotor motor, double value) {
        if (motor == null) return;
        motor.kP_Stable = value;
    }

    /** @return the stable proportional gain currently in effect. */
    public static double getStableKp(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.kP_Stable;
    }

    // =============================================================================================
    // Filter and scheduling constants
    // =============================================================================================

    /**
     * Sets the velocity low-pass factor, unitless 0..1, where 1.0 is no filtering at all.
     * {@code smooth = ALPHA*raw + (1-ALPHA)*previousSmooth}.
     *
     * Lower values reject more encoder quantisation noise but add lag, and lag in a velocity loop
     * shows up as overshoot after every shot.
     */
    public static void setAlpha(NGMotor motor, double value) {
        if (motor == null) return;
        motor.ALPHA = value;
    }

    /** @return the velocity low-pass factor currently in effect. */
    public static double getAlpha(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.ALPHA;
    }

    /**
     * Sets the recovery/integral threshold, TICKS/SECOND. This one number does two jobs at once:
     * <ul>
     *   <li>above it (below target only), the recovery proportional gain takes over;</li>
     *   <li>outside +-it, the integral accumulator is FORCIBLY ZEROED, not merely frozen.</li>
     * </ul>
     * So it is simultaneously the "a shot just happened" detector and the anti-windup band.
     */
    public static void setRecoveryThreshold(NGMotor motor, double value) {
        if (motor == null) return;
        motor.RECOVERY_THRESHOLD = value;
    }

    /** @return the recovery/integral threshold currently in effect, ticks/second. */
    public static double getRecoveryThreshold(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.RECOVERY_THRESHOLD;
    }

    // =============================================================================================
    // Diagnostics
    // =============================================================================================

    /**
     * The FILTERED velocity the controller is actually working from, TICKS/SECOND.
     *
     * Worth graphing next to the raw velocity: the gap between the two is the lag the low-pass is
     * adding, and that lag is what the derivative term differentiates.
     */
    public static double getSmoothedVelocity(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.lastSmoothVelocity;
    }

    /**
     * Clears the filter's memory so a fresh spin-up is not dragged by the last run's value.
     * Call while the wheel is stopped.
     */
    public static void resetVelocityFilter(NGMotor motor) {
        if (motor == null) return;
        motor.lastSmoothVelocity = 0.0;
    }
}
