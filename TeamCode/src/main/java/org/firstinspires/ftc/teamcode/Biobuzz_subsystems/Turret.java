package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Hardware layer for the turret: two ganged REV servos, an optional absolute output encoder, and
 * the angle-to-servo-position mapping. Nothing else.
 *
 * <h2>Why there is no aiming logic in here</h2>
 * This class knows how to point the turret at an angle and how to report where it actually is. It
 * does not know what a HIVE CELL is, what the camera saw, or whether it is allowed to shoot. All
 * of that lives in {@link TurretAimController}. Keeping the split means the aim logic can be read
 * and reasoned about without servo trivia in the way, and the servo mapping can be recalibrated
 * without touching control code.
 *
 * <h2>The angle mapping</h2>
 * Two REV 270-degree servos through a 1.5:1 reduction give 270 / 1.5 = 180 degrees at the output.
 * Centre is servo position 0.5, and the full 180-degree output span covers the full 0..1 servo
 * span, so:
 *
 * <pre>servoPosition = 0.5 + degrees / 180</pre>
 *
 * which puts -90 degrees at 0.0 and +90 degrees at 1.0.
 *
 * <h2>The two servos</h2>
 * They are mechanically ganged onto one output, so they must be driven to positions that
 * correspond to the SAME physical angle. If one is mounted mirrored, its command is the
 * complement of the other's ({@code 1 - position}) - see {@link TurretConstants#SERVO_B_REVERSED}.
 * Getting that wrong makes the two servos fight, which stalls both and cooks them, so it is worth
 * checking at low power before the first real run.
 */
public class Turret {

    private final Telemetry telemetry;

    private Servo servoA;
    private Servo servoB;

    /**
     * Optional absolute output encoder, read through a motor port. Null whenever the encoder is
     * disabled in constants or could not be found, in which case every measurement degrades to NaN
     * and the controller runs open-loop.
     */
    private DcMotorEx encoder;

    /** Last angle commanded, DEGREES, CCW-positive, 0 = turret mechanical zero. */
    private double commandedAngleDeg = 0.0;

    /** True when the last command was clamped by a soft limit - i.e. the caller asked for more. */
    private boolean lastCommandClamped = false;

    private String hardwareFault = "";

    /**
     * Grabs the servos and, if configured, the encoder.
     *
     * A missing servo is reported and left null rather than thrown, so that a configuration typo
     * during a competition shows up as telemetry and a dead turret instead of an opmode that will
     * not start at all.
     */
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            servoA = hardwareMap.get(Servo.class, TurretConstants.TURRET_SERVO_A_NAME);
            servoB = hardwareMap.get(Servo.class, TurretConstants.TURRET_SERVO_B_NAME);
        } catch (Exception e) {
            servoA = null;
            servoB = null;
            hardwareFault = "turret servos not found: " + e.getMessage();
            if (telemetry != null) telemetry.addLine("TURRET: " + hardwareFault);
        }

        applyScaleRanges();

        if (TurretConstants.TURRET_ENCODER_PRESENT) {
            try {
                encoder = hardwareMap.get(DcMotorEx.class, TurretConstants.TURRET_ENCODER_NAME);
            } catch (Exception e) {
                // "Encoder configured but absent" is exactly the case that must degrade quietly to
                // open-loop rather than crash: the turret still aims, just without feedback.
                encoder = null;
                hardwareFault = "turret encoder not found: " + e.getMessage();
                if (telemetry != null) telemetry.addLine("TURRET: " + hardwareFault);
            }
        }
    }

    /**
     * Pushes the per-servo {@code scaleRange} calibration from constants onto both servos.
     *
     * {@code scaleRange} narrows the usable band of a servo so that a commanded 0..1 maps onto the
     * part of its travel that is mechanically reachable. Two ganged servos rarely have identical
     * effective ranges, and this is where that difference is absorbed.
     *
     * Public so the tuning opmode can re-apply it live after editing the constants on the
     * dashboard, without restarting the opmode.
     */
    public void applyScaleRanges() {
        try {
            if (servoA != null) {
                servoA.scaleRange(TurretConstants.SERVO_A_SCALE_MIN, TurretConstants.SERVO_A_SCALE_MAX);
            }
            if (servoB != null) {
                servoB.scaleRange(TurretConstants.SERVO_B_SCALE_MIN, TurretConstants.SERVO_B_SCALE_MAX);
            }
        } catch (Exception e) {
            // scaleRange throws if min >= max, which is a constants typo, not a hardware fault.
            hardwareFault = "bad servo scaleRange constants: " + e.getMessage();
            if (telemetry != null) telemetry.addLine("TURRET: " + hardwareFault);
        }
    }

    // =============================================================================================
    // Commanding
    // =============================================================================================

    /**
     * Points the turret at an angle, clamped to the soft limits.
     *
     * @param deg target angle, DEGREES, CCW-positive, 0 = turret mechanical zero (which is
     *            {@link TurretConstants#TURRET_MOUNT_OFFSET_DEG} away from robot forward)
     * @return the angle actually commanded after clamping, DEGREES
     */
    public double setOutputAngleDeg(double deg) {
        double clamped = clampAngle(deg);
        lastCommandClamped = Math.abs(clamped - deg) > 1e-6;
        commandedAngleDeg = clamped;

        // servoPos = 0.5 + deg/180, i.e. the whole 180-degree output span across the full servo
        // span, with the mechanical centre at SERVO_CENTER_POSITION.
        double pos = TurretConstants.SERVO_CENTER_POSITION
                + (clamped / TurretConstants.OUTPUT_RANGE_DEG);

        // Guard against a constants edit (a changed centre or reduction) pushing the computed
        // position outside what a servo will accept, which would throw.
        pos = Math.max(0.0, Math.min(1.0, pos));

        if (servoA != null) servoA.setPosition(pos);
        if (servoB != null) {
            servoB.setPosition(TurretConstants.SERVO_B_REVERSED ? (1.0 - pos) : pos);
        }
        return clamped;
    }

    /**
     * Drives both servos to a RAW position, bypassing the angle mapping and the soft limits.
     *
     * This exists for one job: finding the servo positions at the physical travel limits so that
     * {@link TurretConstants#SERVO_A_SCALE_MIN} and friends can be filled in. The angle mapping
     * cannot be used for that, because the mapping is exactly what is being calibrated.
     *
     * USE ONLY FROM THE TUNING OPMODE, and jog slowly - there is nothing stopping this from
     * driving the turret into its hard stop and stalling both servos.
     *
     * @param pos raw servo position, unitless 0..1 (clamped)
     */
    public void setRawServoPosition(double pos) {
        double p = Math.max(0.0, Math.min(1.0, pos));
        if (servoA != null) servoA.setPosition(p);
        if (servoB != null) servoB.setPosition(TurretConstants.SERVO_B_REVERSED ? (1.0 - p) : p);
        // Keep the reported angle honest about what was actually commanded.
        commandedAngleDeg = (p - TurretConstants.SERVO_CENTER_POSITION)
                * TurretConstants.OUTPUT_RANGE_DEG;
    }

    /** Sends the turret back to its mechanical centre. */
    public void center() {
        setOutputAngleDeg(0.0);
    }

    /** Clamps an angle to the soft travel limits, DEGREES. */
    public static double clampAngle(double deg) {
        if (deg > TurretConstants.TURRET_MAX_ANGLE_DEG) return TurretConstants.TURRET_MAX_ANGLE_DEG;
        if (deg < TurretConstants.TURRET_MIN_ANGLE_DEG) return TurretConstants.TURRET_MIN_ANGLE_DEG;
        return deg;
    }

    /** True when the requested angle was outside the soft limits and had to be clamped. */
    public boolean wasLastCommandClamped() {
        return lastCommandClamped;
    }

    // =============================================================================================
    // Measuring
    // =============================================================================================

    /**
     * The turret's TRUE output angle from the absolute encoder, DEGREES, CCW-positive.
     *
     * @return the measured angle, or {@link Double#NaN} when no encoder is installed or it could
     *         not be read. NaN is deliberate rather than falling back to the commanded angle: the
     *         controller must be able to tell "I know where the turret is" from "I am assuming the
     *         servo went where I told it", because those two justify very different amounts of
     *         trust in the aim.
     */
    public double getMeasuredAngleDeg() {
        if (encoder == null) return Double.NaN;
        try {
            double ticks = encoder.getCurrentPosition() - TurretConstants.TURRET_ENCODER_ZERO_OFFSET_TICKS;
            if (TurretConstants.TURRET_ENCODER_REVERSED) ticks = -ticks;
            return ticks * (360.0 / TurretConstants.TURRET_ENCODER_TICKS_PER_REV);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /** True when an output encoder is installed and readable, i.e. closed-loop aiming is available. */
    public boolean hasEncoder() {
        return encoder != null;
    }

    /**
     * The best available estimate of where the turret is pointed, DEGREES.
     * The measured angle when an encoder exists, otherwise the commanded angle.
     * Use this for display and for the crosshair; use {@link #getMeasuredAngleDeg()} when the
     * distinction between measured and assumed matters.
     */
    public double getAngleDeg() {
        double measured = getMeasuredAngleDeg();
        return Double.isNaN(measured) ? commandedAngleDeg : measured;
    }

    /** The last angle commanded, DEGREES, regardless of whether the turret got there. */
    public double getCommandedAngleDeg() {
        return commandedAngleDeg;
    }

    /**
     * True when the turret is sitting against (or within
     * {@link TurretConstants#TURRET_HARD_STOP_MARGIN_DEG} of) a soft travel limit.
     *
     * This gates firing: a turret pinned at its limit is not pointed where the controller asked,
     * so the aim error telemetry would look fine while the shot missed.
     */
    public boolean isAtHardStop() {
        double angle = getAngleDeg();
        return angle >= (TurretConstants.TURRET_MAX_ANGLE_DEG - TurretConstants.TURRET_HARD_STOP_MARGIN_DEG)
            || angle <= (TurretConstants.TURRET_MIN_ANGLE_DEG + TurretConstants.TURRET_HARD_STOP_MARGIN_DEG);
    }

    /** True when the servos were found and the turret can actually be commanded. */
    public boolean isHardwareOk() {
        return servoA != null && servoB != null;
    }

    /** Last hardware problem seen, or "" when everything is fine. Telemetry only. */
    public String getHardwareFault() {
        return hardwareFault;
    }

    /** Adds turret hardware state to telemetry. Called by opmodes, never internally per-loop. */
    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Turret commanded (deg)", "%.1f", commandedAngleDeg);
        double measured = getMeasuredAngleDeg();
        telemetry.addData("Turret measured (deg)", Double.isNaN(measured) ? "no encoder"
                : String.format("%.1f", measured));
        telemetry.addData("Turret at hard stop", isAtHardStop());
        if (!hardwareFault.isEmpty()) telemetry.addData("Turret fault", hardwareFault);
    }
}
