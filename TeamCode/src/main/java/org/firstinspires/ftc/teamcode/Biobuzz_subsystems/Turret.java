package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The POLLEN turret: one DC motor ({@link TurretConstants#TURRET_MOTOR_NAME}, an AndyMark NeverRest
 * 19.2:1 through a 55T -> 220T stage), its built-in encoder, and a position controller. It also
 * holds the aim GEOMETRY (where the turret has to point to hit a field point), so the teleop and
 * the autos aim with exactly the same maths.
 *
 * <h2>Angles</h2>
 * DEGREES, CCW-positive (turret turned to the robot's LEFT is +), 0 = the turret's zero, which is
 * {@link TurretConstants#TURRET_MOUNT_OFFSET_DEG} from robot forward (0 = straight forward).
 * Travel is +-{@link TurretConstants#TURRET_LIMIT_DEG}: that ONE constant is the travel limit for
 * every opmode (teleop, every auto, the tuning opmode), read live, so changing it on Dashboard
 * changes them all.
 *
 * <h2>Zero</h2>
 * The motor encoder only counts relative to where it was reset, so the turret must face its zero
 * (straight forward) when it is reset:
 * <ul>
 *   <li>Every AUTO resets it at INIT ({@code rezero = true}) - line the turret up forward first.</li>
 *   <li>TeleOp KEEPS the auto's zero if an auto ran since the Robot Controller app started (the
 *       hub keeps counting between opmodes), otherwise resets at INIT - line it up forward first.</li>
 * </ul>
 *
 * <h2>Control</h2>
 * Position -> velocity -> power, all in degrees: the position error asks for a velocity (capped by
 * {@link TurretConstants#TURRET_MAX_VEL_DEG_S}, and by what can still brake at
 * {@link TurretConstants#TURRET_DECEL_DEG_S2}), plus the caller's feedforward rate (how fast the
 * target is moving in the robot frame, e.g. while the chassis turns), and the power is
 * kS + kV * velocity + kVel * velocity error. Never pushes further out past a travel limit.
 */
public class Turret {

    /** An opmode reset the encoder since the Robot Controller app started (see "Zero" above). */
    private static boolean zeroedThisSession = false;

    private final Telemetry telemetry;
    private DcMotorEx motor;
    private String hardwareFault = "";

    private double targetDeg = 0.0;
    private double ffDegS = 0.0;
    private boolean lastCommandClamped = false;
    private double angleDeg = 0.0, velocityDegS = 0.0;
    private double lastPower = Double.NaN;
    private boolean manual = false;
    private double manualPower = 0.0;
    private boolean keptZero = false;

    /**
     * @param rezero true = reset the encoder here (the turret must be facing its zero). false = keep
     *               the zero an earlier opmode set this session, or reset if there was none.
     */
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, boolean rezero) {
        this.telemetry = telemetry;
        try {
            motor = hardwareMap.get(DcMotorEx.class, TurretConstants.TURRET_MOTOR_NAME);
            motor.setDirection(TurretConstants.TURRET_MOTOR_REVERSED
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (rezero || !zeroedThisSession) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                zeroedThisSession = true;
            } else {
                keptZero = true;
            }
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
            lastPower = 0.0;
        } catch (Exception e) {
            motor = null;
            hardwareFault = "'" + TurretConstants.TURRET_MOTOR_NAME + "' not found: " + e.getMessage();
            if (telemetry != null) telemetry.addLine("TURRET: " + hardwareFault);
        }
    }

    /** Old constructor shape: keeps the session zero (teleop behaviour). */
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    // =============================================================================================
    // Limits and units - all derived live from TurretConstants
    // =============================================================================================

    /** Encoder ticks per turret DEGREE: motor ticks/rev x gear ratio / 360. */
    public static double ticksPerDeg() {
        return TurretConstants.TURRET_MOTOR_TICKS_PER_REV * TurretConstants.TURRET_GEAR_RATIO / 360.0;
    }

    public static double maxAngleDeg() {
        return Math.abs(TurretConstants.TURRET_LIMIT_DEG);
    }

    public static double minAngleDeg() {
        return -Math.abs(TurretConstants.TURRET_LIMIT_DEG);
    }

    /** Clamps an angle to the travel limits, DEGREES. */
    public static double clampAngle(double deg) {
        return Math.max(minAngleDeg(), Math.min(maxAngleDeg(), deg));
    }

    public static double normalizeDeg(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d <= -180.0) d += 360.0;
        return d;
    }

    // =============================================================================================
    // Commanding - call update() once per loop after setting the target
    // =============================================================================================

    /**
     * Where the turret should point and how fast that point is moving.
     * @param deg   turret angle, DEGREES (clamped to the travel limits)
     * @param ffDegS feedforward, DEGREES/SECOND: how fast {@code deg} is changing right now
     */
    public void setTarget(double deg, double ffDegS) {
        double clamped = clampAngle(deg);
        lastCommandClamped = Math.abs(clamped - deg) > 1e-6;
        targetDeg = clamped;
        this.ffDegS = lastCommandClamped ? 0.0 : ffDegS;
        manual = false;
    }

    /** Hold an angle (e.g. 0 = forward for the autos that shoot straight ahead). Call every loop. */
    public void hold(double deg) {
        setTarget(deg, 0.0);
        update();
    }

    /** Open-loop power, for the tuning opmode. Still stops at the travel limits. */
    public void setManualPower(double power) {
        manual = true;
        manualPower = power;
    }

    /** Old API: point at an angle now. Returns the angle actually commanded. */
    public double setOutputAngleDeg(double deg) {
        setTarget(deg, 0.0);
        update();
        return targetDeg;
    }

    public void center() {
        setOutputAngleDeg(0.0);
    }

    /** Reads the encoder only, never powers the motor (INIT: nothing may move before START). */
    public void refresh() {
        if (motor == null) return;
        double tpd = ticksPerDeg();
        angleDeg = motor.getCurrentPosition() / tpd;
        velocityDegS = motor.getVelocity() / tpd;
    }

    /** Reads the encoder and writes the motor power. Once per loop. */
    public void update() {
        if (motor == null) return;
        double tpd = ticksPerDeg();
        angleDeg = motor.getCurrentPosition() / tpd;
        velocityDegS = motor.getVelocity() / tpd;

        double power;
        if (manual) {
            power = manualPower;
        } else {
            double err = targetDeg - angleDeg;
            double mag = Math.abs(err);
            double vFb = mag < TurretConstants.TURRET_DEADBAND_DEG ? 0.0
                    : Math.min(TurretConstants.TURRET_MAX_VEL_DEG_S,
                    Math.min(Math.sqrt(2.0 * TurretConstants.TURRET_DECEL_DEG_S2 * mag),
                            TurretConstants.TURRET_POS_KP * mag));
            double vCmd = ffDegS + Math.copySign(vFb, err);
            power = TurretConstants.TURRET_KV * vCmd
                    + TurretConstants.TURRET_KVEL * (vCmd - velocityDegS);
            if (Math.abs(vCmd) > 1e-6) power += Math.copySign(TurretConstants.TURRET_KS, vCmd);
        }
        double max = TurretConstants.TURRET_MAX_POWER;
        power = Math.max(-max, Math.min(max, power));
        // Never drive further out once past a limit (an overshoot, or a turret moved by hand).
        double over = TurretConstants.TURRET_LIMIT_OVERSHOOT_DEG;
        if (angleDeg >= maxAngleDeg() + (manual ? 0.0 : over) && power > 0) power = 0.0;
        if (angleDeg <= minAngleDeg() - (manual ? 0.0 : over) && power < 0) power = 0.0;

        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > 0.004 || (power == 0.0 && lastPower != 0.0)) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public void stop() {
        manual = true;
        manualPower = 0.0;
        if (motor != null) motor.setPower(0.0);
        lastPower = 0.0;
    }

    // =============================================================================================
    // Measuring
    // =============================================================================================

    /** Turret angle from the encoder (as of the last update()), DEGREES. */
    public double getAngleDeg() {
        return angleDeg;
    }

    public double getMeasuredAngleDeg() {
        return motor == null ? Double.NaN : angleDeg;
    }

    public double getVelocityDegS() {
        return velocityDegS;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public double getCommandedAngleDeg() {
        return targetDeg;
    }

    public boolean hasEncoder() {
        return motor != null;
    }

    /** The last target was outside the travel limits and was clamped (the chassis has to turn). */
    public boolean wasLastCommandClamped() {
        return lastCommandClamped;
    }

    /** Within the on-target tolerance of the target and not swinging past it. */
    public boolean onTarget() {
        return motor != null && !manual
                && Math.abs(targetDeg - angleDeg) <= TurretConstants.AIM_READY_TOLERANCE_DEG
                && Math.abs(velocityDegS - ffDegS) <= TurretConstants.TURRET_ON_TARGET_MAX_RATE_DEG_S;
    }

    /** Within TURRET_HARD_STOP_MARGIN_DEG of a travel limit. */
    public boolean isAtHardStop() {
        return angleDeg >= maxAngleDeg() - TurretConstants.TURRET_HARD_STOP_MARGIN_DEG
                || angleDeg <= minAngleDeg() + TurretConstants.TURRET_HARD_STOP_MARGIN_DEG;
    }

    public boolean isHardwareOk() {
        return motor != null;
    }

    public String getHardwareFault() {
        return hardwareFault;
    }

    /** TeleOp kept the zero an auto set earlier this session instead of resetting it. */
    public boolean keptAutoZero() {
        return keptZero;
    }

    /** Reset the encoder here: the turret must be facing its zero (forward) right now. */
    public void rezero() {
        if (motor == null) return;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        zeroedThisSession = true;
        keptZero = false;
        angleDeg = 0.0;
        lastPower = Double.NaN;
    }

    public void telemetry() {
        if (telemetry == null) return;
        if (motor == null) {
            telemetry.addData("Turret", "MISSING - " + hardwareFault);
            return;
        }
        telemetry.addData("Turret", "%.1f deg (target %.1f%s)  %.0f deg/s  power %.2f%s",
                angleDeg, targetDeg, lastCommandClamped ? ", AT LIMIT" : "", velocityDegS,
                Double.isNaN(lastPower) ? 0.0 : lastPower, onTarget() ? "  ON TARGET" : "");
    }

    // =============================================================================================
    // Aim geometry - shared by the teleop and the autos
    // =============================================================================================

    /** Field position of the turret pivot for a robot pose, INCHES. */
    public static double[] pivotField(Pose2d pose) {
        double h = pose.heading.toDouble(), c = Math.cos(h), s = Math.sin(h);
        double px = TurretConstants.TURRET_PIVOT_X_IN, py = TurretConstants.TURRET_PIVOT_Y_IN;
        return new double[]{pose.position.x + c * px - s * py, pose.position.y + s * px + c * py};
    }

    /**
     * Turret angle that puts the shot line through the field point (tx, ty), DEGREES, normalized to
     * (-180, 180] (not yet clamped to the travel limits). Accounts for the pivot being off robot
     * centre and for a shot line that does not pass through the pivot (TURRET_SHOT_LATERAL_IN).
     */
    public static double aimAngleDeg(Pose2d pose, double tx, double ty) {
        double[] p = pivotField(pose);
        double dx = tx - p[0], dy = ty - p[1];
        double r = Math.max(1e-6, Math.hypot(dx, dy));
        double lat = TurretConstants.TURRET_SHOT_LATERAL_IN;
        double correction = Math.abs(lat) < r ? Math.asin(lat / r) : 0.0;
        double shotRobot = Math.atan2(dy, dx) - pose.heading.toDouble() - correction;
        return normalizeDeg(Math.toDegrees(shotRobot) - TurretConstants.TURRET_MOUNT_OFFSET_DEG);
    }

    /**
     * How fast {@link #aimAngleDeg} is changing, DEGREES/SECOND, for a robot moving at field
     * velocity (vx, vy) in/s and turning at omega rad/s CCW: the bearing rate minus the turn.
     */
    public static double aimRateDegS(Pose2d pose, double tx, double ty, double vx, double vy, double omega) {
        double[] p = pivotField(pose);
        double dx = tx - p[0], dy = ty - p[1];
        double r2 = Math.max(1.0, dx * dx + dy * dy);
        return Math.toDegrees((dy * vx - dx * vy) / r2 - omega);
    }

    /** Distance the ball flies from the shooter exit to (tx, ty) once aimed, INCHES. */
    public static double exitDistanceIn(Pose2d pose, double tx, double ty) {
        double[] p = pivotField(pose);
        double r = Math.hypot(tx - p[0], ty - p[1]);
        double lat = TurretConstants.TURRET_SHOT_LATERAL_IN;
        return Math.sqrt(Math.max(0.0, r * r - lat * lat)) - TurretConstants.TURRET_EXIT_FORWARD_IN;
    }

    /**
     * The equivalent of {@code desiredDeg} (+-360) nearest {@code currentDeg}, so a target moving
     * across the back of the robot (+180 / -180) does not swing the turret the long way round.
     */
    public static double unwrapNear(double desiredDeg, double currentDeg) {
        double best = desiredDeg;
        for (double k = -360.0; k <= 360.0; k += 360.0) {
            if (Math.abs(desiredDeg + k - currentDeg) < Math.abs(best - currentDeg)) best = desiredDeg + k;
        }
        return best;
    }

    /**
     * The turret target to use for an aim angle: {@code desiredDeg} itself, or its +-360 twin when
     * the target has only just crossed behind the robot (within TURRET_UNWRAP_HYST_DEG past the
     * limit), so the turret waits at the limit instead of swinging the long way round on every
     * small move across the back. Clamp the result with setTarget.
     */
    public static double chooseTargetDeg(double desiredDeg, double currentDeg) {
        double u = unwrapNear(desiredDeg, currentDeg);
        if (u != desiredDeg && Math.abs(u) <= maxAngleDeg() + TurretConstants.TURRET_UNWRAP_HYST_DEG) return u;
        return desiredDeg;
    }

    /** Degrees the chassis has to turn (CCW +) so {@code desiredDeg} fits inside the travel minus a margin. */
    public static double chassisExcessDeg(double desiredDeg) {
        double m = TurretConstants.TURRET_CHASSIS_ASSIST_MARGIN_DEG;
        double hi = maxAngleDeg() - m, lo = minAngleDeg() + m;
        if (desiredDeg > hi) return desiredDeg - hi;
        if (desiredDeg < lo) return desiredDeg - lo;
        return 0.0;
    }
}
