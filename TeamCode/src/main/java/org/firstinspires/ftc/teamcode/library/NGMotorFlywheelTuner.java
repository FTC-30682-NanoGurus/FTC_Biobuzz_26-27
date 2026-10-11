package org.firstinspires.ftc.teamcode.library;


public final class NGMotorFlywheelTuner {

    private NGMotorFlywheelTuner() { }

    public static void setRecoveryKp(NGMotor motor, double value) {
        if (motor == null) return;
        motor.kP_Recovery = value;
    }

    public static double getRecoveryKp(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.kP_Recovery;
    }

    public static void setStableKp(NGMotor motor, double value) {
        if (motor == null) return;
        motor.kP_Stable = value;
    }

    public static double getStableKp(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.kP_Stable;
    }

    public static void setAlpha(NGMotor motor, double value) {
        if (motor == null) return;
        motor.ALPHA = value;
    }

    public static double getAlpha(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.ALPHA;
    }

    public static void setRecoveryThreshold(NGMotor motor, double value) {
        if (motor == null) return;
        motor.RECOVERY_THRESHOLD = value;
    }

    public static double getRecoveryThreshold(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.RECOVERY_THRESHOLD;
    }


    public static double getSmoothedVelocity(NGMotor motor) {
        return (motor == null) ? Double.NaN : motor.lastSmoothVelocity;
    }

    public static void resetVelocityFilter(NGMotor motor) {
        if (motor == null) return;
        motor.lastSmoothVelocity = 0.0;
    }
}
