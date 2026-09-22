package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The intake rollers: on, off, and backwards. Nothing more.
 *
 * There is deliberately no game logic in here - no "stop when full", no automatic reversing on a
 * jam, no interaction with the launcher. Those decisions depend on what the driver is doing and on
 * what the rest of the robot is up to, so they belong in the opmode where they can be read in one
 * place. This class only owns the motor.
 */
public class Intake {

    private final Telemetry telemetry;
    private DcMotorEx motor;

    /** What the intake was last told to do. Telemetry only. */
    private String state = "STOPPED";
    private String hardwareFault = "";

    /**
     * Grabs the intake motor.
     *
     * A missing motor is reported and left null rather than thrown, so a config typo costs the
     * intake rather than the whole opmode.
     */
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            motor = hardwareMap.get(DcMotorEx.class, TurretConstants.INTAKE_MOTOR_NAME);
            motor.setDirection(TurretConstants.INTAKE_MOTOR_REVERSED
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            // FLOAT, not BRAKE: a braked roller that a ball is resting against holds it there and
            // makes the next pickup worse.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            motor = null;
            hardwareFault = "intake motor not found: " + e.getMessage();
            if (telemetry != null) telemetry.addLine("INTAKE: " + hardwareFault);
        }
    }

    /** Runs the rollers inward at {@link TurretConstants#INTAKE_POWER}. */
    public void start() {
        setPower(TurretConstants.INTAKE_POWER);
        state = "INTAKING";
    }

    /** Stops the rollers. */
    public void stop() {
        setPower(0.0);
        state = "STOPPED";
    }

    /** Runs the rollers outward at {@link TurretConstants#INTAKE_REVERSE_POWER}, to clear a jam. */
    public void reverse() {
        setPower(TurretConstants.INTAKE_REVERSE_POWER);
        state = "REVERSING";
    }

    /** Sets raw roller power, unitless -1..1. Safe when the motor is missing. */
    public void setPower(double power) {
        if (motor == null) return;
        motor.setPower(power);
    }

    /** Current roller power, unitless -1..1, or 0 when the motor is missing. */
    public double getPower() {
        return (motor == null) ? 0.0 : motor.getPower();
    }

    /** "INTAKING", "STOPPED" or "REVERSING". Telemetry only. */
    public String getState() {
        return state;
    }

    /** True when the motor was found. */
    public boolean isHardwareOk() {
        return motor != null;
    }

    /** Adds intake state to telemetry. */
    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Intake", "%s (%.2f)", state, getPower());
        if (!hardwareFault.isEmpty()) telemetry.addData("Intake fault", hardwareFault);
    }
}
