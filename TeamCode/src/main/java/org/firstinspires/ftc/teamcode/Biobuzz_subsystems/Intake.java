package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private final Telemetry telemetry;
    private DcMotorEx motor;

    private String state = "STOPPED";
    private String hardwareFault = "";

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            motor = hardwareMap.get(DcMotorEx.class, TurretConstants.INTAKE_MOTOR_NAME);
            motor.setDirection(TurretConstants.INTAKE_MOTOR_REVERSED
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            motor = null;
            hardwareFault = "intake motor not found: " + e.getMessage();
            if (telemetry != null) telemetry.addLine("INTAKE: " + hardwareFault);
        }
    }

    public void start() {
        setPower(TurretConstants.INTAKE_POWER);
        state = "INTAKING";
    }

    public void stop() {
        setPower(0.0);
        state = "STOPPED";
    }

    public void reverse() {
        setPower(TurretConstants.INTAKE_REVERSE_POWER);
        state = "REVERSING";
    }

    public void setPower(double power) {
        if (motor == null) return;
        motor.setPower(power);
    }

    public double getPower() {
        return (motor == null) ? 0.0 : motor.getPower();
    }

    public String getState() {
        return state;
    }

    public boolean isHardwareOk() {
        return motor != null;
    }

    public void telemetry() {
        if (telemetry == null) return;
        telemetry.addData("Intake", "%s (%.2f)", state, getPower());
        if (!hardwareFault.isEmpty()) telemetry.addData("Intake fault", hardwareFault);
    }
}
