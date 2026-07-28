package org.firstinspires.ftc.teamcode.DECODE_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGServo;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class DiffyClaw extends Subsystem {
    public double wrist_angle = 0;
    private double claw_angle = 0;
    private double claw_position = 0.95;

    public static double SERVO_1_REAL_RANGE = 265;
    public static double SERVO_2_REAL_RANGE = 265;

    public static double OFFSET = 0;
    public NGServo left;
    public NGServo right;
    public NGServo claw;

    Telemetry telemetry;

    TrafficLight trafficLight;

    public DiffyClaw(HardwareMap hardwareMap, Telemetry telemetry){
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        claw = new NGServo(hardwareMap, telemetry, RobotConstants.claw_servo);
        left = new NGServo(hardwareMap, telemetry, RobotConstants.left_servo);
        right = new NGServo(hardwareMap, telemetry, RobotConstants.right_servo);
        this.telemetry = telemetry;
    }
    public void mountTrafficLight(TrafficLight trafficLight){
        this.trafficLight = trafficLight;
    }
    public double getClawAngle(){
        return claw_angle;
    }
    public double constrainClawAngle(double wrist_angle, double claw_angle) {

        double lowerBound = Math.abs(wrist_angle - 135) - 135;
        double upperBound = -1 * Math.abs(wrist_angle - 135) + 135;
        if(claw_angle < lowerBound){
            return lowerBound;
        }
        else if(claw_angle > upperBound){
            return upperBound;
        }
        return claw_angle;
    }
    public double constrainWristAngle(double wrist_angle) {

       return Math.min(Math.max(0, wrist_angle), 180);
    }

    public double constrainClaw(double claw_position){
        if(claw_position < RobotConstants.claw_flat){
            return RobotConstants.claw_flat;
        }
        return claw_position;
    }
    public void setWristAngle(double wrist_angle){
        this.wrist_angle = wrist_angle;
    }
    public void setClawAngle(double claw_angle){
        this.claw_angle = claw_angle;
    }
    public void moveClaw(double claw_position){
        this.claw_position = claw_position;
        claw.setPosition(constrainClaw(claw_position));
    }
    @Override
    public void update() {
        wrist_angle = constrainWristAngle(wrist_angle);
        claw_angle = constrainClawAngle(wrist_angle, claw_angle);
        claw_position = constrainClaw(claw_position);
        left.setPosition((wrist_angle/SERVO_1_REAL_RANGE) - (claw_angle/SERVO_1_REAL_RANGE) + OFFSET);
        right.setPosition(1 - (wrist_angle/SERVO_2_REAL_RANGE) - (claw_angle/SERVO_2_REAL_RANGE) - OFFSET);
    }


    @Override
    public void telemetry() {
        telemetry.addData("Diffy Servo Wrist Angle", wrist_angle);
        telemetry.addData("Diffy Servo Left Position", left.getPosition());
        telemetry.addData("Diffy Servo Claw Angle", claw_angle);
        telemetry.addData("Diffy Servo Right Position", right.getPosition());
        telemetry.addData("Diffy Servo Claw Position", claw_position);
    }

    @Override
    public void init() {
        wrist_angle = 0;
        claw_angle = 0;
    }
}
