package org.firstinspires.ftc.teamcode.DECODE_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class VihasCameraArm extends Subsystem {
    public NGMotor rigging_motor;
    public NGServo rigging_servo;
    private Telemetry telemetry;

    public static PIDCoefficients PID = new PIDCoefficients(0.005,0,0);
    public static int unlatch_height = 6000;
    public static int full_extend = 12400;

    private ElapsedTime timer;
    public static int off_ground = 5500;

    private boolean level_one = false;
    public VihasCameraArm(HardwareMap hardwareMap, Telemetry telemetry){
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor);
        rigging_servo = new NGServo(hardwareMap, telemetry,RobotConstants.rigging_servo);
        rigging_servo.setInit_pos(0);
        rigging_motor.setDirection(DcMotor.Direction.REVERSE);
        timer = new ElapsedTime();
        this.telemetry = telemetry;
    }
    public void reset(){

        rigging_motor.resetEncoder();
    }
    public class updateAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }
    public Action updateAction(){
        return new updateAction();
    }

    public VihasCameraArm(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        this(hardwareMap, telemetry);
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor, timer);        rigging_servo = new NGServo(hardwareMap, telemetry,RobotConstants.rigging_servo);
        rigging_servo.mountTimer(timer);

        rigging_motor.setDirection(DcMotor.Direction.REVERSE);
        this.timer = timer;

    }

    public void setManualPower(double power){
        rigging_motor.setManualPower(power);
    }
    public void setServoPosition(double position){
        rigging_servo.setPosition(position);
    }

    public void level1(){
        rigging_servo.setPosition(0.6);
        level_one = true;
    }
    public void retract_wand(){
        rigging_servo.setPosition(0);
        level_one = false;
    }
    public void camera(){
        rigging_servo.setPosition(0.78);
    }
    public boolean isLevelOne(){
        return level_one;
    }
    @Override
    public void update() {
        rigging_motor.setPID(PID.p, PID.i, PID.d);
        rigging_motor.update();
    }

    @Override
    public void telemetry() {
        rigging_motor.telemetry();
    }
    public void init_without_moving(){
        rigging_motor.init();

        rigging_motor.setUseMotionProfile(false);
        rigging_motor.setMax(20000);
        rigging_motor.setMin(-20000);
    }
    @Override
    public void init() {
        rigging_servo.init();
        init_without_moving();


    }
}
