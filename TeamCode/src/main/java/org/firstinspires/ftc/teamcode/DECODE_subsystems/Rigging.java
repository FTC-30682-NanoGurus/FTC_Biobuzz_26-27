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
public class Rigging extends Subsystem {
    public NGMotor rigging_motor;
    public NGServo left;
    public NGServo right;
    public static double left_pos_open = 0.3;
    public static double left_pos_closed = 0;
    public static double right_pos_open = 0.7;
    public static double right_pos_closed = 1;
    private Telemetry telemetry;


    private boolean opened = false;
    public static PIDCoefficients PID = new PIDCoefficients(0.005,0,0);
    private ElapsedTime timer;

    private double delay = 3.14, delay2 = 4;
    private boolean ascended = false;

    private boolean up_pressed = false;
    private boolean down_pressed = false;
    private double current_time = 0, current_time2 = 0;
    public Rigging(HardwareMap hardwareMap, Telemetry telemetry){
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor);
        rigging_motor.setDirection(DcMotor.Direction.REVERSE);
        left = new NGServo(hardwareMap, telemetry, RobotConstants.left_rigging_servo);
        right = new NGServo(hardwareMap, telemetry, RobotConstants.right_rigging_servo);
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

    public Rigging(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        this(hardwareMap, telemetry);
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor, timer);
        rigging_motor.setDirection(DcMotor.Direction.REVERSE);

        this.timer = timer;

    }
    public void toggleServos(){
        if(opened){
            closeServos();
        }else{
           openServos();
        }
    }
    public void disableServos(){
        left.disableServo();
        right.disableServo();
    }
    public void closeServos(){
        enableServos();
        opened = false;
        left.setPosition(left_pos_closed);
        right.setPosition(right_pos_closed);
    }
    public void enableServos(){
        left.enableServo();
        right.enableServo();
        current_time = timer.seconds();
    }
    public void openServos(){
        enableServos();
        opened = true;
        left.setPosition(left_pos_open);
        right.setPosition(right_pos_open);
    }
    public void setPower(double power){
        rigging_motor.setManualPower(power);
    }
    public void setUpPressed(boolean on){
        up_pressed = on;
    }
    public void setDownPressed(boolean on){
        down_pressed = on;
    }
    public void ascend(){
        setPower(1);
        ascended = true;
        current_time2 = timer.seconds();
    }
    public void abortAscent(){
        ascended = false;
        rigging_motor.setManualPower(0);
    }

    @Override
    public void update() {
        if(timer.seconds() - current_time > delay){
            disableServos();
        }

        if(!ascended){
            if(up_pressed){
                rigging_motor.setManualPower(-1);
            }else if(down_pressed){
                rigging_motor.setManualPower(1);
            }else{
                rigging_motor.setManualPower(0);
            }
        }else{
            if(up_pressed || down_pressed){
                ascended = false;
            }
            if(timer.seconds() - current_time2 < delay2){
                rigging_motor.setManualPower(1);
            }else{
                rigging_motor.setManualPower(0);
                ascended = false;
            }
        }

        rigging_motor.update();
    }

    @Override
    public void telemetry() {
        rigging_motor.telemetry();
        telemetry.addData("Left Position", left.getPosition());
        telemetry.addData("Right Position", right.getPosition());
    }
    public void init_without_moving(){
        rigging_motor.init();
        rigging_motor.setUseMotionProfile(false);
        rigging_motor.setMax(20000);
        rigging_motor.setMin(-20000);
    }
    @Override
    public void init() {
        init_without_moving();
       closeServos();

    }
}
