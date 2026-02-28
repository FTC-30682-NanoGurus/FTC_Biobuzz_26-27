package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class NGServo extends Subsystem{
    ServoImplEx servo;
    Telemetry telemetry;
    private double targetPosition = 0;
    private double lastTime = 0;
    private double speed = 0;
    private ElapsedTime timer;
    private String name;
    private double init_pos = 0;

    private double last_pos = -10;
    private boolean PWMEnabled = true;
    private double min = 0;
    private double max = 1;


    public NGServo(HardwareMap hardwareMap, Telemetry telemetry, String name){
        servo = hardwareMap.get(ServoImplEx.class, name);
        this.telemetry = telemetry;
        this.name = name;
//        servoController = (ServoControllerEx) servo;
        timer = new ElapsedTime();
    }
    public boolean isPWMEnabled(){
        return PWMEnabled;
    }
    public void setInit_pos(double position){
        init_pos = position;
    }
    public void mountTimer(ElapsedTime timer){
        this.timer = timer;
    }
    public void setMax(double max){
        this.max = max;
    }
    public void setMin(double min){
        this.min = min;
    }
    public void disableServo(){
        servo.setPwmDisable();
        PWMEnabled = false;
    }
    public void enableServo(){
        servo.setPwmEnable();
        PWMEnabled = true;
    }
    public void setPosition(double position) {
        if (position > max){
            position = max;
            }
            if(position < min)

            {
                position = min;
            }
        if(PWMEnabled && Math.abs(position - last_pos) > 0.005) {
            last_pos = position;
            servo.setPosition(position);
            speed = 0;
            targetPosition = position;
        }
    }
    public void setPosition(double position, double speed){
        if (position > max){
            position = max;
        }
        if(position < min)

        {
            position = min;
        }
        targetPosition = position;
        this.speed = speed;
    }
    public double getPosition(){
        return servo.getPosition();
    }
    @Override
    public void update() {
        if(!PWMEnabled){
            return;
        }
        if(speed == 0){
            return;
        }
        double currentTime = timer.time(TimeUnit.SECONDS);
        double elapsedTime = (currentTime - lastTime) ; // Convert milliseconds to seconds
        lastTime = currentTime;
        double currentPosition = servo.getPosition();
        // If the servo is already at the target position, do nothing
        if (servo.getPosition() == targetPosition) return;

        // Calculate the direction to move (positive if target is higher, negative if lower)
        double direction = Math.signum(targetPosition - servo.getPosition());

        // Calculate how much to move based on speed and time
        double positionChange = speed * elapsedTime * direction;

        // Update the current position (don't overshoot the target)
        if (Math.abs(targetPosition - currentPosition) < Math.abs(positionChange)) {
            currentPosition = targetPosition; // Reached target
        } else {
            currentPosition += positionChange;
        }

        // Update the servo's position
        servo.setPosition(currentPosition);
    }

    @Override
    public void telemetry() {
        telemetry.addData(name + " position: ", servo.getPosition());
        telemetry.addData(name + " previous position: ", last_pos);
        telemetry.addData(name + " enabled: ", PWMEnabled);
        if(speed != 0){
            telemetry.addData(name + " speed: ", speed);
        }
    }

    @Override
    public void init() {
        servo.setPosition(init_pos);
    }
}
