package org.firstinspires.ftc.teamcode.DECODE_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Distance extends Subsystem {
    private Rev2mDistanceSensor sensorDistance;
    private double past_distance_reading = 0;
    private double current_dist = 0;
    private double filtered_position = 0;
    public static double filter_value = 0.65;
    private double delay = 0;

    private boolean on = false;
    private ElapsedTime timer;
    private String name = "";

    private int calls_per_loop = 0;
    private boolean filter_on = false;
    Telemetry telemetry;
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, String name){
        this.telemetry = telemetry;
        timer = new ElapsedTime();

        sensorDistance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, name);
        this.name = name;

    }
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, String name, ElapsedTime timer){
        this(hardwareMap, telemetry, name);
        this.timer = timer;


    }
    public double getDist() {
        calls_per_loop += 1;
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
    public int getCalls(){
        return calls_per_loop;
    }
    public void clearCalls(){
        calls_per_loop = 0;
    }
    public void setFilter(double filter){
        filter_value = filter;
    }
    public void setOn(boolean on){
        this.on = on;
    }
    public boolean isOn(){
        return on;
    }
    public boolean timeout(){
        return sensorDistance.didTimeoutOccur();
    }
    @Override
    public void telemetry() {
        telemetry.addData(name + " is on: ", on);
        if(!on){
            return;
        }
        telemetry.addData(name + " Distance (in)", getDist());
        telemetry.addData(name + " did time out", Boolean.toString(sensorDistance.didTimeoutOccur()));
        telemetry.addData(name + " Filtered Distance (in)", getFilteredDist());
    }
    public void update(){
        if(!on){
            filter_on = false;
            return;
        }
        if(timer.milliseconds() - delay > 50){
            filter_on = true;
            past_distance_reading = current_dist;
            current_dist = getDist();
            if(Math.abs(past_distance_reading - current_dist) > 1){
                filtered_position = current_dist;
            }else {
                filtered_position = filter_value * current_dist + (1 - filter_value) * past_distance_reading;
            }
            delay = timer.milliseconds();
        }
    }
    public double getFilteredDist(){

        return filtered_position;
    }
    @Override
    public void init() {
        return;
    }
    public class updateAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put(name + " Filtered Distance", getFilteredDist());
            update();
            return true;
        }
    }
    public class waitAction implements Action {
        double target_distance = 0;
        boolean first= true;
        boolean direction = true;
        public waitAction(double target_distance){
            this.target_distance = target_distance;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                setOn(true);
                direction = getDist() < target_distance;
                first = false;
            }
            if(!filter_on){
                return true;
            }
            telemetryPacket.put(name + " Filtered Distance In Wait Action", getFilteredDist());
            boolean exit = direction ? getFilteredDist() < target_distance : getFilteredDist() > target_distance;
            setOn(exit);
            return exit;
        }
    }
    public Action waitAction(double target){
        return new waitAction(target);
    }
    public Action updateAction(){
        return new updateAction();
    }
}