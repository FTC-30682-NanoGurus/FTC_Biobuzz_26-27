package org.firstinspires.ftc.teamcode.DECODE_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.library.Subsystem;


public class TrafficLight extends Subsystem {
    private enum COLOR {RED, GREEN, OFF_RED, OFF}
    private COLOR current_color = COLOR.OFF;
    private String name = "";
    private ElapsedTime timer;
    private Telemetry telemetry;

    private double red_timer = 0;
    private double green_timer = 0;
    private double offred_timer = 0;

    private int red_count = 0;
    private int green_count = 0;
    private int offred_count = 0;

    private double red_delay = 0.5;
    private double green_delay = 0.5;
    private double offred_delay = 0.5;

    private boolean red_on = false;
    private boolean green_on = false;
    private boolean offred_on = false;

    LED red;
    LED green;
    public TrafficLight(String name, HardwareMap hardwareMap, Telemetry telemetry, String red_name, String green_name){
        this.name = name;
        this.telemetry = telemetry;
        red = hardwareMap.get(LED.class, red_name);
        green = hardwareMap.get(LED.class, green_name);
        timer = new ElapsedTime();
    }
    public TrafficLight(String name, HardwareMap hardwareMap, Telemetry telemetry, String red_name, String green_name, ElapsedTime timer){
        this.name = name;
        this.telemetry = telemetry;
        red = hardwareMap.get(LED.class, red_name);
        green = hardwareMap.get(LED.class, green_name);
        this.timer = timer;
    }
    public void flashRed(){
        flashRed(0.1, 3);
    }
    public void flashRed(double delay){
        flashRed(delay, 3);
    }
    public void flashRed(double delay, int count){
        red_on = true;
        red_timer = timer.time();
        red_delay = delay;
        red_count = count;
    }
    public void flashRed(int count){
        flashRed(0.1, count);
    }
    public void flashGreen() {
        flashGreen(0.1, 3);
    }

    public void flashGreen(double delay) {
        flashGreen(delay, 3);
    }
    public void red(boolean on){
        red_on = on;
    }
    public void green(boolean on){
        green_on = on;

    }
    public void orange(boolean on){
        offred_on = on;


    }
    public void flashGreen(double delay, int count) {
        green_on = true;
        green_timer = timer.time();
        green_delay = delay;
        green_count = count;
    }

    public void flashGreen(int count) {
        flashGreen(0.1, count);
    }
    public void flashOffred() {
        flashOffred(0.1, 3);
    }

    public void flashOffred(double delay) {
        flashOffred(delay, 3);
    }

    public void flashOffred(double delay, int count) {
        offred_on = true;
        offred_timer = timer.time();
        offred_delay = delay;
        offred_count = count;
    }

    public void flashOffred(int count) {
        flashOffred(0.1, count);
    }


    @Override
    public void update() {
        if(red_count > 0 && timer.time() - red_timer > red_delay){
            red_timer = timer.time();
            if(red_on){
                red_count -= 1;
            }
            red_on = !red_on;
        }
        if(green_count > 0 && timer.time() - green_timer > green_delay){
            green_timer = timer.time();
            if(green_on){
                green_count -= 1;
            }
            green_on = !green_on;
        }
        if(offred_count > 0 && timer.time() - offred_timer > offred_delay){
            offred_timer = timer.time();
            if(offred_on){
                offred_count -= 1;
            }
            offred_on = !offred_on;
        }


        if(red_on || offred_on){
            red.on();
        }else{
            red.off();
        }
        if(green_on || offred_on){
            green.on();
        }else{
            green.off();
        }
    }
    public class disableLight implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            red(false);
            green(false);
            return false;
        }
    }
    public Action disable(){
        return new disableLight();
    }

    public Action warnHuman(){
        return new trafficLightAction();
    }
    public class trafficLightAction implements Action {

        ElapsedTime timer;
        private double current_time = 0;
        private int stage = 0;

        private double delay = 0;
        private boolean wait = false;
        public trafficLightAction(){

            timer = new ElapsedTime();
            timer.reset();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(stage == 0 && !wait){
                wait = true;
                delay = 0.25;
                green(true);
                current_time = timer.time();
            }else if(stage == 1 && !wait){
                wait = true;
                red(true);
                delay = 0.125;
                current_time = timer.time();
            }else if(stage == 2 && !wait){
                wait = true;
                green(false);
                return false;
            }
            if(wait && timer.time() - current_time > delay){
                stage += 1;
                wait = false;
            }
            return true;
        }
    }
    public class updateAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }
    public Action updateAction(){
        return new updateAction();
    }
    @Override
    public void telemetry() {
        if(red_on && green_on){
            telemetry.addData(name + "Color", "Orange");
        }else if(red_on){
            telemetry.addData(name + "Color", "Red");
        }else if(green_on){
            telemetry.addData(name + "Color", "Green");
        }
    }

    @Override
    public void init() {
        red_on = false;
        green_on = false;
        update();

    }
}
