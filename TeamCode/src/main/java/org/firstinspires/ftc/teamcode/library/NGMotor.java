package org.firstinspires.ftc.teamcode.library;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.util.Range;

public class NGMotor extends Subsystem {
    DcMotorEx pid_motor;

    private boolean on = true;
    public int targetPos = 0;
    private int startPos = 0;
    public double P = 0.0005, I = 0.0002, D = 0;
    public  double F = 0;
    double error, lastError;
    private boolean reversed_encoder = false;
    public boolean manual = false;
    private boolean reached = false;
    private boolean useMotionProfile = false;

    private boolean motionProfileOverride = false;

    public boolean alternate = false;

    private AlternateEncoder alternateEncoder;
    private int distance = 0;
    private ElapsedTime timer;
    private double time_passed = 0, time_stop = 0, time_started = 0;
    public int maxHardstop = 10000;
    private String name = "";
    public int minHardstop = 0;
    private double holding_power = 0;
    private double max_integral_component = 0.3;
    private double out = 0;
    private double completed_time = 0;
    private int reached_range = 10;
    private double integralSum = 0;

    private double MAX_VEL = 1;
    private double MAX_ACCEL = 1;

    private double MAX_DECEL = -1;

    private double manual_power_time = 0;
    private boolean waiting_for_manual = false;

    private boolean exit_with_time = false;

    private boolean low_external_hardstop = false;
    private boolean high_external_hardstop = false;

    public boolean power_damp = false;
    private double power_damp_coefficient = 0.7;
    private double previous_power = 0;
    private double compensation_power = 0;

    private double motion_profile_exit_time = 0;

    private double caching_tolerance = 0.005;
    private double cached_power = Double.NaN;
    public double targetVelocity = 0.0; //flywheel target velocity
    private double kP, kI, kD, kF;      //PIDF for flywheels

    private double flywheelIntegralSum = 0;
    private double flywheelLastError = 0;
    HardwareMap hardwareMap;
    private ElapsedTime pidTimer = new ElapsedTime();
    Telemetry telemetry;

    //Flywheel PIDF Controller Constants
    double kP_Recovery = 0.082;//0.06
    double kP_Stable  = 0.008;//0.008
    public static double kLoad = 0.8;

    double ALPHA = 0.7;
    double RECOVERY_THRESHOLD = 60.0;
    double lastSmoothVelocity = 0.0;

    ElapsedTime feederTimer = new ElapsedTime();
    boolean wasFeederActive = false; // Tracks state changes
    public static double kRamp = 1.0;

    // --- Voltage sensor caching -------------------------------------------------------------
    // getVoltage() is a LynxGetADC command: a real hub round trip that the bulk read does NOT
    // cover. Battery voltage does not move meaningfully inside one control loop, so we resolve
    // the sensor once and re-read it on a timer instead of every single updateFlywheels() call.
    private VoltageSensor voltageSensor = null;
    private double cachedVoltage = 12.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    /** How often to actually hit the hub for battery voltage. Set to 0 to read every call. */
    public static double VOLTAGE_REFRESH_MS = 250.0;

    /** Set false to skip this motor's per-loop telemetry writes (string building + boxing). */
    private boolean telemetryEnabled = true;

    public void setTelemetryEnabled(boolean on) {
        this.telemetryEnabled = on;
    }

    private double getBatteryVoltage() {
        if (voltageSensor == null) {
            if (hardwareMap == null) return cachedVoltage; // ctor without a hardwareMap
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
            return cachedVoltage;
        }
        if (voltageTimer.milliseconds() >= VOLTAGE_REFRESH_MS) {
            double v = voltageSensor.getVoltage();
            if (v > 0) cachedVoltage = v; // never divide by a bogus 0 reading
            voltageTimer.reset();
        }
        return cachedVoltage;
    }

    public NGMotor(HardwareMap hardwareMap, Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        this.name = name;
        this.hardwareMap = hardwareMap;
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);

        timer = new ElapsedTime();
    }

    public void setCustomVelocityPID(double targetVel, double P, double I, double D, double F) {
        this.targetVelocity = targetVel;
        this.kP = P;
        this.kI = I;
        this.kD = D;
        this.kF = F;

        this.flywheelIntegralSum = 0;
        this.flywheelLastError = 0;
        this.pidTimer.reset();
    }

    public void updateFlywheels(boolean isFeederActive) {

        double rawVelocity = pid_motor.getVelocity();
        double currentVoltage = getBatteryVoltage();
        double loopTime = pidTimer.seconds();
        pidTimer.reset();

        if (loopTime <= 0) loopTime = 0.001;

        double smoothVelocity = (ALPHA * rawVelocity) + ((1 - ALPHA) * lastSmoothVelocity);
        lastSmoothVelocity = smoothVelocity;

        double error = targetVelocity - smoothVelocity;

        double P_Term;
        if (error > RECOVERY_THRESHOLD) {
            P_Term = error * kP_Recovery;
        } else {
            P_Term = error * kP_Stable;
        }

        if (Math.abs(error) < RECOVERY_THRESHOLD) {
            flywheelIntegralSum += error * loopTime;
        } else {
            flywheelIntegralSum = 0;
        }

        double maxISum = (kI != 0) ? 0.1 / kI : 0;
        if (flywheelIntegralSum > maxISum) flywheelIntegralSum = maxISum;
        if (flywheelIntegralSum < -maxISum) flywheelIntegralSum = -maxISum;

        double I_Term = kI * flywheelIntegralSum;

        double derivative = (error - flywheelLastError) / loopTime;
        flywheelLastError = error;
        double D_Term = kD * derivative;

        double F_Term = kF * targetVelocity;

        double Load_Term = 0;
        if (isFeederActive) {
            // Timer reset
            if (!wasFeederActive) {
                feederTimer.reset();
                wasFeederActive = true;
            }

            // Base boost + (Extra boost * seconds held)
            Load_Term = kLoad + (feederTimer.seconds() * kRamp);

        } else {
            // Reset state when not shooting
            wasFeederActive = false;
            Load_Term = 0;
        }

        double calculatedPower = P_Term + I_Term + D_Term + F_Term + Load_Term;

        double compensatedPower = calculatedPower * (12.0 / currentVoltage);

        double finalPower = Range.clip(compensatedPower, -1.0, 1.0);
        pid_motor.setPower(finalPower);

        // Tuning Telemetry
        if (telemetryEnabled) {
            telemetry.addData("Current Velocity", rawVelocity);
            telemetry.addData("Error", error);
            telemetry.addData("FeedForward Power: ", Load_Term * 500);
            //telemetry.addData("Load Boost", isFeederActive ? "ACTIVE" : "OFF");
        }
    }

    public void loopTelemetry(double loopMs){
        telemetry.addData("Hz", "%.0f", 1000 / loopMs);
        telemetry.addData("Loop Time", "%.1f ms", loopMs);
    }

    public void addPower(double power){
        compensation_power = power;
    }

    public void setPowerDamp(boolean on){
        power_damp = on;
    }
    public void setExitWithTime(boolean on){
        exit_with_time = on;
    }
    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPower){
        pid_motor.setZeroPowerBehavior(zeroPower);
    }
    public void enableMotor(boolean on){
        this.on = on;
    }
    public NGMotor(HardwareMap hardwareMap, Telemetry telemetry, String name, ElapsedTime timer) {
        this.telemetry = telemetry;
        this.name = name;
        this.hardwareMap = hardwareMap; // was never assigned here; updateFlywheels() would NPE
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);



        this.timer = timer;

    }
    public void setExternalDownHardstop(boolean on){
        low_external_hardstop = on;
    }

    public void setExternalUpHardstop(boolean on){
        high_external_hardstop = on;

    }
    public void setAlternateEncoder(AlternateEncoder alternateEncoder){
        this.alternateEncoder = alternateEncoder;
    }
    public void resetEncoder(){
        pid_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setReachedRange(int range){
        reached_range = range;
    }
    public void setPID(double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = 0;
    }
    public void setPIDF(double P, double I, double D, double F){
        setPID(P,I,D);
        this.F = F;
    }

    public void setMaxIntegral(double max_integral){
        this.max_integral_component = max_integral;
    }
    public void setDirection(DcMotor.Direction power){
        pid_motor.setDirection(power);
    }

    public void setMin(int min){
        minHardstop = min;
    }

    public void setMax(int max){
        maxHardstop = max;
    }
    public void setMaxVelocity(double speed){
        MAX_VEL = speed;
    }
    public void setMaxAcceleration(double accel){
        MAX_ACCEL = accel;
    }

    public void setMaxDeceleration(double decel){ MAX_DECEL = decel;}
    public boolean isBusy(){
        return !(reached);
    }
    public boolean isCompletedFor(double time){
        return !(reached && timer.time(TimeUnit.SECONDS) - completed_time > time);

    }
    @Override
    public void init(){
        pid_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public double getCurrent(){
        return pid_motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getCurrentAlert(){
        return pid_motor.getCurrentAlert(CurrentUnit.AMPS);
    }
    public double getVelocity(){
        return pid_motor.getVelocity();
    }
    public void setVelocity(double vel){
        pid_motor.setVelocity(vel);
    }
    @Override
    public void telemetry(){
        telemetry.addData(name + "'s Power", getPower());
        telemetry.addData(name + "'s Position", getCurrentPosition());
        telemetry.addData(name + "'s Target Position", targetPos);
        telemetry.addData(name + "'s Max Hardstop", maxHardstop);
        telemetry.addData(name + "'s Min Hardstop", minHardstop);
        telemetry.addData(name + " Exceeding Constraints", exceedingConstraints());
        telemetry.addData(name + "Power Cache", cached_power);
        telemetry.addData(name + " Busy Status", isBusy());
        telemetry.addData(name + "Current Draw", getCurrent());
        telemetry.addData(name + "Current Cooked", getCurrentAlert());
        telemetry.addData(name + " P", P);
        telemetry.addData(name + " I", I);
        telemetry.addData(name + " D", D);
        telemetry.addData(name + " F", F);


        telemetry.addData(name + "Velocity", getVelocity());
        telemetry.addData("Out Power", out);


    }


    public boolean exceedingConstraints(){
        boolean over_max = getCurrentPosition() > maxHardstop;
        boolean under_min = getCurrentPosition() < minHardstop;
        if (low_external_hardstop && getPower() < 0){
            return true;
        }
        return getPower() > 0 ? over_max : under_min;

    }
    public boolean exceedingConstraints(double power){
        boolean over_max = getCurrentPosition() > maxHardstop;
        boolean under_min = getCurrentPosition() < minHardstop;
        if (low_external_hardstop && power < 0){
            return true;
        }
        if(high_external_hardstop && power > 0){
            return true;
        }
        if (!reversed_encoder) {
            return power > 0 ? over_max : under_min;
        }
        return power < 0 ? over_max : under_min;

    }

    public void setManualPower(double power){
        if(power != 0){
            manual = true;
            waiting_for_manual = false;
            setPower(power);
        }
        if(power == 0 && manual && !waiting_for_manual){
            setPower(0);
            move_async_pid(pid_motor.getCurrentPosition());
            manual_power_time = timer.time();
            waiting_for_manual = true;
            manual = false;

        }
    }
    public void setZeroPowerBehavior_Brake(){
        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setZeroPowerBehavior_Float(){
        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        if(exceedingConstraints(power)) {
            power = 0;
        }
        if (Math.abs(power - cached_power) >= caching_tolerance || (power == 0.0 && cached_power != 0.0) || (power >= 1.0 && !(cached_power >= 1.0)) || (power <= -1.0 && !(cached_power <= -1.0)) || Double.isNaN(cached_power)) {
            pid_motor.setPower(power + holding_power);
            cached_power = power+holding_power;
        }

    }
    public void setDrivePower(double power) {
        if (Math.abs(power - cached_power) >= caching_tolerance || (power == 0.0 && cached_power != 0.0) || (power >= 1.0 && !(cached_power >= 1.0)) || (power <= -1.0 && !(cached_power <= -1.0)) || Double.isNaN(cached_power)) {
            pid_motor.setPower(power + holding_power);
            cached_power = power+holding_power;
        }

    }
    public void setUseMotionProfile(boolean on){
        motionProfileOverride = on;
    }
    public void setReversedEncoder(boolean reversed){
        this.reversed_encoder = reversed;
    }
    public int getCurrentPosition() {
        if(alternate){
            return alternateEncoder.getCurrentPosition();
        }
        return pid_motor.getCurrentPosition();
    }
    public void setAbsPower(double power){
        pid_motor.setPower(power);

    }
    public void move_async_pid(int target){
        target = Math.min(target, maxHardstop);
        target = Math.max(target, minHardstop);
        if(targetPos != target){
            useMotionProfile = false;
            reached = false;
            integralSum = 0;
            if(MAX_DECEL == -1){
                motion_profile_exit_time =  Control.motionProfileTime(MAX_ACCEL, MAX_VEL, distance);
            }else{
                motion_profile_exit_time = Control.motionProfileTime(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance);
            }
            time_stop = timer.seconds();
            time_started = timer.seconds();
            startPos = getCurrentPosition();
            distance = target - getCurrentPosition();

        }
        telemetry.addData(name + "'s Distance", distance);
        telemetry.addData(name + "'s Target", target);
        targetPos = target;
    }

    public void move_async(int target) {
        target = Math.min(target, maxHardstop);
        target = Math.max(target, minHardstop);
        if(targetPos != target){
            if(Math.abs(targetPos - target) < 50){
                useMotionProfile = false;
            }else{
                useMotionProfile = true;
                integralSum = 0;//potentially remove this
            }
            reached = false;
//              integralSum = 0;
            if(MAX_DECEL == -1){
                motion_profile_exit_time =  Control.motionProfileTime(MAX_ACCEL, MAX_VEL, distance);
            }else{
                motion_profile_exit_time = Control.motionProfileTime(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance);
            }
            time_stop = timer.seconds();
            time_started = timer.seconds();
            startPos = getCurrentPosition();
            distance = target - getCurrentPosition();

        }
        telemetry.addData(name + "'s Distance", distance);
        telemetry.addData(name + "'s Target", target);
        targetPos = target;

    }

    public void move_sync(int target_position){
        move_async(target_position);
        while (isBusy()){
            update();
        }
        return;
    }
    public void move_sync(int target_position, BulkRead bulkRead){
        move_async(target_position);
        while (isBusy()){
            bulkRead.clearCache();
            update();
        }
        return;
    }
    public double getPower(){
        return pid_motor.getPower();
    }

    public void update() {
        //temporarily disabled because is bad
//        if(waiting_for_manual && timer.time() - manual_power_time > 0.1){
//            waiting_for_manual = false;o
//            manual = false;
//            move_async(pid_motor.getCurrentPosition());
//            targetPos = pid_motor.getCurrentPosition();
//        }
        if(manual){
            return;
        }
        if(!on){
            return;
        }
//        telemetry.addData(name + " P", P);
//        telemetry.addData(name + " I",I);
//        telemetry.addData(name + " D", D);
//        telemetry.addData(name + " F", F);
        // Obtain the encoder position and calculate the error

        if(useMotionProfile && motionProfileOverride){
            double motion_profile_target_pos;
            double motion_profile_velocity;
            if(MAX_DECEL == -1){
                motion_profile_target_pos = Control.motionProfile(MAX_ACCEL, MAX_VEL, distance, timer.seconds() - time_started) + startPos;
                motion_profile_velocity = Control.motionProfileVelo(MAX_ACCEL, MAX_VEL, distance, timer.seconds() - time_started);
                error =  motion_profile_target_pos - getCurrentPosition();
            }else{
                 motion_profile_target_pos = Control.motionProfile( MAX_VEL, MAX_ACCEL, MAX_DECEL, distance, timer.seconds() - time_started) + startPos;
                motion_profile_velocity = Control.motionProfileVelo( MAX_VEL, MAX_ACCEL, MAX_DECEL, distance, timer.seconds() - time_started);
                error =  motion_profile_target_pos - getCurrentPosition();
            }
            telemetry.addData(name + "'s Motion Profile Target Pos", motion_profile_target_pos);
            telemetry.addData(name + "'s Motion Profile Velocity", motion_profile_velocity);
        }else {
            error = targetPos - getCurrentPosition();
        }

// Calculate time passed
        time_passed = timer.seconds() - time_stop;
          // Update time_stop early

// Proportional term
        out = P * error;

// Integral term with windup protection
        integralSum += error * time_passed;
        if (I * integralSum <= max_integral_component){
            out += I * integralSum;
        } else {
            out += max_integral_component;
        }

// Derivative term, ensuring time_passed is not zero
        if(time_passed > 0) {
            double derivative = (error - lastError) / time_passed;
            out += D * derivative;
        }
// Feedforward term
        out += F;
        out += compensation_power;
// Check if the target has been reached (based on an error threshold)
        boolean new_reached = exit_with_time ? (timer.seconds() - time_started > motion_profile_exit_time) : Math.abs(getCurrentPosition() - targetPos) < reached_range && Math.abs(out) < 0.3;
        if(new_reached && !reached){
            reached = true;
            completed_time = timer.seconds();
        }
        if (power_damp){
            double new_power = out * power_damp_coefficient + previous_power * (1 - power_damp_coefficient);
            setPower(new_power);
            previous_power = out;

        }else {
// Set motor power output
            setPower(out);
        }
// Update lastError for the next iteration
        lastError = error;
        time_stop = timer.seconds();
    }




//    public double getBatteryVoltage() {
//        double result = Double.POSITIVE_INFINITY;
//        for (VoltageSensor sensor : hardware.voltageSensor) {
//            double voltage = sensor.getVoltage();
//            if (voltage > 0) {
//                result = Math.min(result, voltage);
//            }
//        }
//        return result;
//    }
}