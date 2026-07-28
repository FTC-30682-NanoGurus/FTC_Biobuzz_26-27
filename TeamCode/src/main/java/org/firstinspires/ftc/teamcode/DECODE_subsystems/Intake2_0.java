package org.firstinspires.ftc.teamcode.DECODE_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;

@Config
public class Intake2_0 {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;
    public NGMotor rollers;
    public static NGMotor flywheels;
    public static Servo hoodAdjuster;
    public NGMotor transferRollers;
    public NGMotor interTransfer;
    public boolean autoFinished = false;

    public Intake2_0(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        timer = new ElapsedTime();
        rollers = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.rollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        transferRollers = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.transferRollers);
        interTransfer = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.interTransfer);
        interTransfer.setDirection(DcMotor.Direction.FORWARD);
        transferRollers.setDirection(DcMotor.Direction.REVERSE);
        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();

        hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);
    }
    public Intake2_0(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer) {
        this(hardwareMap, telemetry);
        this.timer = timer;
    }

    public void setAutoFinished(){
        autoFinished = true;
    }
    public boolean isAutoFinished(){
        return autoFinished;
    }

    public Action setFinished() { return new InstantAction(() -> setAutoFinished()); }

    public Action isFinished() { return new InstantAction(() -> isAutoFinished());}

    public void runRollers(double power){
        rollers.setPower(power);
        interTransfer.setPower(power);
    }
    public void stopRollers(){
        rollers.setPower(0);
        interTransfer.setPower(0);
    }
    public void runFlywheels(double vel){
        flywheels.setCustomVelocityPID(vel, 0.0085, 0.015, 0.0001, 0.000426);
    }
    public void stopFlywheels(){
        flywheels.setCustomVelocityPID(0, 0.0085, 0.015, 0.0001, 0.000426);
    }
    public static void initFlywheels(){
        flywheels.setCustomVelocityPID(0, 0.0085, 0.015, 0.0001, 0.000426);
    }
    public void runTransferRollers(double power){
        transferRollers.setPower(power);
        interTransfer.setPower(power);
    }
    public void stopTransferRollers(){
        transferRollers.setPower(0);
        interTransfer.setPower(0);
    }
    public void hoodAdjusterToPos(double position){
        hoodAdjuster.setPosition(position);
    }
    public void updateVelocity(){
        if(DECODERobotConstants.flywheelsActive) {
            flywheels.updateFlywheels(transferRollers.getPower() > 0);
        }
    }
    /*public Action updateFlywheelPID(){
        return new InstantAction(() -> updateVelocity());
    }*/

    public class updateAction implements Action{

        @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheels.updateFlywheels(transferRollers.getPower() > 0.1);

                telemetry.addData("FW Current Vel", "%.1f", flywheels.getVelocity());

                return true;
            }
        }
    public Action updateFlywheelPID(){
        return new updateAction();
    }

    public static void initHood(){
        hoodAdjuster.setPosition(0.9);
    }

    /*public void updateFlywheels(){
        flywheels.update();
    }*/
    public void prepShooter(double power){
        //hoodAdjuster.setPosition(0.5);
        flywheels.setPower(power);
        telemetry.addLine("Ready To Shoot");
        telemetry.addData("Flywheel Velocity: ", flywheels.getVelocity());
        telemetry.update();
    }
    public void shoot(){
        transferArtifacts();
        telemetry.update();
    }
    public void transferArtifacts(){
        rollers.setPower(1);
        transferRollers.setPower(1);
        telemetry.update();
    }
    public void resetOuttake(){
        rollers.setPower(0);
        transferRollers.setPower(0);
        flywheels.setPower(0);
        telemetry.clear();
        telemetry.addLine("Reset Complete");
        telemetry.update();
    }
    public Action runShooter(double vel, double time){
            return new SequentialAction(
                    new InstantAction(() -> runFlywheels(vel)),
                    new SleepAction(time),
                    new InstantAction(() -> stopFlywheels())
            );
    }
    public Action stopShooter(){
        return new SequentialAction(
                new InstantAction(() -> stopFlywheels())
        );
    }
    public Action shoot(double time, double vel){
                return new SequentialAction(
                        new InstantAction(() -> runFlywheels(vel)),
                        new SleepAction(time),
                        new InstantAction(() -> stopFlywheels())
                );
    }
    public Action collect(double time) {
        return new SequentialAction(
                        new InstantAction(() -> runRollers(1)),
                        new SleepAction(time),
                        new InstantAction(() -> stopRollers())
        );
    }
    public Action preventEscape(double time){
        return new SequentialAction(
                new InstantAction(() -> runRollers(0.3)),
                new SleepAction(time),
                new InstantAction(() -> stopRollers())
        );
    }
    public Action transferUsingRollersForTime(double time, double p){
        return new SequentialAction(
                        new InstantAction(() -> runTransferRollers(p)),
                        new InstantAction(() -> runRollers(p)),
                        new SleepAction(time),
                        new InstantAction(() -> stopTransferRollers()),
                        new InstantAction(() -> stopRollers())
        );
    }
    public Action transferUsingRollers(double p){
                return new ParallelAction(
                        new InstantAction(() -> runTransferRollers(p)),
                        new InstantAction(() -> runRollers(p))
                );
        }
        public Action disableTransfer(){
            return new ParallelAction(
                    new InstantAction(() -> stopTransferRollers()),
                    new InstantAction(() -> stopRollers())
            );
        }
        public Action setHoodAdjuster(double pos){
            return new SequentialAction(
                    new InstantAction(() -> hoodAdjusterToPos(pos))
            );
        }
    }

