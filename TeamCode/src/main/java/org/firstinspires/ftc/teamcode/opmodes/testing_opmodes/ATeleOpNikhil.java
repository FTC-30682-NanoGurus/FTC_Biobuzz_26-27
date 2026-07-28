package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;

import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class ATeleOpNikhil extends LinearOpMode{
    MecaTank mecaTank;
    ElapsedTime timer;
    private NGMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor rollers;
    private NGMotor flywheels;
    private DcMotor transferRollers;
    private Servo hoodAdjuster;


    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        rollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.rollers);
        transferRollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.transferRollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);
        mecaTank = new MecaTank(hardwareMap, telemetry, timer);

        transferRollers.setDirection(DcMotor.Direction.REVERSE);

        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();

        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hoodAdjuster.setPosition(DECODERobotConstants.hoodStartPos);

        flywheels.setCustomVelocityPID(0.0, 0.008, 0.015, 0.0001, 0.000426);

        double hoodPos = hoodAdjuster.getPosition();
        boolean lastUp = false;
        boolean lastDown = false;

        double shooterCloseVel = DECODERobotConstants.closeZoneShootingVel;
        double shooterFarVel = DECODERobotConstants.farZoneShootingVel;
        double hoodClosePos = DECODERobotConstants.closeShootPos;
        double hoodFarPos = DECODERobotConstants.farShootPos;
        boolean shootReady = false;
        boolean flywheelsActive = false;

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            flywheels.updateFlywheels(gamepad2.right_trigger > 0.1);
            boolean hoodUp = gamepad2.dpad_up;
            boolean hoodDown = gamepad2.dpad_down;
            telemetry.addData("hoodAdjuster Position: ", hoodAdjuster.getPosition());
            telemetry.addData("Flywheel Vel: ", flywheels.getVelocity());
            telemetry.update();

            /*if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_y==0) {
                frontRight.setDrivePower(-gamepad1.left_stick_y);
                backRight.setDrivePower(-gamepad1.left_stick_y);
            }else if(gamepad1.right_stick_y != 0 && gamepad1.left_stick_y==0){
                frontLeft.setDrivePower(-gamepad1.right_stick_y);
                backLeft.setDrivePower(-gamepad1.right_stick_y);
            }else if(gamepad1.right_trigger!=0){
                strafeRight(1, gamepad1.right_trigger);
            }else if(gamepad1.left_trigger!=0){
                strafeLeft(1, gamepad1.left_trigger);
            }/*else if(gamepad1.left_stick_y==0 && gamepad1.right_stick_y==0){
                frontLeft.setDrivePower(0);
                frontRight.setDrivePower(0);
                backLeft.setDrivePower(0);
                backRight.setDrivePower(0);
            }else if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_y != 0){
                else{
                    frontRight.setDrivePower(gamepad1.right_stick_y);
                    backRight.setDrivePower(gamepad1.right_stick_y);
                    frontLeft.setDrivePower(gamepad1.left_stick_y);
                    backLeft.setDrivePower(gamepad1.left_stick_y);
                    }else if(gamepad1.left_stick_y < -0.1 && gamepad1.right_stick_y > 0.1 || gamepad1.left_stick_y > 0.1 && gamepad1.right_stick_y < -0.1){
                frontRight.setDrivePower(-gamepad1.right_stick_y);
                backRight.setDrivePower(-gamepad1.right_stick_y);
                frontLeft.setDrivePower(-gamepad1.left_stick_y);
                backLeft.setDrivePower(-gamepad1.left_stick_y);
            }*/
            mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

            if(gamepad2.a){
                rollers.setPower(1.0);
                transferRollers.setPower(0);
            }else if(gamepad2.b){
                rollers.setPower(0);
            }
            if(gamepad2.y){
                resetOuttake();
                flywheelsActive = false;
            }else if(gamepad2.right_trigger > 0.1){
                shoot();
            }else if(gamepad1.dpad_up){
                hoodPos = hoodClosePos;
                flywheels.setCustomVelocityPID(shooterCloseVel, 0.008, 0.015, 0.0001, 0.000426);
            }else if(gamepad1.dpad_down){
                hoodPos = hoodFarPos;
                flywheels.setCustomVelocityPID(shooterFarVel, 0.008, 0.015, 0.0001, 0.000426);
            }else if(gamepad2.x){
                prepShooter(shooterFarVel);
                hoodPos = hoodFarPos;
                flywheelsActive = true;
            }else if(gamepad2.right_bumper){
                flywheels.setCustomVelocityPID(-800, 0.008, 0.015, 0.0001, 0.000426);
                transferRollers.setPower(-0.7);
                rollers.setPower(-0.9);
                flywheelsActive = false;
            }else if(gamepad2.left_bumper){
                flywheels.setCustomVelocityPID(0, 0.008, 0.015, 0.0001, 0.000426);
            }else if(hoodUp && !lastUp) {
                hoodPos -= 0.15;
            }else if (hoodDown && !lastDown) {
                hoodPos += 0.15;
            }

            lastUp = hoodUp;
            lastDown = hoodDown;
            hoodPos = Range.clip(hoodPos, 0, 1);
            /*if(flywheelsActive){
                sleep(2000);
                if(flywheels.getVelocity() >= 1275 && flywheels.getVelocity() <= 1325){
                telemetry.addLine("READY TO SHOOT");
                telemetry.update();
                shootReady = true;
                }else{
                    telemetry.addData("Current Velocity: ", flywheels.getVelocity());
                    shootReady = false;
                }
                if(!shootReady){
                    flywheels.setVelocity(shooterTargetVel-100);
                    telemetry.addLine("Updating Velocity...");
                    telemetry.update();
                }else{
                    flywheels.setVelocity(shooterTargetVel);
                }
            }*/


            hoodAdjuster.setPosition(hoodPos);
            telemetry.update();
        }
    }
    private void strafeLeft(double L_STRAFE_POWER, double left_trigger){
        double posPower = sameSignSqrt(left_trigger);
        double negPower = sameSignSqrt(-left_trigger);
        frontRight.setDrivePower(L_STRAFE_POWER*negPower);
        backLeft.setDrivePower(L_STRAFE_POWER*negPower);
        frontLeft.setDrivePower(L_STRAFE_POWER*posPower);
        backRight.setDrivePower(L_STRAFE_POWER*posPower);
    }
    private void strafeRight(double R_STRAFE_POWER, double right_trigger){
        double posPower = sameSignSqrt(right_trigger);
        double negPower = sameSignSqrt(-right_trigger);
        frontRight.setDrivePower(R_STRAFE_POWER*posPower);
        backLeft.setDrivePower(R_STRAFE_POWER*posPower);
        frontLeft.setDrivePower(R_STRAFE_POWER*negPower);
        backRight.setDrivePower(R_STRAFE_POWER*negPower);
    }
    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    private void prepShooter(double vel){
        flywheels.setCustomVelocityPID(vel, 0.008, 0.015, 0.0001, 0.000426);
        transferRollers.setPower(0);
    }
    private void shoot(){
        transferArtifacts();
        telemetry.update();
    }
    private void transferArtifacts(){
        rollers.setPower(1.0);
        transferRollers.setPower(0.8);
        telemetry.update();
    }
    private void resetOuttake(){
        rollers.setPower(0);
        transferRollers.setPower(0);
    }
}
