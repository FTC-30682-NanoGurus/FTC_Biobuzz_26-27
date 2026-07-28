package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.DecodeCAM;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;

import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class ATeleOp_FusedControl extends LinearOpMode{
    MecaTank mecaTank;
    private DecodeCAM camera;
    ElapsedTime timer;
    private DcMotorEx rollers;
    private NGMotor flywheels;
    private DcMotorEx transferRollers;
    private DcMotorEx interTransfer;
    private Servo hoodAdjuster;

    private enum Alliance {
        BLUE,
        RED
    }
    private Alliance currentAlliance = Alliance.BLUE;

    private static final double GOAL_X = -58.3727;
    private static final double BLUE_GOAL_Y = -55.6425;
    private static final double RED_GOAL_Y = 55.6425;
    private double targetGoalY = BLUE_GOAL_Y;
    private boolean autoAimActive = false;

    private boolean hasCalibrated = false;

    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        rollers = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.rollers);
        interTransfer = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.interTransfer);
        transferRollers = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.transferRollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);

        mecaTank = new MecaTank(hardwareMap, telemetry, timer);

        camera = new DecodeCAM();

        Pose2d startPose = null;
        if(PoseStorage.getLastBluePose() != null || PoseStorage.getLastRedPose() != null){
            if(currentAlliance == Alliance.BLUE) {
            startPose = PoseStorage.getLastBluePose(); //new Pose2d(5, -10, Math.toRadians(55));
            }else if(currentAlliance == Alliance.RED){
            startPose = PoseStorage.getLastRedPose(); //new Pose2d(5, 10, Math.toRadians(55));
            }
        }else{
            if(currentAlliance == Alliance.BLUE) {
                startPose = new Pose2d(5, -10, Math.toRadians(55));
            }else if(currentAlliance == Alliance.RED){
                startPose = new Pose2d(5, 10, Math.toRadians(55));
            }
        }

        mecaTank = new MecaTank(hardwareMap, telemetry, startPose);

        camera.init(hardwareMap.appContext, hardwareMap, telemetry);

        transferRollers.setDirection(DcMotor.Direction.REVERSE);

        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();

        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        interTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        interTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

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

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                currentAlliance = Alliance.RED;
                targetGoalY = RED_GOAL_Y;
            } else if (gamepad1.dpad_right) {
                currentAlliance = Alliance.BLUE;
                targetGoalY = BLUE_GOAL_Y;
            }

            telemetry.addData("INIT Status", "READY");
            telemetry.addData("Alliance (D-PAD L/R)", currentAlliance);
            telemetry.addData("Target Goal (X, Y)", "%.2f, %.2f", GOAL_X, targetGoalY);
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        long loopStartTime = System.currentTimeMillis();
        boolean poseAcquired = false;

        while(!poseAcquired && (System.currentTimeMillis() - loopStartTime) < 500) {
            Pose2d absoluteCameraPose = camera.getAbsoluteRobotPose();

            if (absoluteCameraPose != null) {
                Pose2d currentRRPose = mecaTank.getPoseEstimate();

                //Distance between Dead Wheels and Camera
                double difference = Math.hypot(
                        absoluteCameraPose.position.x - currentRRPose.position.x,
                        absoluteCameraPose.position.y - currentRRPose.position.y
                );

                if (difference > 1.35) {
                    Pose2d smoothPose = new Pose2d(
                            absoluteCameraPose.position.x,
                            absoluteCameraPose.position.y,
                            currentRRPose.heading.toDouble() // Trust Dead Wheel Rotation
                    );

                    mecaTank.setPoseEstimate(smoothPose);
                    hasCalibrated = true;
                }
                /*double smoothHeading = mecaTank.getPoseEstimate().heading.toDouble();

                // 2. Create a "Hybrid" Pose
                // Use Camera for Location (X, Y)
                // Use Dead Wheels for Rotation (Heading)
                Pose2d hybridPose = new Pose2d(
                        absoluteCameraPose.position.x,
                        absoluteCameraPose.position.y,
                        smoothHeading // Ignore camera heading!
                );

                // 3. Update Road Runner
                mecaTank.setPoseEstimate(hybridPose);
                telemetry.addLine("POSE ACQUIRED from Camera. Full Control Ready.");
                poseAcquired = true;*/
            } else {
                mecaTank.updateAutoAlign();
                telemetry.addLine("Seeking Pose... Move robot slightly to acquire AprilTag.");
            }
            telemetry.update();
            sleep(20);
        }
        if (!poseAcquired) {
            telemetry.addLine("Warning: Camera pose not acquired. Relying on odometry from default/Auto end pose.");
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            mecaTank.updateAutoAlign();
            flywheels.updateFlywheels(gamepad2.right_trigger > 0);
            Pose2d currentPose = mecaTank.getPoseEstimate();

            if (!autoAimActive) {
                Pose2d absoluteCameraPose = camera.getAbsoluteRobotPose();
                if (absoluteCameraPose != null) {
                    mecaTank.setPoseEstimate(absoluteCameraPose);
                    hasCalibrated = true;
                }
            }

            double rawStrafe = -gamepad1.right_stick_x;
            double rawForward = -gamepad1.right_stick_y;
            double manualTurnInput = gamepad1.right_stick_x;

            double strafeInput = (Math.abs(rawStrafe) > 0.05) ? rawStrafe : 0.0;
            double forwardInput = (Math.abs(rawForward) > 0.05) ? rawForward : 0.0;

            boolean hoodUp = gamepad2.dpad_right;
            boolean hoodDown = gamepad2.dpad_left;
            //telemetry.addData("hoodAdjuster Position: ", hoodAdjuster.getPosition());
            //telemetry.addData("Flywheel Vel: ", flywheels.getVelocity());
            //telemetry.update();

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                autoAimActive = true;
            } else if (gamepad1.y) {
                autoAimActive = false;
            }

            if (autoAimActive) {
                double dx = GOAL_X - currentPose.position.x;
                double dy = targetGoalY - currentPose.position.y;

                double targetHeadingRad = Math.atan2(dy, dx);
                double targetHeadingDeg = Math.toDegrees(targetHeadingRad);

                double finalTurnPower = mecaTank.calculateAutoTurnPower(
                        targetHeadingDeg,
                        Math.toDegrees(currentPose.heading.toDouble())
                );

                mecaTank.driveRobotCentric(strafeInput, forwardInput, finalTurnPower);

                double distanceToGoal = Math.hypot(dx, dy);

                telemetry.addData("Target Goal", "%s (%.1f, %.1f)",
                        currentAlliance, GOAL_X, targetGoalY);
                telemetry.addData("Distance", "%.1f in", distanceToGoal);
            } else {
                /*Pose2d absoluteCameraPose = camera.getAbsoluteRobotPose();
                if (absoluteCameraPose != null) {
                    mecaTank.setPoseEstimate(absoluteCameraPose);
                    telemetry.addData("Pose Status", "RE-CALIBRATED from AprilTag");
                } else {
                    telemetry.addData("Pose Status", "Coasting on Odometry (Full)");
                }*/
                mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            }

            boolean PosOverride = (Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1);
            //Automated Base Parking
            /*if(gamepad1.b){
                if(currentAlliance == Alliance.BLUE) {
                    mecaTank.moveToPositionBlocking(this, 30, 32, 90, 30, 4, PosOverride);
                }else if(currentAlliance == Alliance.RED){
                    mecaTank.moveToPositionBlocking(this, 30, -32, -90, 30, 4, PosOverride);
                }
            }

            //Automated far shooting position
            if(gamepad1.x){
                if(currentAlliance == Alliance.BLUE) {
                    mecaTank.moveToPositionBlocking(this, 54, -10, 35, 25, 4, PosOverride);
                }else if(currentAlliance == Alliance.RED){
                    mecaTank.moveToPositionBlocking(this, 54, 10, -35, 25, 4, PosOverride);
                }
            }*/

            if (!hasCalibrated) telemetry.addData("Status", "NOT CALIBRATED (Drive to see Tag)");
            else telemetry.addData("Status", "CALIBRATED");

            telemetry.addData("Current Pose (X, Y, H)", "%.2f, %.2f, %.2f",
                    currentPose.position.x,
                    currentPose.position.y,
                    Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.update();
            flywheels.getCurrent();
            if(gamepad2.a){
                rollers.setPower(1.0);
                interTransfer.setPower(0.85);
                transferRollers.setPower(0);
            }else if(gamepad2.b){
                rollers.setPower(0);
                interTransfer.setPower(0);
            }

            if(gamepad2.y){
                resetOuttake();
                flywheelsActive = false;
            }else if(gamepad2.right_trigger > 0.1){
                shoot();
            }else if(gamepad2.dpad_up){
                hoodPos = hoodClosePos;
                flywheels.setCustomVelocityPID(shooterCloseVel, 0.008, 0.015, 0.0001, 0.000426);
            }else if(gamepad2.dpad_down){
                hoodPos = hoodFarPos;
                flywheels.setCustomVelocityPID(shooterFarVel, 0.008, 0.015, 0.0001, 0.000426);
            }else if(gamepad2.x){
                prepShooter(shooterCloseVel);
                hoodPos = hoodClosePos;
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

            hoodAdjuster.setPosition(hoodPos);

            telemetry.addData("Intake current: ", rollers.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("interTransfer Current: ", interTransfer.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Transfer Current: ", transferRollers.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
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
        interTransfer.setPower(1.0);
        transferRollers.setPower(1.0); //0.8 before
        telemetry.update();
    }
    private void resetOuttake(){
        rollers.setPower(0);
        transferRollers.setPower(0);
        interTransfer.setPower(0);
    }
}