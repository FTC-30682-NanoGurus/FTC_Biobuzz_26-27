package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.DecodeCAM;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.TargetingComputer;

import com.acmerobotics.roadrunner.Pose2d;

@Config
@TeleOp
public class ATeleOpDECODE extends LinearOpMode {
    FtcDashboard dashboard;
    Telemetry DStelemetry;
    BulkRead bulkRead;
    MecaTank mecaTank;
    MecanumDrive drive;
    private DecodeCAM camera;
    LED trafficLightRed;
    LED trafficLightGreen;
    LED trafficLightOrange;
    private TargetingComputer computer;
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
    private double targetGoalY;
    private Boolean isAutoVel = false;

    private boolean autoAimActive = false;
    private boolean hasCalibrated = false;

    private long highCurrentStartTime = 0;
    private boolean isIntaking = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();

        rollers = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.rollers);
        interTransfer = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.interTransfer);
        transferRollers = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.transferRollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, "hoodAdjuster");

        trafficLightRed = hardwareMap.get(LED.class, "front_led_red");
        trafficLightOrange = hardwareMap.get(LED.class, "front_led_orange");

        mecaTank = new MecaTank(hardwareMap, telemetry, timer);
        bulkRead = new BulkRead(hardwareMap);
        camera = new DecodeCAM();

        camera.init(hardwareMap.appContext, hardwareMap, telemetry);

        transferRollers.setDirection(DcMotor.Direction.REVERSE);
        interTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        interTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        while (!isStarted() && !isStopRequested()) {
            bulkRead.clearCache();

            if (gamepad1.dpad_left) {
                currentAlliance = Alliance.RED;
                targetGoalY = RED_GOAL_Y;
            } else if (gamepad1.dpad_right) {
                currentAlliance = Alliance.BLUE;
                targetGoalY = BLUE_GOAL_Y;
            }

            telemetry.addData("INIT Status", "READY");
            telemetry.addData("Alliance", currentAlliance);
            telemetry.addData("Target Goal", "%.2f, %.2f", GOAL_X, targetGoalY);
            telemetry.update();
        }

        waitForStart();

        Pose2d startPose = null;

        // Default backup poses
        if (currentAlliance == Alliance.BLUE) {
            startPose = new Pose2d(-35, -12, Math.toRadians(240));
            targetGoalY = BLUE_GOAL_Y;
        } else if(currentAlliance == Alliance.RED){
            startPose = new Pose2d(-30, 12, Math.toRadians(-240));
            targetGoalY = RED_GOAL_Y;
        }

        // Initialize Drives
        mecaTank = new MecaTank(hardwareMap, telemetry, startPose);
        drive = new MecanumDrive(hardwareMap, startPose); // FIXED: Added this so brake logic doesn't crash

        double goalY = targetGoalY;
        computer = new TargetingComputer(startPose, GOAL_X, goalY);

        long loopStartTime = System.currentTimeMillis();

        boolean poseAcquired = false;

        while (!poseAcquired && (System.currentTimeMillis() - loopStartTime) < 500 && opModeIsActive()) {

            Pose2d rawOdo = mecaTank.getPoseEstimate();
            Pose2d rawCam = camera.getAbsoluteRobotPose();

            Pose2d fusedPose = computer.update(rawOdo, rawCam, 0.0);
            mecaTank.setPoseEstimate(fusedPose);
            drive.pose = fusedPose; // Sync RoadRunner drive

            if (computer.getConfidence() > 50.0) {
                poseAcquired = true;
                hasCalibrated = true;
            } else {
                mecaTank.updateAutoAlign();
            }
            telemetry.update();
        }

        double loopTime = 0;
        double lastLoopTime = System.nanoTime();

        Pose2d lockedPose = null;

        while (!isStopRequested() && opModeIsActive()) {

            bulkRead.clearCache(); // Keep this first!

            mecaTank.updatePoseEstimate();
            mecaTank.updateAutoAlign();
            double currentSpeed = mecaTank.getRobotVelocity();

            // OPTIMIZATION: Removed the 3 constant .getCurrent() calls here.
            // They wasted ~15ms per loop. The one you need is now inside the intake block.

            boolean autoReset = false;

            flywheels.updateFlywheels(gamepad2.right_trigger > 0.1);

            Pose2d rawOdo = mecaTank.getPoseEstimate();
            Pose2d rawCam = camera.getAbsoluteRobotPose();

            trafficLightRed.off();
            trafficLightOrange.off();

            boolean isShooting = gamepad2.right_trigger > 0.1;
            autoAimActive = gamepad1.right_stick_button;

            if (isShooting) {

                double speed = mecaTank.getRobotVelocity();
                if (lockedPose == null) {
                    if (speed < 10.0) {
                        lockedPose = mecaTank.getPoseEstimate();
                    } else {
                        // This uses "drive" which is now safely initialized
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                    }
                }

                boolean readyToFire = false;
                if (lockedPose != null) {
                    readyToFire = mecaTank.holdPosition(lockedPose);
                }

                if (computer.getDistanceToGoal() > 100) {
                    //far safety
                    rollers.setPower(0.3); interTransfer.setPower(0.3); transferRollers.setPower(0.3);
                }
                else if (readyToFire) {
                    shoot();
                    trafficLightRed.off();
                }
                else {
                    rollers.setPower(0); interTransfer.setPower(0); transferRollers.setPower(0);
                    trafficLightRed.on();
                }

            } else {
                lockedPose = null;
                mecaTank.releaseLock();

                mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

                transferRollers.setPower(0);
                trafficLightRed.off();
            }


            if (autoAimActive) {
                rawCam = null;
            }

            Pose2d smartPose = computer.update(rawOdo, rawCam, currentSpeed);

            mecaTank.setPoseEstimate(smartPose);

            // Update Calibration Status
            if (computer.getConfidence() > 30.0) hasCalibrated = true;

            double rawStrafe = -gamepad1.right_stick_x;
            double rawForward = -gamepad1.right_stick_y;

            double strafeInput = (Math.abs(rawStrafe) > 0.05) ? rawStrafe : 0.0;
            double forwardInput = (Math.abs(rawForward) > 0.05) ? rawForward : 0.0;

            if (autoAimActive) {
                double dx = GOAL_X - smartPose.position.x;
                double dy = targetGoalY - smartPose.position.y;
                double targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));

                double finalTurnPower = mecaTank.calculateAutoTurnPower(
                        targetHeadingDeg,
                        Math.toDegrees(smartPose.heading.toDouble())
                );
                mecaTank.driveRobotCentric(strafeInput, forwardInput, finalTurnPower);
            }

            boolean PosOverride = (Math.abs(gamepad1.right_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2);

            // Automated Base Parking
            if(gamepad1.b){
                if(currentAlliance == Alliance.BLUE) {
                    mecaTank.moveToPositionBlocking(this, smartPose, 26, 36, 270, 75, 5, PosOverride);
                } else if(currentAlliance == Alliance.RED){
                    mecaTank.moveToPositionBlocking(this, smartPose, 26, -36, 90, 75, 5, PosOverride);
                }
            }

            // Automated far shooting position
            if(gamepad1.x){
                if(currentAlliance == Alliance.BLUE) {
                    mecaTank.moveToAngleBlocking(this, smartPose, 220, 60, 1.5, PosOverride);
                } else if(currentAlliance == Alliance.RED){
                    mecaTank.moveToAngleBlocking(this, smartPose, -220, 60, 1.5, PosOverride);
                }
            }

            boolean startIntake = gamepad2.a;
            boolean stopIntake = gamepad2.y;

            if (startIntake) {
                rollers.setPower(1.0);
                interTransfer.setPower(1.0);
                transferRollers.setPower(0);
                isIntaking = true;
            }

            if (stopIntake) {
                resetOuttake();
                isIntaking = false;
                highCurrentStartTime = 0;
            }

            if (isIntaking) {
                // OPTIMIZATION: Only check current when actually intaking
                double intakeCurrent = rollers.getCurrent(CurrentUnit.MILLIAMPS);

                if (intakeCurrent > 4000) {
                    if (highCurrentStartTime == 0) {
                        highCurrentStartTime = System.currentTimeMillis();
                    } else if ((System.currentTimeMillis() - highCurrentStartTime) > 40) {
                        resetOuttake();
                        isIntaking = false;
                        highCurrentStartTime = 0;
                        trafficLightRed.on();
                    }
                } else {
                    highCurrentStartTime = 0; //Reset counter
                }
            }

            if(gamepad2.dpad_up){
                flywheels.setCustomVelocityPID(shooterCloseVel, 0.0092, 0.016, 0.0001, 0.000426);
                hoodAdjuster.setPosition(hoodClosePos);
                isAutoVel = false;
            } else if(gamepad2.dpad_down){
                flywheels.setCustomVelocityPID(shooterFarVel, 0.0092, 0.016, 0.0001, 0.000426);
                hoodAdjuster.setPosition(hoodFarPos);
                isAutoVel = false;
            } else if(gamepad2.x){
                prepShooter();
            } else if(gamepad2.b){
                flywheels.setCustomVelocityPID(800,0.0092, 0.016, 0.0001, 0.000426);
                isAutoVel = false;
            } else if(gamepad2.right_bumper){
                flywheels.setCustomVelocityPID(-800, 0.0092, 0.016, 0.0001, 0.000426);
                transferRollers.setPower(-0.7);
                rollers.setPower(-0.9);
                interTransfer.setPower(-0.9);
                isAutoVel = false;
            } else if(gamepad2.left_bumper){
                flywheels.setCustomVelocityPID(0, 0.0092, 0.016, 0.0001, 0.000426);
                isAutoVel = false;
            }

            if(isAutoVel) {
                TargetingComputer.ShotData solution = computer.getShooterSolution();
                flywheels.setCustomVelocityPID(solution.velocity, 0.06, 0.0004, 0, 0.00064);
                hoodPos = solution.hoodPosition;
                hoodAdjuster.setPosition(hoodPos);
            }

            double currentLoopTime = System.nanoTime();
            loopTime = (currentLoopTime - lastLoopTime) / 1e6;
            lastLoopTime = currentLoopTime;

            telemetry.addData("Loop Time", "%.1f ms", loopTime);
            telemetry.addData("Frequency", "%.1f Hz", 1000 / loopTime);
            telemetry.update();
        }
    }

    private void prepShooter(){
        //flywheels.setCustomVelocityPID(vel, 0.0085, 0.015, 0.0001, 0.000426);
        isAutoVel = true;
        transferRollers.setPower(0);
    }
    private void shoot(){
        transferArtifacts();
    }
    private void transferArtifacts(){
        rollers.setPower(1.0);
        interTransfer.setPower(1.0);
        transferRollers.setPower(1.0);
    }
    private void resetOuttake(){
        rollers.setPower(0);
        transferRollers.setPower(0);
        interTransfer.setPower(0);
    }
}