package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

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
import org.firstinspires.ftc.teamcode.subsystems.DecodeCAM;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.subsystems.TargetingComputer;

import com.acmerobotics.roadrunner.Pose2d;

@Config
@TeleOp
public class ATeleOpDECODE_Efficient extends LinearOpMode {

    //Hardware
    BulkRead bulkRead;
    MecaTank mecaTank;
    MecanumDrive drive;
    private DecodeCAM camera;
    private TargetingComputer computer;
    private DcMotorEx rollers, transferRollers, interTransfer;
    private NGMotor flywheels;
    private Servo hoodAdjuster;

    //State trackers
    private Pose2d lockedPose = null;
    private Alliance currentAlliance = Alliance.BLUE;
    private double targetGoalY = BLUE_GOAL_Y;
    private boolean isAutoVel = false;
    private boolean autoAimActive = false;
    private boolean hasCalibrated = false;
    private boolean isIntaking = false;
    private long highCurrentStartTime = 0;

    //Timers
    private ElapsedTime camTimer = new ElapsedTime();
    private ElapsedTime currentTimer = new ElapsedTime();
    private double lastRollerCurrent = 0;

    //Alliance-based constants
    private static final double GOAL_X = -58.3727;
    private static final double BLUE_GOAL_Y = -55.6425;
    private static final double RED_GOAL_Y = 55.6425;

    private enum Alliance { BLUE, RED }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rollers = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.rollers);
        interTransfer = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.interTransfer);
        transferRollers = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.transferRollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, "hoodAdjuster");

        bulkRead = new BulkRead(hardwareMap);
        camera = new DecodeCAM();
        camera.init(hardwareMap.appContext, hardwareMap, telemetry);

        //Motor config
        transferRollers.setDirection(DcMotor.Direction.REVERSE);
        interTransfer.setDirection(DcMotorSimple.Direction.FORWARD);

        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        interTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();
        flywheels.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheels.setCustomVelocityPID(0.0, 0.008, 0.015, 0.0001, 0.000426);

        while (!isStarted() && !isStopRequested()) {
            bulkRead.clearCache();
            if (gamepad1.dpad_left) {
                currentAlliance = Alliance.RED;
                targetGoalY = RED_GOAL_Y;
            } else if (gamepad1.dpad_right) {
                currentAlliance = Alliance.BLUE;
                targetGoalY = BLUE_GOAL_Y;
            }
            telemetry.addData("ALLIANCE", currentAlliance);
            telemetry.update();
        }

        waitForStart();

        Pose2d startPose;
        if (currentAlliance == Alliance.BLUE) {
            startPose = new Pose2d(-35, -12, Math.toRadians(240));
        } else {
            startPose = new Pose2d(-30, 12, Math.toRadians(-240));
        }

        drive = new MecanumDrive(hardwareMap, startPose);
        mecaTank = new MecaTank(hardwareMap, telemetry, startPose);
        computer = new TargetingComputer(startPose, GOAL_X, targetGoalY);

        // Warm Up Loop
        long loopStartTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - loopStartTime) < 500 && opModeIsActive()) {
            Pose2d rawOdo = mecaTank.getPoseEstimate();
            Pose2d rawCam = camera.getAbsoluteRobotPose();
            Pose2d fusedPose = computer.update(rawOdo, rawCam, 0.0);

            mecaTank.setPoseEstimate(fusedPose);
            drive.pose = fusedPose;
        }

        double lastLoopTime = System.nanoTime();
        camTimer.reset();
        currentTimer.reset();

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();

            mecaTank.updatePoseEstimate();
            mecaTank.updateAutoAlign();
            Pose2d currentPose = mecaTank.getPoseEstimate();
            drive.pose = currentPose;

            double currentSpeed = mecaTank.getRobotVelocity();

            boolean isShooting = gamepad2.right_trigger > 0.1;
            boolean startIntake = gamepad2.a;
            boolean stopIntake = gamepad2.y;

            autoAimActive = gamepad1.right_stick_button;

            if (isShooting) {

                if (lockedPose == null) {
                    if (currentSpeed < 10.0) {
                        lockedPose = currentPose;
                    } else {
                        // TOO FAST
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
                    }
                }

                boolean readyToFire = false;
                if (lockedPose != null) {
                    readyToFire = mecaTank.holdPosition(lockedPose);
                }

                if (computer.getDistanceToGoal() > 100) {
                    farShoot();
                }
                else if (readyToFire) {
                    shoot();
                }
                else {
                    resetOuttake();
                }

            } else {

                lockedPose = null;
                mecaTank.releaseLock();

                if (gamepad1.a) {
                    double resetHeading = (currentAlliance == Alliance.BLUE) ? Math.toRadians(-90) : Math.toRadians(90);
                    Pose2d resetPose = new Pose2d(currentPose.position, resetHeading);
                    mecaTank.setPoseEstimate(resetPose);
                    drive.pose = resetPose;
                }

                mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

                transferRollers.setPower(0);
            }

            Pose2d rawCam = null;

            //Camera update (every 200ms)
            if (!isShooting && camTimer.milliseconds() > 200) {
                rawCam = camera.getAbsoluteRobotPose();
                camTimer.reset();
            }

            Pose2d smartPose = computer.update(currentPose, rawCam, currentSpeed);
            mecaTank.setPoseEstimate(smartPose);

            if (startIntake) {
                intake();
                isIntaking = true;
            }

            if (stopIntake) {
                resetOuttake();
                isIntaking = false;
                highCurrentStartTime = 0;
            }

            if(gamepad2.right_bumper){
                ejectArtifacts();
                isAutoVel = false;
            }

            if (isIntaking) {

                if (currentTimer.milliseconds() > 50) {
                    lastRollerCurrent = rollers.getCurrent(CurrentUnit.MILLIAMPS);
                    currentTimer.reset();
                }
                //Current threshold: 3000
                //Time threshold: 20 ms
                if (lastRollerCurrent > 3000) {
                    if (highCurrentStartTime == 0) {
                        highCurrentStartTime = System.currentTimeMillis();
                    } else if ((System.currentTimeMillis() - highCurrentStartTime) > 20) {
                        resetOuttake();
                        isIntaking = false;
                        highCurrentStartTime = 0;
                    }
                } else {
                    highCurrentStartTime = 0;
                }
            }

            flywheels.updateFlywheels(isShooting);

            if(gamepad2.dpad_up){
                flywheelCloseMode();
                isAutoVel = false;
            } else if(gamepad2.dpad_down){
                flywheelFarMode();
                isAutoVel = false;
            }

            if(gamepad2.x){
                isAutoVel = true;
            }else if(gamepad2.left_bumper){
                stopFlywheel();
                isAutoVel = false;
            }

            if(isAutoVel) {
                TargetingComputer.ShotData solution = computer.getShooterSolution();
                //flywheels.setCustomVelocityPID(solution.velocity, 0.06, 0.0004, 0, 0.00046);
                flywheels.setCustomVelocityPID(solution.velocity, 0, 0.0004, 0.0004, 0.0009);
                hoodAdjuster.setPosition(solution.hoodPosition);
                telemetry.addData("target vel: ", solution.velocity);
            }

            double currentLoopTime = System.nanoTime();
            double loopMs = (currentLoopTime - lastLoopTime) / 1e6;
            lastLoopTime = currentLoopTime;

            telemetry.addData("Loop Time (ms)", loopMs);
            //flywheels.loopTelemetry(loopMs);
            telemetry.update();
        }
    }

    private void flywheelCloseMode(){
        flywheels.setCustomVelocityPID(DECODERobotConstants.closeZoneShootingVel, 0.0092, 0.016, 0.0001, 0.000426);
        hoodAdjuster.setPosition(DECODERobotConstants.closeShootPos);
    }
    private void flywheelFarMode(){
        flywheels.setCustomVelocityPID(DECODERobotConstants.farZoneShootingVel, 0.0092, 0.016, 0.0001, 0.000426);
        hoodAdjuster.setPosition(DECODERobotConstants.farShootPos);
    }

    private void stopFlywheel(){ flywheels.setCustomVelocityPID(0, 0.0092, 0.016, 0.0001, 0.00035); }
    private void intake(){rollers.setPower(1.0); interTransfer.setPower(0.1); transferRollers.setPower(0); }
    private void shoot(){ transferArtifacts(); }
    private void farShoot(){rollers.setPower(0.3); interTransfer.setPower(0.3); transferRollers.setPower(0.3);}
    private void transferArtifacts(){ rollers.setPower(1.0); interTransfer.setPower(1.0); transferRollers.setPower(1.0); }
    private void resetOuttake(){ rollers.setPower(0); transferRollers.setPower(0); interTransfer.setPower(0); }
    private void ejectArtifacts(){
        flywheels.setCustomVelocityPID(-800, 0.0092, 0.016, 0.0001, 0.000426);
        transferRollers.setPower(-0.7);
        rollers.setPower(-0.9);
        interTransfer.setPower(-0.9);
    }
}
