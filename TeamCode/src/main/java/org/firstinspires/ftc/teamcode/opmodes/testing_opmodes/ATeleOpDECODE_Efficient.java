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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.DecodeCAM;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.TargetingComputer;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Latency-optimised twin of ATeleOpDECODE.
 *
 * Controls, state machine and tuning constants are identical to the original - every change here
 * removes redundant hardware I/O, redundant object construction or redundant logging. The places
 * where an optimisation *could* have shifted robot behaviour are called out inline.
 */
@Config
@TeleOp
public class ATeleOpDECODE_Efficient extends LinearOpMode {

    //Hardware
    BulkRead bulkRead;
    MecaTank mecaTank;
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
    private ElapsedTime telemetryTimer = new ElapsedTime();
    private double lastRollerCurrent = 0;

    //Alliance-based constants
    private static final double GOAL_X = -58.3727;
    private static final double BLUE_GOAL_Y = -55.6425;
    private static final double RED_GOAL_Y = 55.6425;

    /**
     * How often the driver station / dashboard packet is actually sent. FtcDashboard's telemetry
     * adapter serialises and sockets a packet on EVERY update() call - unlike the driver station
     * telemetry, it does not rate-limit itself.
     */
    public static double TELEMETRY_INTERVAL_MS = 100.0;

    /**
     * Measured loop rate of this opmode BEFORE these optimisations, in Hz.
     *
     * TargetingComputer used to bleed confidence by a flat 2.0 per loop, so its behaviour was tied
     * to loop rate. We convert that to a per-second rate using this number, which keeps the fusion
     * filter behaving exactly as it did. ASSUMPTION: the pre-optimisation loop ran at ~50 Hz.
     * If your logged loop time said otherwise, set this to the real value - it is the one constant
     * here that needs your measurement to be exactly faithful.
     */
    public static double PRE_OPT_LOOP_HZ = 50.0;

    /**
     * Servo deadband. 0.0 = write whenever the commanded position differs at all, which is exactly
     * what the original did while still skipping the (very common) identical re-write when the
     * robot is parked and shooting. Raise to ~0.003 to also skip sub-resolution jitter, at the cost
     * of the hood lagging a commanded move by that much.
     */
    public static double HOOD_DEADBAND = 0.0;

    private static final PoseVelocity2d ZERO_POWER = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // Last value actually written to each raw motor / the hood servo. NaN forces the first write.
    // Plain DcMotorEx.setPower() issues a Lynx command every call - it has no cache of its own,
    // unlike NGMotor.setDrivePower(). These helpers only suppress writes of an IDENTICAL value,
    // so the commanded power at any instant is unchanged.
    private double lastRollers = Double.NaN;
    private double lastInterTransfer = Double.NaN;
    private double lastTransferRollers = Double.NaN;
    private double lastHood = Double.NaN;

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
        // Tag overlays off: they are rendered per frame on the vision thread and only feed the
        // dashboard preview. No pose maths reads them.
        camera.init(hardwareMap.appContext, hardwareMap, telemetry, false);

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
            // 50 Hz is far more than enough to catch a dpad press, and stops us hammering the
            // dashboard socket flat out before the match even starts.
            sleep(20);
        }

        waitForStart();

        Pose2d startPose;
        if (currentAlliance == Alliance.BLUE) {
            startPose = new Pose2d(-35, -12, Math.toRadians(240));
        } else {
            startPose = new Pose2d(-30, 12, Math.toRadians(-240));
        }

        // NOTE: this opmode used to build its OWN MecanumDrive here in addition to the one MecaTank
        // builds internally - two localizers, two LazyImus and two sets of motor handles for the
        // same four motors. The local copy's pose was only ever written to, never integrated, so
        // every `drive.pose = ...` assignment against it was dead. We now use mecaTank.drive.
        // MecanumDrive.PARAMS uses LEFT/BACKWARD, identical to MecaTank's own LazyImu, so the final
        // IMU configuration after construction is unchanged.
        mecaTank = new MecaTank(hardwareMap, telemetry, startPose);
        MecanumDrive drive = mecaTank.drive;

        computer = new TargetingComputer(startPose, GOAL_X, targetGoalY);
        computer.setTimeBasedDecay(2.0 * PRE_OPT_LOOP_HZ);

        // MecanumDrive's constructor forces every hub to AUTO, which was silently overriding the
        // MANUAL mode BulkRead set up above and making clearCache() meaningless. Take it back now
        // that every drivetrain object exists.
        bulkRead.setManual();

        // This opmode drives the pose estimate itself, once per loop.
        mecaTank.setAutoPoseUpdate(false);

        // Static flags - restored in the finally block so a later auto/tuning opmode in the same
        // app session still gets its RoadRunner logs.
        boolean prevLogPose = MecanumDrive.LOG_POSE_HISTORY;
        boolean prevLogInputs = TwoDeadWheelLocalizer.LOG_INPUTS;
        MecanumDrive.LOG_POSE_HISTORY = false;
        TwoDeadWheelLocalizer.LOG_INPUTS = false;

        try {
            // Warm Up Loop
            // Left structurally as-is on purpose: it converges the fused pose by repeated
            // application of TargetingComputer's gain, so changing how many times it iterates would
            // change the pose the match starts from.
            long loopStartTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - loopStartTime) < 500 && opModeIsActive()) {
                Pose2d rawOdo = mecaTank.getPoseEstimate();
                Pose2d rawCam = camera.getAbsoluteRobotPose();
                Pose2d fusedPose = computer.update(rawOdo, rawCam, 0.0);

                mecaTank.setPoseEstimate(fusedPose); // this already sets drive.pose
            }

            double lastLoopTime = System.nanoTime();
            double loopMsMax = 0;
            camTimer.reset();
            currentTimer.reset();
            telemetryTimer.reset();

            while (!isStopRequested() && opModeIsActive()) {
                bulkRead.clearCache();

                // Only pose update in the loop. This previously ran up to three times per
                // iteration (here, inside updateAutoAlign(), and inside holdPosition()), and each
                // one costs two IMU reads that the bulk read does not cover. Pose integration is
                // delta-based, so one sample per loop integrates the same total motion.
                mecaTank.updatePoseEstimate();
                mecaTank.updateAutoAlign();
                Pose2d currentPose = mecaTank.getPoseEstimate();

                // Send the driver station / dashboard packet on a timer. addData is gated by the
                // same flag: FTC telemetry only auto-clears on update(), so adding items on a loop
                // that will not update() would just pile up duplicate lines.
                boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

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
                            drive.setDrivePowers(ZERO_POWER);
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
                        mecaTank.setPoseEstimate(resetPose); // this already sets drive.pose
                    }

                    mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

                    setTransferRollers(0);
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

                    // getCurrent() is a LynxGetADC command, not covered by the bulk read - hence
                    // the existing 50 ms throttle. Unchanged.
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

                flywheels.setTelemetryEnabled(sendTelemetry);
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
                    setHood(solution.hoodPosition);
                    if (sendTelemetry) telemetry.addData("target vel: ", solution.velocity);
                }

                double currentLoopTime = System.nanoTime();
                double loopMs = (currentLoopTime - lastLoopTime) / 1e6;
                lastLoopTime = currentLoopTime;
                if (loopMs > loopMsMax) loopMsMax = loopMs;

                if (sendTelemetry) {
                    telemetry.addData("Loop Time (ms)", loopMs);
                    telemetry.addData("Loop Time Max (ms)", loopMsMax);
                    telemetry.addData("Hz", 1000.0 / loopMs);
                    //flywheels.loopTelemetry(loopMs);
                    telemetry.update();
                    loopMsMax = 0;
                    telemetryTimer.reset();
                }
            }
        } finally {
            MecanumDrive.LOG_POSE_HISTORY = prevLogPose;
            TwoDeadWheelLocalizer.LOG_INPUTS = prevLogInputs;
        }
    }

    // --- Cached raw-motor / servo writes -------------------------------------------------------
    // Identical-value writes are suppressed; any change in commanded value still goes out the same
    // loop it was requested. Behaviour at the motor is unchanged.

    private void setRollers(double power) {
        if (power != lastRollers) {
            rollers.setPower(power);
            lastRollers = power;
        }
    }

    private void setInterTransfer(double power) {
        if (power != lastInterTransfer) {
            interTransfer.setPower(power);
            lastInterTransfer = power;
        }
    }

    private void setTransferRollers(double power) {
        if (power != lastTransferRollers) {
            transferRollers.setPower(power);
            lastTransferRollers = power;
        }
    }

    private void setHood(double position) {
        if (Double.isNaN(lastHood) || Math.abs(position - lastHood) > HOOD_DEADBAND) {
            hoodAdjuster.setPosition(position);
            lastHood = position;
        }
    }

    private void flywheelCloseMode(){
        flywheels.setCustomVelocityPID(DECODERobotConstants.closeZoneShootingVel, 0.0092, 0.016, 0.0001, 0.000426);
        setHood(DECODERobotConstants.closeShootPos);
    }
    private void flywheelFarMode(){
        flywheels.setCustomVelocityPID(DECODERobotConstants.farZoneShootingVel, 0.0092, 0.016, 0.0001, 0.000426);
        setHood(DECODERobotConstants.farShootPos);
    }

    private void stopFlywheel(){ flywheels.setCustomVelocityPID(0, 0.0092, 0.016, 0.0001, 0.00035); }
    private void intake(){ setRollers(1.0); setInterTransfer(0.1); setTransferRollers(0); }
    private void shoot(){ transferArtifacts(); }
    private void farShoot(){ setRollers(0.3); setInterTransfer(0.3); setTransferRollers(0.3); }
    private void transferArtifacts(){ setRollers(1.0); setInterTransfer(1.0); setTransferRollers(1.0); }
    private void resetOuttake(){ setRollers(0); setTransferRollers(0); setInterTransfer(0); }
    private void ejectArtifacts(){
        flywheels.setCustomVelocityPID(-800, 0.0092, 0.016, 0.0001, 0.000426);
        setTransferRollers(-0.7);
        setRollers(-0.9);
        setInterTransfer(-0.9);
    }
}
