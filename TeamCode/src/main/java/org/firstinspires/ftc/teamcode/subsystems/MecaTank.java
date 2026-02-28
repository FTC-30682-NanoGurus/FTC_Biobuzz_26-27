package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.Control;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Pose2d;

@Config
public class MecaTank extends Subsystem {
    private NGMotor frontLeft, frontRight, backLeft, backRight;

    public  TrafficLight trafficLight;
    private LazyImu imu; // IMU for field-centric control

    public Distance distance;
    public Distance rear_distance;
    private Telemetry telemetry;
    private double MAX_DRIVE_SPEED = 1;

    private boolean left_strafe = false;
    private boolean right_strafe = false;
    private boolean force_exit = false;

    private boolean fast_drive_direction = false;

    private boolean use_dead_wheel = false;
    private double distance_to_target = 0;
    private double starting_motion_profile_time = 0;

    private double fast_drive_speed = 0;
    private double starting_pos = 0;
    private double time_stop = 0;
    double currentFilterEstimate = 0;
    double previousFilterEstimate = 0;
    public static double kP = 0.02;  // Proportional constant
    public static double kI = 0.001;  // Integral constant
    public static double kD = 0.0005;

    public static double kP_heading = 0.04;
    public static double kD_heading = 0.002;
    public static boolean  motion_profile = true;
    public static double MAX_VEL = 5;
    public static double MAX_ACCEL = 0.8;
    public static double MAX_DECEL = -0.2;
    public static double kF = -0.05;// Feedforward constant
    double previousError = 0;
    double integral = 0;
    private boolean fast_drive = false;
    private double target = 0;

    private boolean update_distance = true;

    private boolean front_distance = true;
    public static double a = 0.6;
    private boolean auto_move = false;
    private double previousHeadingError = 0;
    double error;

    double targetHeading = 0;
    //public final Encoder par, perp;
    //private org.firstinspires.ftc.teamcode.Pose2d currentPose = new org.firstinspires.ftc.teamcode.Pose2d(0, 0, 0);
    private Pose2d currentPose = new Pose2d(0,0, 0);
    private static final double HEADING_KP = 0.012;
    private static final double HEADING_KS = 0.8701732592256377;
    private static final double DRIVE_KP = 0.008;   //Power per inch of error
    private static final double STRAFE_KP = 0.11;
    private static final double MAX_TURN_POWER = 0.85;
    private int prevParallelTicks = 0; //Tracks Forward/Backward
    private int prevPerpTicks = 0;     //Tracks Strafing
    private double prevImuHeadingRad = 0.0; //Tracks Heading
    private static final double TICKS_PER_INCH = 355.15835;
    private static final double TRACK_WIDTH_INCHES = 10.81983;
    private static final double DISTANCE_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE = 2.0;
    private double lockTargetLF, lockTargetLR, lockTargetRF, lockTargetRR;
    private boolean wasLocked = false;
    private double kP_DRIVE_LOCK = 0.005;
    private ElapsedTime timer;
    public MecanumDrive drive;
    public DecodeCAM camera;

    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose){
        frontLeft = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.fl);
        frontRight = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.fr);
        backLeft = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.bl);
        backRight = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.br);

        //distance = new Distance(hardwareMap, telemetry, RobotConstants.distance);
//        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance);
        imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        //trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.init();
        backRight.init();

        timer = new ElapsedTime();
        this.telemetry = telemetry;

        drive = new MecanumDrive(hardwareMap, startPose);
        imu.get().resetYaw();
    }
    public void overrideDriveIfShooting(boolean isShooting) {
        drive.updateLock(isShooting);
    }

    public void updateAutoAlign() {
        updatePoseEstimate();
        telemetry.addData("Deadwheel Pose: ", "%.2f, %.2f", drive.pose.position.x, drive.pose.position.y);
        /*int currentParallelTicks = backLeft.getCurrentPosition();
        int currentPerpTicks = backRight.getCurrentPosition();
        double currentImuHeadingRad = imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        int deltaParallelTicks = currentParallelTicks - prevParallelTicks;
        int deltaPerpTicks = currentPerpTicks - prevPerpTicks;
        double dHeading = AngleUnit.normalizeRadians(currentImuHeadingRad - prevImuHeadingRad);

        prevParallelTicks = currentParallelTicks;
        prevPerpTicks = currentPerpTicks;
        prevImuHeadingRad = currentImuHeadingRad;

        double dY_local = (double)deltaParallelTicks / TICKS_PER_INCH; // Forward/Backward travel
        double dX_local = (double)deltaPerpTicks / TICKS_PER_INCH;     // Strafing travel
        double currentHeading = currentPose.getHeading();
        double theta = currentHeading + (dHeading / 2.0);

        double dX_field = dX_local * Math.cos(theta) - dY_local * Math.sin(theta);
        double dY_field = dX_local * Math.sin(theta) + dY_local * Math.cos(theta);

        this.currentPose = new Pose2d(
                currentPose.getX() + dX_field,
                currentPose.getY() + dY_field,
                currentPose.getHeading() + dHeading // IMU-driven heading update
        );*/
    }

    public boolean holdPosition(Pose2d targetPose) {
        updatePoseEstimate();
        Pose2d currentPose = getPoseEstimate();

        // 1. Calculate Errors
        double errorX = targetPose.position.x - currentPose.position.x;
        double errorY = targetPose.position.y - currentPose.position.y;
        double errorH = targetPose.heading.toDouble() - currentPose.heading.toDouble();

        // Fix Angle Wrap (-180 to 180)
        while (errorH > Math.PI) errorH -= 2 * Math.PI;
        while (errorH <= -Math.PI) errorH += 2 * Math.PI;

        // 2. The Deadzone (Ignore Flywheel Vibration)
        // If error is tiny, pretend it is zero so motors don't jitter
        if (Math.abs(errorX) < 1.0) errorX = 0;
        if (Math.abs(errorY) < 1.0) errorY = 0;
        if (Math.abs(errorH) < Math.toRadians(2.0)) errorH = 0;

        //Calculate Power (PID)
        // Tune these if the robot shakes (too high) or drifts (too low)
        double xPower = errorX * 0.12;
        double yPower = errorY * 0.12;
        double hPower = errorH * 1.5;

        // 4. Field Centric Rotation
        double botHeading = currentPose.heading.toDouble();
        double rotX = xPower * Math.cos(-botHeading) - yPower * Math.sin(-botHeading);
        double rotY = xPower * Math.sin(-botHeading) + yPower * Math.cos(-botHeading);

        // 5. Apply Power
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), hPower));

        double velocity = getRobotVelocity(); // Ensure you have this helper method
        boolean isPositionCorrect = (errorX == 0 && errorY == 0 && errorH == 0);

        return (velocity < 2.0 && isPositionCorrect);
    }

    public void releaseLock() {

        // xIntegralSum = 0;
        // yIntegralSum = 0;
        // hIntegralSum = 0;
    }

    /**
     * Helper to get speed (hypotenuse of x and y velocity)
     */

    public void setPoseEstimate(Pose2d newPose) {
        drive.pose = newPose;
        /* this.currentPose = newPose;

        imu.get().resetYaw();
        prevImuHeadingRad = 0.0;*/
    }
    public Pose2d getPoseEstimate() {
        return drive.pose;
    }
    private PoseVelocity2d currentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public double getRobotVelocity() {
        if (currentVelocity == null || currentVelocity.linearVel == null) return 0.0;

        return currentVelocity.linearVel.norm();
    }

    public void driveFieldCentric(double strafe, double forward, double turn, double currentHeading) {
        double heading = drive.pose.heading.toDouble();

        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotY, rotX),
                turn
        ));
        /*double inputPower = Math.hypot(strafe, forward);
        double inputAngle = Math.atan2(forward, strafe);

        double headingRad = Math.toRadians(currentHeading);
        double relativeAngle = inputAngle - headingRad;

        double rotatedStrafe = inputPower * Math.cos(relativeAngle);
        double rotatedForward = inputPower * Math.sin(relativeAngle);

        double lfPower = rotatedForward + rotatedStrafe + turn;
        double rfPower = rotatedForward - rotatedStrafe - turn;
        double lbPower = rotatedForward - rotatedStrafe + turn;
        double rbPower = rotatedForward + rotatedStrafe - turn;

        double maxPower = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        maxPower = Math.max(maxPower, Math.max(Math.abs(lbPower), Math.abs(rbPower)));

        if (maxPower > 1.0) {
            lfPower /= maxPower; rfPower /= maxPower;
            lbPower /= maxPower; rbPower /= maxPower;
        }

        frontLeft.setPower(lfPower); frontRight.setPower(rfPower);
        backLeft.setPower(lbPower); backRight.setPower(rbPower);

        double headingInRadians = Math.toRadians(currentHeading);
        double rotatedStrafe = strafe * Math.cos(headingInRadians) - forward * Math.sin(headingInRadians);
        double rotatedForward = strafe * Math.sin(headingInRadians) + forward * Math.cos(headingInRadians);

        double lfPower = rotatedForward + rotatedStrafe + turn;
        double rfPower = rotatedForward - rotatedStrafe - turn;
        double lbPower = rotatedForward - rotatedStrafe + turn;
        double rbPower = rotatedForward + rotatedStrafe - turn;

        double maxPower = Math.abs(lfPower);
        maxPower = Math.max(maxPower, Math.abs(rfPower));
        maxPower = Math.max(maxPower, Math.abs(lbPower));
        maxPower = Math.max(maxPower, Math.abs(rbPower));

        if (maxPower > 1.0) {
            lfPower /= maxPower;
            rfPower /= maxPower;
            lbPower /= maxPower;
            rbPower /= maxPower;
        }

        frontLeft.setPower(lfPower);
        frontRight.setPower(rfPower);
        backLeft.setPower(lbPower);
        backRight.setPower(rbPower);
        */
    }
    public double calculateAutoTurnPower(double targetHeading, double currentHeading) {
        double error = targetHeading - currentHeading;
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;

        double turnPower = error * 0.015; //Multiply error by kP
        return Math.max(-MAX_TURN_POWER, Math.min(turnPower, MAX_TURN_POWER));

        /*double headingError = AngleUnit.DEGREES.normalize(targetHeading - currentHeading);
        double turnPower = headingError * HEADING_KP;
        return Math.max(-MAX_TURN_POWER, Math.min(turnPower, MAX_TURN_POWER));*/
    }

    public void driveRobotCentric(double strafe, double forward, double turn) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forward, strafe),
                turn
        ));
        /*double lfPower = forward + strafe + turn;
        double rfPower = forward - strafe - turn;
        double lbPower = forward - strafe + turn;
        double rbPower = forward + strafe - turn;

        // Normalize speeds
        double maxPower = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        maxPower = Math.max(maxPower, Math.max(Math.abs(lbPower), Math.abs(rbPower)));

        if (maxPower > 1.0) {
            lfPower /= maxPower; rfPower /= maxPower;
            lbPower /= maxPower; rbPower /= maxPower;
        }

        frontLeft.setPower(lfPower); frontRight.setPower(rfPower);
        backLeft.setPower(lbPower); backRight.setPower(rbPower);*/
    }

    public void moveToAngleBlocking(LinearOpMode opMode, Pose2d pose, double targetHeadingDeg, double angVel, double timeoutSec, boolean manualOverride) {
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        Action moveAction = drive.actionBuilder(pose)
                .turnTo(targetHeadingRad, new TurnConstraints(angVel, 60, 80))
                .build();

        ElapsedTime moveTimer = new ElapsedTime();

        boolean active = true;

        com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();

        while (active && opMode.opModeIsActive() && moveTimer.seconds() < timeoutSec && !manualOverride) {
            active = moveAction.run(packet);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
    public void moveToPositionBlocking(LinearOpMode opMode, Pose2d pose, double targetX, double targetY, double targetHeadingDeg, double transVel, double timeoutSec, boolean manualOverride) {
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        Action moveAction = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(targetX, targetY), targetHeadingRad, new TranslationalVelConstraint(transVel))
                .build();

        ElapsedTime moveTimer = new ElapsedTime();

        boolean active = true;

        com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();

        while (active && opMode.opModeIsActive() && moveTimer.seconds() < timeoutSec && !manualOverride) {
            active = moveAction.run(packet);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        /*ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec && !manualOverride) {
            updateAutoAlign();
            Pose2d pose = getPoseEstimate();

            //Field-Centric Error
            double xError = targetX - pose.getX();
            double yError = targetY - pose.getY();
            double headingError = targetHeadingDeg - Math.toDegrees(pose.getHeading());

            // Normalized Heading Error (-180 - 180)
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            double distanceError = Math.hypot(xError, yError);
            if (distanceError < DISTANCE_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
                break;
            }

            //Rotated Robot-Centric Error
            double currentHeadingRad = pose.getHeading();
            double forward = xError * Math.cos(-currentHeadingRad) - yError * Math.sin(-currentHeadingRad);
            double strafe = xError * Math.sin(-currentHeadingRad) + yError * Math.cos(-currentHeadingRad);

            //P-Control
            double forwardPower = forward * DRIVE_KP;
            double strafePower = strafe * STRAFE_KP;
            double turnPower = headingError * HEADING_KP;

            //Normalized and Clipped Powers
            double driveScale = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (driveScale > maxSpeed) {
                forwardPower = (forwardPower / driveScale) * maxSpeed;
                strafePower = (strafePower / driveScale) * maxSpeed;
            }

            double lf = forwardPower + strafePower + turnPower;
            double rf = forwardPower - strafePower - turnPower;
            double lb = forwardPower - strafePower + turnPower;
            double rb = forwardPower + strafePower - turnPower;

            double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));
            if (max > 1.0) {
                lf /= max; rf /= max; lb /= max; rb /= max;
            }
            driveRobotCentric(strafePower, forwardPower, turnPower);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);*/
    }



    public Rotation2d getCurrentHeading() {
        return drive.pose.heading;
    }

    public boolean isFrontDistance(){
        return front_distance;
    }

    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        //this(hardwareMap, telemetry);
        this.timer = timer;
    }
    public void setDistanceType(boolean front){
        this.front_distance = front;
    }

    public void mountFrontDistance(Distance distance){
        update_distance = false;
        this.distance = distance;
    }
    public void mountRearDistance(Distance distance){
        update_distance = false;
        this.rear_distance = distance;
    }
    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    private double getHeading() {
        return imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double xRotated = x * cosA - y * sinA;
        double yRotated = x * sinA + y * cosA;
        return new double[] {xRotated, yRotated};
    }
   public void forceExit(){
        force_exit = true;
   }
   void set_individual_powers(double fl_power, double fr_power, double bl_power, double br_power){
        frontRight.setDrivePower(fr_power);
        frontLeft.setDrivePower(fl_power);
        backLeft.setDrivePower(bl_power);
        backRight.setDrivePower(br_power);
    }
    public void setUseDeadwheel(boolean on){
        use_dead_wheel = on;
    }
    public double getDistance(){

        if(front_distance){
            return distance.getFilteredDist();
        }
        return rear_distance.getFilteredDist();
    }
    public double getLinearDeadwheel(){
        return (backLeft.getCurrentPosition() * PARAMS.inPerTick);
    }
    public void clearLinearDeadwheel(){
        backLeft.resetEncoder();
    }
    public void setDrivePowers(double left_stick_x, double left_stick_y, double right_stick_x) {
        double heading = getHeading();  // Get robot heading

        // Rotate joystick inputs for field-centric control
        double[] rotated = rotateVector(left_stick_x, left_stick_y, -heading);
        double x = rotated[0];
        double y = rotated[1];

        // Mecanum drive equations
        double frontLeftPower = y + x + right_stick_x;
        double frontRightPower = y - x - right_stick_x;
        double backLeftPower = y - x + right_stick_x;
        double backRightPower = y + x - right_stick_x;

        // Normalize powers so that no value exceeds 1
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Apply the calculated power values to the motors
        frontLeft.setDrivePower(frontLeftPower * MAX_DRIVE_SPEED);
        frontRight.setDrivePower(frontRightPower * MAX_DRIVE_SPEED);
        backLeft.setDrivePower(backLeftPower * MAX_DRIVE_SPEED);
        backRight.setDrivePower(backRightPower * MAX_DRIVE_SPEED);
    }

    public void setDrivePowers(double left_stick_y, double right_stick_y, double left_trigger, double right_trigger){
//        if(auto_move){
//            return;
////            if(left_stick_y == 0 && right_stick_y == 0 && left_trigger == 0 && right_trigger == 0){
////                return;
////            }

        left_strafe = left_trigger != 0;
        right_strafe = right_trigger != 0;
        if (left_strafe) {
            double posPower = sameSignSqrt(left_trigger);
            double negPower = sameSignSqrt(-left_trigger);
            frontLeft.setDrivePower(negPower * MAX_DRIVE_SPEED);
            backRight.setDrivePower(negPower * MAX_DRIVE_SPEED);
            frontRight.setDrivePower(posPower * MAX_DRIVE_SPEED);
            backLeft.setDrivePower(posPower * MAX_DRIVE_SPEED);
            return;
        }
        if (right_strafe) {
            double posPower = sameSignSqrt(right_trigger);
            double negPower = sameSignSqrt(-right_trigger);
            frontLeft.setDrivePower(posPower * MAX_DRIVE_SPEED);
            backRight.setDrivePower(posPower * MAX_DRIVE_SPEED);
            frontRight.setDrivePower(negPower * MAX_DRIVE_SPEED);
            backLeft.setDrivePower(negPower * MAX_DRIVE_SPEED);
            return;
        }

        double leftPower = sameSignSqrt(-left_stick_y);
        double rightPower = sameSignSqrt(-right_stick_y);
        frontLeft.setDrivePower(leftPower * MAX_DRIVE_SPEED);
        backLeft.setDrivePower(leftPower * MAX_DRIVE_SPEED);
        frontRight.setDrivePower(rightPower * MAX_DRIVE_SPEED);
        backRight.setDrivePower(rightPower * MAX_DRIVE_SPEED);
    }
    public void LivePIDToDistance(double distance){
        if (target != distance){
            previousError = 0;
            integral = 0;
            distance_to_target = distance - getDistance();
            starting_motion_profile_time = timer.time();
            starting_pos = getDistance();
            if (use_dead_wheel && !front_distance){
                distance_to_target *= -1;
            }
            clearLinearDeadwheel();
            motion_profile = !(Math.abs(distance_to_target) < 2);
        }
        fast_drive = false;
        auto_move = true;
        imu.get().resetYaw();
        targetHeading = getHeading();
        target = distance;
    }
    public void PIDToDistance(double distance){
        if (target != distance || !auto_move){
            previousError = 0;
            integral = 0;
            distance_to_target = distance - getDistance();
            starting_motion_profile_time = timer.seconds();
            starting_pos = getDistance();
            auto_move = true;
            fast_drive = false;
            if (use_dead_wheel && !front_distance){
                distance_to_target *= -1;
            }
            clearLinearDeadwheel();
            motion_profile = !(Math.abs(distance_to_target) < 2);
            imu.get().resetYaw();
            targetHeading = getHeading();
            target = distance;
        }
    }
    public void DrivePastDistance(double distance, double speed){
        if(target != distance) {
            fast_drive = true;
            auto_move = true;
            target = distance;
            fast_drive_direction = front_distance ? (getDistance() > distance) : (getDistance() < distance);
            fast_drive_speed = fast_drive_direction ? speed : -speed;

        }

    }



    @Override
    public void update() {
//        if(update_distance) {
//            distance.update();
//            rear_distance.update();
//        }
        trafficLight.update();


        if(force_exit){
            auto_move = false;
            force_exit = false;
            frontLeft.setDrivePower(0);
            backLeft.setDrivePower(0);
            backRight.setDrivePower(0);
            frontRight.setDrivePower(0);
        }
        // Declare variables outside of the loop for PID

        if (auto_move) {
            double currentHeading = getHeading();// Get current heading (yaw)
            telemetry.addData("Auto Drive Direction", fast_drive_direction);
            telemetry.addData("Auto Drive Front Distance", front_distance);
            telemetry.addData("Auto Drive Reading Distance Value", getDistance());
            telemetry.addData("Auto Drive Target", target);
            telemetry.addData("Auto Drive Speed", fast_drive_speed);
            telemetry.addData("Auto Drive Target Heading", targetHeading);
            telemetry.addData("Auto Drive Current Heading", currentHeading);
            if (fast_drive) {



                boolean completed = fast_drive_direction ? (front_distance ? (getDistance() < target) : (getDistance() > target)) : (front_distance ? (getDistance() > target) : (getDistance() < target));

                if (completed) {
                    fast_drive = false;
                    auto_move = false;
                    frontLeft.setDrivePower(0);
                    backLeft.setDrivePower(0);
                    backRight.setDrivePower(0);
                    frontRight.setDrivePower(0);
                    trafficLight.red(false);
                    trafficLight.green(true);
                    trafficLight.flashGreen(1, 1);
                } else {
                    trafficLight.red(true);
                    frontLeft.setDrivePower(fast_drive_speed );
                    frontRight.setDrivePower(fast_drive_speed );
                    backLeft.setDrivePower(fast_drive_speed );
                    backRight.setDrivePower(fast_drive_speed );
                }

            } else {
                if(use_dead_wheel) {
                    if(motion_profile){
                        error = Control.motionProfile(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance_to_target, timer.seconds() - starting_motion_profile_time) - getLinearDeadwheel();
                    }else{
                        error = distance_to_target - getLinearDeadwheel();
                    }
                }else {
                    if (motion_profile) {
                        error = (Control.motionProfile(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance_to_target, timer.seconds() - starting_motion_profile_time) + starting_pos) - getDistance();
                    } else {
                        error = target - getDistance();
                    }
                    error *= front_distance ? -1 : 1;
                }
                telemetry.addData("Auto Drive Error", error);

                double time_passed = timer.seconds() - time_stop;

                if (Math.abs(getDistance() - target) < 0.5) {
                    frontLeft.setDrivePower(0);
                    backLeft.setDrivePower(0);
                    backRight.setDrivePower(0);
                    frontRight.setDrivePower(0);
                    auto_move = false;
                    trafficLight.red(false);
                    trafficLight.green(true);
                    trafficLight.flashGreen(1, 1);

                    previousError = 0;  // Reset for future runs
                    integral = 0;       // Reset for future runs
                } else {
                    trafficLight.red(true);

                    double errorChange = (error - previousError);
                    currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
                    previousFilterEstimate = currentFilterEstimate;

                    integral += error * time_passed;
                    double derivative = currentFilterEstimate / time_passed;
                    double power = kP * error + kI * integral + kD * derivative;

                    power = Math.max(-0.5, Math.min(0.5, power));
                    power -= kF * Math.signum(error);

                    // Calculate heading correction
                    double headingError = currentHeading - targetHeading;
                    double headingCorrection = kP_heading * headingError +
                            kD_heading * (headingError - previousHeadingError) / time_passed;
                    previousHeadingError = headingError;

                    // Adjust motor powers for heading correction
                    frontLeft.setDrivePower(power + headingCorrection);
                    backLeft.setDrivePower(power + headingCorrection);
                    backRight.setDrivePower(power - headingCorrection);
                    frontRight.setDrivePower(power - headingCorrection);

                    previousError = error;
                    time_stop = timer.seconds();
                }
            }
        }

    }
    public void turnToAngle(double targetAngle, LinearOpMode opMode) {
        double currentAngle = getHeading();
        double angleDifference = getAngleDifference(targetAngle, currentAngle);

        // Proportional gain (tune this value)
        double kP = 0.014;
        double turnPower;

        // Use a loop to keep turning until the target is reached
        while (opMode.opModeIsActive() && Math.abs(angleDifference) > 1.0) { // Tolerance of 1 degree
            turnPower = angleDifference * kP;

            // Limit the power for controlled turning
            turnPower = Math.max(-0.8, Math.min(0.8, turnPower));

            // Apply power to motors for turning (tank turn in place)
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);

            currentAngle = getHeading();
            angleDifference = getAngleDifference(targetAngle, currentAngle);

            // Optional: Add telemetry in your OpMode to tune kP
            // opMode.telemetry.addData("Angle Difference", angleDifference);
            // opMode.telemetry.update();
            telemetry.addData("heading: ", getHeading());
        }

        // Stop motors after reaching the target
        stopMotors();
    }


    // Helper method to calculate the shortest angle difference and normalize it
    private double getAngleDifference(double target, double current) {
        double difference = target - current;
        // Normalize angle to be between -180 and 180 degrees
        while (difference > 180) difference -= 360;
        while (difference <= -180) difference += 360;
        return difference;
    }

    // Helper method to stop all motors
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public boolean isBusy(){
        return auto_move;
    }
    @Override
    public void telemetry() {
        //trafficLight.telemetry();;
//        distance.telemetry();
//        rear_distance.telemetry();
        telemetry.addData("Robot Heading", getHeading());
        telemetry.addData("Front Right Motor Power", frontRight.getPower());
        telemetry.addData("Front Left Motor Power", frontLeft.getPower());
        telemetry.addData("Back Right Motor Power", backRight.getPower());
        telemetry.addData("Back Left Motor Power", backLeft.getPower());
        telemetry.addData("MecaTank Perceived Distance", getDistance());

    }

    @Override
    public void init() {

    }

    public void setMaxPower(double v) {
        MAX_DRIVE_SPEED = v;
    }
}