package org.firstinspires.ftc.teamcode.roadrunner;

import static com.acmerobotics.roadrunner.Curves.project;
import static org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank.MAX_DECEL;
import static org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank.kI;
import static org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank.kP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Control;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Distance;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.TrafficLight;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public final class MecanumDrive {
    public Pose2d lockedPose = null;
    public static double LOCK_XY_GAIN = 0.08;
    public static double LOCK_HEADING_GAIN = 0.7;

    public static class Params {
        
        // IMU orientation
       public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // drive model parameters
        public double inPerTick = 0.00303299391;
        public double lateralInPerTick = 0.0023898724154397585;
        public double trackWidthTicks = 3990.098938918289;

        // feedforward parameters (in tick units)
        public double kS = 0.7633453734715174;
        public double kV = 0.0004977751093390224;
        public double kA = 0.0001;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 4;//8;
        public double lateralGain = 4;//7.3;
        public double headingGain = 6;//6; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn


    }
    public static Params PARAMS = new Params();
    public void updateLock(boolean isShooting) {
        // 1. If we are NOT shooting, just reset and do nothing
        if (!isShooting) {
            lockedPose = null;
            return; // Let TeleOp handle driving
        }

        // 2. If this is the FIRST frame of shooting, lock the current position
        if (lockedPose == null) {
            lockedPose = pose; // 'pose' is the variable RR updates automatically
        }

        // 3. Calculate the error (Where we are vs Where we want to be)
        // Field-centric difference
        Vector2d positionError = lockedPose.position.minus(pose.position);

        // 4. Rotate that error into the ROBOT'S perspective
        // (If the error is "North" but we are facing "East", we need to drive "Left")
        Vector2d robotRelativeError = pose.heading.inverse().times(positionError);

        // 5. Calculate Heading Error
        // .log() converts the Rotation2d difference into a number (radians)
        double targetHeading = lockedPose.heading.toDouble();
        double currentHeading = pose.heading.toDouble();

// 2. Subtract
        double headingError = targetHeading - currentHeading;

// 3. Normalize (Handle the -180 to 180 wrap manually)
        while (headingError > Math.PI)  headingError -= 2 * Math.PI;
        while (headingError <= -Math.PI) headingError += 2 * Math.PI;

        // 6. Apply the P-Controller Gains
        // We send this to setDrivePowers, which takes (Forward, Strafe, Turn)
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        robotRelativeError.x * LOCK_XY_GAIN,
                        robotRelativeError.y * LOCK_XY_GAIN
                ),
                headingError * LOCK_HEADING_GAIN
        ));
    }
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx frontLeft, backLeft, backRight, frontRight;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final TwoDeadWheelLocalizer localizer;
    public Pose2d pose;

    public static double kF = -0.05;// Feedforward constant
    double previousError = 0;
    double integral = 0;
    private boolean fast_drive = false;
    private double target = 0;

    private boolean update_distance = true;

    private boolean front_distance = true;
    public static double a = 0.6;
    private boolean auto_move = false;
    private boolean use_dead_wheel = false;
    double error;

    private boolean fast_drive_direction = false;
    private boolean force_exit = false;


    private double currentFilterEstimate = 0, previousFilterEstimate = 0;
    private double distance_to_target = 0;
    private double starting_motion_profile_time = 0;

    private double fast_drive_speed = 0;
    private double starting_pos = 0;
    private double time_stop = 0;

    private boolean use_motion_profile = false;

    Distance distance, rear_distance;
    ElapsedTime timer;

    Telemetry telemetry;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    TrafficLight trafficLight;
    public class DriveLocalizer implements Localizer {
        public final Encoder frontLeft, backLeft, backRight, frontRight;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            frontLeft = new OverflowEncoder(new RawEncoder(MecanumDrive.this.frontLeft));
            backLeft = new OverflowEncoder(new RawEncoder(MecanumDrive.this.backLeft));
            backRight = new OverflowEncoder(new RawEncoder(MecanumDrive.this.backRight));
            frontRight = new OverflowEncoder(new RawEncoder(MecanumDrive.this.frontRight));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed

        }


        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = frontLeft.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = backLeft.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = backRight.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = frontRight.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public class WaitForStationary implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !localizer.isStationary();
        }
    }
    public Action isStationary(){
        return new WaitForStationary();
    }
    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        frontLeft = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.fl);
        backLeft = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.bl);
        backRight = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.br);
        frontRight = hardwareMap.get(DcMotorEx.class, DECODERobotConstants.fr);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        DECODERobotConstants.imu = lazyImu;
        DECODERobotConstants.imu_init = true;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), PARAMS.inPerTick);
        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }
    public void mountTrafficLight(TrafficLight light){
        this.trafficLight = light;
    }
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeft.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeft.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRight.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRight.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }





    public Action moveUsingDistance(Distance distance){
        return new MoveUsingDistanceAction(distance);
    }
    public Action moveUsingDistance(Distance distance, double target, double TOO_CLOSE, double TOO_FAR){
        return new MoveUsingDistanceAction(distance, target, TOO_CLOSE, TOO_FAR);
    }
    public Action moveUsingDistance(Distance distance, double target, double TOO_CLOSE, double TOO_FAR, double GIVE_UP){
        return new MoveUsingDistanceAction(distance, target, TOO_CLOSE, TOO_FAR, GIVE_UP);
    }    public Action moveUsingDistance(Distance distance, double target, double TOO_CLOSE, double TOO_FAR, boolean front_distance){
        return new MoveUsingDistanceAction(distance, target, TOO_CLOSE, TOO_FAR, front_distance);
    }
    public Action moveUsingDistance(Distance distance, double target, double speed, boolean front){
        return new MoveUsingDistanceAction(distance, target, speed, front);
    }

    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }




    public class MoveUsingDistanceAction implements Action {

        private Distance distance;
        private double TOO_FAR = RobotConstants.TOO_FAR;
        private double TOO_CLOSE = RobotConstants.TOO_CLOSE;

        private ElapsedTime timer;
        private double currentTime = 0;
        private double target = 0;

        private boolean first = true;

        private double distance_to_target = 0;
        private double start_position = 0;
        private double starting_motion_profile_time = 0;

        private double time_stop = 0;

        private double integral = 0;

        private double GIVE_UP = 100;
        private double SPEED = 0;
        private boolean fast = false;
        private boolean front_distance = true;

        private boolean use_motion_profile = false;
        private boolean fast_drive_direction;

        private double power = 0;

        public MoveUsingDistanceAction(Distance distance){
            this.distance = distance;
            target = RobotConstants.TARGET;  // Midpoint distance

            timer = new ElapsedTime();
            currentTime = timer.time(TimeUnit.SECONDS);


        }
        public MoveUsingDistanceAction(Distance distance, double target, double TOO_CLOSE, double TOO_FAR){
            this.distance = distance;
            this.target = target;
            timer = new ElapsedTime();
            fast = false;
            currentTime = timer.time(TimeUnit.SECONDS);
            this.TOO_CLOSE = TOO_CLOSE;
            this.TOO_FAR = TOO_FAR;
        }
        public MoveUsingDistanceAction(Distance distance, double target, double TOO_CLOSE, double TOO_FAR, double GIVE_UP){
            this.distance = distance;
            this.target = target;
            timer = new ElapsedTime();
            currentTime = timer.time(TimeUnit.SECONDS);
            this.TOO_CLOSE = TOO_CLOSE;
            fast = false;
            this.TOO_FAR = TOO_FAR;
            this.GIVE_UP = GIVE_UP;
        }
        public MoveUsingDistanceAction(Distance distance, double target, double TOO_CLOSE, double TOO_FAR, boolean front_distance){
            this.distance = distance;
            this.target = target;
            timer = new ElapsedTime();
            currentTime = timer.time(TimeUnit.SECONDS);
            this.TOO_CLOSE = TOO_CLOSE;
            fast = false;
            this.front_distance = front_distance;
            this.TOO_FAR = TOO_FAR;
        }
        public MoveUsingDistanceAction(Distance distance, double target, double speed, boolean front_distance){
            this.distance = distance;
            this.target = target;
            timer = new ElapsedTime();
            fast = true;
            currentTime = timer.seconds();
            this.TOO_CLOSE = 0;
            this.TOO_FAR = 0;
            this.front_distance = front_distance;
            fast_drive_direction = front_distance ? (distance.getFilteredDist() > target) : (distance.getFilteredDist() < target);
            this.SPEED = fast_drive_direction ? speed : -speed;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                timer.reset();
                first = false;
                distance.setOn(true);
                currentTime = timer.seconds();
                distance_to_target = target - distance.getFilteredDist();
                use_motion_profile = !(Math.abs(distance_to_target) < 2);
                starting_motion_profile_time = timer.seconds();
                start_position = distance.getFilteredDist();
                time_stop = timer.seconds();
                if(distance.getDist() > GIVE_UP){
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    trafficLight.flashOffred(0.5, 2);

                    return false;
                }

            }

            updatePoseEstimate();
            if(fast){
                boolean completed = fast_drive_direction ? (front_distance ? ( distance.getFilteredDist() < target) : (distance.getFilteredDist() > target)) : (front_distance ? (distance.getFilteredDist() > target) : (distance.getFilteredDist() < target));
                if(completed){
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    trafficLight.red(false);
                    trafficLight.green(true);
                    trafficLight.flashGreen(1, 1);
                    distance.setOn(false);
                    return false;
                }
                trafficLight.red(true);
                frontLeft.setPower(SPEED);
                frontRight.setPower(SPEED);
                backLeft.setPower(SPEED);
                backRight.setPower(SPEED);


            }
            double time_passed = timer.seconds() - time_stop;
            double error;
            if (use_motion_profile) {
                error = (Control.motionProfile(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance_to_target, timer.seconds() - starting_motion_profile_time) + starting_pos) - distance.getFilteredDist();
            } else {
                error = target - distance.getFilteredDist();
            }
            error *= front_distance ? -1 : 1;
            double real_error = distance.getFilteredDist() - target;
            telemetryPacket.put("Distance", distance.getDist());
            telemetryPacket.put("Error", error);
            telemetryPacket.put("Time", timer.time(TimeUnit.SECONDS) - currentTime);
            telemetryPacket.put("Active", !(timer.time(TimeUnit.SECONDS) - currentTime > 0.2 && (Math.abs(error) < 0.5)));

            if (timer.time(TimeUnit.SECONDS) - currentTime > 1 || (Math.abs(power) < 0.3 &&
                    (Math.abs(real_error) < 0.5 || (distance.getDist() > TOO_CLOSE && distance.getDist() < TOO_FAR))
            )) {

                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);
                trafficLight.red(false);
                trafficLight.green(true);
                trafficLight.flashGreen(1, 1);
                return false;
            }

            trafficLight.red(true);

// Apply proportional control based on error
            // filter out hight frequency noise to increase derivative performance
            integral += error * time_passed;  // Accumulate the error over time
            // PID output
            power = kP * error + kI * integral;

            // Limit power to a safe range (optional, depending on your motor controller)
            power = Math.max(-1.0, Math.min(1.0, power));
            power -= kF * Math.signum(power);
// Set motor power proportionally to the error
            frontLeft.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
            frontRight.setPower(power);

            time_stop = timer.seconds();


            return true;

        }
    }
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;
        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration ) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            Pose2d error = txWorldTarget.value().minusExp(pose);

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
            frontRight.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);

//            if(Math.abs(error.position.x) > ERROR_THRESHOLD || Math.abs(error.position.y) > ERROR_THRESHOLD){
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//                return false;
//            }


            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }
    public final class FollowTrajectoryActionFast implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;
        public FollowTrajectoryActionFast(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            Pose2d error = txWorldTarget.value().minusExp(pose);
            if (t >= timeTrajectory.duration - 0.25) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
            frontRight.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);

//            if(Math.abs(error.position.x) > ERROR_THRESHOLD || Math.abs(error.position.y) > ERROR_THRESHOLD){
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//                return false;
//            }


            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }
    public final class FollowTrajectoryActionPP implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;
        private double disp = 0;
        private List<Double> disps, vel, accel;
        public FollowTrajectoryActionPP(TimeTrajectory t) {
            timeTrajectory = t;

            disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));


            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
                vel.add(t.path.get(disps.get(i), 2).position.norm().value());
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration ) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }
            disp = project(timeTrajectory.path, pose.position, disp);
            Pose2dDual<Arclength> temp = timeTrajectory.path.get(disp, 3);

            DualNum<Time> arclength = new DisplacementProfile(disps, vel, accel).get(disp);
            Pose2dDual<Time> txWorldTarget = temp.reparam(arclength);

            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            Pose2d error = txWorldTarget.value().minusExp(pose);

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
            frontRight.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);

//            if(Math.abs(error.position.x) > ERROR_THRESHOLD || Math.abs(error.position.y) > ERROR_THRESHOLD){
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//                return false;
//            }


            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeft.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            backLeft.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            backRight.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            frontRight.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
    /**
     * Per-update pose history + FlightRecorder logging. Keep true for autos and for any RoadRunner
     * tuning opmode (the tuners read these logs). Teleop can turn it off: the pose history is only
     * consumed by drawPoseHistory() during trajectory actions, and the PoseMessage allocation
     * happens on every update even though DownsampledWriter only writes every 50 ms.
     */
    public static boolean LOG_POSE_HISTORY = true;

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());
        DECODERobotConstants.pose = pose;

        if (LOG_POSE_HISTORY) {
            poseHistory.add(pose);
            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }

            estimatedPoseWriter.write(new PoseMessage(pose));
        }

        return twist.velocity().value();
    }
    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
    public TrajectoryActionBuilder fastActionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryActionFast::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public TrajectoryActionBuilder ppActionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryActionPP::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}