package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

@Config
@Autonomous
public class Biobuzz_Left_PreLoad_Auto extends NGAutoOpMode{

    public static double SHOOT_AT_S = 1;

    @Override
    public void runOpMode() throws InterruptedException{

        double TRACK_WIDTH = MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks;
        double LATERAL_MULT = MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick;
        double MAX_WHEEL_VEL = 64;
        double MAX_PATH_ACCEL = 100;
        double MAX_WHEEL_ACCEL = 120;
        double MAX_LATERAL_ACCEL = 150;
        MecanumKinematics robotKinematics = new MecanumKinematics(TRACK_WIDTH, LATERAL_MULT);

        VelConstraint cornerVel = (robotPose, posePath, disp) -> {
            double curvature = posePath.get(disp, 3).position.drop(2).value().norm();
            return curvature < 1e-6 ? 1e9 : Math.sqrt(MAX_LATERAL_ACCEL / curvature);
        };
        VelConstraint turnRateVel = (robotPose, posePath, disp) -> {
            Rotation2dDual<Arclength> heading = posePath.get(disp, 3).heading;
            double headingCurvature = Math.abs(heading.real.get(0) * heading.imag.get(2) - heading.imag.get(0) * heading.real.get(2));
            return headingCurvature < 1e-9 ? 1e9 : Math.sqrt(MAX_WHEEL_ACCEL / (headingCurvature * TRACK_WIDTH));
        };
        VelConstraint maxVel = new MinVelConstraint(Arrays.asList(
                robotKinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                cornerVel,
                turnRateVel
        ));

        AccelConstraint maxAccel = (robotPose, posePath, disp) -> {
            Vector2d travel = robotPose.heading.value().inverse().times(robotPose.position.drop(1).value());
            double turnPerInch = Math.abs(robotPose.heading.velocity().value());
            double wheelPerInch = Math.abs(travel.x) + LATERAL_MULT * Math.abs(travel.y) + turnPerInch * TRACK_WIDTH;
            double accel = Math.min(MAX_PATH_ACCEL, MAX_WHEEL_ACCEL / wheelPerInch);
            return new MinMax(-accel, accel);
        };

        Pose2d beginPose = new Pose2d(-65, -11, Math.toRadians(0));
        initAuto(beginPose);

        Pose2d leftShot = new Pose2d(-52, -14, Math.toRadians(0));
        Pose2d parkPose = new Pose2d(-28.5, -60, Math.toRadians(90));

        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, null, telemetry, maxVel, maxAccel,
                parkPose, leftShot, beginPose);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepare(leftShot, parkPose);

        Action toLeftShot = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(leftShot.position, Math.toRadians(-75), maxVel, maxAccel)
                .build();

        Action shootPreLoad = new SequentialAction(
                kit.waitUntilMatchTime(() -> false, SHOOT_AT_S),
                kit.preloadShot());

        Action parkPath = drive.actionBuilder(leftShot)
                .setTangent(Math.toRadians(-86.75))
                .splineToSplineHeading(parkPose, Math.toRadians(-86.75), maxVel, maxAccel)
                .build();

        Action park = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftShot, parkPath, parkPose, null));

        Action main = new SequentialAction(
                toLeftShot,
                shootPreLoad,
                park);

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Paths Built - Ready To Start");
            telemetry.addData("Preload shot at", "%.1f s", SHOOT_AT_S);
            telemetry.update();
        }
        if (isStopRequested()) return;

        kit.start();
        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        intake2_0.updateFlywheelPID(),
                        intake2_0.runShooter(pollenShootingVel, nectarShootingVel, 30),
                        kit.guardPark(main)
                )
        );
    }
}
