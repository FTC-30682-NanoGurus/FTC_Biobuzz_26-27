package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.dashboard.config.Config;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

@Config
@Autonomous
public class Biobuzz_2_Tip_Auto extends NGAutoOpMode{
    public static double WALL_SET_DWELL_S = 0.25;

    @Override
    public void runOpMode() throws InterruptedException{

        VelConstraint intakeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > 49.0) {
                return 15;
            } else {
                return 40;
            }
        };

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
        double INTAKE_ZONE_Y = -42;
        double INTAKE_VEL = 15;
        VelConstraint cornerIntakeVel = (robotPose, _path, _disp) -> {
            boolean inIntakeZone = robotPose.position.x.value() > 50.0 && robotPose.position.y.value() < INTAKE_ZONE_Y;
            return inIntakeZone ? INTAKE_VEL : 60;
        };
        VelConstraint maxIntakeVel = new MinVelConstraint(Arrays.asList(cornerIntakeVel, maxVel));

        AccelConstraint maxAccel = (robotPose, posePath, disp) -> {
            Vector2d travel = robotPose.heading.value().inverse().times(robotPose.position.drop(1).value());
            double turnPerInch = Math.abs(robotPose.heading.velocity().value());
            double wheelPerInch = Math.abs(travel.x) + LATERAL_MULT * Math.abs(travel.y) + turnPerInch * TRACK_WIDTH;
            double accel = Math.min(MAX_PATH_ACCEL, MAX_WHEEL_ACCEL / wheelPerInch);
            return new MinMax(-accel, accel);
        };

        Pose2d beginPose = new Pose2d(61, -14, Math.toRadians(180));
        initAuto(beginPose);

        Pose2d leftShot = new Pose2d(-52, -14, Math.toRadians(0));
        Pose2d leftShotBack = new Pose2d(-52, -14, Math.toRadians(360));
        Pose2d wallSet = new Pose2d(62, -58, Math.toRadians(360));
        Pose2d leftNear = new Pose2d(-53.48, -43.82, Math.toRadians(30));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d parkPose = new Pose2d(-49.5, -58, Math.toRadians(90));

        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, null, telemetry, maxVel, maxAccel,
                parkPose, leftShot, beginPose);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepare(leftNear, flowerOne);
        kit.prepare(leftShotBack, parkPose);
        kit.prepare(leftNear, parkPose);

        Action shootPreLoad = new ParallelAction(
                drive.actionBuilder(beginPose).waitSeconds(2).build(),
                kit.transfer());

        Action IntakeWallSetandShoot = new SequentialAction(
                kit.withWallCollect(drive.actionBuilder(beginPose)
                        .setTangent(Math.toRadians(-155))
                        .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                        .waitSeconds(WALL_SET_DWELL_S)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                        .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                        .splineToSplineHeading(leftNear, Math.toRadians(180), maxVel, maxAccel)
                        .build()),
                kit.transfer());

        Action IntakeFromFlowerOne = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.route(leftNear, flowerOne)),
                new SleepAction(0.4),
                intake2_0.collectFromFlower(3));

        Action shootFlowerOnePollen = new SequentialAction(
                drive.actionBuilder(flowerOne)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                        .splineToSplineHeading(leftShotBack, Math.toRadians(90), maxVel, maxAccel)
                        .build(),
                kit.transfer());

        Action park = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftShotBack, kit.route(leftShotBack, parkPose), parkPose, null));

        Action main = new SequentialAction(
                shootPreLoad,
                IntakeWallSetandShoot,
                IntakeFromFlowerOne,
                shootFlowerOnePollen,
                park);

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;
        while (!isStarted() && !isStopRequested()) {
            AllianceSelector.poll(gamepad1, gamepad2);
            AllianceSelector.show(telemetry);
            telemetry.addLine("Paths Built - Ready To Start");
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
