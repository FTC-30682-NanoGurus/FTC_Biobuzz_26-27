package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
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
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;

import java.util.Arrays;

@Config
@Autonomous
public class Biobuzz_2_Tip_Auto extends NGAutoOpMode{
    public static double WALL_SET_DWELL_S = 0.25;

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

        Pose2d beginPose = HiveAutoKit.sideShot(Side.RIGHT, 49.05, 0);
        initAuto(beginPose);

        Vector2d gardenEnd = new Vector2d(beginPose.position.x, -61);
        Pose2d leftShot = HiveAutoKit.sideShot(Side.LEFT, 41.3, 183);
        Pose2d leftAngled = HiveAutoKit.sideShotFrom(Side.LEFT, new Vector2d(-58.0, -34.6));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, leftAngled.heading.toDouble());

        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, null, telemetry, maxVel, maxAccel,
                parkPose, leftShot, beginPose);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepare(leftShot, parkPose);
        kit.prepare(leftAngled, parkPose);
        kit.prepareFlower(leftShot, flowerOne);
        kit.prepareFlower(leftShot, flowerTwo);
        kit.prepareFromFlower(flowerOne, leftShot);
        kit.prepareFromFlower(flowerTwo, leftAngled);

        double F1_TO_SHOT_S = 1.29 + BiobuzzRobotConstants.flowerCollectTime + 1.38 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F2_FROM_LEFT_S = 3.44 + BiobuzzRobotConstants.flowerCollectTime + 3.25 + BiobuzzRobotConstants.volleyS() + 0.3;

        Pose2d f1Staging = HiveAutoKit.flowerStaging(flowerOne);
        double leftShotToF1Tangent = Math.atan2(f1Staging.position.y - leftShot.position.y,
                f1Staging.position.x - leftShot.position.x) + Math.toRadians(40);

        Action shootPreLoad = kit.preloadShot();

        Action wallSetToLeftShot = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(90), maxVel, maxAccel)
                .build(), HiveAutoKit.SIDE_WALL_COLLECT_START_S);

        Action leftShotToFlowerOne = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.approachFlower(drive.actionBuilder(leftShot)
                        .setTangent(leftShotToF1Tangent), flowerOne).build()),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerOneToLeftShot = kit.departFlower(drive.actionBuilder(flowerOne), flowerOne)
                .splineToSplineHeading(leftShot, Math.toRadians(-170), maxVel, maxAccel)
                .build();

        Action leftShotToFlowerTwo = kit.approachFlower(drive.actionBuilder(leftShot)
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-24, -39, Math.toRadians(0)), Math.toRadians(5), maxVel, maxAccel)
                        .splineToSplineHeading(new Pose2d(10, -39.5, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, intake2_0.deployFlowerRamp()), flowerTwo)
                .afterTime(0, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime))
                .build();

        Action flowerTwoToLeftAngled = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-42, -38, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftAngled, Math.toRadians(150), maxVel, maxAccel)
                .build();

        Action parkFromAngled = drive.actionBuilder(leftAngled)
                .strafeToConstantHeading(parkPose.position, maxVel, maxAccel)
                .build();

        Action park = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftAngled, parkFromAngled, parkPose, null));

        Action main = new SequentialAction(
                shootPreLoad,
                wallSetToLeftShot,
                kit.transfer(),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_TO_SHOT_S, leftShot)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftShotToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftShot, leftShot),
                            kit.transfer());
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_LEFT_S, leftAngled)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftShotToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftAngled, leftAngled),
                            kit.transfer());
                }),
                park);

        kit.checkPrebuilt("wallSetToLeftShot", wallSetToLeftShot);
        kit.checkPrebuilt("leftShotToFlowerOne", leftShotToFlowerOne);
        kit.checkPrebuilt("flowerOneToLeftShot", flowerOneToLeftShot);
        kit.checkPrebuilt("leftShotToFlowerTwo", leftShotToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeftAngled", flowerTwoToLeftAngled);
        kit.checkPrebuilt("parkFromAngled", parkFromAngled);

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;
        while (!isStarted() && !isStopRequested()) {
            AllianceSelector.poll(gamepad1, gamepad2);
            AllianceSelector.show(telemetry);
            telemetry.addLine("Paths Built - Ready To Start");
            telemetry.addData("HIVE clearance", kit.clearanceSummary());
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
        PoseStorage.storeAutoEndPose(AllianceSelector.red(), drive.pose);
    }
}
