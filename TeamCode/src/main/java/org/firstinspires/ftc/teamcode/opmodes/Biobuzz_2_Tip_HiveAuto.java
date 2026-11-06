package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

@Config
@Autonomous
public class Biobuzz_2_Tip_HiveAuto extends NGAutoOpMode {
    public static double WALL_SET_DWELL_S = 0.25;
    public static double TIP_START_WAIT_S = 3.5;

    @Override
    public void runOpMode() throws InterruptedException {

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
        Pose2d rightShot = new Pose2d(61, -14, Math.toRadians(180));
        Pose2d wallSet = new Pose2d(62, -58, Math.toRadians(360));
        Pose2d rightNear = new Pose2d(56.69, -40.01, Math.toRadians(155));
        Pose2d leftNear = new Pose2d(-55.46, -40.00, Math.toRadians(25.0));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, Math.toRadians(90));

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepareDefaults();
        kit.prepareFlower(leftNear, flowerOne);
        kit.prepareFromFlower(flowerTwo, leftNear);
        kit.prepareFromFlower(flowerOne, leftNear);
        kit.prepareFlower(rightNear, flowerTwo);
        kit.prepare(leftNear, parkPose);
        kit.prepare(leftShotBack, parkPose);

        double F1_BLOCK_S = 1.96 + BiobuzzRobotConstants.flowerCollectTime + 1.71 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F1_TO_NEAR_S = 1.96 + BiobuzzRobotConstants.flowerCollectTime + 1.93 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F2_FROM_LEFT_S = 2.90 + BiobuzzRobotConstants.flowerCollectTime + 3.09 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F2_FROM_RIGHT_S = 1.98 + BiobuzzRobotConstants.flowerCollectTime + 3.09 + BiobuzzRobotConstants.volleyS() + 0.3;

        Action wallSetToLeftNear = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -42, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -40, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftNear, Math.toRadians(180), maxVel, maxAccel)
                .build());

        Action wallSetToRightNear = kit.withWallCollect(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                .setTangent(Math.toRadians(-155))
                                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                                .waitSeconds(WALL_SET_DWELL_S)
                                .build(),
                        drive.actionBuilder(wallSet)
                                .setTangent(Math.atan2(10, -4))
                                .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                                .splineToSplineHeading(rightNear, Math.atan2(rightNear.position.y + 48, rightNear.position.x - 58), maxVel, maxAccel)
                                .build()));

        Action leftNearToFlowerOne = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(leftNear, flowerOne)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action backToShoot = kit.departFlower(drive.actionBuilder(flowerOne), flowerOne)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(155), maxVel, maxAccel)
                .build();

        Action leftToFlowerTwo = kit.approachFlower(drive.actionBuilder(leftShotBack)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(0, -41, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, intake2_0.deployFlowerRamp()), flowerTwo)
                .afterTime(0, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime))
                .build();

        Action rightNearToFlowerTwo = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(rightNear, flowerTwo)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerTwoToLeftNear = drive.actionBuilder(flowerTwo)
                .strafeToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), maxVel, maxAccel)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0, -40, Math.toRadians(0)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-30, -40, Math.toRadians(0)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftNear, Math.toRadians(180), maxVel, maxAccel)
                .build();

        Action flowerOneToLeftNear = kit.routeFromFlower(flowerOne, leftNear);

        Action flowerOneBlock = new SequentialAction(
                kit.flowerLeg(leftNear, leftNearToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                kit.fromFlowerLeg(flowerOne, backToShoot, leftShotBack),
                kit.shotPoint(Side.LEFT));

        Action flowerOneBlockToNear = new SequentialAction(
                kit.flowerLeg(leftNear, leftNearToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                kit.fromFlowerLeg(flowerOne, flowerOneToLeftNear, leftNear),
                kit.shotPoint(Side.LEFT));

        Action parkFromNear = drive.actionBuilder(leftNear)
                .setTangent(Math.toRadians(-60))
                .splineToSplineHeading(parkPose, Math.toRadians(-90), maxVel, maxAccel)
                .build();

        Action park = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftNear, parkFromNear, parkPose, null));

        Action tippedPlan = new SequentialAction(
                wallSetToLeftNear,
                kit.shotPoint(Side.LEFT),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_BLOCK_S, leftShotBack)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return flowerOneBlock;
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_LEFT_S, leftNear)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    kit.log("flower two - safety set, always shot at LEFT");
                    return new SequentialAction(
                            kit.flowerLeg(leftShotBack, leftToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftNear, leftNear),
                            kit.committedShot(Side.LEFT));
                }),
                park);

        Action notTippedPlan = new SequentialAction(
                wallSetToRightNear,
                kit.shotPoint(Side.RIGHT, 8.0),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_RIGHT_S, leftNear)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(rightNear, rightNearToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftNear, leftNear),
                            kit.shotPoint(Side.LEFT));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_TO_NEAR_S, leftNear)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return flowerOneBlockToNear;
                }),
                park);

        Action main = new SequentialAction(
                kit.preloadShot(),
                kit.waitForOwnTipStart(Side.RIGHT, TIP_START_WAIT_S),
                HiveAutoKit.branch(() -> {
                    if (kit.ownTipStarted(Side.RIGHT)) {
                        kit.log("RIGHT CELL tip started - wall set to the LEFT side");
                        return tippedPlan;
                    }
                    kit.log("no tip start in " + TIP_START_WAIT_S + " s - assuming no tip, wall set at RIGHT");
                    return notTippedPlan;
                }));

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;

        kit.checkPrebuilt("wallSetToLeftNear", wallSetToLeftNear);
        kit.checkPrebuilt("wallSetToRightNear", wallSetToRightNear);
        kit.checkPrebuilt("leftNearToFlowerOne", leftNearToFlowerOne);
        kit.checkPrebuilt("parkFromNear", parkFromNear);
        kit.checkPrebuilt("backToShoot", backToShoot);
        kit.checkPrebuilt("leftToFlowerTwo", leftToFlowerTwo);
        kit.checkPrebuilt("rightNearToFlowerTwo", rightNearToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeftNear", flowerTwoToLeftNear);
        kit.checkPrebuilt("flowerOneToLeftNear", flowerOneToLeftNear);

        while (!isStarted() && !isStopRequested()) {
            if (AllianceSelector.poll(gamepad1, gamepad2)) hive.setAlliance(AllianceSelector.red());
            hive.update(drive.pose);
            AllianceSelector.show(telemetry);
            telemetry.addData("Alliance check", hive.ownTagsSeen(2.0) ? "OK - our HIVE tags in view"
                    : hive.otherAllianceTagsSeen(2.0) ? "CHECK ALLIANCE - only the other alliance's tags seen" : "our HIVE tags not seen yet");
            telemetry.addLine("Paths Built - Ready To Start");
            telemetry.addData("HIVE", hive.describe());
            telemetry.addData("HIVE clearance", kit.clearanceSummary());
            Side up = hive.upCell(1.0);
            telemetry.addData("CELL up", up == null ? "unknown" : up);
            telemetry.update();
        }
        if (isStopRequested()) {
            hive.close();
            return;
        }

        kit.start();
        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        intake2_0.updateFlywheelPID(),
                        kit.background(),
                        intake2_0.runShooter(pollenShootingVel, nectarShootingVel, 30),
                        kit.guardPark(main)
                )
        );
        hive.close();
    }
}
