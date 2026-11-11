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
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

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

        Pose2d beginPose = HiveAutoKit.sideShot(Side.RIGHT, 49.05, 0);
        initAuto(beginPose);

        Vector2d gardenEnd = new Vector2d(beginPose.position.x, -61);
        Pose2d leftShot = HiveAutoKit.sideShot(Side.LEFT, 41.3, 183);
        Pose2d leftAngled = HiveAutoKit.sideShotFrom(Side.LEFT, new Vector2d(-58.0, -34.6));
        Pose2d rightAngled = HiveAutoKit.sideShot(Side.RIGHT, 49.05, -25);
        Pose2d rightShot = beginPose;
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, leftAngled.heading.toDouble());

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepareDefaults();
        kit.prepare(leftAngled, parkPose);
        kit.prepareFlower(leftShot, flowerOne);
        kit.prepareFlower(leftShot, flowerTwo);
        kit.prepareFlower(rightAngled, flowerTwo);
        kit.prepareFromFlower(flowerOne, leftShot);
        kit.prepareFromFlower(flowerOne, leftAngled);
        kit.prepareFromFlower(flowerTwo, leftShot);
        kit.prepareFromFlower(flowerTwo, leftAngled);

        double F1_TO_SHOT_S = 1.29 + BiobuzzRobotConstants.flowerCollectTime + 1.38 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F1_TO_ANGLED_S = 1.29 + BiobuzzRobotConstants.flowerCollectTime + 1.62 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F2_FROM_LEFT_S = 3.44 + BiobuzzRobotConstants.flowerCollectTime + 3.25 + BiobuzzRobotConstants.volleyS() + 0.3;
        double F2_FROM_RIGHT_S = 1.66 + BiobuzzRobotConstants.flowerCollectTime + 3.27 + BiobuzzRobotConstants.volleyS() + 0.3;

        Pose2d f1Staging = HiveAutoKit.flowerStaging(flowerOne);
        Pose2d f2Staging = HiveAutoKit.flowerStaging(flowerTwo);
        double leftShotToF1Tangent = Math.atan2(f1Staging.position.y - leftShot.position.y,
                f1Staging.position.x - leftShot.position.x) + Math.toRadians(40);
        double rightAngledToF2Tangent = Math.atan2(f2Staging.position.y - rightAngled.position.y,
                f2Staging.position.x - rightAngled.position.x) - Math.toRadians(20);

        Action wallSetToLeftShot = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(90), maxVel, maxAccel)
                .build(), HiveAutoKit.SIDE_WALL_COLLECT_START_S);

        Action wallSetToRightAngled = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(rightAngled, Math.toRadians(140), maxVel, maxAccel)
                .build(), HiveAutoKit.SIDE_WALL_COLLECT_START_S);

        Action leftShotToFlowerOne = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.approachFlower(drive.actionBuilder(leftShot)
                        .setTangent(leftShotToF1Tangent), flowerOne).build()),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerOneToLeftShot = kit.departFlower(drive.actionBuilder(flowerOne), flowerOne)
                .splineToSplineHeading(leftShot, Math.toRadians(-170), maxVel, maxAccel)
                .build();

        Action flowerOneToLeftAngled = kit.departFlower(drive.actionBuilder(flowerOne), flowerOne)
                .splineToSplineHeading(leftAngled, Math.toRadians(-150), maxVel, maxAccel)
                .build();

        Action leftShotToFlowerTwo = kit.approachFlower(drive.actionBuilder(leftShot)
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-24, -39, Math.toRadians(0)), Math.toRadians(5), maxVel, maxAccel)
                        .splineToSplineHeading(new Pose2d(10, -39.5, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, intake2_0.deployFlowerRamp()), flowerTwo)
                .afterTime(0, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime))
                .build();

        Action rightAngledToFlowerTwo = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.approachFlower(drive.actionBuilder(rightAngled)
                        .setTangent(rightAngledToF2Tangent), flowerTwo).build()),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerTwoToLeftAngled = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-42, -38, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftAngled, Math.toRadians(150), maxVel, maxAccel)
                .build();

        Action flowerTwoToLeftShot = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(90), maxVel, maxAccel)
                .build();

        Action parkFromAngled = drive.actionBuilder(leftAngled)
                .strafeToConstantHeading(parkPose.position, maxVel, maxAccel)
                .build();

        Action park = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftAngled, parkFromAngled, parkPose, null));

        Action tippedPlan = new SequentialAction(
                wallSetToLeftShot,
                kit.shotPoint(Side.LEFT),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_TO_SHOT_S, leftShot)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftShotToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftShot, leftShot),
                            kit.shotPoint(Side.LEFT));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_LEFT_S, leftAngled)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    kit.log("flower two - safety set, always shot at LEFT");
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftShotToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftAngled, leftAngled),
                            kit.committedShot(Side.LEFT));
                }),
                park);

        Action notTippedPlan = new SequentialAction(
                wallSetToRightAngled,
                kit.shotPoint(Side.RIGHT, 8.0),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_RIGHT_S, leftShot)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(rightAngled, rightAngledToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftShot, leftShot),
                            kit.shotPoint(Side.LEFT));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_TO_ANGLED_S, leftAngled)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftShotToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftAngled, leftAngled),
                            kit.shotPoint(Side.LEFT));
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

        kit.checkPrebuilt("wallSetToLeftShot", wallSetToLeftShot);
        kit.checkPrebuilt("wallSetToRightAngled", wallSetToRightAngled);
        kit.checkPrebuilt("leftShotToFlowerOne", leftShotToFlowerOne);
        kit.checkPrebuilt("flowerOneToLeftShot", flowerOneToLeftShot);
        kit.checkPrebuilt("flowerOneToLeftAngled", flowerOneToLeftAngled);
        kit.checkPrebuilt("leftShotToFlowerTwo", leftShotToFlowerTwo);
        kit.checkPrebuilt("rightAngledToFlowerTwo", rightAngledToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeftAngled", flowerTwoToLeftAngled);
        kit.checkPrebuilt("flowerTwoToLeftShot", flowerTwoToLeftShot);
        kit.checkPrebuilt("parkFromAngled", parkFromAngled);

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
        PoseStorage.storeAutoEndPose(AllianceSelector.red(), drive.pose);
        hive.close();
    }
}
