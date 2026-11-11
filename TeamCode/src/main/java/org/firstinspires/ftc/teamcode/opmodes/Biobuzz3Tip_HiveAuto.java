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
import com.acmerobotics.roadrunner.SleepAction;
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
public class Biobuzz3Tip_HiveAuto extends NGAutoOpMode {
    public static double MAX_WAIT_FOR_ALLIANCE_TIP_S = 4.2;
    public static double WALL_SET_DWELL_S = 0.25;

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

        Pose2d beginPose = new Pose2d(-64.5, -9.5, Math.toRadians(90));
        initAuto(beginPose);

        Pose2d leftShot = HiveAutoKit.sideShot(Side.LEFT, 41.3, 183);
        Pose2d rightShot = HiveAutoKit.sideShot(Side.RIGHT, 49.05, 0);
        Pose2d rightAngled = HiveAutoKit.sideShot(Side.RIGHT, 49.05, -25);
        Vector2d gardenEnd = new Vector2d(rightShot.position.x, -61);
        Pose2d gardenPose = new Pose2d(gardenEnd, rightShot.heading.toDouble());
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, leftShot.heading.toDouble());
        Pose2d parkPoseFromRight = new Pose2d(-49.5, -58, Math.toRadians(-135));

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepareDefaults();
        kit.prepare(rightAngled, parkPose);
        kit.prepare(gardenPose, parkPose);
        kit.prepareFlower(leftShot, flowerOne);
        kit.prepareFlower(leftShot, flowerTwo);
        kit.prepareFromFlower(flowerOne, leftShot);
        kit.prepareFromFlower(flowerTwo, leftShot);
        kit.prepareFromFlower(flowerTwo, rightShot);

        double FLOWER_TWO_BLOCK_S = 3.44 + BiobuzzRobotConstants.flowerCollectTime + 1.90 + BiobuzzRobotConstants.volleyS();
        double WALL_TO_LEFT_S = 5.99 + BiobuzzRobotConstants.volleyS();
        double WALL_NEAR_S = 3.26 + BiobuzzRobotConstants.volleyS();
        double WALL_ONLY_S = 2.7;
        double SOLO_FLOWER_BLOCK_S = 1.29 + BiobuzzRobotConstants.flowerCollectTime + 1.38 + BiobuzzRobotConstants.volleyS();

        Pose2d f1Staging = HiveAutoKit.flowerStaging(flowerOne);
        double leftShotToF1Tangent = Math.atan2(f1Staging.position.y - leftShot.position.y,
                f1Staging.position.x - leftShot.position.x) + Math.toRadians(40);

        Action toLeftShot = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(leftShot.position, leftShot.heading, maxVel, maxAccel)
                .build();

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

        Action flowerTwoToLeftShot = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(90), maxVel, maxAccel)
                .build();

        Action crossToRight = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(30, -38, Math.toRadians(-90)), Math.toRadians(20), maxVel, maxAccel)
                .splineToSplineHeading(rightShot, Math.toRadians(30), maxVel, maxAccel)
                .build();

        Action wallToLeft = kit.withWallCollect(drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(10, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(90), maxVel, maxAccel)
                .build(), HiveAutoKit.SIDE_WALL_COLLECT_START_S);

        Action wallToNear = kit.withWallCollect(drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(rightAngled, Math.toRadians(140), maxVel, maxAccel)
                .build(), HiveAutoKit.SIDE_WALL_COLLECT_START_S);

        Action wallOnly = kit.withWallCollect(drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .build(), HiveAutoKit.SIDE_WALL_COLLECT_START_S);

        Action soloToRight = drive.actionBuilder(leftShot)
                .setTangent(Math.toRadians(-60))
                .splineToSplineHeading(new Pose2d(-24, -39, Math.toRadians(180)), Math.toRadians(0), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(20, -39, Math.toRadians(180)), Math.toRadians(0), maxVel, maxAccel)
                .splineToSplineHeading(rightShot, Math.toRadians(30), maxVel, maxAccel)
                .build();

        Action parkFromLeftShot = drive.actionBuilder(leftShot)
                .strafeToConstantHeading(parkPose.position, maxVel, maxAccel)
                .build();

        Action parkFromRightAngled = drive.actionBuilder(rightAngled)
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(20, -40, Math.toRadians(180)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(parkPoseFromRight, Math.toRadians(-135), maxVel, maxAccel)
                .build();

        Action parkLeft = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftShot, parkFromLeftShot, parkPose, null));

        Action parkRight = new SequentialAction(
                kit.beginPark(),
                kit.leg(rightAngled, parkFromRightAngled, parkPoseFromRight, null));

        Action wallBlock = HiveAutoKit.branch(() -> {
            if (kit.robotSide() != Side.RIGHT) {
                kit.log("not at the RIGHT pose - going to park");
                return parkLeft;
            }
            Side up = hive.upCell(1.5);
            if (up == Side.LEFT && kit.fits(WALL_TO_LEFT_S, leftShot)) {
                kit.log("LEFT CELL up - wall set to LEFT");
                return new SequentialAction(wallToLeft, kit.shotPoint(Side.LEFT), parkLeft);
            }
            if (up != Side.LEFT && kit.fits(WALL_NEAR_S, rightAngled)) {
                kit.log(up == Side.RIGHT ? "RIGHT CELL up - wall set, angled shot at RIGHT" : "no view - wall set, angled shot at RIGHT");
                return new SequentialAction(wallToNear, kit.shotPoint(Side.RIGHT, 8.0), parkRight);
            }
            if (kit.fits(WALL_ONLY_S, gardenPose)) {
                kit.log("no time to shoot the wall set - collecting it for TELEOP");
                return new SequentialAction(wallOnly, parkLeft);
            }
            kit.log("no time for the wall set");
            return parkLeft;
        });

        Action flowerTwoBlock = new SequentialAction(
                kit.flowerLeg(leftShot, leftShotToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                HiveAutoKit.branch(() -> {
                    if (hive.upCell(15.0) == Side.LEFT) {
                        kit.log("LEFT CELL still up - flower two goes LEFT");
                        return new SequentialAction(kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftShot, leftShot), kit.shotPoint(Side.LEFT), parkLeft);
                    }
                    return new SequentialAction(
                            kit.fromFlowerLeg(flowerTwo, crossToRight, rightShot),
                            kit.shotPoint(Side.RIGHT),
                            wallBlock);
                }));

        Action nominal = new SequentialAction(
                kit.transfer(),
                kit.flowerLeg(leftShot, leftShotToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                kit.fromFlowerLeg(flowerOne, flowerOneToLeftShot, leftShot),
                kit.shotPoint(Side.LEFT),
                HiveAutoKit.branch(() -> {
                    if (kit.fits(FLOWER_TWO_BLOCK_S, rightShot)) return flowerTwoBlock;
                    kit.log("no time for flower two");
                    return parkLeft;
                }));

        Action partnerDidNotTip = new SequentialAction(
                kit.leg(leftShot, soloToRight, rightShot, null),
                kit.shotPoint(Side.RIGHT),
                kit.leg(rightShot, wallToLeft, leftShot, null),
                kit.shotPoint(Side.LEFT),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(SOLO_FLOWER_BLOCK_S, leftShot)) {
                        kit.log("no time for flower one");
                        return parkLeft;
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftShotToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftShot, leftShot),
                            kit.shotPoint(Side.LEFT),
                            parkLeft);
                }));

        Action main = new SequentialAction(
                toLeftShot,
                kit.waitForAllianceTip(MAX_WAIT_FOR_ALLIANCE_TIP_S),
                HiveAutoKit.branch(() -> {
                    HiveAutoKit.TipResult tip = kit.allianceTipResult();
                    if (tip == HiveAutoKit.TipResult.NOT_TIPPED) {
                        kit.log("partner did not tip - tipping RIGHT ourselves");
                        return partnerDidNotTip;
                    }
                    kit.log(tip == HiveAutoKit.TipResult.TIPPED ? "partner tipped - LEFT CELL up" : "no camera frames - shooting LEFT as planned");
                    return nominal;
                }));

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;

        kit.checkPrebuilt("toLeftShot", toLeftShot);
        kit.checkPrebuilt("leftShotToFlowerOne", leftShotToFlowerOne);
        kit.checkPrebuilt("flowerOneToLeftShot", flowerOneToLeftShot);
        kit.checkPrebuilt("leftShotToFlowerTwo", leftShotToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeftShot", flowerTwoToLeftShot);
        kit.checkPrebuilt("crossToRight", crossToRight);
        kit.checkPrebuilt("wallToLeft", wallToLeft);
        kit.checkPrebuilt("wallToNear", wallToNear);
        kit.checkPrebuilt("wallOnly", wallOnly);
        kit.checkPrebuilt("soloToRight", soloToRight);
        kit.checkPrebuilt("parkFromLeftShot", parkFromLeftShot);
        kit.checkPrebuilt("parkFromRightAngled", parkFromRightAngled);

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
