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

        Pose2d beginPose = new Pose2d(-65, -11, Math.toRadians(0));
        initAuto(beginPose);

        Pose2d leftShot = new Pose2d(-52, -14, Math.toRadians(0));
        Pose2d leftShotAfterFlower = new Pose2d(-52, -14, Math.toRadians(1));
        Pose2d wallShotLeft = new Pose2d(-52, -16, Math.toRadians(0));
        Pose2d rightShot = new Pose2d(61, -14, Math.toRadians(-179.5));
        Pose2d rightShotSolo = new Pose2d(61, -14, Math.toRadians(180));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(-179.5));
        Pose2d flowerOneSolo = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d wallSet = new Pose2d(62, -58, Math.toRadians(360));
        Pose2d nearShot = new Pose2d(56.69, -40.01, Math.toRadians(155));
        Pose2d parkPose = new Pose2d(-21, -59.5, Math.toRadians(180));

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepareDefaults();
        kit.prepare(leftShot, rightShotSolo);
        kit.prepareFromFlower(flowerTwo, leftShot);
        kit.prepare(wallSet, parkPose);
        kit.prepare(rightShot, wallShotLeft);
        kit.prepare(nearShot, parkPose);
        kit.prepare(wallShotLeft, parkPose);

        double FLOWER_TWO_BLOCK_S = 5.90 + 1.84 + 1.0;
        double WALL_TO_LEFT_S = 6.14 + 1.0;
        double WALL_NEAR_S = 4.69 + 1.0;
        double WALL_ONLY_S = 4.6;
        double SOLO_FLOWER_BLOCK_S = 4.98 + 1.71 + 1.0;

        Action toLeftShot = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel)
                .build();

        Action toFlowerOne = kit.approachFlower(drive.actionBuilder(leftShot)
                        .afterTime(0, intake2_0.deployFlowerRamp())
                        .setTangent(Math.toRadians(-32)), flowerOne)
                .afterTime(0, intake2_0.collectFromFlower(3))
                .build();

        Action backFromFlowerOne = kit.departFlower(drive.actionBuilder(flowerOne), flowerOne)
                .splineToSplineHeading(leftShotAfterFlower, Math.toRadians(155), maxVel, maxAccel)
                .build();

        Action toFlowerTwo = kit.approachFlower(drive.actionBuilder(leftShotAfterFlower)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(0, -41, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, intake2_0.deployFlowerRamp()), flowerTwo)
                .afterTime(0, intake2_0.collectFromFlower(3))
                .build();

        Action crossToRight = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(rightShot, Math.toRadians(15), maxVel, maxAccel)
                .build();

        Action wallToLeft = kit.withWallCollect(drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-24, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(120), maxVel, maxAccel)
                .build());

        Action wallToNear = kit.withWallCollect(
                new SequentialAction(
                        drive.actionBuilder(rightShot)
                                .setTangent(Math.toRadians(-155))
                                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                                .waitSeconds(WALL_SET_DWELL_S)
                                .build(),
                        drive.actionBuilder(wallSet)
                                .setTangent(Math.atan2(10, -4))
                                .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                                .splineToSplineHeading(nearShot, Math.atan2(nearShot.position.y + 48, nearShot.position.x - 58), maxVel, maxAccel)
                                .build()));

        Action wallOnly = drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-155))
                .afterTime(2.1, intake2_0.collectToGate(2.5))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .build();

        Action parkFromLeft = kit.route(wallShotLeft, parkPose);

        Action soloWallSet = kit.withWallCollect(drive.actionBuilder(rightShotSolo)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(-220), maxVel, maxAccel)
                .build());

        Action soloFlower = kit.approachFlower(drive.actionBuilder(wallShotLeft)
                        .afterTime(0, intake2_0.deployFlowerRamp())
                        .setTangent(Math.toRadians(-27)), flowerOneSolo)
                .afterTime(0.4, intake2_0.collectFromFlower(3))
                .build();

        Action soloBack = kit.departFlower(drive.actionBuilder(flowerOneSolo), flowerOneSolo)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(155), maxVel, maxAccel)
                .build();

        Action parkBlock = new SequentialAction(
                kit.beginPark(),
                kit.leg(wallShotLeft, parkFromLeft, parkPose, null));

        Action wallBlock = HiveAutoKit.branch(() -> {
            if (kit.robotSide() != Side.RIGHT) {
                kit.log("not at the RIGHT pose - going to park");
                return parkBlock;
            }
            Side up = hive.upCell(1.5);
            if (up == Side.LEFT && kit.fits(WALL_TO_LEFT_S, leftShot)) {
                kit.log("LEFT CELL up - wall set to LEFT");
                return new SequentialAction(wallToLeft, kit.shotPoint(Side.LEFT), parkBlock);
            }
            if (up != Side.LEFT && kit.fits(WALL_NEAR_S, nearShot)) {
                kit.log(up == Side.RIGHT ? "RIGHT CELL up - wall set, angled shot at RIGHT" : "no view - wall set, angled shot at RIGHT");
                return new SequentialAction(wallToNear, kit.shotPoint(Side.RIGHT, 8.0), parkBlock);
            }
            if (kit.fits(WALL_ONLY_S, wallSet)) {
                kit.log("no time to shoot the wall set - collecting it for TELEOP");
                return new SequentialAction(wallOnly, parkBlock);
            }
            kit.log("no time for the wall set");
            return parkBlock;
        });

        Action flowerTwoBlock = new SequentialAction(
                kit.flowerLeg(leftShotAfterFlower, toFlowerTwo, flowerTwo, intake2_0.collectFromFlower(3)),
                HiveAutoKit.branch(() -> {
                    if (hive.upCell(15.0) == Side.LEFT) {
                        kit.log("LEFT CELL still up - flower two goes LEFT");
                        return new SequentialAction(kit.leaveFlower(kit.routeFromFlower(drive.pose, leftShot)), kit.shotPoint(Side.LEFT), parkBlock);
                    }
                    return new SequentialAction(
                            kit.fromFlowerLeg(flowerTwo, crossToRight, rightShot),
                            kit.shotPoint(Side.RIGHT),
                            wallBlock);
                }));

        Action nominal = new SequentialAction(
                kit.transfer(),
                kit.flowerLeg(leftShot, toFlowerOne, flowerOne, intake2_0.collectFromFlower(3)),
                kit.leaveFlower(backFromFlowerOne),
                kit.shotPoint(Side.LEFT),
                HiveAutoKit.branch(() -> {
                    if (kit.fits(FLOWER_TWO_BLOCK_S, rightShot)) return flowerTwoBlock;
                    kit.log("no time for flower two");
                    return parkBlock;
                }));

        Action partnerDidNotTip = new SequentialAction(
                HiveAutoKit.branch(() -> kit.route(drive.pose, rightShotSolo)),
                kit.shotPoint(Side.RIGHT),
                kit.leg(rightShotSolo, soloWallSet, wallShotLeft, null),
                kit.shotPoint(Side.LEFT),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(SOLO_FLOWER_BLOCK_S, leftShot)) {
                        kit.log("no time for flower one");
                        return parkBlock;
                    }
                    return new SequentialAction(
                            kit.flowerLeg(wallShotLeft, soloFlower, flowerOneSolo, intake2_0.collectFromFlower(3)),
                            kit.leaveFlower(soloBack),
                            kit.shotPoint(Side.LEFT),
                            parkBlock);
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
        kit.checkPrebuilt("toFlowerOne", toFlowerOne);
        kit.checkPrebuilt("backFromFlowerOne", backFromFlowerOne);
        kit.checkPrebuilt("toFlowerTwo", toFlowerTwo);
        kit.checkPrebuilt("crossToRight", crossToRight);
        kit.checkPrebuilt("wallToLeft", wallToLeft);
        kit.checkPrebuilt("wallToNear", wallToNear);
        kit.checkPrebuilt("wallOnly", wallOnly);
        kit.checkPrebuilt("parkFromLeft", parkFromLeft);
        kit.checkPrebuilt("soloWallSet", soloWallSet);
        kit.checkPrebuilt("soloFlower", soloFlower);
        kit.checkPrebuilt("soloBack", soloBack);

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
