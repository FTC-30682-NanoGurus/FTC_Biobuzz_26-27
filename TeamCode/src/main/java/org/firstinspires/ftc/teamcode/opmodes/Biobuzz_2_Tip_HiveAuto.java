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
public class Biobuzz_2_Tip_HiveAuto extends NGAutoOpMode {
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

        Pose2d beginPose = new Pose2d(61, -14, Math.toRadians(180));
        initAuto(beginPose);

        Pose2d leftShot = new Pose2d(-52, -14, Math.toRadians(0));
        Pose2d leftShotBack = new Pose2d(-52, -14, Math.toRadians(360));
        Pose2d rightShot = new Pose2d(61, -14, Math.toRadians(180));
        Pose2d wallSet = new Pose2d(62, -58, Math.toRadians(360));
        Pose2d rightNear = new Pose2d(56.69, -40.01, Math.toRadians(155));
        Pose2d leftNear = new Pose2d(-53.48, -43.82, Math.toRadians(30));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, Math.toRadians(90));

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepareDefaults();
        kit.prepare(leftNear, flowerOne);
        kit.prepare(flowerTwo, leftNear);
        kit.prepare(rightNear, flowerTwo);
        kit.prepare(leftNear, parkPose);
        kit.prepare(leftShotBack, parkPose);

        double F1_BLOCK_S = 1.47 + 0.4 + 3.0 + 1.66 + 1.0 + 0.3;
        double F2_FROM_LEFT_S = 2.61 + 3.0 + 2.57 + 1.0 + 0.3;
        double F2_FROM_RIGHT_S = 1.71 + 3.0 + 2.57 + 1.0 + 0.3;

        Action preloadWait = drive.actionBuilder(beginPose)
                .waitSeconds(2)
                .build();

        Action wallSetToLeftNear = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
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
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.route(leftNear, flowerOne)),
                new SleepAction(0.4),
                intake2_0.collectFromFlower(3));

        Action backToShoot = drive.actionBuilder(flowerOne)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(leftShotBack, Math.toRadians(90), maxVel, maxAccel)
                .build();

        Action leftToFlowerTwo = drive.actionBuilder(leftShotBack)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(-10)), Math.toRadians(0), maxVel, maxAccel)
                .afterTime(0, intake2_0.deployFlowerRamp())
                .splineToSplineHeading(flowerTwo, Math.toRadians(-90), maxVel, maxAccel)
                .afterTime(0, intake2_0.collectFromFlower(3))
                .build();

        Action rightNearToFlowerTwo = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.route(rightNear, flowerTwo)),
                intake2_0.collectFromFlower(3));

        Action flowerTwoToLeftNear = kit.route(flowerTwo, leftNear);

        Action flowerOneBlock = new SequentialAction(
                kit.leg(leftNear, leftNearToFlowerOne, flowerOne, intake2_0.collectFromFlower(3)),
                kit.leg(flowerOne, backToShoot, leftShotBack, null),
                kit.shotPoint(Side.LEFT));

        Action park = new SequentialAction(
                kit.beginPark(),
                HiveAutoKit.branch(() -> kit.route(drive.pose, parkPose)));

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
                    kit.log("flower two - backup shot at the LEFT CELL if it has not tipped");
                    return new SequentialAction(
                            kit.leg(leftShotBack, leftToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(3)),
                            kit.leg(flowerTwo, flowerTwoToLeftNear, leftNear, null),
                            kit.shotPoint(Side.LEFT));
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
                            kit.leg(rightNear, rightNearToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(3)),
                            kit.leg(flowerTwo, flowerTwoToLeftNear, leftNear, null),
                            kit.shotPoint(Side.LEFT));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_BLOCK_S, leftShotBack)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return flowerOneBlock;
                }),
                park);

        Action main = new SequentialAction(
                new ParallelAction(preloadWait, kit.transfer()),
                kit.waitForTipStart(Side.RIGHT, HiveAutoKit.OWN_TIP_GRACE_S),
                HiveAutoKit.branch(() -> {
                    if (hive.settledUp(Side.RIGHT, 0.4)) {
                        kit.log("RIGHT CELL did not tip - wall set, angled shot at RIGHT");
                        return notTippedPlan;
                    }
                    kit.log(hive.tipping(Side.RIGHT, 5.0) ? "RIGHT CELL seen tipping" : "no RIGHT CELL at rest in view - assuming it tipped");
                    return tippedPlan;
                }));

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;

        kit.checkPrebuilt("wallSetToLeftNear", wallSetToLeftNear);
        kit.checkPrebuilt("wallSetToRightNear", wallSetToRightNear);
        kit.checkPrebuilt("leftNearToFlowerOne", leftNearToFlowerOne);
        kit.checkPrebuilt("backToShoot", backToShoot);
        kit.checkPrebuilt("leftToFlowerTwo", leftToFlowerTwo);
        kit.checkPrebuilt("rightNearToFlowerTwo", rightNearToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeftNear", flowerTwoToLeftNear);

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
