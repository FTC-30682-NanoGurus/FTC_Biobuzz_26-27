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
        Pose2d parkPose = new Pose2d(-53.5, -57, Math.toRadians(90));

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, true);
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        double FLOWER_TWO_BLOCK_S = 5.61 + 1.84 + 1.0;
        double WALL_TO_LEFT_S = 5.89 + 1.0;
        double WALL_TO_RIGHT_S = 5.6 + 1.0;
        double WALL_ONLY_S = 4.6;
        double SOLO_FLOWER_BLOCK_S = 5.08 + 1.66 + 1.0;

        Action toLeftShot = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel)
                .build();

        Action toFlowerOne = drive.actionBuilder(leftShot)
                .afterTime(0, intake2_0.deployFlowerRamp())
                .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(flowerOne, Math.toRadians(210), maxVel, maxAccel)
                .afterTime(0, intake2_0.collectFromFlower(3))
                .build();

        Action backFromFlowerOne = drive.actionBuilder(flowerOne)
                .setTangent(Math.toRadians(40))
                .splineToSplineHeading(leftShotAfterFlower, Math.toRadians(165), maxVel, maxAccel)
                .build();

        Action toFlowerTwo = drive.actionBuilder(leftShotAfterFlower)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(-10)), Math.toRadians(0), maxVel, maxAccel)
                .afterTime(0, intake2_0.deployFlowerRamp())
                .splineToSplineHeading(flowerTwo, Math.toRadians(-90), maxVel, maxAccel)
                .afterTime(0, intake2_0.collectFromFlower(3))
                .build();

        Action crossToRight = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(rightShot, Math.toRadians(15), maxVel, maxAccel)
                .build();

        Action wallToLeft = drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-155))
                .afterTime(2.1, intake2_0.collect(2.5))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-24, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(120), maxVel, maxAccel)
                .build();

        Action wallToRight = drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-155))
                .afterTime(2.1, intake2_0.collect(2.5))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(44, -38, Math.toRadians(-90)), Math.toRadians(90), maxVel, maxAccel)
                .splineToLinearHeading(rightShot, Math.toRadians(30), maxVel, maxAccel)
                .build();

        Action wallOnly = drive.actionBuilder(rightShot)
                .setTangent(Math.toRadians(-155))
                .afterTime(2.1, intake2_0.collect(2.5))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .build();

        Action parkFromLeft = drive.actionBuilder(wallShotLeft)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50, -26, Math.toRadians(20)), Math.toRadians(-90), maxVel, maxAccel)
                .splineToSplineHeading(parkPose, Math.toRadians(-90), maxVel, maxAccel)
                .build();

        Action soloWallSet = drive.actionBuilder(rightShotSolo)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .afterTime(0.7, intake2_0.collect(4))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(-220), maxVel, maxAccel)
                .build();

        Action soloFlower = drive.actionBuilder(wallShotLeft)
                .afterTime(0, intake2_0.deployFlowerRamp())
                .setTangent(Math.toRadians(-172))
                .splineToSplineHeading(new Pose2d(-52.6, -17.5, Math.toRadians(180)), Math.toRadians(-112), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(-55, -22), Math.toRadians(-82), maxVel, maxAccel)
                .afterTime(0.4, intake2_0.collectFromFlower(3))
                .build();

        Action soloBack = drive.actionBuilder(flowerOneSolo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(360)), Math.toRadians(90), maxVel, maxAccel)
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
            if (up != Side.LEFT && kit.fits(WALL_TO_RIGHT_S, rightShot)) {
                kit.log(up == Side.RIGHT ? "RIGHT CELL up - wall set back to RIGHT" : "no view - wall set back to RIGHT");
                return new SequentialAction(wallToRight, kit.shotPoint(Side.RIGHT), parkBlock);
            }
            if (kit.fits(WALL_ONLY_S, wallSet)) {
                kit.log("no time to shoot the wall set - collecting it for TELEOP");
                return new SequentialAction(wallOnly, parkBlock);
            }
            kit.log("no time for the wall set");
            return parkBlock;
        });

        Action flowerTwoBlock = new SequentialAction(
                kit.leg(leftShotAfterFlower, toFlowerTwo, flowerTwo, intake2_0.collectFromFlower(3)),
                HiveAutoKit.branch(() -> {
                    if (hive.upCell(15.0) == Side.LEFT) {
                        kit.log("LEFT CELL still up - flower two goes LEFT");
                        return new SequentialAction(kit.route(drive.pose, leftShot), kit.shotPoint(Side.LEFT), parkBlock);
                    }
                    return new SequentialAction(
                            kit.leg(flowerTwo, crossToRight, rightShot, null),
                            kit.shotPoint(Side.RIGHT),
                            wallBlock);
                }));

        Action nominal = new SequentialAction(
                kit.transfer(),
                kit.leg(leftShot, toFlowerOne, flowerOne, intake2_0.collectFromFlower(3)),
                backFromFlowerOne,
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
                            kit.leg(wallShotLeft, soloFlower, flowerOneSolo, intake2_0.collectFromFlower(3)),
                            soloBack,
                            kit.shotPoint(Side.LEFT),
                            parkBlock);
                }));

        Action main = new SequentialAction(
                toLeftShot,
                HiveAutoKit.waitUntil(() -> hive.settledUp(Side.LEFT, 0.4), MAX_WAIT_FOR_ALLIANCE_TIP_S),
                HiveAutoKit.branch(() -> {
                    boolean leftUp = hive.settledUp(Side.LEFT, 0.6);
                    if (!leftUp && hive.settledUp(Side.RIGHT, 0.6)) {
                        kit.log("partner did not tip - tipping RIGHT ourselves");
                        return partnerDidNotTip;
                    }
                    kit.log(leftUp ? "partner tipped - LEFT CELL up" : "no HIVE view - shooting LEFT as planned");
                    return nominal;
                }));

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;

        while (!isStarted() && !isStopRequested()) {
            hive.update(drive.pose);
            telemetry.addLine("Paths Built - Ready To Start");
            telemetry.addData("HIVE", hive.describe());
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
