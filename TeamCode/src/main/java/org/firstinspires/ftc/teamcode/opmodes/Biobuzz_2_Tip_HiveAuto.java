package org.firstinspires.ftc.teamcode.opmodes;

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

@Autonomous
public class Biobuzz_2_Tip_HiveAuto extends NGAutoOpMode {
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
        Pose2d wallShotLeft = new Pose2d(-52, -16, Math.toRadians(0));
        Pose2d rightShot = new Pose2d(61, -14, Math.toRadians(180));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d parkPose = new Pose2d(-50, -57, Math.toRadians(90));

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, true);
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, rightShot);

        double FLOWER_BLOCK_S = 5.08 + 1.66 + 1.0;

        Action preloadWait = drive.actionBuilder(beginPose)
                .waitSeconds(2)
                .build();

        Action wallSetToLeft = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                .afterTime(0.7, intake2_0.collect(4))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(-220), maxVel, maxAccel)
                .build();

        Action wallSetToRight = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                .afterTime(0.7, intake2_0.collect(4))
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(44, -38, Math.toRadians(-90)), Math.toRadians(90), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-179.5)), Math.toRadians(30), maxVel, maxAccel)
                .build();

        Action toFlowerOne = drive.actionBuilder(wallShotLeft)
                .afterTime(0, intake2_0.deployFlowerRamp())
                .setTangent(Math.toRadians(-172))
                .splineToSplineHeading(new Pose2d(-52.6, -17.5, Math.toRadians(180)), Math.toRadians(-112), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(-55, -22), Math.toRadians(-82), maxVel, maxAccel)
                .afterTime(0.4, intake2_0.collectFromFlower(3))
                .build();

        Action backToShoot = drive.actionBuilder(flowerOne)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(360)), Math.toRadians(90), maxVel, maxAccel)
                .build();

        Action parkFromLeft = drive.actionBuilder(leftShot)
                .strafeToLinearHeading(new Vector2d(-50, -57), Math.toRadians(90), maxVel, maxAccel)
                .build();

        Action main = new SequentialAction(
                new ParallelAction(preloadWait, kit.transfer()),
                HiveAutoKit.branch(() -> {
                    if (hive.settledUp(Side.RIGHT, 0.4)) {
                        kit.log("RIGHT CELL did not tip - wall set goes back to RIGHT");
                        return new SequentialAction(wallSetToRight, kit.shotPoint(Side.RIGHT));
                    }
                    kit.log(hive.settledUp(Side.LEFT, 1.0) ? "tip seen - LEFT CELL up" : "no RIGHT CELL in view - assuming it tipped");
                    return new SequentialAction(wallSetToLeft, kit.shotPoint(Side.LEFT));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(FLOWER_BLOCK_S, leftShot)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.leg(wallShotLeft, toFlowerOne, flowerOne, intake2_0.collectFromFlower(3)),
                            backToShoot,
                            kit.shotPoint(Side.LEFT));
                }),
                kit.beginPark(),
                kit.leg(leftShot, parkFromLeft, parkPose, null)
        );

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
