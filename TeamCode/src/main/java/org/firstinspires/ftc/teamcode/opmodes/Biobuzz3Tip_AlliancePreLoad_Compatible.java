package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

@Config
@Autonomous
public class Biobuzz3Tip_AlliancePreLoad_Compatible extends NGAutoOpMode{
    public static double MAX_WAIT_FOR_ALLIANCE_TIP_S = 4.2;

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

        Pose2d beginPose = new Pose2d(-65, -11, Math.toRadians(0));
        initAuto(beginPose);

        TrajectoryActionBuilder waitForAlliancePreLoadPath = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel);

        TrajectoryActionBuilder turnToFlowerOne = waitForAlliancePreLoadPath.endTrajectory().fresh()
                .afterTime(0, intake2_0.deployFlowerRamp())
                .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(-55, -22, Math.toRadians(-179.5)), Math.toRadians(210), maxVel, maxAccel)
                .afterTime(0, intake2_0.collectFromFlower(3));

        TrajectoryActionBuilder turnToShoot = turnToFlowerOne.endTrajectory().fresh()
                .setTangent(Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(165), maxVel, maxAccel);

        TrajectoryActionBuilder toFlowerTwo = turnToShoot.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(-10)), Math.toRadians(0), maxVel, maxAccel)
                .afterTime(0, intake2_0.deployFlowerRamp())
                .splineToSplineHeading(new Pose2d(24, -55, Math.toRadians(-90)), Math.toRadians(-90), maxVel, maxAccel)
                .afterTime(0, intake2_0.collectFromFlower(3));

        TrajectoryActionBuilder crossToShoot = toFlowerTwo.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-179.5)), Math.toRadians(15), maxVel, maxAccel);

        TrajectoryActionBuilder ToWallSetandShooting = crossToShoot.endTrajectory().fresh()
                .setTangent(Math.toRadians(-155))
                .afterTime(2.1, intake2_0.collect(2.5))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-24, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-52, -16, Math.toRadians(360)), Math.toRadians(120), maxVel, maxAccel);

        TrajectoryActionBuilder strafeToParking = ToWallSetandShooting.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50, -26, Math.toRadians(20)), Math.toRadians(-90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-53.5, -57, Math.toRadians(90)), Math.toRadians(-90), maxVel, maxAccel);

        Pose2d soloStart = new Pose2d(61, -14, Math.toRadians(180));
        Pose2d parkPose = new Pose2d(-53.5, -57, Math.toRadians(90));

        TrajectoryActionBuilder soloPreLoadWait = drive.actionBuilder(soloStart)
                .waitSeconds(2);

        TrajectoryActionBuilder soloWallSetandShooting = soloPreLoadWait.endTrajectory().fresh()
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                .afterTime(0.7, intake2_0.collect(4))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-52, -16, Math.toRadians(360)), Math.toRadians(-220), maxVel, maxAccel);

        TrajectoryActionBuilder soloTurnToFlowerOne = soloWallSetandShooting.endTrajectory().fresh()
                .afterTime(0, intake2_0.deployFlowerRamp())
                .setTangent(Math.toRadians(-172))
                .splineToSplineHeading(new Pose2d(-52.6, -17.5, Math.toRadians(180)), Math.toRadians(-112), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(-55, -22), Math.toRadians(-82), maxVel, maxAccel)
                .afterTime(0.4, intake2_0.collectFromFlower(3));

        TrajectoryActionBuilder soloTurnToShoot = soloTurnToFlowerOne.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(360)), Math.toRadians(90), maxVel, maxAccel);

        TrajectoryActionBuilder soloStrafeToParking = soloTurnToShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -57), Math.toRadians(90), maxVel, maxAccel);

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action shootPreLoad = waitForAlliancePreLoadPath.build();
        Action IntakeFromFlowerOne = turnToFlowerOne.build();
        Action shootFlowerOnePollen = turnToShoot.build();
        Action IntakeFromFlowerTwo = toFlowerTwo.build();
        Action shootFlowerTwoPollen = crossToShoot.build();
        Action IntakeWallSetandShoot = ToWallSetandShooting.build();
        Action park = strafeToParking.build();

        Action soloShootPreLoad = soloPreLoadWait.build();
        Action soloIntakeWallSetandShoot = soloWallSetandShooting.build();
        Action soloIntakeFromFlowerOne = soloTurnToFlowerOne.build();
        Action soloShootFlowerOnePollen = soloTurnToShoot.build();
        Action soloPark = soloStrafeToParking.build();

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, true);
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, new Pose2d(-52, -14, Math.toRadians(0)), soloStart);

        telemetry.addLine("Paths Built");
        telemetry.update();

        Action regularPath = new SequentialAction(
                new SequentialAction(
                        intake2_0.transferUsingRollersForTime(1.0, 0.8)
                ),
                new SequentialAction(
                        IntakeFromFlowerOne,
                        shootFlowerOnePollen,
                        intake2_0.transferUsingRollersForTime(1.0, 0.8)
                ),
                new SequentialAction(
                        IntakeFromFlowerTwo,
                        shootFlowerTwoPollen,
                        intake2_0.transferUsingRollersForTime(1.0, 0.8)
                ),
                new SequentialAction(
                        IntakeWallSetandShoot,
                        intake2_0.transferUsingRollersForTime(1.0, 0.8)
                ),
                new SequentialAction(
                        kit.beginPark(),
                        park
                )
        );

        Action allianceDidNotTip = new SequentialAction(
                HiveAutoKit.branch(() -> kit.route(drive.pose, soloStart)),
                new ParallelAction(
                        soloShootPreLoad,
                        intake2_0.transferUsingRollersForTime(1.0, 0.8)
                ),
                new SequentialAction(
                        soloIntakeWallSetandShoot,
                        intake2_0.transferUsingRollersForTime(1.0, 0.8)
                ),
                HiveAutoKit.branch(() -> {
                    if (kit.fits(5.08 + 1.66 + 1.0, new Pose2d(-52, -14, Math.toRadians(0)))) {
                        return new SequentialAction(
                                soloIntakeFromFlowerOne,
                                soloShootFlowerOnePollen,
                                intake2_0.transferUsingRollersForTime(1.0, 0.8),
                                kit.beginPark(),
                                soloPark
                        );
                    }
                    kit.log("no time for flower one");
                    return new SequentialAction(
                            kit.beginPark(),
                            HiveAutoKit.branch(() -> kit.route(drive.pose, parkPose))
                    );
                })
        );

        Action main = new SequentialAction(
                shootPreLoad,
                HiveAutoKit.waitUntil(() -> hive.settledUp(Side.LEFT, 0.4), MAX_WAIT_FOR_ALLIANCE_TIP_S),
                HiveAutoKit.branch(() -> {
                    if (!hive.settledUp(Side.LEFT, 0.6) && hive.settledUp(Side.RIGHT, 0.6)) {
                        kit.log("alliance did not tip - going RIGHT to run the 2 tip auto");
                        return allianceDidNotTip;
                    }
                    kit.log(hive.settledUp(Side.LEFT, 0.6) ? "alliance tipped - shooting" : "no HIVE view - shooting as planned");
                    return regularPath;
                })
        );

        double pollenShootingVel = BiobuzzRobotConstants.pollenFixedShootingVel;
        double nectarShootingVel = BiobuzzRobotConstants.nectarFixedShootingVel;

        while (!isStarted() && !isStopRequested()) {
            hive.update(drive.pose);
            telemetry.addLine("Paths Built - Ready To Start");
            telemetry.addData("HIVE", hive.describe());
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
