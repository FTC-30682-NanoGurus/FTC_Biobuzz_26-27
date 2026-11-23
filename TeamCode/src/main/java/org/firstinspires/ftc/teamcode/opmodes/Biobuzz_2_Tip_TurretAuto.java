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
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

import java.util.Arrays;

@Config
@Autonomous
public class Biobuzz_2_Tip_TurretAuto extends NGAutoOpMode {
    public static double WALL_SET_DWELL_S = 0.25;
    public static double TIP_START_WAIT_S = 3.5;
    public static double BUDGET_SLACK_S = 0.0;

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

        Pose2d wallSet = new Pose2d(62, -58, Math.toRadians(360));
        Pose2d leftZero = new Pose2d(-55.46, -40.00, Math.toRadians(0));
        Pose2d leftDown = new Pose2d(-55.46, -40.00, Math.toRadians(-90));
        Pose2d rightNear = new Pose2d(56.69, -40.01, Math.toRadians(-75));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, Math.toRadians(-90));

        Turret turret = intake2_0.turret;
        Side[] aim = {Side.RIGHT};

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftDown, beginPose);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepare(leftDown, parkPose);
        kit.prepare(leftZero, parkPose);
        kit.prepare(rightNear, parkPose);
        kit.prepareFlower(leftZero, flowerOne);
        kit.prepareFlower(leftDown, flowerOne);
        kit.prepareFlower(rightNear, flowerTwo);
        kit.prepareFromFlower(flowerOne, leftDown);

        double volley = BiobuzzRobotConstants.volleyS() + BUDGET_SLACK_S;
        double F1_FROM_ZERO_S = 1.97 + BiobuzzRobotConstants.flowerCollectTime + 1.71 + volley;
        double F1_FROM_DOWN_S = 1.78 + BiobuzzRobotConstants.flowerCollectTime + 1.71 + volley;
        double F2_FROM_LEFT_S = 2.73 + BiobuzzRobotConstants.flowerCollectTime + 2.72 + volley;
        double F2_FROM_RIGHT_S = 1.67 + BiobuzzRobotConstants.flowerCollectTime + 2.72 + volley;

        Action wallSetToLeft = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -42, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -40, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(leftZero.position, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .build());

        Action wallSetToRight = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(rightNear, Math.toRadians(90), maxVel, maxAccel)
                .build());

        Action leftZeroToFlowerOne = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(leftZero, flowerOne)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action leftDownToFlowerOne = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(leftDown, flowerOne)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerOneToLeftDown = kit.routeFromFlower(flowerOne, leftDown);

        Action leftDownToFlowerTwo = kit.approachFlower(drive.actionBuilder(leftDown)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-28, -40, Math.toRadians(-90)), Math.toRadians(0), maxVel, maxAccel)
                        .splineToLinearHeading(new Pose2d(4, -41, Math.toRadians(-90)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, intake2_0.deployFlowerRamp()), flowerTwo)
                .afterTime(0, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime))
                .build();

        Action rightToFlowerTwo = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(rightNear, flowerTwo)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerTwoToLeftDown = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(180), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(-30, -40), Math.toRadians(180), maxVel, maxAccel)
                .splineToConstantHeading(leftDown.position, Math.toRadians(180), maxVel, maxAccel)
                .build();

        Action parkFromLeftDown = drive.actionBuilder(leftDown)
                .strafeToConstantHeading(parkPose.position, maxVel, maxAccel)
                .build();

        Action park = new SequentialAction(
                kit.beginPark(),
                kit.leg(leftDown, parkFromLeftDown, parkPose, null));

        Action aimLeft = HiveAutoKit.instant(() -> aim[0] = Side.LEFT);

        Action tippedPlan = new SequentialAction(
                aimLeft,
                wallSetToLeft,
                kit.aimedShot(turret),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_FROM_ZERO_S, leftDown)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftZero, leftZeroToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftDown, leftDown),
                            kit.aimedShot(turret));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_LEFT_S, leftDown)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    kit.log("flower two - safety set, always shot at LEFT");
                    return new SequentialAction(
                            kit.flowerLeg(leftDown, leftDownToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftDown, leftDown),
                            kit.aimedShot(turret));
                }),
                park);

        Action notTippedPlan = new SequentialAction(
                wallSetToRight,
                kit.aimedShot(turret),
                aimLeft,
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_RIGHT_S, leftDown)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(rightNear, rightToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftDown, leftDown),
                            kit.aimedShot(turret));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_FROM_DOWN_S, leftDown)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftDown, leftDownToFlowerOne, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftDown, leftDown),
                            kit.aimedShot(turret));
                }),
                park);

        Action main = new SequentialAction(
                kit.aimedShot(turret, BiobuzzRobotConstants.preloadSpinUpMaxS),
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

        kit.checkPrebuilt("wallSetToLeft", wallSetToLeft);
        kit.checkPrebuilt("wallSetToRight", wallSetToRight);
        kit.checkPrebuilt("leftZeroToFlowerOne", leftZeroToFlowerOne);
        kit.checkPrebuilt("leftDownToFlowerOne", leftDownToFlowerOne);
        kit.checkPrebuilt("flowerOneToLeftDown", flowerOneToLeftDown);
        kit.checkPrebuilt("leftDownToFlowerTwo", leftDownToFlowerTwo);
        kit.checkPrebuilt("rightToFlowerTwo", rightToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeftDown", flowerTwoToLeftDown);
        kit.checkPrebuilt("parkFromLeftDown", parkFromLeftDown);

        while (!isStarted() && !isStopRequested()) {
            if (AllianceSelector.poll(gamepad1, gamepad2)) hive.setAlliance(AllianceSelector.red());
            hive.update(drive.pose);
            turret.refresh();
            AllianceSelector.show(telemetry);
            telemetry.addData("Alliance check", hive.ownTagsSeen(2.0) ? "OK - our HIVE tags in view"
                    : hive.otherAllianceTagsSeen(2.0) ? "CHECK ALLIANCE - only the other alliance's tags seen" : "our HIVE tags not seen yet");
            telemetry.addData("Turret", "%.1f deg - must face FORWARD at INIT (zeroed there)", turret.getAngleDeg());
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
                        kit.guardPark(main),
                        kit.turretTracker(turret, () -> aim[0])
                )
        );
        PoseStorage.storeAutoEndPose(AllianceSelector.red(), drive.pose);
        turret.stop();
        hive.close();
    }
}
