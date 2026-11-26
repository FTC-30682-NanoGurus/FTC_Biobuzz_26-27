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
        double INTAKE_ZONE_Y = -48;
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

        Vector2d gardenEnd = new Vector2d(61, -61);
        Pose2d leftShot = new Pose2d(-55.46, -40.00, Math.toRadians(0));
        Pose2d leftHigh = new Pose2d(-55.46, -40.00, Math.toRadians(100));
        Pose2d flowerOneShot = new Pose2d(-52, -14, Math.toRadians(-20));
        Pose2d rightShot = new Pose2d(56.69, -40.01, Math.toRadians(235));
        Pose2d flowerOne = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-49.5, -58, Math.toRadians(0));

        Turret turret = intake2_0.turret;
        Side[] aim = {Side.RIGHT};

        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(drive, intake2_0, hive, telemetry, maxVel, maxAccel,
                parkPose, leftShot, beginPose);

        telemetry.addLine("Preparing fastest routes...");
        telemetry.update();
        kit.prepare(leftShot, parkPose);
        kit.prepare(leftHigh, parkPose);
        kit.prepare(flowerOneShot, parkPose);
        kit.prepare(rightShot, parkPose);
        kit.prepareFlower(leftShot, flowerOne);
        kit.prepareFlower(rightShot, flowerTwo);
        kit.prepareFromFlower(flowerOne, flowerOneShot);

        double volley = BiobuzzRobotConstants.volleyS() + BUDGET_SLACK_S;
        double F1_TIPPED_S = 1.97 + BiobuzzRobotConstants.flowerCollectTime + 1.69 + volley;
        double F2_TIPPED_S = 2.91 + BiobuzzRobotConstants.flowerCollectTime + 2.78 + volley;
        double F2_FROM_RIGHT_S = 1.88 + BiobuzzRobotConstants.flowerCollectTime + 2.78 + volley;
        double F1_NOT_TIPPED_S = 1.97 + BiobuzzRobotConstants.flowerCollectTime + 1.69 + volley;

        Action wallSetToLeft = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(60, -36, Math.toRadians(270)), Math.toRadians(-90), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(61, -48), Math.toRadians(-90), maxVel, maxAccel)
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(30, -40, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -40, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(leftShot.position, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .build(), HiveAutoKit.TOP_WALL_COLLECT_START_S);

        Action wallSetToRight = kit.withWallCollect(drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(60, -36, Math.toRadians(270)), Math.toRadians(-90), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(61, -48), Math.toRadians(-90), maxVel, maxAccel)
                .splineToConstantHeading(gardenEnd, Math.toRadians(-90), maxIntakeVel, maxAccel)
                .waitSeconds(WALL_SET_DWELL_S)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(rightShot, Math.toRadians(120), maxVel, maxAccel)
                .build(), HiveAutoKit.TOP_WALL_COLLECT_START_S);

        Action leftToFlowerOneTipped = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(leftShot, flowerOne)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action leftToFlowerOneNotTipped = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(leftShot, flowerOne)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerOneToShot = kit.routeFromFlower(flowerOne, flowerOneShot);

        Action flowerOneToLeftHigh = kit.departFlower(drive.actionBuilder(flowerOne), flowerOne)
                .splineToSplineHeading(leftHigh, Math.toRadians(180), maxVel, maxAccel)
                .build();

        Action shotToFlowerTwo = kit.approachFlower(drive.actionBuilder(flowerOneShot)
                        .setTangent(Math.toRadians(-60))
                        .splineToLinearHeading(new Pose2d(0, -42, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, intake2_0.deployFlowerRamp()), flowerTwo)
                .afterTime(0, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime))
                .build();

        Action rightToFlowerTwo = new SequentialAction(
                new ParallelAction(intake2_0.deployFlowerRamp(), kit.flowerRoute(rightShot, flowerTwo)),
                intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime));

        Action flowerTwoToLeftTipped = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(6, -42, Math.toRadians(0)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-30, -40, Math.toRadians(0)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(180), maxVel, maxAccel)
                .build();

        Action flowerTwoToLeftNotTipped = drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(HiveAutoKit.flowerDepart(flowerTwo), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(6, -42, Math.toRadians(0)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-30, -40, Math.toRadians(0)), Math.toRadians(180), maxVel, maxAccel)
                .splineToSplineHeading(leftShot, Math.toRadians(180), maxVel, maxAccel)
                .build();

        Action parkFromLeft = drive.actionBuilder(leftShot)
                .strafeToConstantHeading(parkPose.position, maxVel, maxAccel)
                .build();

        Action parkFromLeftHigh = drive.actionBuilder(leftHigh)
                .strafeToConstantHeading(parkPose.position, maxVel, maxAccel)
                .build();

        Action aimLeft = HiveAutoKit.instant(() -> aim[0] = Side.LEFT);

        Action tippedPlan = new SequentialAction(
                aimLeft,
                wallSetToLeft,
                kit.aimedShot(turret),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_TIPPED_S, flowerOneShot)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftToFlowerOneTipped, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToShot, flowerOneShot),
                            kit.aimedShot(turret));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_TIPPED_S, leftShot)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    kit.log("flower two - safety set, always shot at LEFT");
                    return new SequentialAction(
                            kit.flowerLeg(flowerOneShot, shotToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftTipped, leftShot),
                            kit.aimedShot(turret));
                }),
                kit.beginPark(),
                kit.leg(leftShot, parkFromLeft, parkPose, null));

        Action notTippedPlan = new SequentialAction(
                wallSetToRight,
                kit.aimedShot(turret),
                aimLeft,
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F2_FROM_RIGHT_S, leftShot)) {
                        kit.log("no time for flower two");
                        return kit.note("skipping flower two");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(rightShot, rightToFlowerTwo, flowerTwo, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerTwo, flowerTwoToLeftNotTipped, leftShot),
                            kit.aimedShot(turret));
                }),
                HiveAutoKit.branch(() -> {
                    if (!kit.fits(F1_NOT_TIPPED_S, leftHigh)) {
                        kit.log("no time for flower one");
                        return kit.note("skipping flower one");
                    }
                    return new SequentialAction(
                            kit.flowerLeg(leftShot, leftToFlowerOneNotTipped, flowerOne, intake2_0.collectFromFlower(BiobuzzRobotConstants.flowerCollectTime)),
                            kit.fromFlowerLeg(flowerOne, flowerOneToLeftHigh, leftHigh),
                            kit.aimedShot(turret));
                }),
                kit.beginPark(),
                HiveAutoKit.branch(() -> kit.leg(leftHigh, parkFromLeftHigh, parkPose, null)));

        double maxShotTurret = Math.max(Math.max(Math.abs(HiveAutoKit.turretAngleFor(leftShot, Side.LEFT)),
                Math.abs(HiveAutoKit.turretAngleFor(leftHigh, Side.LEFT))), Math.max(Math.abs(HiveAutoKit.turretAngleFor(flowerOneShot, Side.LEFT)),
                Math.max(Math.abs(HiveAutoKit.turretAngleFor(rightShot, Side.RIGHT)), Math.abs(HiveAutoKit.turretAngleFor(beginPose, Side.RIGHT)))));

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
        kit.checkPrebuilt("leftToFlowerOne", leftToFlowerOneTipped);
        kit.checkPrebuilt("flowerOneToShot", flowerOneToShot);
        kit.checkPrebuilt("flowerOneToLeftHigh", flowerOneToLeftHigh);
        kit.checkPrebuilt("shotToFlowerTwo", shotToFlowerTwo);
        kit.checkPrebuilt("rightToFlowerTwo", rightToFlowerTwo);
        kit.checkPrebuilt("flowerTwoToLeft", flowerTwoToLeftTipped);
        kit.checkPrebuilt("parkFromLeft", parkFromLeft);
        kit.checkPrebuilt("parkFromLeftHigh", parkFromLeftHigh);

        while (!isStarted() && !isStopRequested()) {
            if (AllianceSelector.poll(gamepad1, gamepad2)) hive.setAlliance(AllianceSelector.red());
            hive.update(drive.pose);
            turret.refresh();
            AllianceSelector.show(telemetry);
            telemetry.addData("Alliance check", hive.ownTagsSeen(2.0) ? "OK - our HIVE tags in view"
                    : hive.otherAllianceTagsSeen(2.0) ? "CHECK ALLIANCE - only the other alliance's tags seen" : "our HIVE tags not seen yet");
            telemetry.addData("Turret", "%.1f deg - must face FORWARD at INIT (zeroed there)", turret.getAngleDeg());
            if (maxShotTurret > Turret.maxAngleDeg() - 3.0) {
                telemetry.addLine(String.format("TURRET TRAVEL TOO SMALL: shots need %.0f deg, TURRET_LIMIT_DEG is %.0f", maxShotTurret, Turret.maxAngleDeg()));
            }
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
