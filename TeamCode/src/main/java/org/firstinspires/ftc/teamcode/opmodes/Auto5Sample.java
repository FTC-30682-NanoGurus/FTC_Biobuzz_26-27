package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous
public class Auto5Sample extends NGAutoOpMode {
    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-31, -64, Math.toRadians(0));
        camera = new Camera(hardwareMap, telemetry);
        camera.init();

        initAuto(beginPose);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() > -30.0) {
                return new MinMax(-10,12);
            } else {
                return new MinMax(-30,50);
            }
        };
        telemetry.clear();
        telemetry.addLine("x for Blue Field 1");
        telemetry.addLine("a for Red Field 1");
        telemetry.addLine("y for Blue Field 2");
        telemetry.addLine("b for Red Field 2");
        telemetry.update();
        double fourthSampleAngleDegrees = 123.5;
        double INCHES_FORWARD = -1.8;
        boolean exit = false;
        String side = "No Side";
        Pose2d basket = new Pose2d(-57, -57, Math.toRadians(45));
        Pose2d sample4Pickup = new Pose2d(-57 , -46.5 + INCHES_FORWARD, Math.toRadians(fourthSampleAngleDegrees));
        Pose2d fourthBasket = new Pose2d(-57, -57, Math.toRadians(45));
        Pose2d fifthBasket = new Pose2d(-57, -57, Math.toRadians(45));
        Pose2d firstBasket = new Pose2d(-57, -57, Math.toRadians(45));
        while (!isStopRequested()) {
            while (!isStopRequested()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Field 1");
                    telemetry.update();
                    side = "Blue Field 1";

                    fourthSampleAngleDegrees = 123.5;
                    INCHES_FORWARD = 0.5;
                    basket = new Pose2d(-54, -54, Math.toRadians(45));
                    fourthBasket = new Pose2d(-54.5, -54.5, Math.toRadians(45));
                    fifthBasket = new Pose2d(-55, -55, Math.toRadians(45));
                    sample4Pickup = new Pose2d(-57, -48 + INCHES_FORWARD, Math.toRadians(fourthSampleAngleDegrees));
                    firstBasket = new Pose2d(-54, -54, Math.toRadians(45));


                    /**
                     *   fourthSampleAngleDegrees = 123.5;
                     *                     INCHES_FORWARD = -1.8;
                     *                     basket = new Pose2d(-58, -59, Math.toRadians(45));
                     *                     fourthBasket = new Pose2d(-59, -59, Math.toRadians(45));
                     *                     sample4Pickup = new Pose2d(-57, -46.5 + INCHES_FORWARD, Math.toRadians(fourthSampleAngleDegrees));
                     */

                    break;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Field 2");
                    telemetry.update();
                    side = "Blue Field 2";

                    fourthSampleAngleDegrees = 123.5;
                    INCHES_FORWARD = -0.5;
                    basket = new Pose2d(-54, -54, Math.toRadians(45));
                    fourthBasket = new Pose2d(-54.5, -54.5, Math.toRadians(45));
                    fifthBasket = new Pose2d(-55, -55, Math.toRadians(45));

                    sample4Pickup = new Pose2d(-57, -47.5 + INCHES_FORWARD, Math.toRadians(fourthSampleAngleDegrees));
                    firstBasket = new Pose2d(-55, -55, Math.toRadians(45));


                    break;
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Field 1");
                    telemetry.update();
                    side = "Red Field 1";

                    fourthSampleAngleDegrees = 123.5;
                    INCHES_FORWARD = -0;
                    basket = new Pose2d(-54, -54, Math.toRadians(45));
                    fourthBasket = new Pose2d(-54, -54, Math.toRadians(45));
                    fifthBasket = new Pose2d(-55, -55, Math.toRadians(45));
                    sample4Pickup = new Pose2d(-57, -48 + INCHES_FORWARD, Math.toRadians(fourthSampleAngleDegrees));
                    firstBasket = new Pose2d(-55, -55, Math.toRadians(45));


                    break;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Field 2");
                    telemetry.update();

                    fourthSampleAngleDegrees = 123.5;
                    INCHES_FORWARD = -0.5;
                    basket = new Pose2d(-54, -54, Math.toRadians(45));
                    fourthBasket = new Pose2d(-54.5, -54.5, Math.toRadians(45));
                    fifthBasket = new Pose2d(-55, -55, Math.toRadians(45));

                    sample4Pickup = new Pose2d(-57, -47.5 + INCHES_FORWARD, Math.toRadians(fourthSampleAngleDegrees));
                    firstBasket = new Pose2d(-55, -55, Math.toRadians(45));



                    side = "Red Field 2";

                    break;
                }
            }
            sleep(200);
            telemetry.addLine("Press x to confirm, press y to restart");
            telemetry.update();
            while (!isStopRequested()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    exit = true;
                    telemetry.addLine("Ready to go");
                    telemetry.update();
                    break;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    exit = false;
                    telemetry.clear();
                    telemetry.addLine("x for Blue Field 1");
                    telemetry.addLine("a for Red Field 1");
                    telemetry.addLine("y for Blue Field 2");
                    telemetry.addLine("b for Red Field 2");
                    telemetry.update();
                    break;
                }
            }
            sleep(200);
            if(exit){
                break;
            }
        }
        //rigging.disableServos();
        camera.stopCamera();
        telemetry.addLine("Ready to go");
        telemetry.update();
        Pose2d sample2Pickup = new Pose2d(-48, -49.5 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample3Pickup = new Pose2d(-58, -50 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample5Pickup = new Pose2d(-26, -12, Math.toRadians(0));
        if(side.equals("Blue Field 1") || side.equals("Red Field 1")){
            sample2Pickup = new Pose2d(-48, -48 + INCHES_FORWARD, Math.toRadians(90));
        }

        TrajectoryActionBuilder firstSamplePath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firstBasket, Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,50));

        TrajectoryActionBuilder secondSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(sample2Pickup, Math.toRadians(0), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15,35));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.actionBuilder(sample2Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample2Pickup.position.y - basket.position.y, sample2Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,50));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(sample3Pickup, Math.toRadians(90), new TranslationalVelConstraint(30),  new ProfileAccelConstraint(-15,35));

        TrajectoryActionBuilder scoreThirdSamplePath = drive.actionBuilder(sample3Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample3Pickup.position.y - basket.position.y, sample3Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,50));

        TrajectoryActionBuilder fourthSamplePath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(sample4Pickup, Math.toRadians(90), new TranslationalVelConstraint(30),  new ProfileAccelConstraint(-15,35));
        TrajectoryActionBuilder scoreFourthSamplePath = drive.actionBuilder(sample4Pickup)
                .setReversed(true)
                .setTangent(Math.toRadians(fourthSampleAngleDegrees + 180))
                .splineToLinearHeading(fourthBasket, Math.toRadians(225), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-25,50));
//new Pose2d(-59, -56, Math.toRadians(45))
        TrajectoryActionBuilder fifthSamplePath = scoreFourthSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-54, -45, Math.toRadians(0)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(sample5Pickup, Math.toRadians(0), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,40));
        TrajectoryActionBuilder scoreFifthSamplePath = drive.actionBuilder(sample5Pickup)
                .setReversed(true)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(fifthBasket, Math.toRadians(225));
        TrajectoryActionBuilder parkPath = scoreFifthSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(sample5Pickup, Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30,50));

        FailoverAction firstSample = new FailoverAction(firstSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction secondSample = new FailoverAction( secondSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreSecondSample = scoreSecondSamplePath.build();
        FailoverAction thirdSample = new FailoverAction(thirdSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreThirdSample = scoreThirdSamplePath.build();
        FailoverAction fourthSample = new FailoverAction( fourthSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreFourthSample = scoreFourthSamplePath.build();
        Action fifthSample = fifthSamplePath.build();
        Action scoreFifthSample = scoreFifthSamplePath.build();
        Action park = parkPath.build();

        telemetry.clear();
        telemetry.addLine("Paths built for " + side);
        telemetry.update();
        
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        intake.updateAction(),
                        //trafficLight.updateAction(),
                        new SequentialAction(
//                                slideCollectSampleAndScore(firstSample, RobotConstants.claw_flat, true),
                                new ParallelAction(
                                        firstSample,
                                        new SequentialAction(
                                                new SleepAction(0.1),
                                                intake.raiseArm(false)
                                        )
                                ),

                                new InstantAction(() -> intake.arm.setManualPower(0.5)),
                                new InstantAction(() -> intake.closeClaw(-0.05)),
                                new SleepAction(0.2),
                                new InstantAction(() -> intake.arm.setManualPower(0)),
                                intake.scoreSlidePickup(),
                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat)),
                                goToSampleWithSlides(secondSample),
                                slideCollectSampleAndScore(scoreSecondSample, RobotConstants.claw_flat, true),
                                goToSampleWithSlides(thirdSample),
                                slideCollectSampleAndScore(scoreThirdSample, RobotConstants.claw_flat, true),
                                goToSampleWithSlides(fourthSample, 90-fourthSampleAngleDegrees, 300),
                                slideCollectSampleAndScore(scoreFourthSample, RobotConstants.claw_open, true),
                                new ParallelAction(
                                        new InstantAction(() -> intake.arm.setManualPower(0)),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                new InstantAction(() -> intake.turnAndRotateClaw(90,0))
                                        ),
                                        new SequentialAction(
                                            new SleepAction(0.1),
                                            intake.slideAction(0, 600),
                                            new InstantAction(
                                                    () -> {
                                                        vihasCameraArm.camera();

                                                    }
                                            ),
                                            intake.armAction(400)

                                        ),
                                        new SequentialAction(
                                            fifthSample,
                                            new InstantAction(() -> camera.startCamera())
                                        )

                                ),
                                drive.isStationary(),
                                camera.waitForYellow(),
                                obtainSampleWithCamera(camera, drive),

                                slideCollectSampleAndScore(scoreFifthSample, new SleepAction(0.8), RobotConstants.claw_open, true),
                                new SleepAction(0.15),
                                new InstantAction(() -> intake.arm.setManualPower(0)),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(1),
                                                new InstantAction(() -> intake.moveWrist(0))
                                        ),
                                        new SequentialAction(
                                                new SleepAction(0.4),
                                                intake.slideAction(0, 600),
                                                intake.armAction(0)
                                        ),
                                        park,
                                        new InstantAction(() -> vihasCameraArm.level1())
                                )
                        )
                )
        );
    }
}