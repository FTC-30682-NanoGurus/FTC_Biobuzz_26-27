package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.NullAction;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Autonomous
@Disabled
public class Auto1Specimen extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-9, -64, Math.toRadians(180));
        double INCHES_FORWARD = 0;
        initAuto(beginPose);
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() > -30.0) {
                return new MinMax(-10,12);
            } else {
                return new MinMax(-30,50);
            }
        };

        double thirdSampleAngleDegrees = 123.5;
        Pose2d basket = new Pose2d(-60, -57, Math.toRadians(45));
        Pose2d sample1Pickup = new Pose2d(-48, -50 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample2Pickup = new Pose2d(-58, -50 + INCHES_FORWARD, Math.toRadians(90));
        Pose2d sample3Pickup = new Pose2d(-55 , -46 + INCHES_FORWARD, Math.toRadians(thirdSampleAngleDegrees));


        FailoverAction firstDelay = new FailoverAction(new SleepAction(5), new NullAction());
        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-2,-45, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-2,-36), Math.toRadians(90));

        TrajectoryActionBuilder firstSamplePath = drive.actionBuilder(new Pose2d(-2, -36.5, Math.toRadians(270)))
                .setReversed(false)
                .setTangent(Math.toRadians(225))
                .afterDisp(22, new InstantAction(firstDelay::failover))
                .splineToLinearHeading(sample1Pickup, Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20,50));

        TrajectoryActionBuilder scoreFirstSamplePath = drive.actionBuilder(sample1Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample1Pickup.position.y - basket.position.y, sample1Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(sample2Pickup, Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32));

        TrajectoryActionBuilder scoreSecondSamplePath = drive.actionBuilder(sample2Pickup)
                .setReversed(true)
                .setTangent(Math.PI + Math.atan2(sample2Pickup.position.y - basket.position.y, sample2Pickup.position.x - basket.position.x))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40));

        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(sample3Pickup, Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32));
        TrajectoryActionBuilder scoreThirdSamplePath = drive.fastActionBuilder(sample3Pickup)
                .setReversed(true)
                .setTangent(Math.toRadians(thirdSampleAngleDegrees + 180))
                .splineToLinearHeading( new Pose2d(-61, -58, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,30));

        TrajectoryActionBuilder parkPath = scoreThirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-26, -12, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,40));

        FailoverAction scoreSpecimen = new FailoverAction(scoreSpecimenPath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        FailoverAction firstSample = new FailoverAction(firstSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreFirstSample = scoreFirstSamplePath.build();
        FailoverAction secondSample = new FailoverAction( secondSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreSecondSample = scoreSecondSamplePath.build();
        FailoverAction thirdSample = new FailoverAction(thirdSamplePath.build(), new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))));
        Action scoreThirdSample = scoreThirdSamplePath.build();
        Action park = parkPath.build();
        intake.moveArm(400);
        while(!isStopRequested() && opModeInInit()){
            intake.update();
        }

        Actions.runBlocking(
                new ParallelAction(
                        intake.updateAction(),
                        //trafficLight.updateAction(),
                        new SequentialAction(
                                scoreSpecimenSampleSide(scoreSpecimen),
//                                new ParallelAction(
//                                        intake.armAction(1000),
//
//                                        intake.slideAction(200),
//                                        new InstantAction(() -> intake.moveWrist(70)),
//                                        scoreSpecimen
//                                ),
//                                intake.armAction(1300),
//                                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat)),
                                goToSampleWithSlides(firstSample, firstDelay),

                                slideCollectSampleAndScore(scoreFirstSample, RobotConstants.claw_flat),
                                goToSampleWithSlides(secondSample),
                                slideCollectSampleAndScore(scoreSecondSample, RobotConstants.claw_flat),
                                goToSampleWithSlides(thirdSample, 90-thirdSampleAngleDegrees),
                                slideCollectSampleAndScore(scoreThirdSample, RobotConstants.claw_flat),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(1),
                                                new InstantAction(() -> intake.moveWrist(0))
                                        ),
                                        new SequentialAction(
                                                intake.slideAction(0),
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