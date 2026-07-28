package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;


@Disabled
public class DistanceRRPathTest extends TestingOpMode {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();

        Pose2d beginPose = new Pose2d(-10, -63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        intake = new Intake(hardwareMap, telemetry);
        intake.init();

        intake.calculateOffset();
        intake.moveClaw(0.99);
        waitForStart();
        TrajectoryActionBuilder scoreSpecimenPath = drive.actionBuilder(beginPose)
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(0.6)))
                .splineToLinearHeading(new Pose2d(-6, -33.5 , Math.toRadians(270)), Math.toRadians(90));
//                .lineToY(-40);
        TrajectoryActionBuilder firstSamplePath = scoreSpecimenPath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-6, -50, Math.toRadians(270)), Math.toRadians(270));

//                .afterTime(0.6, new InstantAction(() -> intake.moveClaw(0.9)))
//                .splineTo(new Vector2d(-46,-29 ), Math.toRadians(180), new TranslationalVelConstraint(5), new ProfileAccelConstraint(-10,10));



        TrajectoryActionBuilder scoreFirstSamplePath = firstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-45, -57, Math.toRadians(270)), Math.toRadians(270));

        TrajectoryActionBuilder secondSamplePath = scoreFirstSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .afterTime(0.5, new InstantAction(() -> intake.moveClaw(0.7)))
                .splineToLinearHeading(new Pose2d(-50, -44, Math.toRadians(90)), Math.toRadians(90));


        TrajectoryActionBuilder scoreSecondSamplePath = secondSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-43, -55, Math.toRadians(45)), Math.toRadians(225));
        TrajectoryActionBuilder thirdSamplePath = scoreSecondSamplePath.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd(new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position + 0.05)))
                .splineToSplineHeading(new Pose2d(-40,-40, Math.toRadians(180)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47,-27), Math.toRadians(180));
        TrajectoryActionBuilder scoreThirdSamplePath = thirdSamplePath.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-43, -55, Math.toRadians(45)), Math.toRadians(225));
//        TrajectoryActionBuilder backToStartPath = scoreFirstSamplePath.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(-10, -50, Math.toRadians(270)),0);



        Action scoreSpecimen = scoreSpecimenPath.build();
        Action firstSample = firstSamplePath.build();
        Action scoreFirstSample = scoreFirstSamplePath.build();
        Action secondSample = secondSamplePath.build();
        Action scoreSecondSample = scoreSecondSamplePath.build();
        Action thirdSample = thirdSamplePath.build();
        Action scoreThirdSample = scoreThirdSamplePath.build();
//        Action backToStart = backToStartPath.build();
//        Action scoreAction = new SequentialAction(
//                intake.armAction(1450),
//                new InstantAction(() -> intake.openClaw()),
//                new SleepAction(0.1),
//                new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
//                new SleepAction(0.3),
//                intake.slideAction(0)
//        );
//        Action raiseArm = new SequentialAction(
//                intake.armAction(1400, 600),
//                new ParallelAction(
////                        intake.slideAction(slide_extension),
//                        intake.armAction(1400)
//                ),
//                new InstantAction(() -> intake.moveWrist(0.9))
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        intake.updateAction(),
//                        new SequentialAction(
//    //                            intake.armAction(0),
//                                new InstantAction(() -> intake.moveClaw(0.7)),
//                                new SleepAction(0.2),
//                                new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
//                                new SleepAction(0.4),
//                                drive.moveUsingDistance(intake.distance, 2.5, 4),
//                                new InstantAction(() -> intake.moveClaw(0.9)),
//                                new SleepAction(35)
////                                new SleepAction(0.3),
////                                new InstantAction(() -> intake.moveClaw(0.7)),
////                                new SleepAction(0.3),
////                                new InstantAction(() -> intake.moveClaw(0.95))
//
//                        )
//                )
//        );
        Actions.runBlocking(
                new ParallelAction(
                    intake.updateAction(),
                    new SequentialAction(
                            new ParallelAction(
                                    scoreSpecimen,
                                    intake.armAction(1550)
                            )
//                            drive.moveUsingDistance(intake.distance, 20.8, 21, 20)
                            ,new SleepAction(0.3),
                                    new InstantAction(() -> intake.moveClaw(0.7)),
                                    new SleepAction(0.4),
                                    new ParallelAction(
                                            firstSample,
                                            intake.armAction(0)
                                    )
//                            new InstantAction(() -> intake.moveClaw(.7)),
//                            new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
//                            new SleepAction(0.5),
//                            intake.grab(0.9)
                    )));
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
}