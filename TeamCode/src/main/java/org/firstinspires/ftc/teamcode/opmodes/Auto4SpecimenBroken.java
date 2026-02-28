package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.specimen_deliver;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
//@Config
//@Autonomous
public class Auto4SpecimenBroken extends NGAutoOpMode {
    Distance rear_distance;
    public static int SPECIMEN_COUNT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(10, -64, Math.toRadians(270 ));
        initAuto(beginPose);

        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);

        TrajectoryActionBuilder scoreFirstSpecimenPath = drive.actionBuilder(beginPose)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .setReversed(false)
                .lineToY(-33.1);
        TrajectoryActionBuilder clearSamples = scoreFirstSpecimenPath.endTrajectory().fresh()
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(15, -42), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30,40))
                .splineToConstantHeading(new Vector2d(30, -42), Math.toRadians(0), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,50))
                .splineToConstantHeading(new Vector2d(32, -5), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,40))

                .splineToConstantHeading(new Vector2d(45, -8), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30,40))
                .splineToConstantHeading(new Vector2d(45, -50), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,50))
                .splineToConstantHeading(new Vector2d(45, -5), Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,40))

                .splineToConstantHeading(new Vector2d(54, -8), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30,40))
                .splineToConstantHeading(new Vector2d(54, -50), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,50))
                .splineToConstantHeading(new Vector2d(54, -5),  Math.toRadians(90), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,40))

                .splineToConstantHeading(new Vector2d(65, -8), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30,40))
                .splineToConstantHeading(new Vector2d(65, -50), Math.toRadians(270), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,50))
                .splineToConstantHeading(new Vector2d(25, -58), Math.toRadians(180), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20,40));

        TrajectoryActionBuilder moveForward1 = clearSamples.endTrajectory().fresh()
                .lineToX(32, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-7,50));

        TrajectoryActionBuilder scoreSecondSpecimenPath = moveForward1.endTrajectory().fresh()
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToLinearHeading(new Pose2d(1, -33.1, Math.toRadians(270)), Math.toRadians(90));
        TrajectoryActionBuilder pickThirdSpecimenPath = scoreSecondSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(1, -42, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(25, -58, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20,40));
        TrajectoryActionBuilder moveForward2 = pickThirdSpecimenPath.endTrajectory().fresh()
                .lineToX(32, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-7,50));

        TrajectoryActionBuilder scoreThirdSpecimenPath = moveForward2.endTrajectory().fresh()
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToLinearHeading(new Pose2d(4, -33.1, Math.toRadians(270)), Math.toRadians(90));
        TrajectoryActionBuilder pickFourthSpecimenPath = scoreThirdSpecimenPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(4, -42, Math.toRadians(270)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(25, -58, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20,40));
        TrajectoryActionBuilder moveForward3 = pickFourthSpecimenPath.endTrajectory().fresh()
                .lineToX(32, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-7,50));
        TrajectoryActionBuilder scoreFourthSpecimenPath = moveForward3.endTrajectory().fresh()
                .setReversed(true)
                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.specimen_deliver)))
                .splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(270)), Math.toRadians(90));
        TrajectoryActionBuilder moveBackPath =scoreFourthSpecimenPath.endTrajectory().fresh()
                .lineToY(-40);
        Action clearSampleAction = clearSamples.build();
        Action scoreFirstSpecimen = scoreFirstSpecimenPath.build();
        Action scoreSecondSpecimen = scoreSecondSpecimenPath.build();
        Action pickThirdSpecimen = pickThirdSpecimenPath.build();
        Action scoreThirdSpecimen = scoreThirdSpecimenPath.build();
        Action pickFourthSpecimen = pickFourthSpecimenPath.build();
        Action scoreFourthSpecimen = scoreFourthSpecimenPath.build();
        Action moveForwardAction1= moveForward1.build();
        Action moveForwardAction2 = moveForward2.build();
        Action moveForwardAction3 = moveForward3.build();
        Action moveBack = moveBackPath.build();


        Action auto;
        SPECIMEN_COUNT = 4;
//        if(SPECIMEN_COUNT == 4){
            auto = new SequentialAction(
                    new ParallelAction(
                            new SequentialAction(
                                intake.armAction(ARM_LIMIT,100),
                                new InstantAction(() ->intake.moveWrist(specimen_deliver )),
                                intake.slideAction(200),
                                intake.armAction(ARM_LIMIT)
                            ),
                            scoreFirstSpecimen
                    ),
                    intake.slideAction(0),
                    new ParallelAction(
                        clearSampleAction,
                        new SequentialAction(
                                openClawAfterDistance(7.5, rear_distance),
                                new InstantAction(() -> intake.moveWrist(0)),
                                intake.armAction(0)
                        )
                    ),
                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                    //trafficLight.warnHuman(),
                    moveForwardAction1,
                    intake.grab(RobotConstants.claw_closed),
                    new InstantAction(() -> rear_distance.setOn(true)),
                    new ParallelAction(
                            scoreSecondSpecimen,
                            //trafficLight.disable(),
                        intake.armAction(ARM_LIMIT),
                        new InstantAction(() ->intake.moveWrist(specimen_deliver ))
                    ),
                    new ParallelAction(
                            pickThirdSpecimen,
                            new SequentialAction(
                                    openClawAfterDistance(8.5, rear_distance),
                                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                    intake.armAction(0)
                            )
                    ),
                //trafficLight.warnHuman(),
                    moveForwardAction2,
                    intake.grab(RobotConstants.claw_closed),
                    new InstantAction(() -> rear_distance.setOn(true)),
                    new ParallelAction(
                            scoreThirdSpecimen,
                            //trafficLight.disable(),
                            intake.armAction(ARM_LIMIT),
                            new InstantAction(() ->intake.moveWrist(specimen_deliver + 5))),

                    new ParallelAction(
                            pickFourthSpecimen,
                            new SequentialAction(
                                    openClawAfterDistance(8.5, rear_distance),
                                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                    intake.armAction(0)
                            )
                    ),
                    //trafficLight.warnHuman(),
                    moveForwardAction3,
                    new InstantAction(() -> rear_distance.setOn(true)),
                    new ParallelAction(scoreFourthSpecimen,
                            intake.armAction(ARM_LIMIT),
                         new InstantAction(() ->intake.moveWrist(specimen_deliver + 5))
                    ),
                    new ParallelAction(
                            moveBack,
                            new SequentialAction(
                                    openClawAfterDistance(8.5, rear_distance),
                                    new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)),
                                    intake.armAction(0)
                            )
                    )
                );

//        }
//        else if(SPECIMEN_COUNT == 3){
//            auto = new SequentialAction(scoreFirstSpecimen,
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance),
//                    throwFirstSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    throwSecondSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    clearThirdSample,
//                    trafficLight.warnHuman(),
//
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable()),
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance),
//                    pickThirdSpecimen,
//                    trafficLight.warnHuman(),
//
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    new ParallelAction(scoreThirdSpecimen,trafficLight.disable()),
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance));
//        }
//        else if(SPECIMEN_COUNT == 2){
//            auto = new SequentialAction(scoreFirstSpecimen,
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance),
//                    throwFirstSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    throwSecondSample,
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
//                    intake.yeetSample(),
//                    clearThirdSample,
//                    trafficLight.warnHuman(),
//                    drive.moveUsingDistance(intake.distance, 2, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
//                    new ParallelAction(scoreSecondSpecimen,trafficLight.disable()),
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance));
//        }
//        else{
//            auto = new SequentialAction(
//                    scoreFirstSpecimen,
//                    raiseArmForSpecimen(),
//                    scoreSpecimen(rear_distance)
//            );
//        }


        telemetry.clear();
        telemetry.addLine("Arm Lowered, ready to go");
        telemetry.update();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        intake.updateAction(),
                        //trafficLight.updateAction(),
                        rear_distance.updateAction(),
                        intake.distance.updateAction(),
                        auto
                )
        );
        //Arm to 100
        //Slides to 650
        //Wrist to 0.45


    }
    public Action driveAndGrab(){
        return new SequentialAction(
                //drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, 12),
                intake.grab(RobotConstants.claw_closed)
            );
    }
}