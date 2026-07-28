package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Distance;

@Config
@Disabled
@Autonomous
public class Auto3Specimen extends NGAutoOpMode {
    Distance rear_distance;
    public static int SPECIMEN_COUNT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(10, -64, Math.toRadians(0));
        initAuto(beginPose);

        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10,20);
            } else {
                return new MinMax(-50,80);
            }
        };
        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 30;
            } else {
                return 65;
            }
        };

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-15,40);
            } else {
                return new MinMax(-30,50);
            }
        };
        AccelConstraint pickup = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -40) {
                return new MinMax(-6,12);
            } else {
                return new MinMax(-30,50);
            }
        };
        VelConstraint pickupVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -40.0) {
                return 20;
            } else {
                return 40;
            }
        };





        double CHAMBER_Y = -34.5;
        Pose2d firstSample = new Pose2d(31, -34, Math.toRadians(30));
        Pose2d secondSample = new Pose2d(41, -34, Math.toRadians(30));
        Pose2d thirdSample = new Pose2d(48, -35, Math.toRadians(30));
        Pose2d pickupPosition = new Pose2d(40, -50.5,  Math.toRadians(270));
        TrajectoryActionBuilder scoreFirstSpecimenPath = drive.fastActionBuilder(beginPose)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(0,-45, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2,-34), Math.toRadians(90));

        TrajectoryActionBuilder moveToFirstSamplePath = drive.actionBuilder(new Pose2d(4, -34, Math.toRadians(270)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(firstSample, Math.toRadians(30), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15, 24));
        TrajectoryActionBuilder depositFirstSamplePath = moveToFirstSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(firstSample.position.x, -42, Math.toRadians(-70)), Math.toRadians(0), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 25));
        TrajectoryActionBuilder moveToSecondSamplePath = depositFirstSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(secondSample, Math.toRadians(30), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15, 24));
            TrajectoryActionBuilder depositSecondSamplePath = moveToSecondSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(40, -45,  Math.toRadians(270)) ,Math.toRadians(270),new TranslationalVelConstraint(12), new ProfileAccelConstraint(-8, 25));
        TrajectoryActionBuilder collectSecondSpecimenPath = depositSecondSamplePath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(pickupPosition.position.y);
//                .splineToLinearHeading(new Pose2d(secondSample.position.x, -42, Math.toRadians(-60)), Math.toRadians(0), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 25));
        TrajectoryActionBuilder moveToThirdSamplePath = drive.actionBuilder(new Pose2d(secondSample.position.x, -35, Math.toRadians(270)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(thirdSample, Math.toRadians(30), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15, 35));
        TrajectoryActionBuilder depositThirdSamplePath = drive.actionBuilder(thirdSample)
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(pickupPosition,Math.toRadians(270),new TranslationalVelConstraint(20), new ProfileAccelConstraint(-8, 25));

        Pose2d secondSpecimen = new Pose2d(2, CHAMBER_Y, Math.toRadians(270));
        Pose2d thirdSpecimen = new Pose2d(5, CHAMBER_Y, Math.toRadians(270));
        Pose2d fourthSpecimen = new Pose2d(8, CHAMBER_Y, Math.toRadians(270));
        Pose2d fifthSpecimen = new Pose2d(12, CHAMBER_Y, Math.toRadians(270));
        TrajectoryActionBuilder scoreSecondSpecimenPath = drive.fastActionBuilder(pickupPosition)
                .setTangent(Math.toRadians(170))

                .splineToConstantHeading(secondSpecimen.position, Math.toRadians(90)) ;
        TrajectoryActionBuilder collectThirdSpecimenPath = drive.fastActionBuilder(secondSpecimen)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(pickupPosition.position, Math.toRadians(0));
        TrajectoryActionBuilder scoreThirdSpecimenPath = drive.fastActionBuilder(pickupPosition)
                .setTangent(Math.toRadians(170))

                .splineToConstantHeading(thirdSpecimen.position, Math.toRadians(90));
        TrajectoryActionBuilder collectFourthSpecimenPath = drive.fastActionBuilder(thirdSpecimen)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(pickupPosition.position, Math.toRadians(0));
        TrajectoryActionBuilder scoreFourthSpecimenPath = drive.fastActionBuilder(pickupPosition)
                .setTangent(Math.toRadians(170))

                .splineToConstantHeading(fourthSpecimen.position, Math.toRadians(90));
        TrajectoryActionBuilder collectFifthSpecimenPath = drive.fastActionBuilder(fourthSpecimen)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(pickupPosition.position, Math.toRadians(0));
        TrajectoryActionBuilder scoreFifthSpecimenPath = drive.fastActionBuilder(pickupPosition)
                .strafeTo(fifthSpecimen.position);
        TrajectoryActionBuilder parkPath = scoreFourthSpecimenPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(pickupPosition.position, Math.toRadians(180)), Math.toRadians(0));



        Action firstSpecimen = scoreFirstSpecimenPath.build();
        Action moveToFirstSample = moveToFirstSamplePath.build();
        Action depositFirstSample = depositFirstSamplePath.build();
        Action moveToSecondSample = moveToSecondSamplePath.build();
        Action depositSecondSample = depositSecondSamplePath.build();
        Action moveToThirdSample = moveToThirdSamplePath.build();
        Action depositThirdSample = depositThirdSamplePath.build();
        Action scoreSecondSpecimen = scoreSecondSpecimenPath.build();
        Action collectThirdSpecimen = collectThirdSpecimenPath.build();
        Action scoreThirdSpecimen = scoreThirdSpecimenPath.build();
        Action collectFourthSpecimen = collectFourthSpecimenPath.build();
        Action scoreFourthSpecimen = scoreFourthSpecimenPath.build();
        Action collectFifthSpecimen = collectFifthSpecimenPath.build();
        Action scoreFifthSpecimen = scoreFifthSpecimenPath.build();
        Action park = parkPath.build();


        Action auto;
        SPECIMEN_COUNT = 4;
        auto = new SequentialAction(
                new ParallelAction(
                    intake.armAction(1600),

                    intake.slideAction(200),
                    new InstantAction(() -> intake.moveWrist(20)),
                    firstSpecimen
                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat)),
//                intake.slideAction(200, 400),

            transferSample(moveToFirstSample, depositFirstSample, 60, 900),
            transferSample(moveToSecondSample, depositSecondSample, 60, 900,true),
            collectSecondSpecimenPath.build(),
//depositThirdSample
//            transferSample(moveToThirdSample, depositThirdSample, 60, 900, true),
            scoreSpecimen(scoreSecondSpecimen, collectThirdSpecimen),
            scoreSpecimen(scoreThirdSpecimen, collectFourthSpecimen),
            scoreSpecimen(scoreFourthSpecimen, park)
//            ,scoreSpecimen(scoreFifthSpecimen, park)
        );



        telemetry.addLine("paths did the creation");
        telemetry.update();
//        intake.openClaw();
        intake.moveArm(400);
        while(!isStopRequested() && opModeInInit()){
            intake.update();
        }
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