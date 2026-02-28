package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

@Autonomous
public class BlueGoalSideFastAuto extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.resetPose();

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < -24.0) {
                return new MinMax(-2,2);
            } else {
                return new MinMax(-100,100);
            }
        };

/*TrajectoryActionBuilder PathToGate = collectFirstSet.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-7, -44.5), Math.toRadians(270))
                .lineToY(-56, new TranslationalVelConstraint(12));
                    new SleepAction(2);*/

        Pose2d beginPose = new Pose2d(-52, -48, Math.toRadians(235));
        initAuto(beginPose);

        TrajectoryActionBuilder moveBackwardPath = drive.actionBuilder(beginPose)
                .lineToY(-27, new TranslationalVelConstraint(60), new ProfileAccelConstraint(-120, 120))
                .afterTime(1, intake2_0.transferUsingRollersForTime(0.7, 1));

        TrajectoryActionBuilder ToFirstSet = moveBackwardPath.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-18, -24, Math.toRadians(260)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 60))
                .afterTime(0, intake2_0.collect(1.0))
                .splineToLinearHeading(new Pose2d(-18, -55, Math.toRadians(260)), Math.toRadians(-100), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 30))
                .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(232)), Math.toRadians(100), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50));

        TrajectoryActionBuilder ToSecondSet = ToFirstSet.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(16, -23, Math.toRadians(236)), Math.toRadians(-118), new TranslationalVelConstraint(55),  new ProfileAccelConstraint(-120, 120))
                .afterTime(0, intake2_0.collect(1.0))
                .splineToLinearHeading(new Pose2d(16, -56, Math.toRadians(236)), Math.toRadians(-118), new TranslationalVelConstraint(38))
                .splineToSplineHeading(new Pose2d(18, -23, Math.toRadians(250)), Math.toRadians(118), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(232)), Math.toRadians(118), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40));

        TrajectoryActionBuilder ToFirstGateSet = ToSecondSet.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(8, -47.6, Math.toRadians(240)), Math.toRadians(-120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(8, -57))
                .strafeToLinearHeading(new Vector2d(21, -59), Math.toRadians(210), new TranslationalVelConstraint(60))
                .afterTime(0, intake2_0.collect(1.0))
                .strafeToLinearHeading(new Vector2d(10, -59), Math.toRadians(210), new TranslationalVelConstraint(42));

        TrajectoryActionBuilder ToGoal1 = ToFirstGateSet.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(7.7, -33, Math.toRadians(239)), Math.toRadians(90), new TranslationalVelConstraint(60))
                .afterTime(0.8, intake2_0.transferUsingRollersForTime(1.0, 1))
                .splineToSplineHeading(new Pose2d(-38, -25, Math.toRadians(242)), Math.toRadians(-200), new TranslationalVelConstraint(60), smartScore)
                .splineToLinearHeading(new Pose2d(8, -47.6, Math.toRadians(240)), Math.toRadians(-120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(8, -57))
                .strafeToLinearHeading(new Vector2d(21, -59), Math.toRadians(210), new TranslationalVelConstraint(60))
                .afterTime(0, intake2_0.collect(1.0))
                .strafeToLinearHeading(new Vector2d(10, -59), Math.toRadians(210), new TranslationalVelConstraint(42));

        TrajectoryActionBuilder ToGoal2 = ToGoal1.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(7.7, -33, Math.toRadians(239)), Math.toRadians(90), new TranslationalVelConstraint(60))
                .afterTime(0.7, intake2_0.transferUsingRollersForTime(1.0, 1))
                .splineToSplineHeading(new Pose2d(-38, -25, Math.toRadians(242)), Math.toRadians(-200), new TranslationalVelConstraint(60), smartScore)
                .splineToLinearHeading(new Pose2d(8, -47.6, Math.toRadians(240)), Math.toRadians(-120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(8, -57))
                .strafeToLinearHeading(new Vector2d(21, -59), Math.toRadians(210), new TranslationalVelConstraint(60))
                .afterTime(0, intake2_0.collect(1.0))
                .strafeToLinearHeading(new Vector2d(10, -59), Math.toRadians(210), new TranslationalVelConstraint(42));
        /*.splineToSplineHeading(new Pose2d(10.5, -33, Math.toRadians(239)), Math.toRadians(90), new TranslationalVelConstraint(60))
                .afterTime(1.1, intake2_0.transferUsingRollersForTime(1.2, 1))
                .splineToSplineHeading(new Pose2d(-50, -25, Math.toRadians(285)), Math.toRadians(-200), new TranslationalVelConstraint(60), smartScore)
                .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(243)), Math.toRadians(-120), new TranslationalVelConstraint(40));*/
        //.strafeToConstantHeading(new Vector2d(9, -58));

        TrajectoryActionBuilder ToGoal3 = ToGoal2.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(7.7, -33, Math.toRadians(239)), Math.toRadians(90), new TranslationalVelConstraint(60))
                .afterTime(1.2, intake2_0.transferUsingRollersForTime(1.0, 1))
                .splineToSplineHeading(new Pose2d(-50, -25, Math.toRadians(275)), Math.toRadians(-200), new TranslationalVelConstraint(60), smartScore);

        TrajectoryActionBuilder leaveFromGate = ToGoal2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(10, -36), new TranslationalVelConstraint(120));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveBackwardPath.build();
        Action intakeFirstSet = ToFirstSet.build();
        Action intakeSecondSet = ToSecondSet.build();
        Action toGateSet1 = ToFirstGateSet.build();
        Action ShootToIntake1 = ToGoal1.build();
        Action shootToIntake2 = ToGoal2.build();
        Action shoot3 = ToGoal3.build();
        Action leave = leaveFromGate.build();

        telemetry.addLine("Paths Built");
        telemetry.update();

        double shooterTargetVel = DECODERobotConstants.closeZoneShootingVel;
        double hoodShootingPos = DECODERobotConstants.closeShootPos;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        intake2_0.updateFlywheelPID(),
                        new ParallelAction(
                                intake2_0.runShooter(shooterTargetVel, 30),
                                new SequentialAction(
                                        scorePreLoaded,
                                        new SequentialAction(
                                                intakeFirstSet,
                                                intake2_0.transferUsingRollersForTime(0.7, 1)
                                        ),
                                        new SequentialAction(
                                                intakeSecondSet,
                                                intake2_0.transferUsingRollersForTime(0.7, 1)
                                        ),
                                        new SequentialAction(
                                                toGateSet1,
                                                intake2_0.collect(0.8)
                                        ),
                                        new SequentialAction(
                                                ShootToIntake1,
                                                intake2_0.collect(0.8),
                                                shootToIntake2
                                        ),
                                        new SequentialAction(
                                                intake2_0.collect(1),
                                                new Action() {
                                                    private boolean initialized = false;
                                                    private Action conditionalSequence = null;

                                                    @Override
                                                    public boolean run(@NonNull TelemetryPacket packet) {
                                                        if (!initialized) {

                                                            if (getRuntime() < 29) {
                                                                conditionalSequence = new SequentialAction(
                                                                        shoot3
                                                                );
                                                            }else if(getRuntime() > 29){
                                                                conditionalSequence = new SequentialAction(
                                                                        leave
                                                                );
                                                            }
                                                            initialized = true;
                                                        }

                                                        if (conditionalSequence != null) {
                                                            return conditionalSequence.run(packet);
                                                        }
                                                        return false;
                                                    }


                                        }
                                )
                        )
                )
        )
        );
        }
    }
