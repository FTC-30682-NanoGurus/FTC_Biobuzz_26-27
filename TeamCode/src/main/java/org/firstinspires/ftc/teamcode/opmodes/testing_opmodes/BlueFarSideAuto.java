package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
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

@Autonomous
public class BlueFarSideAuto extends NGAutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < -30.0) {
                return new MinMax(-5,5);
            } else {
                return new MinMax(-120,120);
            }
        };

        Pose2d beginPose = new Pose2d(60, -12, Math.toRadians(180));
        initAuto(beginPose);

        TrajectoryActionBuilder ToShootPos = drive.actionBuilder(beginPose)
                .lineToXLinearHeading(52, Math.toRadians(212), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-70, 80));

        TrajectoryActionBuilder ToFirstSet = ToShootPos.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(43, -25, Math.toRadians(-90)), Math.toRadians(5), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-40, 50))
                .afterTime(0, intake2_0.collect(1))
                .splineToLinearHeading(new Pose2d(43, -51, Math.toRadians(-90)), Math.toRadians(70), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-10, 20))
                .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70));

        TrajectoryActionBuilder ToHPset1 = ToFirstSet.endTrajectory().fresh()
                .strafeToSplineHeading(
                        new Vector2d(52, -35),
                        Math.toRadians(-90),
                        new TranslationalVelConstraint(100),
                        new ProfileAccelConstraint(-90, 90)
                )
                .afterTime(0, intake2_0.collect(1.6))
                .splineToLinearHeading(
                        new Pose2d(56, -58,
                                Math.toRadians(-90)),
                        Math.toRadians(120),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-20, 20)
                )
                .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70));

        TrajectoryActionBuilder ToHPset2 = ToHPset1.endTrajectory().fresh()
                .strafeToSplineHeading(
                        new Vector2d(52, -35),
                        Math.toRadians(-90),
                        new TranslationalVelConstraint(100),
                        new ProfileAccelConstraint(-90, 90)
                )
                .afterTime(0, intake2_0.collect(1.6))
                .splineToLinearHeading(
                        new Pose2d(56, -58,
                                Math.toRadians(-90)),
                        Math.toRadians(120),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-20, 20)
                )
                .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70));

        TrajectoryActionBuilder leaveLaunchLine = ToHPset2.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(38, -12), Math.toRadians(-90));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = ToShootPos.build();
        Action getFirstSet = ToFirstSet.build();
        Action getHPset1 = ToHPset1.build();
        Action getHPset2 = ToHPset2.build();
        Action leave = leaveLaunchLine.build();


        telemetry.addLine("Paths Built");
        telemetry.update();

        double shooterTargetVel = DECODERobotConstants.farZoneShootingVel;
        double hoodShootingPos = DECODERobotConstants.farShootPos;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        intake2_0.updateFlywheelPID(),
                        new ParallelAction(
                                intake2_0.runShooter(shooterTargetVel, 30),
                                intake2_0.setHoodAdjuster(hoodShootingPos),
                                new SequentialAction(
                                        new SequentialAction(
                                                scorePreLoaded,
                                                intake2_0.transferUsingRollersForTime(2, 0.3)
                                        ),
                                        new SequentialAction(
                                                getFirstSet,
                                                intake2_0.transferUsingRollersForTime(2, 0.3)
                                        ),
                                        new SequentialAction(
                                                getHPset1,
                                                intake2_0.transferUsingRollersForTime(2, 0.3)
                                        ),
                                        new SequentialAction(
                                                getHPset2,
                                                intake2_0.transferUsingRollersForTime(2, 0.3)
                                        ),
                                        leave
                                )
                        )
                )
        );
    }
}
