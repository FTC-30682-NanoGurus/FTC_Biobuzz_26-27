package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

@Autonomous
public class Biobuzz_2_Tip_Auto extends NGAutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException{

        VelConstraint intakeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > 52.0) {
                return 15;
            } else {
                return 40;
            }
        };

        Pose2d beginPose = new Pose2d(61, -12, Math.toRadians(180));
        initAuto(beginPose);

        TrajectoryActionBuilder moveForwardPath = drive.actionBuilder(beginPose)
                .lineToX(55);
        TrajectoryActionBuilder ToWallSetandShooting = moveForwardPath.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), intakeVel)
                .afterTime(0.7, intake2_0.collect(6))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-26, -40, Math.toRadians(360)), Math.toRadians(170), new TranslationalVelConstraint(55))
                .splineToLinearHeading(new Pose2d(-54, -12, Math.toRadians(360)), Math.toRadians(-220), new TranslationalVelConstraint(45));
        TrajectoryActionBuilder turnToFlowerOne = ToWallSetandShooting.endTrajectory().fresh()
                .lineToXLinearHeading(-58, Math.toRadians(180))
                .afterTime(0.4, intake2_0.collect(5));
        TrajectoryActionBuilder turnToShoot = turnToFlowerOne.endTrajectory().fresh()
                .lineToXLinearHeading(-54, Math.toRadians(360));
        TrajectoryActionBuilder strafeToParking = turnToShoot.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-35, -60), new TranslationalVelConstraint(60));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action moveForward = moveForwardPath.build();
        Action IntakeWallSetandShoot = ToWallSetandShooting.build();
        Action IntakeFromFlowerOne = turnToFlowerOne.build();
        Action shootFlowerOnePollen = turnToShoot.build();
        Action park = strafeToParking.build();



        telemetry.addLine("Paths Built");
        telemetry.update();

    }
}
