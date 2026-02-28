package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.library.NGAutoOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.testing.YellowSampleCamera;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

@Disabled
@Autonomous
public class YellowSampleEvasion extends NGAutoOpMode {
    Intake intake;
    TrafficLight trafficLight;
    BulkRead bulkRead;
    YellowSampleCamera camera;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new YellowSampleCamera(hardwareMap, telemetry);
        camera.init();
        bulkRead = new BulkRead(hardwareMap);
        timer = new ElapsedTime();

        // Starting pose
        Pose2d beginPose = new Pose2d(-60, -48.5, Math.toRadians(70));
        initAuto(beginPose);

        // Constraints (from your example)
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) return new MinMax(-10, 50);
            else return new MinMax(-30, 50);
        };

        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) return new MinMax(-20, 80);
            else return new MinMax(-30, 80);
        };

        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) return new MinMax(-10, 12);
            else return new MinMax(-30, 50);
        };

        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) return 20;
            else return 50;
        };

        waitForStart();
        if (isStopRequested()) return;

        double distanceToSample = -1;
        ElapsedTime waitTimer = new ElapsedTime();
        while (!isStopRequested() && (distanceToSample < 0)) {
            distanceToSample = camera.getDistanceToYellowSample();
            boolean IsSampleDetected = camera.isSampleDetected();
            telemetry.addData("Waiting for yellow sample...", distanceToSample);
            telemetry.addData("Sample Detected?", IsSampleDetected);
            telemetry.addData("Pipeline id", camera.getPipelineIdentity());
            telemetry.addData("Frames processed", camera.getPipelineFrameCount());
            telemetry.update();

            if (waitTimer.seconds() > 10) { // timeout in 10 seconds
                telemetry.addData("Timeout/waiting for camera...", distanceToSample);
                telemetry.update();
                break;
            }
        }
        double threshold = 12.0;

        TrajectoryActionBuilder trajBuilder = drive.actionBuilder(beginPose);

        if (distanceToSample > 0.2 && distanceToSample < threshold) {
            // Sample Detected/go around
            trajBuilder
                    .strafeTo(new Vector2d(-32, -48.5))
                    .setTangent(90)
                    .splineToSplineHeading(new Pose2d(-54, -5, Math.toRadians(90)), Math.toRadians(150));
        } else {
            // Sample Clear/move forward
            trajBuilder
                    .lineToY(-5);
        }

        Action driveAction = new FailoverAction(
                trajBuilder.build(),
                new InstantAction(() -> drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)))
        );

        Actions.runBlocking(driveAction);
    }
}
