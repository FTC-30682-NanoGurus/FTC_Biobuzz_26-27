package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGServo;


public class RudimentaryDiffyTesting extends TestingOpMode {
    public static double left_pos = 0;
    public static double right_pos = 1;
    public static double claw_pos = 0.9;
    NGServo left;
    NGServo right;
    NGServo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = new NGServo(hardwareMap, telemetry, RobotConstants.claw_servo);
        left = new NGServo(hardwareMap, telemetry, RobotConstants.left_servo);
        right = new NGServo(hardwareMap, telemetry, RobotConstants.right_servo);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

           left.setPosition(left_pos);
           right.setPosition(right_pos);
           claw.setPosition(claw_pos);
        }

    }
}
