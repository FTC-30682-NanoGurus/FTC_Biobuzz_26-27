package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;

@TeleOp
@Config
public class ClawReset extends TestingOpMode {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

           intake.moveClaw(RobotConstants.claw_flat);
        }

    }
}


