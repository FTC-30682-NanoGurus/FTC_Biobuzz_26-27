package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;


public class TestDataTrasnfer extends TestingOpMode {
    ElapsedTime timer;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry);
        intake.foldWrist();
        intake.arm.setUseMotionProfile(true);
        intake.slides.setUseMotionProfile(true);
        intake.init();
        sleep(200);
        intake.calculateOffset();
        RobotConstants.auto_transfer = true;
        RobotConstants.intake = intake;
        telemetry.addData("Offset", intake.getOffset());
        telemetry.update();
        RobotConstants.offset = intake.getOffset();
//        intake.closeClaw();
       waitForStart();

    return;


    }
}
