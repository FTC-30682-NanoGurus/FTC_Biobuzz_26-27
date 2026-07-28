package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.DecodeCAM;

@Config
@TeleOp
public class autoTurretTesting extends LinearOpMode {
    DecodeCAM CAM = new DecodeCAM();

    @Override
    public void runOpMode() throws InterruptedException{
        CAM.init(hardwareMap.appContext, hardwareMap, telemetry);
        //CAM.telemetry();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CAM.getGoalTagData();
            telemetry.update();
        }

        CAM.stop();
    }

}
