package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.DecodeCAM;

@Autonomous(name="Obelisk Motif Detection", group="Autonomous")
public class obeliskTesting extends LinearOpMode {
    DecodeCAM CAM = new DecodeCAM();

    @Override
    public void runOpMode() throws InterruptedException{
        CAM.init(hardwareMap.appContext, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            String motif = CAM.getMotif();
            telemetry.addData("Motif", motif);
            telemetry.update();
            CAM.getGoalTagData();
        }

        CAM.stop();
    }

}
