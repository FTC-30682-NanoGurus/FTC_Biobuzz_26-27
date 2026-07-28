package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;


public class ServoTesting extends TestingOpMode {

    Intake intake;
    ElapsedTime timer;
    double current_time = 0;
    double position = 0;
    double increment = 0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if(timer.time() - current_time > 0.3){
                position += increment;
                current_time = timer.time();
            }
            if(position >= 1){
                increment = -0.05;
            }else if(position <= 0){
                increment= 0.05;
            }
            telemetry.addData("Position", position);
            intake.moveWrist(position);
            telemetry.update();
        }

    }
}
