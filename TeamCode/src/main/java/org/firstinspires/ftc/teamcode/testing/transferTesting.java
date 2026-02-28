package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.library.NGAutoOpMode.timer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.MultiClick;
import org.firstinspires.ftc.teamcode.subsystems.Intake2_0;

@Config
@TeleOp
public class transferTesting extends LinearOpMode{
    private DcMotor rollers;
    private Servo rampAdjuster;
    Intake2_0 transfer = new Intake2_0(hardwareMap, telemetry,timer);
    private MultiClick multiClick = new MultiClick(telemetry);
    @Override
    public void runOpMode() throws InterruptedException{
        rollers = hardwareMap.get(DcMotor.class, "rollers");
        rampAdjuster = hardwareMap.get(Servo.class, "rampAdjuster");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            double currentTime = getRuntime();
            multiClick.update("A", currentTime, gamepad1.a);

            if(gamepad1.a){
                rampAdjuster.setPosition(0.5);
            }else if(gamepad1.b){
                rampAdjuster.setPosition(1.0);
            }else if (multiClick.getTaps("A") == 2) { // Check for double click
                telemetry.addData("A Button", "DOUBLE CLICK DETECTED - Resetting rampAdjuster");
                telemetry.update();
                rampAdjuster.setPosition(0);
                multiClick.clearTaps("A"); // clear after handling so it doesn't keep triggering
            }
            if(gamepad1.y){
                rollers.setPower(1);
            }else if(gamepad1.x){
                rollers.setPower(0);
            }
        }
    }
}
