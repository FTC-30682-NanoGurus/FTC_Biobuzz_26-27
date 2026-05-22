package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class Diffy extends LinearOpMode {
    private Servo leftServo;
    private Servo rightServo;

    @Override
    public void runOpMode() throws InterruptedException {
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        waitForStart();
        double lastPos = 0;
        while (opModeIsActive() && isStopRequested()) {
            double increment = 0.05;
            if(gamepad1.dpad_up){
                lastPos = leftServo.getPosition();
                leftServo.setPosition(lastPos+increment);
                rightServo.setPosition(lastPos+increment);
            }else if(!gamepad1.dpad_up){
                leftServo.setPosition(lastPos);
                rightServo.setPosition(lastPos);
            }
        }
    }
}
