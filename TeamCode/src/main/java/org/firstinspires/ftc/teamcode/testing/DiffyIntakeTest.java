package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.NGMotor;

@Config
@TeleOp
public class DiffyIntakeTest extends LinearOpMode {

    NGMotor extension;
    private Servo leftServo, rightServo, clawServo;
    private final Gamepad previous = new Gamepad();
    private final Gamepad current = new Gamepad();

    public static double tiltIncrement = 0.01;
    public static double yawIncrement = 0.005;

    public static double tiltPos = 0.5;
    public static double yawPos = 0.5;

    public static double clawOpenPos = 1.0;
    public static double clawClosedPos = 0.0;
    private boolean clawOpen = false;
    private boolean lastA = false;

    int extendedPos = -300;
    int retractedPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        extension = new NGMotor(hardwareMap, telemetry, "extension");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extension.init();
        extension.setMin(-1000);
        extension.setMax(1000);

        extension.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //Start: Claw closed
        clawServo.setPosition(clawClosedPos);

        while (opModeIsActive()) {

            extension.update();
            extension.setPIDF(0.005, 0.0002, 0, 0);

            previous.copy(current);
            current.copy(gamepad1);

            if(current.left_bumper){
                retract(retractedPos);
            } else if(current.right_bumper){
                extend(extendedPos);
            } else {
                extension.setPower(current.right_stick_y);
            }

            if (current.dpad_up) {
                tiltPos = clamp(tiltPos + tiltIncrement, 0.1, 0.9);
            }
            if (current.dpad_down) {
                tiltPos = clamp(tiltPos - tiltIncrement, 0.1, 0.9);
            }
            if (current.dpad_right) {
                yawPos = clamp(yawPos + yawIncrement, 0.1, 0.9);
            }
            if (current.dpad_left) {
                yawPos = clamp(yawPos - yawIncrement, 0.1, 0.9);
            }

            double leftPos = clamp(tiltPos + yawPos - 0.5, 0.1, 0.9);
            double rightPos = clamp(tiltPos - yawPos + 0.5, 0.1, 0.9);

            leftServo.setPosition(leftPos);
            rightServo.setPosition(rightPos);

            // a to open, a to close
            if (current.a && !lastA) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? clawOpenPos : clawClosedPos);
            }
            lastA = current.a;


            telemetry.addData("Extension pos: ", extension.getCurrentPosition());
            telemetry.addData("Tilt Target", "%.2f", tiltPos);
            telemetry.addData("Yaw Target", "%.2f", yawPos);
            telemetry.addData("Left Servo Command", "%.2f", leftPos);
            telemetry.addData("Right Servo Command", "%.2f", rightPos);
            telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
            telemetry.update();

            idle();
        }
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void extend(int targetPos){
        extension.move_async(targetPos);
    }

    private void retract(int targetPos){
        extension.move_async(targetPos);
    }
}