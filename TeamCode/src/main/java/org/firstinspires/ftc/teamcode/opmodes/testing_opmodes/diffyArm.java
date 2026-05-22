package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

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
public class diffyArm extends LinearOpMode {

    DcMotor extension;
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

        extension = hardwareMap.get(DcMotor.class, "extension");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //extension.init();
        //extension.setMin(20);
        //extension.setMax(-350);

        extension.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //Start: Claw closed
        clawServo.setPosition(clawClosedPos);

        while (opModeIsActive()) {

            //extension.update();
            //extension.setPIDF(0.005, 0.0002, 0, 0);

            previous.copy(current);
            current.copy(gamepad1);

                extension.setPower(gamepad1.right_stick_y);

        }
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void extend(int targetPos){
        //extension.move_async(targetPos);
    }

    private void retract(int targetPos){
        //extension.move_async(targetPos);
    }
}