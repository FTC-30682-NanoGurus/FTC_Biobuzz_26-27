package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="OutreachBot TeleOp", group="TeleOp")
public class OutreachBot extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    Servo clawServo;

    double clawPosition = 0.5; //start position

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Make sure config name is "clawServo"
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        clawServo.setPosition(clawPosition);
    }

    @Override
    public void loop() {

        // Drive
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // Claw control (A = open, B = close)
        if (gamepad1.right_bumper) {
            clawPosition = 1.0; // open
        }
        else  {
            clawPosition = 0.25; // close
        }

        clawServo.setPosition(clawPosition);

        // Telemetry
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Claw Position", clawPosition);
        telemetry.update();
    }
}