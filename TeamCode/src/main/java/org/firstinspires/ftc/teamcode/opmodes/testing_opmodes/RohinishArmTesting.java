package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;

@Config
@TeleOp(name = "Heavy Arm Tester", group = "Testing")
public class RohinishArmTesting extends LinearOpMode {

    // PIDF values to tune in FTC Dashboard
    public static double P = 0.0006;
    public static double I = 0;
    public static double D = 0.0002;
    // FIX: Changed F from 0 to 0.1. A heavy arm requires Feedforward (F) to counteract gravity!
    // You will need to tune this up or down in the dashboard so it holds perfectly steady.
    public static double F = 0.1;

    NGMotor armPivot;
    public NGServo claw;

    public static double openPos = 0;
    public static double closedPos = 0.22;

    boolean wasMovingManually = false;

    @Override
    public void runOpMode() throws InterruptedException {

        armPivot = new NGMotor(hardwareMap, telemetry, "arm");
        claw = new NGServo(hardwareMap, telemetry, "claw");

        armPivot.init();
        armPivot.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        //Safety limits
        //armPivot.setMin(5);
        //armPivot.setMax(500);

        telemetry.addLine("Heavy Arm Tester Initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // Live-update tuning variables
            armPivot.setPIDF(P, I, D, F);

            // --- ARM MANUAL CONTROL ---
            if (gamepad1.dpad_up) {
                // Increased power to 0.8 so it actually has the torque to lift a heavy arm!
                armPivot.setPower(0.8);
                wasMovingManually = true;
            }
            else if (gamepad1.dpad_down) {
                // Increased power to -0.8
                armPivot.setPower(-0.8);
                wasMovingManually = true;
            }
            else {
                // The exact moment you let go of the D-pad...
                if (wasMovingManually) {

                    // Grab the current position
                    int currentPos = armPivot.getCurrentPosition();

                    // Set it as the new target to hold
                    armPivot.move_async_pid(currentPos);

                    // Mark manual movement as finished
                    wasMovingManually = false;
                }
            }

            if(gamepad1.a){
                //claw.setPwmEnable(true);
                claw.setPosition(openPos, 0.5);
            }else if(gamepad1.b){
                //claw.setPwmEnable(true);
                claw.setPosition(closedPos, 0.5);
            }/*else if(gamepad1.x){
                claw.setPwmEnable(false);
            }*/

            // Only update the PID loop when we aren't manually driving the motor
            if (!wasMovingManually) {
                armPivot.update();
            }

            claw.update();

            // --- TELEMETRY ---
            telemetry.addData("Claw Pos: ", claw.getPosition());
            telemetry.addData("Arm Position", armPivot.getCurrentPosition());
            telemetry.addData("Target Position", armPivot.targetPos);
            telemetry.addData("Motor Power", armPivot.getPower());
            telemetry.update();
        }
    }
}