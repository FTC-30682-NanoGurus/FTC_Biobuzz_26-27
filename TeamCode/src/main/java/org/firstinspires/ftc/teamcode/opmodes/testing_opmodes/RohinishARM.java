package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;

@Config
@TeleOp
public class RohinishARM extends LinearOpMode{

    private NGMotor arm;
    private Servo claw;

    public static double openPos = 0.22;
    public static double closedPos = 0.8;

    public static double PIVOT_P = 0.006;
    public static double PIVOT_I = 0.0;
    public static double PIVOT_D = 0.0002;
    public static double PIVOT_F = 0.0;

    public static int armIntakePos = 300;
    public static int armScorePos = 1200;

    double holdingKp = 0.005; // TODO: TUNE THIS TO AVOID BREAKING EVERYTHING
    int targetPosition = 0;
    boolean holdingPosition = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new NGMotor(hardwareMap, telemetry, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        arm.init();

        //arm.setMin(-50);
        //arm.setMax(3000);

        claw.setPosition(openPos);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            //arm.update();

            //arm.setPIDF(PIVOT_P, PIVOT_I, PIVOT_D, PIVOT_F);

            //Arm controls
            if(gamepad1.dpad_up){
                arm.setPower(0.3);
                holdingPosition = false;
            }else if(gamepad1.dpad_down){
                arm.setPower(0.3);
                holdingPosition = false;
            }else{
                if (!holdingPosition) {
                    targetPosition = arm.getCurrentPosition();
                    holdingPosition = true;
                }

                int currentPosition = arm.getCurrentPosition();
                int error = targetPosition - currentPosition;

                double power = error * holdingKp;

                power = Math.max(-1.0, Math.min(1.0, power));

                arm.setPower(power);
                arm.setPower(0);
            }

            //Claw controls
            if(gamepad1.a){
                //claw.setPwmEnable(true);
                claw.setPosition(openPos);
            }else if(gamepad1.b){
                //claw.setPwmEnable(true);
                claw.setPosition(closedPos);
            }/*else if(gamepad1.x){
                claw.setPwmEnable(false);
            }*/

            telemetry.addData("Claw Pos: ", claw.getPosition());
            telemetry.addData("Arm Pos: ", arm.getCurrentPosition());
            telemetry.update();
        }
    }
    private void toIntakePos(){
        arm.move_async(armIntakePos);
        claw.setPosition(openPos);
    }
    private void toScorePos(){
        claw.setPosition(closedPos);
        sleep(200);
        arm.move_async(armScorePos);
    }
    private void score(){
        claw.setPosition(openPos);
        sleep(500);
        toIntakePos();
    }
}
