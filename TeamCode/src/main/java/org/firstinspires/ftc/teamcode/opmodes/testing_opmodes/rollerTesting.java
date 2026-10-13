package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
@TeleOp
public class rollerTesting extends LinearOpMode{
    //private DcMotor roller;
    private DcMotor transferRollers;
    private DcMotor rollers;

    @Override
    public void runOpMode() throws InterruptedException{
        //roller = hardwareMap.get(DcMotor.class, "roller");
        transferRollers = hardwareMap.get(DcMotor.class, "transferRollers");

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
        /*if(gamepad1.a){
            roller.setPower(1.0);
        }else if(gamepad1.b){
            roller.setPower(-1.0);
        }else if(gamepad1.x){
            roller.setPower(0);
        }*/

        if(gamepad1.dpad_up){
            transferRollers.setPower(0.8);
        }else if(gamepad1.dpad_down){
            transferRollers.setPower(-0.8);
        }else if(gamepad1.dpad_right){
            transferRollers.setPower(0);
        }

    }
}
}
