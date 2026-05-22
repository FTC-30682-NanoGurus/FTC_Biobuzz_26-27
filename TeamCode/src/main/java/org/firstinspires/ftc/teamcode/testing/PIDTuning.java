package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;


public class PIDTuning extends TestingOpMode {
    DcMotor motor;
    double error, lastError;

    NGMotor ngMotor;
    public static int target_position = 0;
    public static boolean motion_profile = false;
    public static PIDFCoefficients pidf = new PIDFCoefficients(0,0,0,0);
    public static double arm_at_0 = 0;
    public static double arm_at_90 = 400;
    public static String name = "intake_motor";
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ngMotor = new NGMotor(hardwareMap, telemetry, name);
        ngMotor.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            double arm_angle = (ngMotor.getCurrentPosition() - arm_at_0) / (arm_at_90 - arm_at_0) * 90;
            telemetry.addData("Arm Angle", arm_angle);
            arm_angle = Math.toRadians(arm_angle);
            ngMotor.setPIDF(pidf.p, pidf.i, pidf.d, Math.cos(arm_angle) * pidf.f);
            ngMotor.setUseMotionProfile(motion_profile);
            ngMotor.move_async(target_position);
            ngMotor.telemetry();
            ngMotor.update();
            telemetry.update();
        }
    }
}
