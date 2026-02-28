package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.Control;

public class MotionProfileTest extends TestingOpMode {

    public static double distance = 10;
   public static double max_vel = 10;
   public static double max_accel = 2;
   public static double max_decel = 1;
   private double past_distance = distance;
   private ElapsedTime timer;

    @Override
    public void runOpMode() {
        makeTelemetry();
        timer = new ElapsedTime();
        timer.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Wait for the start button
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Start the opmode loop
        while (opModeIsActive()) {

            if(distance != past_distance){
               timer.reset();
            }
            past_distance = distance;

            telemetry.addData("Distance", Control.motionProfile(max_vel, max_accel, max_decel, distance, timer.seconds()));
            telemetry.addData("Velocity", Control.motionProfileVelo(max_vel, max_accel, max_decel, distance, timer.seconds()));

            telemetry.update();
        }
    }

}