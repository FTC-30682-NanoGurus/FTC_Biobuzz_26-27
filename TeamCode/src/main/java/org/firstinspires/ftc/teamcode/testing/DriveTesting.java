package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;


//@Disabled
public class DriveTesting extends TestingOpMode {
    MecaTank mecaTank;
    public static boolean pid_on = false;
    public static double distance = 8;
    public static boolean fast_drive = false;
    public static double speed = 0.5;
    public static boolean front_distance = true;
    public static boolean field_centric = false;
    public static boolean motion_profile = false;
    public static boolean static_pid = false;

    public void runOpMode(){
        makeTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //mecaTank = new MecaTank(hardwareMap, telemetry);

        mecaTank.init();
        ElapsedTime loopTimer = new ElapsedTime();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            loopTimer.reset();
            mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            mecaTank.setDistanceType(front_distance);
            if(!pid_on) {
                mecaTank.distance.setOn(false);
                mecaTank.rear_distance.setOn(false);
                mecaTank.forceExit();
            }else{
                mecaTank.distance.setOn(true);
                mecaTank.rear_distance.setOn(true);
                if(fast_drive){
                    mecaTank.DrivePastDistance(distance, speed);
                }else if(static_pid){
                    mecaTank.PIDToDistance(distance);
                }
                else {
                    mecaTank.LivePIDToDistance(distance);
                }
            }
            telemetry.addData("Target Distance", distance);
            telemetry.addData("Loop Time", loopTimer.milliseconds());
            mecaTank.telemetry();
            mecaTank.update();
            telemetry.update();
        }
    }
}
