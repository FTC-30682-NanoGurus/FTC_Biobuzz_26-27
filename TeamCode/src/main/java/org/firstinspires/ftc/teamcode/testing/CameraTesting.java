package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Camera;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.TrafficLight;


public class CameraTesting extends TestingOpMode {
    Intake intake;
    TrafficLight trafficLight;

    BulkRead bulkRead;
    Camera camera;
    ElapsedTime timer;
    public static boolean useCameraAngle = false;
    public static boolean useCameraDistance = false;
    public static boolean red_sample = false;
    public static boolean yellow_sample = true;

    public static double wrist_angle = 0;
    public static int arm_position = 235;
    public static int slide_position = 0;

    public static double claw_position = RobotConstants.inside_pickup_open;

    public static boolean score = false;

    public static boolean inside_pick = true;

    public enum CAMERA_COLLECT {
        RETRACT_SLIDES,
        LOWER_ARM
    }
    CAMERA_COLLECT state = CAMERA_COLLECT.RETRACT_SLIDES;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        bulkRead = new BulkRead(hardwareMap);
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
        intake.init();
        camera = new Camera(hardwareMap, telemetry);
        camera.init();


        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();
            camera.useInsidePick(inside_pick);
            if(useCameraAngle){
                if(yellow_sample){
                    intake.turnClaw(camera.getYellow()[0]);
                }else if(red_sample){
                    intake.turnClaw(camera.getRed()[0]);
                }else{
                    intake.turnClaw(camera.getBlue()[0]);
                }
            }

            if(score){
                switch(state){
                    case RETRACT_SLIDES:
                        if (slide_position > 325){
                            slide_position -= 225;
                            useCameraAngle = false;
                            state = CAMERA_COLLECT.LOWER_ARM;
                        }
                        break;
                    case LOWER_ARM:

                        if(intake.slides.isBusy()){
                                break;
                        }
                        score = false;
                        intake.arm.setManualPower(-0.3);
                        sleep(700);
                        intake.arm.setManualPower(0);
                        intake.moveClaw(inside_pick ? RobotConstants.inside_pickup_closed : RobotConstants.claw_closed);
                        claw_position = inside_pick ? RobotConstants.inside_pickup_closed : RobotConstants.claw_closed;
                        sleep(300);
                        intake.moveArm(300);
                        arm_position = 300;
                        state = CAMERA_COLLECT.RETRACT_SLIDES;
                        break;
                }



            }
            intake.moveSlides(slide_position);
            intake.moveArm(arm_position);
            intake.moveWrist(wrist_angle);
            intake.moveClaw(claw_position);
            intake.setSlidePower(-gamepad1.left_stick_y);
            intake.setRotationPower(-gamepad1.right_stick_y);
            intake.update();
            intake.telemetry();
            camera.telemetry();
            telemetry.update();
        }

    }
}
