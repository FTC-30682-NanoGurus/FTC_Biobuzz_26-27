package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;

@Config
public class RobotConstants {
    public static double
            claw_closed = 0.43,
            claw_fully_open = claw_closed - 0.25,
            claw_open= claw_closed - 0.18,
            claw_flat = claw_closed - 0.43,
            claw_floor_pickup = claw_closed - 0.30,
            inside_pickup_closed = claw_closed - 0.16,
            inside_pickup_open = claw_closed - 0.05;

    public static Pose2d pose = new Pose2d(0,0,0);
    public static double wrist_folded = 0, wrist_extended=30, floor_pickup_position = 100, specimen_deliver = 60;//update the 0 and 90 degrees as well.
    public static double TOO_FAR = 3.8, TOO_CLOSE = 2.3, GIVE_UP = 4.3, TARGET = 2.9;
    public static Intake intake;
    public static int ARM_LIMIT = 1600;
    public static String intakeMotor = "intake_motor";//ehub 0
    public static String slidesMotor = "slide_motor";//ehub 1 taped togethwr with encoder
    public static String riggingMotor = "rigging_motor";//ehub 3 (barely reaches)

    public static String claw_servo = "claw_servo";//ehub 4 (yellow servo connector)
    public static String left_servo = "left_servo";//ehub 3 (black tape)
    public static String right_servo = "right_servo";//ehub 1 (copious lack of black tape)

    public static String rigging_servo = "the_wand";
    public static String left_rigging_servo = "ascentLeft";
    public static String right_rigging_servo = "ascentRight";
    public static String camera = "Webcam 1";
    public static String green_led = "front_led_green";
    public static String red_led = "front_led_red";
    public static String fr = "frontRight";//chub 0 && perp deadwheel
    public static String br = "backRight";//chub 1
    public static String fl = "frontLeft";//chub 2
    public static String bl = "backLeft";//chub 3 && par deadwheel

    public static String magnet_sensor = "magnet_sensor";//digital 0
    public static String touch_sensor = "touch_sensor";//TBD
    public static String magnet_sensor2 = "magnet_sensor2";//ehub digital 0
    public static String distance = "sensor_distance";//i2c 1

    public static String rear_distance = "rear_distance";//ehub i2c 1

    public static LazyImu imu;
    public static boolean imu_init = false;
    public static boolean auto_transfer = false;
    public static int offset = 0;
}
