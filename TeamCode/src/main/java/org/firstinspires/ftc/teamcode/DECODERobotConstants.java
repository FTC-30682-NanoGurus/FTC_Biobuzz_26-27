package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;

@Config
public class DECODERobotConstants {
    public static double
            hoodStartPos = 1.0,
            closeShootPos = 1.0,
            farShootPos = 0.65,
            //farShootingVel = 1300,
            farZoneShootingVel = 1560,
            closeZoneShootingVel = 1060;

    public static String fr = "frontRight";
    public static String br = "backRight";
    public static String fl = "frontLeft";
    public static String bl = "backLeft";
    public static LazyImu imu;
    public static boolean imu_init = false;

    public static Pose2d pose = new Pose2d(0,0,0);
    public static String rollers = "rollers";
    public static String transferRollers = "transferRollers";
    public static String interTransfer = "interTransfer";
    public static String flywheels = "flywheels";
    public static String hoodAdjuster = "hoodAdjuster";
    public static boolean flywheelsActive;
}
