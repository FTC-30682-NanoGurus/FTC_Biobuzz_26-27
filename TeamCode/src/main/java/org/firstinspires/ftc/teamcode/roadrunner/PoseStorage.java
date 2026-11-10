package org.firstinspires.ftc.teamcode.roadrunner;
import com.acmerobotics.roadrunner.Pose2d;

/**
 * The pose the last autonomous ended at, per alliance, in FIELD coordinates (red drivers' view:
 * red side y < 0). Static, so it survives from AUTO into TELEOP until the Robot Controller app
 * restarts.
 */
public class PoseStorage {
    static Pose2d blueAutoEndPose;
    static Pose2d redAutoEndPose;
    public static void storeBluePose(Pose2d pose){
        blueAutoEndPose = pose;
    }
    public static Pose2d getLastBluePose() { return blueAutoEndPose; }
    public static void storeRedPose(Pose2d pose){
        redAutoEndPose = pose;
    }
    public static Pose2d getLastRedPose(){
        return redAutoEndPose;
    }
    public static void resetPose() {
        blueAutoEndPose = null;
        redAutoEndPose = null;
    }

    /**
     * ALLIANCE FRAME (how the BIOBUZZ autos drive: red view for both alliances) to FIELD
     * coordinates. Red is unchanged; blue is the same spot turned 180 degrees about the centre.
     */
    public static Pose2d allianceToField(boolean red, Pose2d p) {
        if (red) return p;
        return new Pose2d(-p.position.x, -p.position.y, p.heading.toDouble() + Math.PI);
    }

    /** For a BIOBUZZ auto: stores its final pose (ALLIANCE FRAME) for TeleOp, in field coordinates. */
    public static void storeAutoEndPose(boolean red, Pose2d allianceFramePose) {
        Pose2d field = allianceToField(red, allianceFramePose);
        if (red) redAutoEndPose = field;
        else blueAutoEndPose = field;
    }
}
