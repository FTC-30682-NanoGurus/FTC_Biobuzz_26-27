package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class DecodeCAM extends Subsystem{
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    OpenCvCamera camera;
    Telemetry telemetry;
    public String motif;
    private static final Pose2d TAG_24_POSE = new Pose2d(-58.3727f, 55.6425f, Math.toRadians(0)); // Red Goal
    private static final Pose2d TAG_20_POSE = new Pose2d(-58.3727f, -55.6425f, Math.toRadians(0)); // Blue Goal


    public void init(android.content.Context appContext, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, Telemetry telemetry) {
        init(appContext, hardwareMap, telemetry, true);
    }

    /**
     * @param drawAnnotations when false, skips the tag ID / outline / axes / cube overlays.
     *
     * Those overlays are rendered per camera frame on the vision thread, which competes with the
     * opmode thread for the Control Hub's very limited CPU. They are purely a debugging aid - the
     * pose maths in getAbsoluteRobotPose() does not read them - so turning them off for a
     * competition teleop buys back CPU without changing a single robot decision.
     *
     * The camera resolution is pinned in this path. 640x480 is the VisionPortal default, so this
     * is a no-op in the normal case; it just stops a webcam from silently negotiating something
     * larger and much more expensive to decode.
     */
    public void init(android.content.Context appContext, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, Telemetry telemetry, boolean drawAnnotations) {
        this.telemetry = telemetry;
        if (aprilTagProcessor == null) {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .setOutputUnits(INCH, AngleUnit.RADIANS)
                    .setCameraPose(new Position(INCH, 0, 5, 4, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 43.3, 0, 0))
                    .setDrawTagID(drawAnnotations)
                    .setDrawTagOutline(drawAnnotations)
                    .setDrawAxes(drawAnnotations)
                    .setDrawCubeProjection(drawAnnotations)
                    .build();

            VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor);

            if (!drawAnnotations) {
                portalBuilder.setCameraResolution(new android.util.Size(640, 480));
            }

            visionPortal = portalBuilder.build();
           boolean cameraReady = visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY;
            if(cameraReady) {
                visionPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
                visionPortal.getCameraControl(ExposureControl.class).setExposure(6000, TimeUnit.MICROSECONDS);
                visionPortal.getCameraControl(GainControl.class).setGain(50);
            }
        }
    }

    public Pose2d getAbsoluteRobotPose() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (currentDetections.isEmpty()) {
            return null;
        }

        // Use the first valid detection
        AprilTagDetection detection = currentDetections.get(0);

        if (detection.metadata != null && detection.ftcPose != null) {

            double tagFieldX = detection.metadata.fieldPosition.get(0);
            double tagFieldY = detection.metadata.fieldPosition.get(1);

            Quaternion q = detection.metadata.fieldOrientation;

            double yawRad = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            double tagHeadingDeg = Math.toDegrees(yawRad);

            double headingRad = Math.toRadians(tagHeadingDeg);
            double camX = detection.ftcPose.y;
            double camY = -detection.ftcPose.x;

            double rotatedX = camX * Math.cos(headingRad) - camY * Math.sin(headingRad);
            double rotatedY = camX * Math.sin(headingRad) + camY * Math.cos(headingRad);

            double robotX = detection.robotPose.getPosition().x; //tagFieldX - rotatedX;
            double robotY = detection.robotPose.getPosition().y; //tagFieldY - rotatedY;

            double robotHeadingDeg = tagHeadingDeg + 90 + detection.ftcPose.yaw;
            //telemetry.addData("Camera X, Y, Heading: ", "%.2f, %.2f, %.2f", robotX, robotY, robotHeadingDeg);
            return new Pose2d(
                    robotX,
                    robotY,
                    Math.toRadians(robotHeadingDeg)
            );
        }
        return null;
        /*List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        AprilTagDetection visibleTag = null;
        Pose2d tagFieldPose = null;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20) {
                    visibleTag = detection;
                    tagFieldPose = TAG_20_POSE;
                    break;
                } else if (detection.id == 24) {
                    visibleTag = detection;
                    tagFieldPose = TAG_24_POSE;
                    break;
                }
            }
        }

        if (visibleTag == null) {
            return null;
        }

        double camRelTagX = visibleTag.ftcPose.x;
        double camRelTagY = visibleTag.ftcPose.y;
        double camRelTagYaw_deg = visibleTag.ftcPose.yaw;

        double camFieldHeading_rad = AngleUnit.normalizeRadians(
                tagFieldPose.getHeading() - Math.toRadians(camRelTagYaw_deg)
        );

        double camFieldX = tagFieldPose.getX() - (camRelTagX * Math.cos(camFieldHeading_rad) - camRelTagY * Math.sin(camFieldHeading_rad));
        double camFieldY = tagFieldPose.getY() - (camRelTagX * Math.sin(camFieldHeading_rad) + camRelTagY * Math.cos(camFieldHeading_rad));

        double robotFieldHeading_rad = camFieldHeading_rad;

        double robotRelCamX_field = CAMERA_X_OFFSET * Math.cos(robotFieldHeading_rad) - CAMERA_Y_OFFSET * Math.sin(robotFieldHeading_rad);
        double robotRelCamY_field = CAMERA_X_OFFSET * Math.sin(robotFieldHeading_rad) + CAMERA_Y_OFFSET * Math.cos(robotFieldHeading_rad);

        double robotFieldX = camFieldX - robotRelCamX_field; //+ 60;
        double robotFieldY = camFieldY - robotRelCamY_field; //- 15;

        telemetry.addData("Pose from Tag", String.format("ID %d -> (%.1f, %.1f, %.1f°)",
                visibleTag.id, robotFieldX, robotFieldY, Math.toDegrees(robotFieldHeading_rad)));

        return new Pose2d(robotFieldX, robotFieldY, robotFieldHeading_rad);
         */
    }
    public String getMotif() {
        if (aprilTagProcessor == null) return "Not Initialized";

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) return "No Tag";

        for (AprilTagDetection tag : detections) {
            switch (tag.id) {
                case 21: motif = "GPP";
                case 22: motif = "PGP";
                case 23: motif = "PPG";
            }
            return motif;
        }
        return "Unknown Motif";
    }

    public void getGoalTagData() {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));

                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));



                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }


    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void telemetry() {

    }

    @Override
    public void init() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 100);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
}
