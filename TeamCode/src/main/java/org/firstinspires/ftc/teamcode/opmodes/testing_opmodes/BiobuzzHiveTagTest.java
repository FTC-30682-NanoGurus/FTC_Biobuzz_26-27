package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ HIVE AprilTag test: ground distance to each of OUR alliance's two HIVE CELL clusters, and
 * whether each CELL is up or tipped down.
 *
 * <h2>The tags (BIOBUZZ Competition Manual 9.9, AprilTag production sheets)</h2>
 * Every CELL has one 4-tag cluster of 3.25 in 36h11 tags on its BOTTOM face, facing down toward the
 * tiles. One HIVE per alliance, two CELLS per HIVE, so four clusters on the field:
 * <pre>
 *   IDs 30-33  red  CELL, side of the field opposite the audience (scoring side)
 *   IDs 34-37  red  CELL, audience side
 *   IDs 38-41  blue CELL, audience side
 *   IDs 42-45  blue CELL, side of the field opposite the audience (scoring side)
 * </pre>
 *
 * <h2>The other alliance's tags are never used</h2>
 * The tag library handed to the processor holds ONLY the selected alliance's eight IDs, so the other
 * alliance's tags get no metadata and no pose. On top of that, every detection is checked against
 * the selected alliance's ID list and anything else is dropped before any maths or telemetry. The
 * decoder still sees every 36h11 code in the image - that cannot be switched off - but nothing it
 * reads outside our eight IDs reaches this opmode's results.
 *
 * <h2>Ground distance</h2>
 * Each visible tag's camera-frame position is rotated through the camera mount (roll, then pitch,
 * then yaw) and shifted by the mount offset, all from {@link TurretConstants}, giving its position
 * in the ROBOT frame. The cluster point is the mean of the visible tags, and the ground distance is
 * the horizontal length of that point from the robot centre - its height is dropped. With the mount
 * still at the TurretConstants defaults (0 offset, 0 angles) this is the distance from the LENS,
 * and any real camera pitch will make it read short: measure the mount before trusting it. The
 * "tag height" line is the check - it should match the CELL bottom height above the tiles.
 *
 * The cluster's member layout is not published in the manual text, so a partly visible cluster is
 * averaged over the tags that are seen; expect the point to shift by a couple of inches as tags
 * come and go.
 *
 * <h2>Tipped or not</h2>
 * FIRST's cluster guidance: a cluster whose roll is within +/-90 deg is right-side up, which means
 * its CELL is facing up (scorable). Outside that it is upside down: the CELL is tipped down. The
 * state only changes after {@link #TIP_CONFIRM_FRAMES} frames agree. Each HIVE always has exactly
 * one CELL up, so when only one of the two clusters is visible the other CELL's state is shown as
 * INFERRED from it. A tipped-down CELL's tags face away from the floor, so it is often not visible
 * at all - that is expected, not a fault.
 *
 * <h2>Controls</h2>
 * <pre>
 * INIT:  gamepad1 B = RED alliance, gamepad1 X = BLUE alliance (can be changed until START)
 * RUN:   nothing - push the robot around the HIVE and read telemetry
 * </pre>
 */
@Config
@TeleOp(name = "BIOBUZZ Hive Tag Test", group = "testing")
public class BiobuzzHiveTagTest extends LinearOpMode {
    /** BIOBUZZ HIVE tag edge length, INCHES (manual 9.9). */
    public static double TAG_SIZE_IN = 3.25;
    /** Frames in a row that must agree before a CELL's up/down state changes. */
    public static int TIP_CONFIRM_FRAMES = 3;
    /** |roll| below this, DEGREES, is right-side up (CELL up). FIRST's guidance is 90. */
    public static double UPRIGHT_ROLL_LIMIT_DEG = 90.0;
    /**
     * Manual exposure for this test. OFF by default: TurretConstants' 3 ms is tuned for a match-lit
     * field and a fast-turning turret, and in an ordinary room it gives an almost black image. The
     * robot is pushed slowly in this test, so auto exposure costs nothing here.
     */
    public static boolean MANUAL_EXPOSURE = false;
    /** FTC Dashboard camera stream rate. Keep it low: every frame goes over the hub's WiFi. */
    public static int DASHBOARD_FPS = 10;

    private enum Alliance { NONE, RED, BLUE }

    private enum CellState { UNKNOWN, UP, DOWN }

    /** One CELL's cluster: its four IDs, a label, and the debounced tip state. */
    private static class Cell {
        final String label;
        final int firstId;
        CellState state = CellState.UNKNOWN;
        CellState pending = CellState.UNKNOWN;
        int pendingFrames = 0;

        // Filled each loop
        final List<Integer> seenIds = new ArrayList<>();
        double fwdIn, leftIn, upIn, groundIn, bearingDeg, meanRollDeg;
        int uprightCount, invertedCount;

        Cell(String label, int firstId) {
            this.label = label;
            this.firstId = firstId;
        }

        boolean owns(int id) {
            return id >= firstId && id < firstId + 4;
        }

        boolean visible() {
            return !seenIds.isEmpty();
        }
    }

    private Cell[] cells;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean cameraControlsApplied = false;
    private String cameraReport = "not applied yet";

    @Override
    public void runOpMode() {
        // ---- INIT: pick the alliance, and open the camera as soon as it is picked ------------------
        // The camera opens DURING INIT, not at START: the Driver Station camera preview only works
        // in INIT, so a camera opened after START can never be seen there. The tag library depends
        // on the alliance, so the first pick locks it in - restart the opmode to change it.
        Alliance alliance = Alliance.NONE;
        while (!isStarted() && !isStopRequested()) {
            if (alliance == Alliance.NONE) {
                if (gamepad1.b) alliance = Alliance.RED;
                if (gamepad1.x) alliance = Alliance.BLUE;
                if (alliance != Alliance.NONE) startVision(alliance);
            }
            telemetry.addLine("BIOBUZZ HIVE TAG TEST");
            if (alliance == Alliance.NONE) {
                telemetry.addLine("Which alliance are you on?");
                telemetry.addLine("   gamepad1 B = RED  (tags 30-37)");
                telemetry.addLine("   gamepad1 X = BLUE (tags 38-45)");
            } else {
                telemetry.addData("Alliance", "%s (locked - restart the opmode to change)", alliance);
                telemetry.addData("Camera", visionPortal == null ? "FAILED TO OPEN - check the config name"
                        : String.valueOf(visionPortal.getCameraState()));
                telemetry.addLine("Driver Station: 3-dot menu > Camera Stream, tap the image to refresh.");
                telemetry.addLine("Or open FTC Dashboard (192.168.43.1:8080/dash) for a live stream.");
            }
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            shutdownVision();
            return;
        }

        // Started without a choice: keep asking, nothing is detected until one is made.
        while (alliance == Alliance.NONE && opModeIsActive()) {
            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (alliance != Alliance.NONE) startVision(alliance);
            telemetry.addLine("NO ALLIANCE SELECTED - nothing will be detected.");
            telemetry.addLine("gamepad1 B = RED, gamepad1 X = BLUE");
            telemetry.update();
            idle();
        }
        if (!opModeIsActive() || visionPortal == null) {
            if (visionPortal == null && opModeIsActive()) {
                telemetry.addLine("CAMERA FAILED TO OPEN: check that '" + TurretConstants.WEBCAM_NAME
                        + "' is in the active robot configuration.");
                telemetry.update();
                while (opModeIsActive()) idle();
            }
            shutdownVision();
            return;
        }

        int firstOurId = cells[0].firstId;
        int lastOurId = cells[1].firstId + 3;

        // ---- RUN -----------------------------------------------------------------------------------
        while (opModeIsActive()) {
            if (!cameraControlsApplied
                    && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                applyCameraControls();
            }

            for (Cell cell : cells) {
                cell.seenIds.clear();
                cell.fwdIn = cell.leftIn = cell.upIn = 0;
                cell.meanRollDeg = 0;
                cell.uprightCount = cell.invertedCount = 0;
            }

            for (AprilTagDetection det : aprilTag.getDetections()) {
                // Only our alliance's eight IDs. Anything else - the other alliance's HIVE, stray
                // tags - is dropped here, before it can touch a single number below.
                if (det.metadata == null || det.ftcPose == null) continue;
                Cell owner = null;
                for (Cell cell : cells) if (cell.owns(det.id)) owner = cell;
                if (owner == null) continue;

                double[] robot = cameraToRobot(det.ftcPose.x, det.ftcPose.y, det.ftcPose.z);
                owner.seenIds.add(det.id);
                owner.fwdIn += robot[0];
                owner.leftIn += robot[1];
                owner.upIn += robot[2];

                double roll = wrapDeg(det.ftcPose.roll - TurretConstants.CAMERA_MOUNT_ROLL_DEG);
                owner.meanRollDeg += roll;
                if (Math.abs(roll) < UPRIGHT_ROLL_LIMIT_DEG) owner.uprightCount++;
                else owner.invertedCount++;
            }

            for (Cell cell : cells) {
                if (!cell.visible()) continue;
                int n = cell.seenIds.size();
                cell.fwdIn /= n;
                cell.leftIn /= n;
                cell.upIn /= n;
                cell.meanRollDeg /= n;
                cell.groundIn = Math.hypot(cell.fwdIn, cell.leftIn);
                cell.bearingDeg = Math.toDegrees(Math.atan2(cell.leftIn, cell.fwdIn));
                CellState seen = cell.uprightCount >= cell.invertedCount ? CellState.UP : CellState.DOWN;
                debounce(cell, seen);
            }

            // ---- TELEMETRY -----------------------------------------------------------------------
            telemetry.addData("ALLIANCE", alliance);
            telemetry.addData("Using tag IDs", "%d-%d only (other alliance's tags ignored)", firstOurId, lastOurId);
            telemetry.addData("Camera", "%s | %s", visionPortal.getCameraState(), cameraReport);
            telemetry.addLine();

            for (int i = 0; i < 2; i++) {
                Cell cell = cells[i];
                Cell other = cells[1 - i];
                telemetry.addLine("--- " + cell.label + " (IDs " + cell.firstId + "-" + (cell.firstId + 3) + ") ---");
                if (cell.visible()) {
                    telemetry.addData("Tags seen", "%s (%d/4)", cell.seenIds, cell.seenIds.size());
                    telemetry.addData(">>> GROUND DISTANCE", "%.1f in", cell.groundIn);
                    telemetry.addData("Bearing", "%.1f deg (+ = left)", cell.bearingDeg);
                    telemetry.addData("Tag height above robot origin", "%.1f in", cell.upIn);
                    telemetry.addData("Roll", "%.1f deg  (upright %d / upside-down %d)",
                            cell.meanRollDeg, cell.uprightCount, cell.invertedCount);
                } else {
                    telemetry.addLine("Not visible");
                }
                telemetry.addData(">>> CELL", stateText(cell, other));
                telemetry.addLine();
            }
            telemetry.update();
        }

        shutdownVision();
    }

    /** Our two CELLS, a library holding only their eight tags, the processor and the camera. */
    private void startVision(Alliance alliance) {
        cells = alliance == Alliance.RED
                ? new Cell[]{new Cell("RED SCORING-SIDE CELL (away from audience)", 30),
                             new Cell("RED AUDIENCE-SIDE CELL", 34)}
                : new Cell[]{new Cell("BLUE AUDIENCE-SIDE CELL", 38),
                             new Cell("BLUE SCORING-SIDE CELL (away from audience)", 42)};

        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (Cell cell : cells) {
            for (int id = cell.firstId; id < cell.firstId + 4; id++) {
                library.addTag(id, cell.label + " tag " + id, TAG_SIZE_IN, DistanceUnit.INCH);
            }
        }

        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(library.build())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(TurretConstants.CAMERA_FX, TurretConstants.CAMERA_FY,
                            TurretConstants.CAMERA_CX, TurretConstants.CAMERA_CY)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();
            aprilTag.setDecimation(TurretConstants.APRILTAG_DECIMATION);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, TurretConstants.WEBCAM_NAME))
                    .setCameraResolution(new Size(TurretConstants.STREAM_WIDTH, TurretConstants.STREAM_HEIGHT))
                    // MJPEG, like TurretVision and PollenCamera: a fraction of the USB bandwidth of
                    // the default uncompressed YUY2, which is what keeps a C920 stable on the hub.
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build();
            FtcDashboard.getInstance().startCameraStream(visionPortal, DASHBOARD_FPS);
        } catch (Exception e) {
            visionPortal = null;
        }
    }

    /**
     * Stops the dashboard stream first, then closes the camera - but only once it has finished
     * opening. Closing a camera that is still mid-open (stop pressed right after the pick) is the
     * classic way to get "stuck in stop()", so this waits briefly for the open to settle first.
     */
    private void shutdownVision() {
        FtcDashboard.getInstance().stopCameraStream();
        if (visionPortal == null) return;
        long deadline = System.currentTimeMillis() + 1500;
        while (System.currentTimeMillis() < deadline) {
            VisionPortal.CameraState state = visionPortal.getCameraState();
            if (state == VisionPortal.CameraState.STREAMING || state == VisionPortal.CameraState.ERROR
                    || state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) break;
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        try {
            visionPortal.close();
        } catch (Exception ignored) {
        }
        visionPortal = null;
    }

    /**
     * Camera frame (SDK ftcPose: x right, y out of the lens, z up) to robot frame
     * (x forward, y left, z up), through the mount roll, pitch, yaw and offset in TurretConstants.
     */
    private static double[] cameraToRobot(double xRight, double yOut, double zUp) {
        double f = yOut, l = -xRight, u = zUp;

        double roll = Math.toRadians(TurretConstants.CAMERA_MOUNT_ROLL_DEG);    // + = clockwise from behind
        double l1 = l * Math.cos(roll) - u * Math.sin(roll);
        double u1 = l * Math.sin(roll) + u * Math.cos(roll);

        double pitch = Math.toRadians(TurretConstants.CAMERA_MOUNT_PITCH_DEG);  // + = tilted up
        double f2 = f * Math.cos(pitch) - u1 * Math.sin(pitch);
        double u2 = f * Math.sin(pitch) + u1 * Math.cos(pitch);

        double yaw = Math.toRadians(TurretConstants.CAMERA_MOUNT_YAW_DEG);      // + = turned left
        double f3 = f2 * Math.cos(yaw) - l1 * Math.sin(yaw);
        double l3 = f2 * Math.sin(yaw) + l1 * Math.cos(yaw);

        return new double[]{
                f3 + TurretConstants.CAMERA_OFFSET_X_IN,
                l3 + TurretConstants.CAMERA_OFFSET_Y_IN,
                u2 + TurretConstants.CAMERA_OFFSET_Z_IN
        };
    }

    private static void debounce(Cell cell, CellState seen) {
        if (seen == cell.state) {
            cell.pending = seen;
            cell.pendingFrames = 0;
            return;
        }
        if (seen == cell.pending) {
            cell.pendingFrames++;
        } else {
            cell.pending = seen;
            cell.pendingFrames = 1;
        }
        if (cell.pendingFrames >= TIP_CONFIRM_FRAMES) {
            cell.state = seen;
            cell.pendingFrames = 0;
        }
    }

    /** Seen state if visible, otherwise inferred from the other CELL (one CELL per HIVE is always up). */
    private static String stateText(Cell cell, Cell other) {
        if (cell.visible() && cell.state != CellState.UNKNOWN) {
            return cell.state == CellState.UP ? "UP (scorable, not tipped down)" : "TIPPED DOWN";
        }
        if (other.visible() && other.state != CellState.UNKNOWN) {
            return other.state == CellState.UP
                    ? "TIPPED DOWN (inferred: other CELL is up)"
                    : "UP (inferred: other CELL is tipped down)";
        }
        return cell.visible() ? "confirming..." : "UNKNOWN (neither CELL visible)";
    }

    private static double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    /** Short manual exposure and high gain, same as TurretVision: less motion blur, steadier tags. */
    private void applyCameraControls() {
        StringBuilder report = new StringBuilder();
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (MANUAL_EXPOSURE) {
                if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    long lo = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                    long hi = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                    long use = Math.max(lo, Math.min(hi, TurretConstants.EXPOSURE_MS));
                    exposure.setExposure(use, TimeUnit.MILLISECONDS);
                    report.append("manual exposure ").append(use).append(" ms");
                } else {
                    report.append("manual exposure unsupported");
                }
                GainControl gain = visionPortal.getCameraControl(GainControl.class);
                if (gain != null) {
                    int want = TurretConstants.GAIN < 0 ? gain.getMaxGain() : TurretConstants.GAIN;
                    int use = Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), want));
                    gain.setGain(use);
                    report.append(", gain ").append(use);
                }
            } else {
                // Set auto EXPLICITLY. A UVC webcam keeps the last exposure it was given until it is
                // unplugged, so after any opmode that set 3 ms manual, every later camera opmode
                // starts almost black unless something switches it back. The C920 reports its auto
                // mode as AperturePriority.
                ExposureControl.Mode[] autoModes = {ExposureControl.Mode.AperturePriority,
                        ExposureControl.Mode.Auto, ExposureControl.Mode.ContinuousAuto};
                String applied = "auto exposure unsupported";
                if (exposure != null) {
                    for (ExposureControl.Mode mode : autoModes) {
                        if (exposure.isModeSupported(mode) && exposure.setMode(mode)) {
                            applied = "auto exposure (" + mode + ")";
                            break;
                        }
                    }
                }
                report.append(applied);
            }
        } catch (Exception e) {
            report.append(" (camera controls failed: ").append(e.getMessage()).append(")");
        }
        cameraReport = report.toString();
        cameraControlsApplied = true;
    }
}
