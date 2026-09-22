package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.Alliance;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretTarget;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretVision;
// ===== BEGIN SDK 12 CLUSTER CODE - DISABLED =====
// import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
// ===== END SDK 12 CLUSTER CODE - DISABLED =====
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Bench opmode for calibrating the turret. It does THREE jobs, and you will want all three before
 * the drive-and-shoot teleop is worth running.
 *
 * <h2>1. Find the shortest reliable exposure</h2>
 * Tags are decoded from where their corners land on the sensor, and motion blur moves those
 * corners. Blur is governed by EXPOSURE TIME, not by brightness, so the goal is the SHORTEST
 * exposure that still decodes, with the lost light bought back by sensor gain. Start at 3 ms and
 * walk it down with dpad up/down while watching the detection count from a realistic distance,
 * ideally while someone pushes the robot. Keep the shortest setting that still holds the cluster,
 * then write it into {@link TurretConstants#EXPOSURE_MS}.
 *
 * <h2>2. Capture the servo range calibration</h2>
 * Jog the turret with the left stick in RAW SERVO UNITS - deliberately not in degrees, because the
 * degree mapping is the thing being calibrated. Creep up on each physical limit, stop as soon as
 * the turret touches it, and press A at one end and B at the other to record the positions. Copy
 * the two recorded numbers into the SERVO_A / SERVO_B scale constants.
 *
 * WATCH THE CURRENT DRAW. Jogging past a hard stop stalls both servos against each other and will
 * cook them in seconds. If the turret refuses to move smoothly in one direction, suspect
 * {@link TurretConstants#SERVO_B_REVERSED} first - two ganged servos fighting behave exactly like
 * a jammed mechanism.
 *
 * <h2>3. Zero the mount offsets</h2>
 * The cluster listing prints name, roll, coverage, bearing and range for everything in frame.
 * Park the robot squarely facing a HIVE, at a known distance, and compare what it prints against
 * what you measured with a tape. A constant bearing offset means
 * {@link TurretConstants#CAMERA_MOUNT_YAW_DEG} is wrong. A range that is consistently short means
 * the camera is pitched and {@link TurretConstants#CAMERA_MOUNT_PITCH_DEG} is wrong.
 *
 * <h2>Controls (gamepad1 - this is a bench opmode, so the strict gamepad split does not apply)</h2>
 * <pre>
 *   dpad up / down ..... exposure +/- 1 ms  (swept between the min and max in constants)
 *   dpad right / left .. gain +/- 8 raw units
 *   left stick X ....... jog the turret in RAW SERVO POSITION
 *   A .................. record the current raw position as the LOW limit
 *   B .................. record the current raw position as the HIGH limit
 *   Y .................. send the turret to its commanded centre
 *   X .................. toggle which alliance getBestTarget() is asked for
 * </pre>
 */
@Config
@TeleOp(name = "Turret Tuning", group = "turret")
public class TurretTuningOpMode extends LinearOpMode {

    /** Raw servo units per second while jogging. Low on purpose - see the hard-stop warning. */
    public static double JOG_RATE_PER_SEC = 0.15;

    private Turret turret;
    private TurretVision vision;

    private double rawServoPos = TurretConstants.SERVO_CENTER_POSITION;
    private double recordedLow = Double.NaN;
    private double recordedHigh = Double.NaN;

    private int exposureMs = TurretConstants.EXPOSURE_MS;
    private int gain = -1; // -1 = camera maximum
    private Alliance alliance = Alliance.RED;

    private boolean prevUp, prevDown, prevLeft, prevRight, prevA, prevB, prevX, prevY;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap, telemetry);
        vision = new TurretVision(hardwareMap, telemetry);

        // Mirror the live stream to the dashboard so the crosshair overlay can be seen from a
        // laptop, not just on the Driver Station preview.
        // Null-guarded: a camera that failed to open must not take the opmode down.
        if (vision.getCameraStreamSource() != null) {
            FtcDashboard.getInstance().startCameraStream(vision.getCameraStreamSource(), 0);
        }

        telemetry.addLine("TURRET TUNING");
        telemetry.addLine("dpad up/dn = exposure   dpad L/R = gain");
        telemetry.addLine("left stick X = jog RAW servo   A = record low   B = record high");
        telemetry.addLine("Y = centre   X = swap alliance");
        telemetry.addLine("");
        telemetry.addLine("WARNING: jogging into a hard stop stalls both servos. Creep up on it.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime loopTimer = new ElapsedTime();
        double lastSeconds = loopTimer.seconds();

        while (opModeIsActive() && !isStopRequested()) {
            double nowSeconds = loopTimer.seconds();
            double dt = nowSeconds - lastSeconds;
            lastSeconds = nowSeconds;

            // Applies manual exposure/gain once the portal reaches STREAMING. Non-blocking.
            vision.update();

            // ---- Exposure and gain sweep -------------------------------------------------------
            boolean changed = false;
            if (gamepad1.dpad_up && !prevUp) {
                exposureMs = Math.min(TurretConstants.EXPOSURE_SWEEP_MAX_MS, exposureMs + 1);
                changed = true;
            }
            if (gamepad1.dpad_down && !prevDown) {
                exposureMs = Math.max(TurretConstants.EXPOSURE_SWEEP_MIN_MS, exposureMs - 1);
                changed = true;
            }
            if (gamepad1.dpad_right && !prevRight) {
                int max = vision.getMaxGain();
                gain = (gain < 0) ? max : Math.min(max, gain + 8);
                changed = true;
            }
            if (gamepad1.dpad_left && !prevLeft) {
                int max = vision.getMaxGain();
                gain = (gain < 0) ? max : Math.max(0, gain - 8);
                changed = true;
            }
            if (changed) vision.setExposureAndGain(exposureMs, gain);

            // ---- Servo jog, in raw units --------------------------------------------------------
            double jog = -gamepad1.left_stick_x; // stick right = positive turret direction
            if (Math.abs(jog) > 0.05) {
                rawServoPos += jog * JOG_RATE_PER_SEC * dt;
                rawServoPos = Math.max(0.0, Math.min(1.0, rawServoPos));
            }
            if (gamepad1.y && !prevY) rawServoPos = TurretConstants.SERVO_CENTER_POSITION;
            turret.setRawServoPosition(rawServoPos);

            if (gamepad1.a && !prevA) recordedLow = rawServoPos;
            if (gamepad1.b && !prevB) recordedHigh = rawServoPos;
            if (gamepad1.x && !prevX) {
                alliance = (alliance == Alliance.RED) ? Alliance.BLUE : Alliance.RED;
            }

            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;

            // ---- Telemetry ----------------------------------------------------------------------
            telemetry.addLine("--- CAMERA ---");
            telemetry.addData("State", vision.getCameraStateString());
            telemetry.addData("Settings applied", vision.areCameraSettingsApplied());
            telemetry.addData("Exposure requested (ms)", exposureMs);
            telemetry.addData("Gain requested", gain < 0 ? ("MAX (" + vision.getMaxGain() + ")") : gain);
            // What the CAMERA actually accepted, which is not always what was asked for - a value
            // outside the device's supported range is coerced, and that is worth seeing while
            // sweeping rather than discovering later.
            telemetry.addData("Exposure ACCEPTED (ms)", vision.getAppliedExposureMs());
            telemetry.addData("Gain ACCEPTED", vision.getAppliedGain());
            telemetry.addData("White balance ACCEPTED (K)", vision.getAppliedWhiteBalanceK());
            telemetry.addData("Focus mode", vision.getAppliedFocusMode());
            telemetry.addData("Setup report", vision.getCameraSetupReport());
            telemetry.addData("Detections / clusters",
                    "%d / %d", vision.getLastDetectionCount(), vision.getLastClusterCount());

            telemetry.addLine("--- SERVO CALIBRATION ---");
            telemetry.addData("Raw servo position", "%.4f", rawServoPos);
            telemetry.addData("Recorded LOW  (A)", Double.isNaN(recordedLow) ? "--"
                    : String.format("%.4f", recordedLow));
            telemetry.addData("Recorded HIGH (B)", Double.isNaN(recordedHigh) ? "--"
                    : String.format("%.4f", recordedHigh));
            telemetry.addData("Servo B reversed", TurretConstants.SERVO_B_REVERSED);
            double measured = turret.getMeasuredAngleDeg();
            telemetry.addData("Encoder angle (deg)", Double.isNaN(measured) ? "no encoder"
                    : String.format("%.2f", measured));

            // ---- Every cluster in frame, unfiltered ---------------------------------------------
            //
            // Printed raw, before any alliance/roll/coverage gate, because the point of this
            // listing is to see what the camera ACTUALLY reports - including the clusters the
            // selector is throwing away and why.
            telemetry.addLine("--- CLUSTERS IN FRAME ---");
            List<AprilTagDetection> detections = vision.getRawDetections();
            int shown = 0;
            // ===== BEGIN SDK 12 CLUSTER CODE - DISABLED =====
            // Full copy at TeamCode/sdk12-cluster-backup/TurretTuningOpMode.java.bak
            //
            // for (AprilTagDetection d : detections) {
            //     if (!(d instanceof AprilTagClusterDetection)) continue;
            //     AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            //     if (c.ftcPose == null) {
            //         telemetry.addData(c.metadata.name, "no pose solution");
            //         continue;
            //     }
            //     telemetry.addData(c.metadata.name,
            //             "roll %.1f  cov %d%%  bearing %.1f  range %.1fin  (x %.1f y %.1f z %.1f)",
            //             c.ftcPose.roll, c.percentClusterFound, c.ftcPose.bearing, c.ftcPose.range,
            //             c.ftcPose.x, c.ftcPose.y, c.ftcPose.z);
            //     shown++;
            // }
            // ===== END SDK 12 CLUSTER CODE - DISABLED =====

            // Raw detection count still works on SDK 10.1.0, so the camera, the exposure sweep and
            // the servo calibration in this opmode are all still usable - only the per-cluster
            // breakdown is gone.
            telemetry.addData("  raw detections", detections.size());
            if (shown == 0) {
                telemetry.addLine("  CLUSTER DECODE DISABLED - needs SDK 12.0.0");
            }

            telemetry.addLine("--- SELECTOR ---");
            telemetry.addData("Asking for", alliance);
            TurretTarget best = vision.getBestTarget(alliance);
            telemetry.addData("Best target", best == null
                    ? ("none: " + vision.getLastRejectReason()) : best.toString());

            telemetry.update();
        }

        vision.close();
    }
}
