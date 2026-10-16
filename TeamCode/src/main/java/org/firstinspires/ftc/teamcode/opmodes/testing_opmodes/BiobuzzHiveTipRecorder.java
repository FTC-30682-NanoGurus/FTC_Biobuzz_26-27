package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher;

import java.util.ArrayList;
import java.util.List;

/**
 * Measures real HIVE tips with the same detector the autos use (HiveWatcher), so its thresholds
 * can be checked on your field.
 *
 * Put the robot still at a shooting pose: (-52, -14) facing +x looks at the LEFT CELL, (61, -14)
 * facing -x at the RIGHT CELL. Two tests:
 *   1. Facing the CELL that is UP, tip it (launch into it): shows when TIPPING fires and how long
 *      its tags stay visible while it sinks.
 *   2. Facing the CELL that is DOWN, tip the other one: the near CELL comes up, so the recorder
 *      times the whole tip, from its first movement to "settled UP".
 * Telemetry shows, for each CELL:
 *   - row: height of the tag row (above the tiles once TurretConstants.CAMERA_OFFSET_Z_IN is set),
 *   - off rest: how far the tags are from where the CELL rested (TIPPING fires at TIP_MOVE_IN),
 * plus every detector event with its time, and the time from the first movement to the end of the
 * tip. Use these to set HiveWatcher.TIP_MAX_S, TIP_MOVE_IN and UP_ROW_HEIGHT_IN, and
 * HiveAutoKit.ALLIANCE_SETTLE_EXTRA_S / TIP_WAIT_MAX_S.
 *
 * INIT: B = red, X = blue (either gamepad). RUNNING: A = clear the log.
 * The robot must not move - its pose is not tracked here.
 *
 * Telemetry and the camera image also go to FTC Dashboard (192.168.43.1:8080/dash), from INIT on.
 */
@Config
@TeleOp(name = "BioBuzz HIVE Tip Recorder", group = "testing")
public class BiobuzzHiveTipRecorder extends LinearOpMode {
    /** FTC Dashboard camera stream rate. Keep it low: every frame goes over the hub's WiFi. */
    public static int DASHBOARD_FPS = 10;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        hive.startDashboardStream(DASHBOARD_FPS);
        Pose2d still = new Pose2d(0, 0, 0);

        while (!isStarted() && !isStopRequested()) {
            if (AllianceSelector.poll(gamepad1, gamepad2)) hive.setAlliance(AllianceSelector.red());
            hive.update(still);
            AllianceSelector.show(telemetry);
            telemetry.addData("HIVE", hive.describe());
            telemetry.update();
        }

        ElapsedTime clock = new ElapsedTime();
        List<String> log = new ArrayList<>();
        double firstMove = -1, lastTipEnd = -1;
        String lastTip = "none yet";

        while (opModeIsActive()) {
            hive.update(still);
            double t = clock.seconds();
            for (String e : hive.drainEvents()) {
                log.add(String.format("%6.2fs %s", t, e));
                if ((e.contains("TIPPING") || e.contains("moving")) && firstMove < 0) firstMove = t;
                if (e.contains("settled UP") && firstMove >= 0) {
                    lastTip = String.format("%.2f s from first movement to a CELL at rest", t - firstMove);
                    lastTipEnd = t;
                    firstMove = -1;
                }
            }
            if (gamepad1.a) {
                log.clear();
                firstMove = -1;
                lastTip = "none yet";
            }

            boolean moving = hive.hiveMoving(HiveWatcher.TIP_MAX_S);
            telemetry.addData("HIVE", hive.describe());
            telemetry.addData("Moving now", hive.hiveMoving(0.5));
            telemetry.addData("Tip in progress",
                    firstMove >= 0 ? String.format("%.2f s since first movement", t - firstMove) : (moving ? "recently" : "no"));
            telemetry.addData("Last tip", lastTip + (lastTipEnd >= 0 ? String.format(" (ended at %.1fs)", lastTipEnd) : ""));
            telemetry.addLine("--- events (A clears) ---");
            for (int i = Math.max(0, log.size() - 12); i < log.size(); i++) telemetry.addLine(log.get(i));
            telemetry.update();
        }
        hive.close();
    }
}
