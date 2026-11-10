package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;
import org.firstinspires.ftc.teamcode.opmodes.Biobuzz3Tip_HiveAuto;
import org.firstinspires.ftc.teamcode.opmodes.Biobuzz_2_Tip_HiveAuto;

import java.util.ArrayList;
import java.util.List;

/**
 * Camera-only test of the autos' HIVE tip detection. Nothing moves: no drive, intake or shooter.
 * It runs the autos' own HiveWatcher + HiveAutoKit code (same wait actions, same queries, same
 * windows, same @Config constants), so a result here is exactly what the auto would decide.
 *
 * Two tests (Y switches between them while no trial is running):
 *   2 TIP  - Biobuzz_2_Tip_HiveAuto. Put the robot at the RIGHT shot (61, -14) facing -x (the auto's
 *            start pose). Press A the moment the preload volley has left: the auto's
 *            waitForOwnTipStart(RIGHT, TIP_START_WAIT_S) runs, then ownTipStarted(RIGHT) picks
 *            TIPPED (wall set at LEFT) or NOT TIPPED (wall set at RIGHT).
 *   3 TIP  - Biobuzz3Tip_HiveAuto / Biobuzz3Tip_AlliancePreLoad_Compatible. Put the robot at the
 *            LEFT shot (-52, -14) facing +x. Press A when the auto would arrive there: the auto's
 *            waitForAllianceTip(MAX_WAIT_FOR_ALLIANCE_TIP_S) runs, then allianceTipResult() gives
 *            TIPPED / NOT_TIPPED / UNKNOWN.
 * The robot must sit still at that pose: the auto passes its pose to HiveWatcher, and here the
 * pose is that fixed shooting pose.
 *
 * Controls (INIT and RUN; INIT has no time limit):
 *   B / X (INIT only) ... alliance RED / BLUE (same selector and stored choice as the autos)
 *   Y ................... switch 2 TIP / 3 TIP test
 *   A ................... start a trial (the auto's real wait), result is frozen when it ends
 *   Right bumper ........ cancel the trial / clear the result and event log
 *
 * Telemetry and the camera image also go to FTC Dashboard (192.168.43.1:8080/dash).
 */
@Config
@TeleOp(name = "BioBuzz HIVE Tip Detect Test", group = "testing")
public class BiobuzzHiveTipDetectTest extends LinearOpMode {
    public static int DASHBOARD_FPS = 10;

    private static final Pose2d RIGHT_SHOT = new Pose2d(61, -14, Math.toRadians(180));
    private static final Pose2d LEFT_SHOT = new Pose2d(-52, -14, Math.toRadians(0));

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HiveWatcher hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        HiveAutoKit kit = new HiveAutoKit(null, null, hive, telemetry, null, null,
                new Pose2d(-49.5, -58, Math.toRadians(90)), LEFT_SHOT, RIGHT_SHOT);
        hive.startDashboardStream(DASHBOARD_FPS);

        boolean twoTip = true;
        boolean lastY = false, lastA = false, lastRb = false;
        Action trial = null;
        ElapsedTime trialClock = new ElapsedTime();
        String result = "none yet - press A";
        List<String> log = new ArrayList<>();

        while (!isStopRequested()) {
            boolean running = isStarted();
            if (!running && AllianceSelector.poll(gamepad1, gamepad2)) hive.setAlliance(AllianceSelector.red());

            Pose2d pose = twoTip ? RIGHT_SHOT : LEFT_SHOT;
            hive.update(pose);
            for (String e : hive.drainEvents()) {
                log.add(String.format("%6.2fs %s", trial != null ? trialClock.seconds() : 0.0, e));
            }
            while (log.size() > 40) log.remove(0);

            boolean y = gamepad1.y || gamepad2.y;
            boolean a = gamepad1.a || gamepad2.a;
            boolean rb = gamepad1.right_bumper || gamepad2.right_bumper;
            if (y && !lastY && trial == null) {
                twoTip = !twoTip;
                result = "none yet - press A";
                log.clear();
            }
            if (a && !lastA && trial == null) {
                trial = twoTip
                        ? kit.waitForOwnTipStart(Side.RIGHT, Biobuzz_2_Tip_HiveAuto.TIP_START_WAIT_S)
                        : kit.waitForAllianceTip(Biobuzz3Tip_HiveAuto.MAX_WAIT_FOR_ALLIANCE_TIP_S);
                trialClock.reset();
                log.clear();
                result = "waiting...";
            }
            if (rb && !lastRb) {
                trial = null;
                result = "none yet - press A";
                log.clear();
            }
            lastY = y;
            lastA = a;
            lastRb = rb;

            if (trial != null && !trial.run(new TelemetryPacket())) {
                double s = trialClock.seconds();
                if (twoTip) {
                    result = kit.ownTipStarted(Side.RIGHT)
                            ? String.format("TIPPED after %.2f s -> wall set at LEFT", s)
                            : String.format("NOT TIPPED (no tip start in %.1f s) -> wall set at RIGHT",
                            Biobuzz_2_Tip_HiveAuto.TIP_START_WAIT_S);
                } else {
                    HiveAutoKit.TipResult tip = kit.allianceTipResult();
                    result = String.format("%s after %.2f s -> %s", tip, s,
                            tip == HiveAutoKit.TipResult.NOT_TIPPED ? "we tip RIGHT ourselves"
                                    : tip == HiveAutoKit.TipResult.TIPPED ? "partner tipped, LEFT up" : "no camera frames, LEFT as planned");
                }
                log.add(String.format("%6.2fs RESULT %s", s, result));
                trial = null;
            }

            if (!running) AllianceSelector.show(telemetry);
            telemetry.addData("Alliance check", hive.ownTagsSeen(2.0) ? "OK - our HIVE tags in view"
                    : hive.otherAllianceTagsSeen(2.0) ? "CHECK ALLIANCE - only the other alliance's tags seen" : "our HIVE tags not seen yet");
            telemetry.addLine(twoTip
                    ? "TEST: 2 TIP  - robot at RIGHT shot (61,-14) facing -x, A when the preload leaves"
                    : "TEST: 3 TIP  - robot at LEFT shot (-52,-14) facing +x, A when the auto arrives");
            telemetry.addLine("Y switch test | A start trial | RB cancel/clear" + (running ? "" : " | B/X alliance"));
            telemetry.addData("Trial", trial != null ? String.format("running %.2f s / max %.1f s", trialClock.seconds(),
                    twoTip ? Biobuzz_2_Tip_HiveAuto.TIP_START_WAIT_S
                            : Biobuzz3Tip_HiveAuto.MAX_WAIT_FOR_ALLIANCE_TIP_S + HiveAutoKit.ALLIANCE_SETTLE_EXTRA_S) : "idle");
            telemetry.addData("RESULT", result);

            telemetry.addLine("--- live (what the auto would see now) ---");
            if (twoTip) {
                telemetry.addData("ownTipStarted(RIGHT)", kit.ownTipStarted(Side.RIGHT));
                telemetry.addData("  RIGHT tipping", hive.tipping(Side.RIGHT, HiveWatcher.TIP_MAX_S));
                telemetry.addData("  RIGHT confirmed down", hive.confirmedDown(Side.RIGHT, HiveWatcher.TIP_MAX_S));
                telemetry.addData("  LEFT settled up", hive.settledUp(Side.LEFT, 0.6));
            } else {
                telemetry.addData("allianceTipResult", kit.allianceTipResult());
                telemetry.addData("  LEFT settled up", hive.settledUp(Side.LEFT, 0.6));
                telemetry.addData("  RIGHT settled up", hive.settledUp(Side.RIGHT, 0.6));
                telemetry.addData("  LEFT down by absence", hive.downByAbsence(Side.LEFT, HiveAutoKit.ALLIANCE_ABSENCE_S));
                telemetry.addData("  LEFT tip activity", hive.tipActivity(Side.LEFT, 1.0));
            }
            Side up = hive.upCell(1.0);
            telemetry.addData("CELL up", up == null ? "unknown" : up);
            telemetry.addData("HIVE moving", hive.hiveMoving(HiveAutoKit.HIVE_STILL_S));
            telemetry.addData("HIVE", hive.describe());
            telemetry.addLine("--- events ---");
            for (int i = Math.max(0, log.size() - 10); i < log.size(); i++) telemetry.addLine(log.get(i));
            telemetry.update();
        }
        hive.close();
    }
}
