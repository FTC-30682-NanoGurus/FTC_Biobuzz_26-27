package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit.ShotChoice;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveAutoKit.TipResult;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;
import org.firstinspires.ftc.teamcode.opmodes.Biobuzz3Tip_HiveAuto;
import org.firstinspires.ftc.teamcode.opmodes.Biobuzz_2_Tip_HiveAuto;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Camera-only preview of the HIVE decisions in Biobuzz_2_Tip_HiveAuto and Biobuzz3Tip_HiveAuto.
 * The robot NEVER moves: no drive powers, no intake, no shooter. Put the robot at a CHECKPOINT's
 * pose (one of the places an auto makes a camera decision), keep it still, and read:
 * <ul>
 *   <li><b>TAGS</b>: everything HiveWatcher has collected - camera state, the alliance check, each
 *       CELL's state / last seen / tag-row height / distance off rest / time missing, every query
 *       the autos make with the windows they use, upCell at those windows, and the event log.</li>
 *   <li><b>DECISION</b>: what the auto decides at this checkpoint, live ("NOW"), and - after pressing
 *       A - the result of running the auto's REAL wait action from this moment ("DECIDED").</li>
 *   <li><b>NEXT PATH</b>: the timeline the auto would then drive, with the exact prebuilt paths
 *       (their planned Road Runner times), the runtime routes RouteOptimizer picks, every time-budget
 *       check with its deadline, and the park time.</li>
 * </ul>
 *
 * <h2>Same logic as the autos</h2>
 * The waits, the shot decision, the alliance-tip result, the budget checks (fits) and the runtime
 * routes are the autos' own HiveAutoKit / HiveWatcher code, called on HiveAutoKit instances built
 * with the autos' poses, constraints and prepared routes. The auto-level branches (which plan,
 * flower-one / flower-two / wall-set choices) and the prebuilt paths are COPIED from the two auto
 * files - if you change a path, pose or budget constant there, change it here too.
 *
 * Decisions further down the timeline than this checkpoint are made by the camera when the robot
 * gets there; the timeline shows them as the camera would answer RIGHT NOW, and assumes future
 * shots find their planned CELL up. Budget checks use the match time: set the time the robot
 * would reach this checkpoint with the dpad (it starts at the nominal time with no waits).
 *
 * <h2>Controls (INIT and RUN - use INIT: it has no 30 s limit and keeps the camera preview)</h2>
 * <pre>
 *   B / X (INIT only) ..... alliance RED / BLUE
 *   Y ..................... switch 2 TIP / 3 TIP auto
 *   Dpad left / right ..... previous / next checkpoint
 *   Dpad up / down ........ match time at this checkpoint +/- TIME_STEP_S
 *   Left bumper ........... match time back to the nominal time
 *   A ..................... run the auto's wait from now, then freeze its decision
 *   Right bumper .......... clear the frozen decision
 * </pre>
 */
@Autonomous(name = "BioBuzz HIVE Decision Preview", group = "testing")
public class BiobuzzHiveDecisionPreview extends LinearOpMode {
    public static double TIME_STEP_S = 0.5;
    public static int EVENT_LINES = 6;
    public static double TELEMETRY_INTERVAL_MS = 250;

    private static final double F1_BLOCK_S = 1.94 + 0.4 + 3.0 + 1.71 + 1.0 + 0.3;
    private static final double F2_FROM_LEFT_S = 2.90 + 3.0 + 3.09 + 1.0 + 0.3;
    private static final double F2_FROM_RIGHT_S = 1.98 + 3.0 + 3.09 + 1.0 + 0.3;

    private static final double FLOWER_TWO_BLOCK_S = 5.90 + 1.84 + 1.0;
    private static final double WALL_TO_LEFT_S = 6.14 + 1.0;
    private static final double WALL_NEAR_S = 4.69 + 1.0;
    private static final double WALL_ONLY_S = 4.6;
    private static final double SOLO_FLOWER_BLOCK_S = 4.98 + 1.71 + 1.0;

    // Biobuzz_2_Tip_HiveAuto poses
    private final Pose2d begin2 = new Pose2d(61, -14, Math.toRadians(180));
    private final Pose2d leftShot2 = new Pose2d(-52, -14, Math.toRadians(0));
    private final Pose2d leftShotBack2 = new Pose2d(-52, -14, Math.toRadians(360));
    private final Pose2d rightShot2 = new Pose2d(61, -14, Math.toRadians(180));
    private final Pose2d wallSet = new Pose2d(62, -58, Math.toRadians(360));
    private final Pose2d rightNear = new Pose2d(56.69, -40.01, Math.toRadians(155));
    private final Pose2d leftNear = new Pose2d(-53.48, -43.82, Math.toRadians(30));
    private final Pose2d flowerOne2 = new Pose2d(-55, -22, Math.toRadians(180));
    private final Pose2d flowerTwo = new Pose2d(24, -55, Math.toRadians(-90));
    private final Pose2d park2 = new Pose2d(-49.5, -58, Math.toRadians(90));

    // Biobuzz3Tip_HiveAuto poses
    private final Pose2d begin3 = new Pose2d(-65, -11, Math.toRadians(0));
    private final Pose2d leftShot3 = new Pose2d(-52, -14, Math.toRadians(0));
    private final Pose2d leftShotAfterFlower = new Pose2d(-52, -14, Math.toRadians(1));
    private final Pose2d wallShotLeft = new Pose2d(-52, -16, Math.toRadians(0));
    private final Pose2d rightShot3 = new Pose2d(61, -14, Math.toRadians(-179.5));
    private final Pose2d rightShotSolo = new Pose2d(61, -14, Math.toRadians(180));
    private final Pose2d flowerOne3 = new Pose2d(-55, -22, Math.toRadians(-179.5));
    private final Pose2d flowerOneSolo = new Pose2d(-55, -22, Math.toRadians(180));
    private final Pose2d nearShot = new Pose2d(56.69, -40.01, Math.toRadians(155));
    private final Pose2d soloBackEnd = new Pose2d(-52, -14, Math.toRadians(1));
    private final Pose2d park3 = new Pose2d(-21, -59.5, Math.toRadians(180));

    private MecanumDrive drive;
    private HiveWatcher hive;
    private HiveAutoKit kit2, kit3;
    private VelConstraint maxVel, maxIntakeVel;
    private AccelConstraint maxAccel;

    private Leg wallSetToLeftNear, wallSetToRightNear, leftNearToFlowerOne, backToShoot, leftToFlowerTwo,
            rightNearToFlowerTwo, flowerTwoToLeftNear;
    private Leg toLeftShot, toFlowerOne, backFromFlowerOne, toFlowerTwo, crossToRight, wallToLeft, wallToNear,
            wallOnly, parkFromLeft, soloWallSet, soloFlower, soloBack;
    private final Map<String, Leg> routeCache = new HashMap<>();

    private final List<Checkpoint> cps2 = new ArrayList<>(), cps3 = new ArrayList<>();

    /** A path with its planned time and a readable description of where it goes. */
    private static final class Leg {
        final String name, via;
        final double seconds;

        Leg(String name, Action action, String via) {
            this.name = name;
            this.via = via;
            this.seconds = HiveAutoKit.motionTime(action);
        }
    }

    /** How decisions further down the timeline are answered: by the camera now, or forced. */
    private static final class Forecast {
        final boolean live;
        final Side f2Up, wallUp;

        Forecast(boolean live, Side f2Up, Side wallUp) {
            this.live = live;
            this.f2Up = f2Up;
            this.wallUp = wallUp;
        }
    }

    private final Forecast liveForecast = new Forecast(true, null, null);

    /** What a checkpoint decided, and the rest of the auto from there. */
    private static final class Decision {
        final String text;
        final Consumer<Sim> next;

        Decision(String text, Consumer<Sim> next) {
            this.text = text;
            this.next = next;
        }
    }

    /** One place where an auto makes a camera decision. */
    private abstract class Checkpoint {
        final int id;
        final String name;
        final Pose2d pose;
        final HiveAutoKit kit;
        final Side shotSide;
        final double freshS;
        double nominalT = Double.NaN;

        Checkpoint(int id, String name, Pose2d pose, HiveAutoKit kit, Side shotSide, double freshS) {
            this.id = id;
            this.name = name;
            this.pose = pose;
            this.kit = kit;
            this.shotSide = shotSide;
            this.freshS = freshS;
        }

        Action makeWait() {
            return shotSide == null ? new NullAction() : kit.waitForShot(freshS);
        }

        boolean postVolleyDecision() {
            return false;
        }

        String waitStatus() {
            switch (kit.shotWait(freshS)) {
                case HIVE_MOVING:
                    return String.format("HIVE moving - waits up to %.1f s for it to settle", HiveAutoKit.TIP_WAIT_MAX_S);
                case NO_EVIDENCE:
                    return String.format("no evidence %.1f s fresh - waits up to %.1f s for some", freshS, HiveAutoKit.GATE_MAX_WAIT_S);
                default:
                    return hive.cameraHealthy() ? "ready - evidence in hand" : "ready - no camera frames, no wait";
            }
        }

        abstract Decision decide(ShotChoice shot);
    }

    /** A shot checkpoint: the auto's shot decision, then {@code then}. */
    private Checkpoint shotCp(int id, String name, Pose2d pose, HiveAutoKit kit, Side side, double freshS,
                              Consumer<Sim> then) {
        return new Checkpoint(id, name, pose, kit, side, freshS) {
            @Override
            Decision decide(ShotChoice shot) {
                return new Decision(HiveAutoKit.describeShot(side, shot), s -> {
                    s.applyShot(side, shot);
                    then.accept(s);
                });
            }
        };
    }

    /** Walks an auto's plan forward in match time, writing one line per step. */
    private final class Sim {
        final HiveAutoKit kit;
        final Forecast forecast;
        final Map<Integer, Double> marks;
        final List<String> lines = new ArrayList<>();
        double t;
        Pose2d pose;

        Sim(HiveAutoKit kit, double t, Pose2d pose, Forecast forecast, Map<Integer, Double> marks) {
            this.kit = kit;
            this.t = t;
            this.pose = pose;
            this.forecast = forecast;
            this.marks = marks;
        }

        void line(String msg) {
            lines.add(String.format("%5.1fs %s", t, msg));
        }

        void drive(Leg l, Pose2d end) {
            line(String.format("%s (%.2f s): %s", l.name, l.seconds, l.via));
            t += l.seconds;
            pose = end;
        }

        void route(Pose2d to) {
            drive(routeLeg(kit, pose, to), to);
        }

        void leg(Pose2d expectedStart, Leg prebuilt, Pose2d target, double thenCollectS) {
            if (near(pose, expectedStart)) {
                drive(prebuilt, target);
                return;
            }
            line("not at " + fmt(expectedStart) + " - rebuilding leg to " + fmt(target));
            route(target);
            if (thenCollectS > 0) {
                line(String.format("collect from FLOWER %.1f s", thenCollectS));
                t += thenCollectS;
            }
        }

        void routeFromFlower(Pose2d to) {
            drive(fromFlowerRouteLeg(kit, pose, to), to);
        }

        void fromFlowerLeg(Pose2d expectedStart, Leg prebuilt, Pose2d target) {
            if (near(pose, expectedStart)) {
                drive(prebuilt, target);
                return;
            }
            line("not at " + fmt(expectedStart) + " - rebuilding leg from FLOWER to " + fmt(target));
            routeFromFlower(target);
        }

        void flowerLeg(Pose2d expectedStart, Leg prebuilt, Pose2d flower, double thenCollectS) {
            if (near(pose, expectedStart)) {
                drive(prebuilt, flower);
                return;
            }
            line("not at " + fmt(expectedStart) + " - rebuilding FLOWER leg to " + fmt(flower) + ", ramp down now");
            drive(flowerRouteLeg(kit, pose, flower), flower);
            line(String.format("collect from FLOWER %.1f s", thenCollectS));
            t += thenCollectS;
        }

        void transfer(String what) {
            line(String.format("%s (%.1f s)", what, HiveAutoKit.TRANSFER_S));
            t += HiveAutoKit.TRANSFER_S;
        }

        void mark(int cp) {
            if (marks != null && !marks.containsKey(cp)) marks.put(cp, t);
        }

        void shot(int cp, Side side) {
            mark(cp);
            transfer("shotPoint(" + side + ") - camera decides there; assumed " + side + " up, volley");
        }

        void applyShot(Side side, ShotChoice c) {
            if (c == ShotChoice.SHOOT_UP || c == ShotChoice.SHOOT_NO_VIEW) {
                transfer("volley into " + side);
            } else if (c == ShotChoice.CROSS) {
                route(kit.shotPose(side.other()));
                transfer("waitForShot, then volley into " + side.other() + " unless it is down again");
            } else {
                line("HOLD - no volley");
            }
        }

        boolean fits(String what, double seconds, Pose2d end) {
            boolean ok = kit.fitsAt(t, seconds, end);
            line(String.format("budget %s: %.2f s + park, start by %.1f s -> %s", what, seconds,
                    kit.latestStart(seconds, end), ok ? "YES" : "NO"));
            return ok;
        }

        void note(String msg) {
            line(msg);
        }

        String source() {
            return forecast.live ? "camera NOW" : "assumed";
        }

        void parked() {
            line("PARKED" + (t > HiveAutoKit.AUTO_LENGTH_S
                    ? " - past 30 s: the park watchdog would abandon the plan earlier and park" : ""));
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Building paths and preparing routes (as the autos do in INIT)...");
        telemetry.update();

        drive = new MecanumDrive(hardwareMap, begin2);
        hive = new HiveWatcher(hardwareMap, telemetry, AllianceSelector.red());
        buildConstraints();
        kit2 = new HiveAutoKit(drive, null, hive, telemetry, maxVel, maxAccel, park2, leftShot2, rightShot2);
        kit3 = new HiveAutoKit(drive, null, hive, telemetry, maxVel, maxAccel, park3, leftShot3, rightShot3);

        kit2.prepareDefaults();
        kit2.prepareFlower(leftNear, flowerOne2);
        kit2.prepareFromFlower(flowerTwo, leftNear);
        kit2.prepareFlower(rightNear, flowerTwo);
        kit2.prepare(leftNear, park2);
        kit2.prepare(leftShotBack2, park2);

        kit3.prepareDefaults();
        kit3.prepare(leftShot3, rightShotSolo);
        kit3.prepareFromFlower(flowerTwo, leftShot3);
        kit3.prepare(wallSet, park3);
        kit3.prepare(rightShot3, wallShotLeft);
        kit3.prepare(nearShot, park3);
        kit3.prepare(wallShotLeft, park3);

        buildLegs();
        buildCheckpoints();
        computeNominalTimes();

        boolean threeTip = true;
        int cpIndex = 0;
        double matchT = startTime(cps3.get(0));
        boolean prevY = false, prevL = false, prevR = false, prevU = false, prevD = false, prevA = false, prevLB = false;
        boolean prevRB = false;

        Action runWait = null;
        int runPhase = 0;
        ShotChoice runShot = null;
        double runDecisionT = 0;
        Decision frozen = null;
        List<String> frozenTimeline = null;
        ElapsedTime runTimer = new ElapsedTime();
        ElapsedTime telemetryTimer = new ElapsedTime();
        ElapsedTime clock = new ElapsedTime();
        TelemetryPacket packet = new TelemetryPacket();
        List<String> events = new ArrayList<>();

        while (!isStopRequested()) {
            if (!isStarted() && AllianceSelector.poll(gamepad1, gamepad2)) {
                hive.setAlliance(AllianceSelector.red());
                events.clear();
            }

            List<Checkpoint> cps = threeTip ? cps3 : cps2;
            boolean changed = false;
            if (gamepad1.y && !prevY) {
                threeTip = !threeTip;
                cps = threeTip ? cps3 : cps2;
                cpIndex = 0;
                changed = true;
            }
            if (gamepad1.dpad_right && !prevR) {
                cpIndex = (cpIndex + 1) % cps.size();
                changed = true;
            }
            if (gamepad1.dpad_left && !prevL) {
                cpIndex = (cpIndex + cps.size() - 1) % cps.size();
                changed = true;
            }
            Checkpoint cp = cps.get(cpIndex);
            if (changed || (gamepad1.left_bumper && !prevLB)) matchT = startTime(cp);
            if (changed || (gamepad1.right_bumper && !prevRB)) {
                runWait = null;
                frozen = null;
                frozenTimeline = null;
            }
            if (gamepad1.dpad_up && !prevU) matchT += TIME_STEP_S;
            if (gamepad1.dpad_down && !prevD) matchT = Math.max(0, matchT - TIME_STEP_S);
            if (gamepad1.a && !prevA) {
                cp.kit.setMatchTime(matchT);
                runWait = cp.makeWait();
                runPhase = 1;
                runShot = null;
                frozen = null;
                frozenTimeline = null;
                runTimer.reset();
            }
            prevY = gamepad1.y;
            prevL = gamepad1.dpad_left;
            prevR = gamepad1.dpad_right;
            prevU = gamepad1.dpad_up;
            prevD = gamepad1.dpad_down;
            prevA = gamepad1.a;
            prevLB = gamepad1.left_bumper;
            prevRB = gamepad1.right_bumper;

            drive.pose = cp.pose;
            hive.update(drive.pose);
            for (String e : hive.drainEvents()) events.add(String.format("%7.1fs %s", clock.seconds(), e));
            while (events.size() > 40) events.remove(0);

            if (runWait != null) {
                if (!runWait.run(packet)) {
                    if (runPhase == 1) {
                        runDecisionT = cp.kit.t();
                        if (cp.shotSide != null) {
                            runShot = cp.kit.shotChoice(cp.shotSide, true, cp.freshS, drive.pose, runDecisionT);
                        }
                        if (cp.postVolleyDecision() && shoots(runShot)) {
                            runWait = new SleepAction(HiveAutoKit.TRANSFER_S);
                            runPhase = 2;
                        } else {
                            runWait = null;
                        }
                    } else {
                        runWait = null;
                    }
                    if (runWait == null) {
                        frozen = cp.decide(runShot);
                        frozenTimeline = simulate(cp, frozen, runDecisionT);
                    }
                }
            } else {
                cp.kit.setMatchTime(matchT);
            }

            if (telemetryTimer.milliseconds() < TELEMETRY_INTERVAL_MS) continue;
            telemetryTimer.reset();

            telemetry.addLine("BIOBUZZ HIVE DECISION PREVIEW - the robot never moves");
            if (!isStarted()) AllianceSelector.show(telemetry);
            else telemetry.addLine("ALLIANCE: " + (hive.isRed() ? "RED" : "BLUE"));
            telemetry.addData("Auto (Y)", threeTip ? "Biobuzz3Tip_HiveAuto" : "Biobuzz_2_Tip_HiveAuto");
            telemetry.addData("Checkpoint (dpad L/R)", "%d/%d %s", cpIndex + 1, cps.size(), cp.name);
            telemetry.addData("Place robot at", fmt(cp.pose));
            telemetry.addData("Match time (dpad U/D, LB)", "%.1f s (nominal %s)", matchT,
                    Double.isNaN(cp.nominalT) ? "not reached" : String.format("%.1f s", cp.nominalT));

            telemetry.addLine("--- TAGS ---");
            telemetry.addData("Camera", "%s, frames %s", hive.cameraState(), hive.cameraHealthy() ? "arriving" : "NOT arriving");
            telemetry.addData("Alliance check", hive.ownTagsSeen(2.0) ? "OK - our HIVE tags in view"
                    : hive.otherAllianceTagsSeen(2.0) ? "CHECK ALLIANCE - only the other alliance's tags seen" : "our HIVE tags not seen yet");
            telemetry.addLine(hive.describe(Side.LEFT));
            telemetry.addLine(hive.describe(Side.RIGHT));
            telemetry.addLine(flags(Side.LEFT));
            telemetry.addLine(flags(Side.RIGHT));
            telemetry.addData("upCell", "0.6s %s | 1.0s %s | 1.5s %s | 8s %s | 15s %s",
                    up(0.6), up(1.0), up(1.5), up(8.0), up(15.0));
            telemetry.addData("HIVE moving", "%s (0.5 s)  %s (%.1f s)", yn(hive.hiveMoving(HiveAutoKit.HIVE_STILL_S)),
                    yn(hive.hiveMoving(HiveWatcher.TIP_MAX_S)), HiveWatcher.TIP_MAX_S);
            for (int i = Math.max(0, events.size() - EVENT_LINES); i < events.size(); i++) telemetry.addLine(events.get(i));

            telemetry.addLine("--- DECISION ---");
            telemetry.addData("Wait", cp.waitStatus());
            Decision now = cp.decide(cp.shotSide == null ? null
                    : cp.kit.shotChoice(cp.shotSide, true, cp.freshS, cp.pose, matchT));
            telemetry.addData("NOW", now.text);
            if (runWait != null) {
                telemetry.addData("A", "running the auto's wait: %.1f s%s", runTimer.seconds(),
                        runPhase == 2 ? " (volley, then the wall-set decision)" : "");
            } else if (frozen != null) {
                telemetry.addData("DECIDED", "at %.1f s: %s  (RB clears)", runDecisionT, frozen.text);
            } else {
                telemetry.addData("A", "run the auto's wait from now and freeze its decision");
            }

            telemetry.addLine(frozen != null ? "--- NEXT PATH (DECIDED) ---" : "--- NEXT PATH (if decided NOW) ---");
            List<String> timeline = frozen != null ? frozenTimeline : simulate(cp, now, matchT);
            for (String l : timeline) telemetry.addLine(l);
            telemetry.update();
        }
        hive.close();
    }

    // =============================================================================================
    // Decisions and plans, copied from the autos
    // =============================================================================================

    private void buildCheckpoints() {
        HiveAutoKit k = kit2;
        double fresh = HiveAutoKit.GATE_FRESH_S;
        cps2.add(new Checkpoint(0, "START: preload volley, then own-tip check on RIGHT", begin2, k, null, 0) {
            @Override
            Action makeWait() {
                return kit.waitForTipStart(Side.RIGHT, HiveAutoKit.OWN_TIP_GRACE_S);
            }

            @Override
            String waitStatus() {
                return String.format("waitForTipStart(RIGHT, %.1f s): tipping %s, settled up %s -> %s",
                        HiveAutoKit.OWN_TIP_GRACE_S, yn(hive.tipping(Side.RIGHT, HiveWatcher.TIP_MAX_S)),
                        yn(hive.settledUp(Side.RIGHT, 0.4)),
                        !hive.cameraHealthy() || hive.tipping(Side.RIGHT, HiveWatcher.TIP_MAX_S)
                                || !hive.settledUp(Side.RIGHT, 0.4) ? "ends now" : "keeps waiting");
            }

            @Override
            Decision decide(ShotChoice shot) {
                if (hive.settledUp(Side.RIGHT, 0.4)) {
                    return new Decision("RIGHT CELL did not tip - wall set, angled shot at RIGHT", BiobuzzHiveDecisionPreview.this::t2NotTipped);
                }
                return new Decision(hive.tipping(Side.RIGHT, 5.0) ? "RIGHT CELL seen tipping"
                        : "no RIGHT CELL at rest in view - assuming it tipped", BiobuzzHiveDecisionPreview.this::t2Tipped);
            }
        });
        cps2.add(shotCp(1, "TIPPED plan: wall-set shot at LEFT from leftNear", leftNear, k, Side.LEFT, fresh, this::t2AfterT1));
        cps2.add(shotCp(2, "TIPPED plan: flower-one shot at LEFT", leftShotBack2, k, Side.LEFT, fresh, this::t2AfterT2));
        cps2.add(shotCp(3, "TIPPED plan: flower-two backup shot at LEFT", leftNear, k, Side.LEFT, fresh, this::park2));
        cps2.add(shotCp(4, "NOT TIPPED plan: angled wall-set shot at RIGHT", rightNear, k, Side.RIGHT, 8.0, this::t2AfterN1));
        cps2.add(shotCp(5, "NOT TIPPED plan: flower-two shot at LEFT", leftNear, k, Side.LEFT, fresh, this::t2AfterN2));
        cps2.add(shotCp(6, "NOT TIPPED plan: flower-one shot at LEFT", leftShotBack2, k, Side.LEFT, fresh, this::park2));

        k = kit3;
        cps3.add(new Checkpoint(0, "START: at LEFT shot, wait for the partner's tip", leftShot3, k, null, 0) {
            @Override
            Action makeWait() {
                return kit.waitForAllianceTip(Biobuzz3Tip_HiveAuto.MAX_WAIT_FOR_ALLIANCE_TIP_S);
            }

            @Override
            String waitStatus() {
                return String.format("waitForAllianceTip(%.1f s + up to %.1f s): LEFT settled up %s, LEFT tip activity %s, LEFT missing %s",
                        Biobuzz3Tip_HiveAuto.MAX_WAIT_FOR_ALLIANCE_TIP_S, HiveAutoKit.ALLIANCE_SETTLE_EXTRA_S,
                        yn(hive.settledUp(Side.LEFT, 0.4)), yn(hive.tipActivity(Side.LEFT, HiveAutoKit.HIVE_STILL_S)),
                        yn(hive.downByAbsence(Side.LEFT, HiveAutoKit.ALLIANCE_ABSENCE_S)));
            }

            @Override
            Decision decide(ShotChoice shot) {
                TipResult tip = kit.allianceTipResult();
                if (tip == TipResult.NOT_TIPPED) {
                    return new Decision("partner did not tip - tipping RIGHT ourselves", BiobuzzHiveDecisionPreview.this::solo3);
                }
                return new Decision(tip == TipResult.TIPPED ? "partner tipped - LEFT CELL up"
                        : "no camera frames - shooting LEFT as planned", BiobuzzHiveDecisionPreview.this::nominal3);
            }
        });
        cps3.add(shotCp(1, "NOMINAL: LEFT shot after flower one", leftShotAfterFlower, k, Side.LEFT, fresh, this::afterN1));
        cps3.add(new Checkpoint(2, "NOMINAL: flower-two decision at flower two", flowerTwo, k, null, 0) {
            @Override
            String waitStatus() {
                return "no wait - decided on arrival, after collecting";
            }

            @Override
            Decision decide(ShotChoice shot) {
                Side up = hive.upCell(15.0);
                return new Decision(up == Side.LEFT ? "LEFT CELL still up - flower two goes LEFT"
                        : "upCell(15 s) = " + (up == null ? "none" : up) + " - cross to the RIGHT shot",
                        s -> f2Decision(s, up));
            }
        });
        cps3.add(shotCp(3, "NOMINAL: flower-two shot at LEFT", leftShot3, k, Side.LEFT, fresh, this::park3));
        cps3.add(new Checkpoint(4, "NOMINAL: RIGHT shot, then wall-set decision", rightShot3, k, Side.RIGHT, fresh) {
            @Override
            boolean postVolleyDecision() {
                return true;
            }

            @Override
            Decision decide(ShotChoice shot) {
                Side wallUp = hive.upCell(1.5);
                return new Decision(HiveAutoKit.describeShot(Side.RIGHT, shot) + "; wall set sees "
                        + (wallUp == null ? "no view" : wallUp + " up"), s -> {
                    s.applyShot(Side.RIGHT, shot);
                    wallBlock3(s, wallUp, "camera at decision");
                });
            }
        });
        cps3.add(shotCp(5, "WALL SET: LEFT shot at wallShotLeft", wallShotLeft, k, Side.LEFT, fresh, this::park3));
        cps3.add(shotCp(6, "WALL SET: angled RIGHT shot at nearShot", nearShot, k, Side.RIGHT, 8.0, this::park3));
        cps3.add(shotCp(7, "SOLO: RIGHT shot", rightShotSolo, k, Side.RIGHT, fresh, this::afterS1));
        cps3.add(shotCp(8, "SOLO: wall-set LEFT shot at wallShotLeft", wallShotLeft, k, Side.LEFT, fresh, this::afterS2));
        cps3.add(shotCp(9, "SOLO: flower-one LEFT shot", soloBackEnd, k, Side.LEFT, fresh, this::park3));
    }

    // ---- Biobuzz_2_Tip_HiveAuto -----------------------------------------------------------------

    private void t2Tipped(Sim s) {
        s.drive(wallSetToLeftNear, leftNear);
        s.shot(1, Side.LEFT);
        t2AfterT1(s);
    }

    private void t2AfterT1(Sim s) {
        t2FlowerOne(s, 2);
        t2AfterT2(s);
    }

    private void t2AfterT2(Sim s) {
        if (!s.fits("flower two", F2_FROM_LEFT_S, leftNear)) {
            s.note("skipping flower two");
        } else {
            s.note("flower two - backup shot at the LEFT CELL if it has not tipped");
            s.flowerLeg(leftShotBack2, leftToFlowerTwo, flowerTwo, 3.0);
            s.fromFlowerLeg(flowerTwo, flowerTwoToLeftNear, leftNear);
            s.shot(3, Side.LEFT);
        }
        park2(s);
    }

    private void t2NotTipped(Sim s) {
        s.drive(wallSetToRightNear, rightNear);
        s.shot(4, Side.RIGHT);
        t2AfterN1(s);
    }

    private void t2AfterN1(Sim s) {
        if (!s.fits("flower two", F2_FROM_RIGHT_S, leftNear)) {
            s.note("skipping flower two");
        } else {
            s.flowerLeg(rightNear, rightNearToFlowerTwo, flowerTwo, 3.0);
            s.fromFlowerLeg(flowerTwo, flowerTwoToLeftNear, leftNear);
            s.shot(5, Side.LEFT);
        }
        t2AfterN2(s);
    }

    private void t2AfterN2(Sim s) {
        t2FlowerOne(s, 6);
        park2(s);
    }

    private void t2FlowerOne(Sim s, int cp) {
        if (!s.fits("flower one", F1_BLOCK_S, leftShotBack2)) {
            s.note("skipping flower one");
            return;
        }
        s.flowerLeg(leftNear, leftNearToFlowerOne, flowerOne2, 3.0);
        s.fromFlowerLeg(flowerOne2, backToShoot, leftShotBack2);
        s.shot(cp, Side.LEFT);
    }

    private void park2(Sim s) {
        s.note("parking");
        s.route(park2);
        s.parked();
    }

    // ---- Biobuzz3Tip_HiveAuto -------------------------------------------------------------------

    private void nominal3(Sim s) {
        s.transfer("preload volley into LEFT");
        s.flowerLeg(leftShot3, toFlowerOne, flowerOne3, 3.0);
        s.drive(backFromFlowerOne, leftShotAfterFlower);
        s.shot(1, Side.LEFT);
        afterN1(s);
    }

    private void afterN1(Sim s) {
        if (!s.fits("flower two", FLOWER_TWO_BLOCK_S, rightShot3)) {
            s.note("no time for flower two");
            park3(s);
            return;
        }
        s.flowerLeg(leftShotAfterFlower, toFlowerTwo, flowerTwo, 3.0);
        s.mark(2);
        Side up = s.forecast.live ? hive.upCell(15.0) : s.forecast.f2Up;
        s.note("flower-two decision, upCell(15 s) " + s.source() + ": " + (up == null ? "none" : up));
        f2Decision(s, up);
    }

    private void f2Decision(Sim s, Side up) {
        if (up == Side.LEFT) {
            s.note("LEFT CELL still up - flower two goes LEFT");
            s.routeFromFlower(leftShot3);
            s.shot(3, Side.LEFT);
            park3(s);
            return;
        }
        s.fromFlowerLeg(flowerTwo, crossToRight, rightShot3);
        s.shot(4, Side.RIGHT);
        Side wallUp = s.forecast.live ? hive.upCell(1.5) : s.forecast.wallUp;
        wallBlock3(s, wallUp, s.source());
    }

    private void wallBlock3(Sim s, Side up, String source) {
        if (s.pose.position.x < 0) {
            s.note("not at the RIGHT pose - going to park");
            park3(s);
            return;
        }
        s.note("wall-set decision, upCell(1.5 s) " + source + ": " + (up == null ? "none" : up));
        if (up == Side.LEFT && s.fits("wall set to LEFT", WALL_TO_LEFT_S, leftShot3)) {
            s.note("LEFT CELL up - wall set to LEFT");
            s.drive(wallToLeft, wallShotLeft);
            s.shot(5, Side.LEFT);
            park3(s);
            return;
        }
        if (up != Side.LEFT && s.fits("angled wall-set shot", WALL_NEAR_S, nearShot)) {
            s.note(up == Side.RIGHT ? "RIGHT CELL up - wall set, angled shot at RIGHT" : "no view - wall set, angled shot at RIGHT");
            s.drive(wallToNear, nearShot);
            s.shot(6, Side.RIGHT);
            park3(s);
            return;
        }
        if (s.fits("wall set only", WALL_ONLY_S, wallSet)) {
            s.note("no time to shoot the wall set - collecting it for TELEOP");
            s.drive(wallOnly, wallSet);
            park3(s);
            return;
        }
        s.note("no time for the wall set");
        park3(s);
    }

    private void solo3(Sim s) {
        s.route(rightShotSolo);
        s.shot(7, Side.RIGHT);
        afterS1(s);
    }

    private void afterS1(Sim s) {
        s.leg(rightShotSolo, soloWallSet, wallShotLeft, 0);
        s.shot(8, Side.LEFT);
        afterS2(s);
    }

    private void afterS2(Sim s) {
        if (!s.fits("flower one", SOLO_FLOWER_BLOCK_S, leftShot3)) {
            s.note("no time for flower one");
            park3(s);
            return;
        }
        s.flowerLeg(wallShotLeft, soloFlower, flowerOneSolo, 3.0);
        s.drive(soloBack, soloBackEnd);
        s.shot(9, Side.LEFT);
        park3(s);
    }

    private void park3(Sim s) {
        s.note("parking");
        s.leg(wallShotLeft, parkFromLeft, park3, 0);
        s.parked();
    }

    /** Arrival time at every checkpoint along the plans with no waits (the dpad's starting point). */
    private void computeNominalTimes() {
        Map<Integer, Double> m2 = new HashMap<>();
        m2.put(0, 2.0);
        t2Tipped(new Sim(kit2, 2.0, begin2, new Forecast(false, null, null), m2));
        t2NotTipped(new Sim(kit2, 2.0 + HiveAutoKit.OWN_TIP_GRACE_S, begin2, new Forecast(false, null, null), m2));
        for (Checkpoint c : cps2) c.nominalT = m2.containsKey(c.id) ? m2.get(c.id) : Double.NaN;

        Map<Integer, Double> m3 = new HashMap<>();
        double arrive = toLeftShot.seconds;
        m3.put(0, arrive);
        double decided = arrive + Biobuzz3Tip_HiveAuto.MAX_WAIT_FOR_ALLIANCE_TIP_S;
        nominal3(new Sim(kit3, decided, leftShot3, new Forecast(false, Side.LEFT, null), m3));
        nominal3(new Sim(kit3, decided, leftShot3, new Forecast(false, Side.RIGHT, Side.LEFT), m3));
        nominal3(new Sim(kit3, decided, leftShot3, new Forecast(false, Side.RIGHT, Side.RIGHT), m3));
        solo3(new Sim(kit3, decided, leftShot3, new Forecast(false, null, null), m3));
        for (Checkpoint c : cps3) c.nominalT = m3.containsKey(c.id) ? m3.get(c.id) : Double.NaN;
    }

    private List<String> simulate(Checkpoint cp, Decision d, double t) {
        Sim s = new Sim(cp.kit, t, cp.pose, liveForecast, null);
        s.note("decided: " + d.text);
        d.next.accept(s);
        return s.lines;
    }

    // =============================================================================================
    // Paths, copied from the autos (intake actions replaced by waits of the same length)
    // =============================================================================================

    private void buildConstraints() {
        double TRACK_WIDTH = MecanumDrive.PARAMS.inPerTick * MecanumDrive.PARAMS.trackWidthTicks;
        double LATERAL_MULT = MecanumDrive.PARAMS.inPerTick / MecanumDrive.PARAMS.lateralInPerTick;
        double MAX_WHEEL_VEL = 64;
        double MAX_PATH_ACCEL = 100;
        double MAX_WHEEL_ACCEL = 120;
        double MAX_LATERAL_ACCEL = 150;
        MecanumKinematics robotKinematics = new MecanumKinematics(TRACK_WIDTH, LATERAL_MULT);

        VelConstraint cornerVel = (robotPose, posePath, disp) -> {
            double curvature = posePath.get(disp, 3).position.drop(2).value().norm();
            return curvature < 1e-6 ? 1e9 : Math.sqrt(MAX_LATERAL_ACCEL / curvature);
        };
        VelConstraint turnRateVel = (robotPose, posePath, disp) -> {
            Rotation2dDual<Arclength> heading = posePath.get(disp, 3).heading;
            double headingCurvature = Math.abs(heading.real.get(0) * heading.imag.get(2) - heading.imag.get(0) * heading.real.get(2));
            return headingCurvature < 1e-9 ? 1e9 : Math.sqrt(MAX_WHEEL_ACCEL / (headingCurvature * TRACK_WIDTH));
        };
        maxVel = new MinVelConstraint(Arrays.asList(
                robotKinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                cornerVel,
                turnRateVel
        ));
        double INTAKE_ZONE_Y = -42;
        double INTAKE_VEL = 15;
        VelConstraint cornerIntakeVel = (robotPose, _path, _disp) -> {
            boolean inIntakeZone = robotPose.position.x.value() > 50.0 && robotPose.position.y.value() < INTAKE_ZONE_Y;
            return inIntakeZone ? INTAKE_VEL : 60;
        };
        maxIntakeVel = new MinVelConstraint(Arrays.asList(cornerIntakeVel, maxVel));

        maxAccel = (robotPose, posePath, disp) -> {
            Vector2d travel = robotPose.heading.value().inverse().times(robotPose.position.drop(1).value());
            double turnPerInch = Math.abs(robotPose.heading.velocity().value());
            double wheelPerInch = Math.abs(travel.x) + LATERAL_MULT * Math.abs(travel.y) + turnPerInch * TRACK_WIDTH;
            double accel = Math.min(MAX_PATH_ACCEL, MAX_WHEEL_ACCEL / wheelPerInch);
            return new MinMax(-accel, accel);
        };
    }

    private void buildLegs() {
        double dwell2 = Biobuzz_2_Tip_HiveAuto.WALL_SET_DWELL_S;
        double dwell3 = Biobuzz3Tip_HiveAuto.WALL_SET_DWELL_S;

        wallSetToLeftNear = new Leg("wallSetToLeftNear", wallCollect(drive.actionBuilder(begin2)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(dwell2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToSplineHeading(leftNear, Math.toRadians(180), maxVel, maxAccel)
                .build()),
                "start -> wall set (62,-58,0) slow, dwell -> (20,-48,0) -> (-33,-44,0) -> leftNear (-53.5,-43.8,30)");

        wallSetToRightNear = new Leg("wallSetToRightNear", wallCollect(new SequentialAction(
                drive.actionBuilder(begin2)
                        .setTangent(Math.toRadians(-155))
                        .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                        .waitSeconds(dwell2)
                        .build(),
                drive.actionBuilder(wallSet)
                        .setTangent(Math.atan2(10, -4))
                        .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                        .splineToSplineHeading(rightNear, Math.atan2(rightNear.position.y + 48, rightNear.position.x - 58), maxVel, maxAccel)
                        .build())),
                "start -> wall set (62,-58,0) slow, dwell -> (58,-48,30) -> rightNear (56.7,-40.0,155)");

        Action r = kit2.flowerRoute(leftNear, flowerOne2);
        leftNearToFlowerOne = new Leg("leftNearToFlowerOne", new SequentialAction(
                new ParallelAction(noop(), r), new SleepAction(0.4), collect(3)),
                "ramp down + route [" + kit2.lastPattern + "] leftNear -> staging -> straight in to flowerOne (-55,-22,180), 0.4 s, collect 3 s");

        backToShoot = new Leg("backToShoot", kit2.departFlower(drive.actionBuilder(flowerOne2), flowerOne2)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(155), maxVel, maxAccel)
                .build(),
                "flowerOne, ramp down -> straight back to " + fmtV(HiveAutoKit.flowerDepart(flowerOne2)) + ", ramp up -> (-52,-14,1)");

        leftToFlowerTwo = new Leg("leftToFlowerTwo", kit2.approachFlower(drive.actionBuilder(leftShotBack2)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(0, -41, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, noop()), flowerTwo)
                .afterTime(0, collect(3))
                .build(),
                "(-52,-14,360) -> (0,-41,-15), ramp down -> staging " + fmt(HiveAutoKit.flowerStaging(flowerTwo))
                        + " -> straight in to flowerTwo (24,-55,-90), collect 3 s");

        r = kit2.flowerRoute(rightNear, flowerTwo);
        rightNearToFlowerTwo = new Leg("rightNearToFlowerTwo", new SequentialAction(
                new ParallelAction(noop(), r), collect(3)),
                "ramp down + route [" + kit2.lastPattern + "] rightNear -> staging -> straight in to flowerTwo (24,-55,-90), collect 3 s");

        r = kit2.routeFromFlower(flowerTwo, leftNear);
        flowerTwoToLeftNear = new Leg("flowerTwoToLeftNear", r,
                "flowerTwo, ramp down -> straight back to " + fmtV(HiveAutoKit.flowerDepart(flowerTwo)) + ", ramp up + route ["
                        + kit2.lastPattern + "] -> leftNear (-53.5,-43.8,30)");

        toLeftShot = new Leg("toLeftShot", drive.actionBuilder(begin3)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel)
                .build(),
                "start (-65,-11,0) -> leftShot (-52,-14,0)");

        toFlowerOne = new Leg("toFlowerOne", kit3.approachFlower(drive.actionBuilder(leftShot3)
                        .afterTime(0, noop())
                        .setTangent(Math.toRadians(-32)), flowerOne3)
                .afterTime(0, collect(3))
                .build(),
                "leftShot, ramp down -> staging " + fmt(HiveAutoKit.flowerStaging(flowerOne3))
                        + " -> straight in to flowerOne (-55,-22,-179.5), collect 3 s");

        backFromFlowerOne = new Leg("backFromFlowerOne", kit3.departFlower(drive.actionBuilder(flowerOne3), flowerOne3)
                .splineToSplineHeading(leftShotAfterFlower, Math.toRadians(155), maxVel, maxAccel)
                .build(),
                "flowerOne, ramp down -> straight back to " + fmtV(HiveAutoKit.flowerDepart(flowerOne3)) + ", ramp up -> (-52,-14,1)");

        toFlowerTwo = new Leg("toFlowerTwo", kit3.approachFlower(drive.actionBuilder(leftShotAfterFlower)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(0, -41, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel)
                        .afterTime(0, noop()), flowerTwo)
                .afterTime(0, collect(3))
                .build(),
                "(-52,-14,1) -> (0,-41,-15), ramp down -> staging " + fmt(HiveAutoKit.flowerStaging(flowerTwo))
                        + " -> straight in to flowerTwo (24,-55,-90), collect 3 s");

        crossToRight = new Leg("crossToRight", drive.actionBuilder(flowerTwo)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(rightShot3, Math.toRadians(15), maxVel, maxAccel)
                .build(),
                "flowerTwo, ramp down -> straight back, ramp up 8 in out -> rightShot (61,-14,-179.5)");

        wallToLeft = new Leg("wallToLeft", wallCollect(drive.actionBuilder(rightShot3)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(dwell3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-24, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(120), maxVel, maxAccel)
                .build()),
                "rightShot -> wall set (62,-58,0) slow, dwell -> (20,-37,0) -> (-24,-37,0) -> wallShotLeft (-52,-16,0)");

        wallToNear = new Leg("wallToNear", wallCollect(new SequentialAction(
                drive.actionBuilder(rightShot3)
                        .setTangent(Math.toRadians(-155))
                        .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                        .waitSeconds(dwell3)
                        .build(),
                drive.actionBuilder(wallSet)
                        .setTangent(Math.atan2(10, -4))
                        .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                        .splineToSplineHeading(nearShot, Math.atan2(nearShot.position.y + 48, nearShot.position.x - 58), maxVel, maxAccel)
                        .build())),
                "rightShot -> wall set (62,-58,0) slow, dwell -> (58,-48,30) -> nearShot (56.7,-40.0,155)");

        wallOnly = new Leg("wallOnly", drive.actionBuilder(rightShot3)
                .setTangent(Math.toRadians(-155))
                .afterTime(2.1, collect(2.5))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .build(),
                "rightShot -> wall set (62,-58,0) slow, collect 2.5 s from 2.1 s (kept for TELEOP)");

        r = kit3.route(wallShotLeft, park3);
        parkFromLeft = new Leg("parkFromLeft", r, "route [" + kit3.lastPattern + "] wallShotLeft -> park (-21,-59.5,180)");

        soloWallSet = new Leg("soloWallSet", wallCollect(drive.actionBuilder(rightShotSolo)
                .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(wallSet, Math.toRadians(25), maxIntakeVel, maxAccel)
                .waitSeconds(dwell3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToLinearHeading(wallShotLeft, Math.toRadians(-220), maxVel, maxAccel)
                .build()),
                "(61,-14,180) -> wall set (62,-58,0) slow, dwell -> (20,-48,0) -> (-33,-44,0) -> wallShotLeft (-52,-16,0)");

        soloFlower = new Leg("soloFlower", kit3.approachFlower(drive.actionBuilder(wallShotLeft)
                        .afterTime(0, noop())
                        .setTangent(Math.toRadians(-27)), flowerOneSolo)
                .afterTime(0.4, collect(3))
                .build(),
                "wallShotLeft, ramp down -> staging " + fmt(HiveAutoKit.flowerStaging(flowerOneSolo))
                        + " -> straight in to flowerOne (-55,-22,180), collect 3 s from 0.4 s");

        soloBack = new Leg("soloBack", kit3.departFlower(drive.actionBuilder(flowerOneSolo), flowerOneSolo)
                .splineToSplineHeading(soloBackEnd, Math.toRadians(155), maxVel, maxAccel)
                .build(),
                "flowerOne, ramp down -> straight back to " + fmtV(HiveAutoKit.flowerDepart(flowerOneSolo)) + ", ramp up -> (-52,-14,1)");
    }

    /** HiveAutoKit.withWallCollect's timing, with the rollers replaced by a wait. */
    private static Action wallCollect(Action d) {
        double run = Math.max(HiveAutoKit.WALL_COLLECT_MIN_S, HiveAutoKit.motionTime(d) - HiveAutoKit.WALL_COLLECT_START_S);
        return new ParallelAction(d, new SequentialAction(new SleepAction(HiveAutoKit.WALL_COLLECT_START_S), new SleepAction(run)));
    }

    private static Action collect(double s) {
        return new SleepAction(s);
    }

    private static Action noop() {
        return new InstantAction(() -> {
        });
    }

    /** kit.route from {@code from} to {@code to}, built once and remembered. */
    private Leg routeLeg(HiveAutoKit kit, Pose2d from, Pose2d to) {
        String key = (kit == kit2 ? "2 " : "3 ") + fmt(from) + fmt(to);
        Leg l = routeCache.get(key);
        if (l == null) {
            Action a = kit.route(from, to);
            StringBuilder via = new StringBuilder("route [" + kit.lastPattern + "] " + fmt(from));
            for (Vector2d c : HiveAutoKit.plan(from.position, to.position).points) {
                via.append(String.format(" -> (%.0f,%.0f)", c.x, c.y));
            }
            via.append(" -> ").append(fmt(to));
            l = new Leg("route", a, via.toString());
            routeCache.put(key, l);
        }
        return l;
    }

    /** kit.routeFromFlower from {@code from} (at a FLOWER) to {@code to}, built once and remembered. */
    private Leg fromFlowerRouteLeg(HiveAutoKit kit, Pose2d from, Pose2d to) {
        String key = (kit == kit2 ? "2D " : "3D ") + fmt(from) + fmt(to);
        Leg l = routeCache.get(key);
        if (l == null) {
            Action a = kit.routeFromFlower(from, to);
            l = new Leg("route from FLOWER", a, "straight back to " + fmtV(HiveAutoKit.flowerDepart(from))
                    + " + route [" + kit.lastPattern + "] -> " + fmt(to));
            routeCache.put(key, l);
        }
        return l;
    }

    /** kit.flowerRoute from {@code from} to the FLOWER intake pose {@code flower}, built once and remembered. */
    private Leg flowerRouteLeg(HiveAutoKit kit, Pose2d from, Pose2d flower) {
        String key = (kit == kit2 ? "2F " : "3F ") + fmt(from) + fmt(flower);
        Leg l = routeCache.get(key);
        if (l == null) {
            Action a = kit.flowerRoute(from, flower);
            l = new Leg("FLOWER route", a, "ramp down + route [" + kit.lastPattern + "] " + fmt(from) + " -> staging "
                    + fmt(HiveAutoKit.flowerStaging(flower)) + " -> straight in to " + fmt(flower));
            routeCache.put(key, l);
        }
        return l;
    }

    // =============================================================================================
    // Helpers
    // =============================================================================================

    /** Not reached on any nominal timeline (only after skips or crossings): start from 15 s. */
    private static double startTime(Checkpoint c) {
        return Double.isNaN(c.nominalT) ? 15.0 : c.nominalT;
    }

    private static boolean shoots(ShotChoice c) {
        return c == ShotChoice.SHOOT_UP || c == ShotChoice.SHOOT_NO_VIEW;
    }

    private String flags(Side s) {
        return String.format("%s: settled up %s | upright %s | TIPPING %s | moving %s | upside down %s | missing>=%.1fs %s | tip activity %s",
                s, yn(hive.settledUp(s, 0.4)), yn(hive.seenUpright(s, 0.6)), yn(hive.tipping(s, HiveWatcher.TIP_MAX_S)),
                yn(hive.moving(s, HiveAutoKit.HIVE_STILL_S)), yn(hive.confirmedDown(s, 0.6)), HiveAutoKit.ALLIANCE_ABSENCE_S,
                yn(hive.downByAbsence(s, HiveAutoKit.ALLIANCE_ABSENCE_S)), yn(hive.tipActivity(s, HiveAutoKit.HIVE_STILL_S)));
    }

    private String up(double maxAgeS) {
        Side s = hive.upCell(maxAgeS);
        return s == null ? "-" : s.toString();
    }

    private static String yn(boolean b) {
        return b ? "Y" : "-";
    }

    private static boolean near(Pose2d a, Pose2d b) {
        double d = Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y);
        double h = Math.toDegrees(a.heading.toDouble() - b.heading.toDouble());
        while (h > 180) h -= 360;
        while (h <= -180) h += 360;
        return d < HiveAutoKit.NEAR_IN && Math.abs(h) < HiveAutoKit.NEAR_DEG;
    }

    private static String fmtV(Vector2d v) {
        return String.format("(%.0f,%.0f)", v.x, v.y);
    }

    private static String fmt(Pose2d p) {
        return String.format("(%.1f, %.1f, %.0f deg)", p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
    }
}
