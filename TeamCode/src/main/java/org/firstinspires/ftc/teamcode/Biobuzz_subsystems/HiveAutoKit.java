package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.HiveWatcher.Side;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Building blocks for the HIVE-aware BIOBUZZ autos: runtime decisions, a match clock with a time
 * budget, HIVE-avoiding routes built on the fly, the "shoot at whichever CELL is really up" step,
 * and the park watchdog.
 *
 * <h2>The rule every decision follows</h2>
 * The original plan is the default. The autos only deviate from it on POSITIVE, SETTLED camera
 * evidence that the planned CELL is not the one that is up. With no camera, no frames, or a robot in
 * the way, every decision falls through to exactly what the original auto did - so vision can make
 * the auto better but never worse.
 *
 * <h2>Prebuilt vs runtime paths</h2>
 * The planned legs are built in INIT, exactly as in the original autos, so they cost nothing during
 * the match. A leg only gets rebuilt at runtime ({@link #leg}) when an earlier decision has left the
 * robot somewhere else than that leg was built to start from. Runtime routes ({@link #route}) go
 * around the HIVE through a corridor at y = {@link #CORRIDOR_Y} when the straight line would hit it.
 *
 * <h2>PARK is guaranteed by {@link #guardPark}</h2>
 * Every loop it compares the time left with a conservative estimate of the drive from wherever the
 * robot is to the park pose. The moment they meet (plus {@link #PARK_MARGIN_S}), it abandons whatever
 * the auto is doing, stops every mechanism and drives to park.
 */
@Config
public class HiveAutoKit {

    /** Length of AUTO, SECONDS. */
    public static double AUTO_LENGTH_S = 30.0;
    /** Kept in hand at every time-budget check and by the park watchdog, SECONDS. */
    public static double PARK_MARGIN_S = 0.6;
    /** Average speed assumed for runtime routes and park estimates, IN/S (deliberately low). */
    public static double ROUTE_SPEED_IN_S = 38.0;
    /** Fixed cost added to every route estimate for accelerating and settling, SECONDS. */
    public static double ROUTE_OVERHEAD_S = 0.8;
    /** Extra estimate per radian of heading change, SECONDS. */
    public static double ROUTE_TURN_S_PER_RAD = 0.3;
    /** Extra estimate per corridor corner, SECONDS (corners force the follower to slow). */
    public static double ROUTE_WAYPOINT_S = 0.25;
    /** How recent HIVE evidence must be for a shot decision, SECONDS. */
    public static double GATE_FRESH_S = 0.6;
    /**
     * Longest a shot waits for evidence when it has none, SECONDS. Must exceed
     * HiveWatcher.ABSENCE_DOWN_S, because a DOWN near CELL is only recognised by staying unseen that
     * long. An UP near CELL is normally seen on the approach, so this costs nothing then; with no
     * camera frames at all the wait is skipped.
     */
    public static double GATE_MAX_WAIT_S = 0.7;
    /** Near LEFT CELL unseen this long during the alliance wait means the alliance did not tip, SECONDS. */
    public static double ALLIANCE_ABSENCE_S = 1.0;
    /** If the LEFT CELL is seen mid-swing when the wait runs out, wait this much longer for it to settle. */
    public static double ALLIANCE_SETTLE_EXTRA_S = 0.6;
    /** A leg's prebuilt path is used only if the robot is this close to where it starts, INCHES / DEG. */
    public static double NEAR_IN = 8.0;
    public static double NEAR_DEG = 25.0;
    /** Corridor used to drive around the HIVE, field inches (red side). */
    public static double CORRIDOR_Y = -44.0;
    public static double CORRIDOR_X = 30.0;
    /** HIVE keep-out for the robot CENTRE: frame half-sizes plus half a robot plus margin. */
    public static double KEEPOUT_X = 31.0;
    public static double KEEPOUT_Y = 36.0;
    /** One volley. Same as the original autos. */
    public static double TRANSFER_S = 1.0;
    public static double TRANSFER_POWER = 0.8;

    public final MecanumDrive drive;
    public final Intake2_0 intake;
    public final HiveWatcher hive;
    private final Telemetry telemetry;
    private final VelConstraint vel;
    private final AccelConstraint accel;
    private final Pose2d parkPose, leftShot, rightShot;

    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final List<String> log = new ArrayList<>();
    private boolean parking = false;

    public HiveAutoKit(MecanumDrive drive, Intake2_0 intake, HiveWatcher hive, Telemetry telemetry,
                       VelConstraint vel, AccelConstraint accel,
                       Pose2d parkPose, Pose2d leftShot, Pose2d rightShot) {
        this.drive = drive;
        this.intake = intake;
        this.hive = hive;
        this.telemetry = telemetry;
        this.vel = vel;
        this.accel = accel;
        this.parkPose = parkPose;
        this.leftShot = leftShot;
        this.rightShot = rightShot;
    }

    /** Call right after START. */
    public void start() {
        clock.reset();
        log("START");
    }

    public double t() {
        return clock.seconds();
    }

    public double remaining() {
        return AUTO_LENGTH_S - t();
    }

    /** True if {@code seconds} of work still leaves time to park from {@code endPose}. */
    public boolean fits(double seconds, Pose2d endPose) {
        return remaining() >= seconds + estimateRoute(endPose, parkPose) + PARK_MARGIN_S;
    }

    public void log(String msg) {
        log.add(String.format("%5.1fs %s", t(), msg));
    }

    public Side robotSide() {
        return drive.pose.position.x >= 0 ? Side.RIGHT : Side.LEFT;
    }

    public Pose2d shotPose(Side side) {
        return side == Side.LEFT ? leftShot : rightShot;
    }

    // =============================================================================================
    // Generic actions
    // =============================================================================================

    /** Picks its action the first time it runs, from the state of the match at that moment. */
    public static Action branch(Supplier<Action> chooser) {
        return new Action() {
            private Action chosen;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (chosen == null) chosen = chooser.get();
                return chosen.run(p);
            }
        };
    }

    /** Ends when the condition holds or after maxS seconds, whichever first. */
    public static Action waitUntil(BooleanSupplier condition, double maxS) {
        return new Action() {
            private long startNanos = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (startNanos < 0) startNanos = System.nanoTime();
                return !(condition.getAsBoolean() || (System.nanoTime() - startNanos) / 1e9 >= maxS);
            }
        };
    }

    /** Ends when the condition holds or the MATCH clock reaches {@code deadlineS}. */
    public Action waitUntilMatchTime(BooleanSupplier condition, double deadlineS) {
        return p -> !(condition.getAsBoolean() || t() >= deadlineS);
    }

    public static Action instant(Runnable r) {
        return p -> {
            r.run();
            return false;
        };
    }

    public Action note(String msg) {
        return instant(() -> log(msg));
    }

    /** Marks the start of the planned park so the watchdog does not interrupt it. */
    public Action beginPark() {
        return instant(() -> {
            parking = true;
            log("parking");
        });
    }

    public Action transfer() {
        return intake.transferUsingRollersForTime(TRANSFER_S, TRANSFER_POWER);
    }

    /**
     * A planned leg. Uses {@code prebuilt} if the robot is where it was built to start; otherwise
     * builds a route from the robot's actual pose to {@code target} and then runs {@code thenIfRebuilt}
     * (whatever the prebuilt leg did on arrival, e.g. collecting from a FLOWER).
     */
    public Action leg(Pose2d expectedStart, Action prebuilt, Pose2d target, Action thenIfRebuilt) {
        return branch(() -> {
            if (near(drive.pose, expectedStart)) return prebuilt;
            log("rebuilding leg to " + fmt(target));
            return thenIfRebuilt == null ? route(drive.pose, target)
                    : new SequentialAction(route(drive.pose, target), thenIfRebuilt);
        });
    }

    // =============================================================================================
    // Shooting at whichever CELL is really up
    // =============================================================================================

    /**
     * The robot has arrived at the shooting pose for {@code side}. Waits at most
     * {@link #GATE_MAX_WAIT_S} for evidence it does not already have, then:
     * <ul>
     *   <li>planned CELL up, or no evidence: shoot (the original behaviour);</li>
     *   <li>the OTHER CELL settled up: if the budget allows, drive around the HIVE to the other
     *       shooting pose and shoot there; if not, hold the artifacts rather than launch into the
     *       bottom of a CELL that is down (wasted, and a possible G417 foul).</li>
     * </ul>
     */
    public Action shotPoint(Side side) {
        return new SequentialAction(
                waitUntil(this::haveEvidence, GATE_MAX_WAIT_S),
                branch(() -> decideShot(side, true)));
    }

    private boolean haveEvidence() {
        return hive.upCell(GATE_FRESH_S) != null || !hive.cameraHealthy();
    }

    public enum TipResult { TIPPED, NOT_TIPPED, UNKNOWN }

    /**
     * Waits at the LEFT shooting pose for the alliance's tip: ends as soon as the LEFT CELL is up and
     * settled, or after maxWaitS - plus up to {@link #ALLIANCE_SETTLE_EXTRA_S} if the LEFT CELL is
     * visibly swinging up at that moment.
     */
    public Action waitForAllianceTip(double maxWaitS) {
        return new SequentialAction(
                waitUntil(() -> hive.settledUp(Side.LEFT, 0.4), maxWaitS),
                waitUntil(() -> hive.settledUp(Side.LEFT, 0.4) || !hive.seenUpright(Side.LEFT, 0.3),
                        ALLIANCE_SETTLE_EXTRA_S));
    }

    /**
     * After {@link #waitForAllianceTip}: TIPPED if the LEFT CELL is seen up; NOT_TIPPED if the RIGHT
     * CELL is seen up or the near LEFT CELL stayed unseen for {@link #ALLIANCE_ABSENCE_S} while the
     * camera was working and the robot sat facing it; UNKNOWN otherwise (e.g. no camera frames).
     */
    public TipResult allianceTipResult() {
        if (hive.settledUp(Side.LEFT, 0.6)) return TipResult.TIPPED;
        if (hive.settledUp(Side.RIGHT, 0.6) || hive.downByAbsence(Side.LEFT, ALLIANCE_ABSENCE_S)) {
            return TipResult.NOT_TIPPED;
        }
        return TipResult.UNKNOWN;
    }

    private Action decideShot(Side side, boolean mayCross) {
        Side up = hive.upCell(GATE_FRESH_S);
        if (up == side.other()) {
            if (!mayCross) {
                log(side + " CELL down again after crossing - holding");
                return new NullAction();
            }
            Pose2d dest = shotPose(side.other());
            double need = estimateRoute(drive.pose, dest) + GATE_MAX_WAIT_S + TRANSFER_S;
            if (fits(need, dest)) {
                log(side + " CELL is DOWN - crossing to shoot " + side.other());
                return new SequentialAction(
                        route(drive.pose, dest),
                        waitUntil(this::haveEvidence, GATE_MAX_WAIT_S),
                        branch(() -> decideShot(side.other(), false)));
            }
            log(side + " CELL is DOWN, no time to cross - holding");
            return new NullAction();
        }
        log(up == side ? side + " CELL up - shooting" : "no HIVE view - shooting " + side + " as planned");
        return transfer();
    }

    // =============================================================================================
    // Park watchdog
    // =============================================================================================

    /**
     * Runs {@code main}, but abandons it the moment the time left only just covers driving to park
     * from where the robot is. Then stops every mechanism and parks from the robot's actual pose.
     * The planned park (after {@link #beginPark}) is never interrupted.
     */
    public Action guardPark(Action main) {
        return new Action() {
            private Action park;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (park == null && !parking
                        && remaining() <= estimateRoute(drive.pose, parkPose) + PARK_MARGIN_S) {
                    log("WATCHDOG: out of time - parking now");
                    intake.stopAllFeeding();
                    parking = true;
                    park = route(drive.pose, parkPose);
                }
                if (park != null) return park.run(p);
                return main.run(p);
            }
        };
    }

    /** HIVE watcher update and telemetry; runs alongside everything else for the whole auto. */
    public Action background() {
        return p -> {
            hive.update(drive.pose);
            if (telemetryTimer.milliseconds() > 200) {
                telemetryTimer.reset();
                telemetry.addData("t", "%.1f s   remaining %.1f s", t(), remaining());
                telemetry.addData("HIVE", hive.describe());
                for (int i = Math.max(0, log.size() - 8); i < log.size(); i++) telemetry.addLine(log.get(i));
                telemetry.update();
            }
            return true;
        };
    }

    // =============================================================================================
    // Routes
    // =============================================================================================

    /** A path from {@code from} to {@code to}, through the corridor if the straight line hits the HIVE. */
    public Action route(Pose2d from, Pose2d to) {
        Vector2d[] wps = waypoints(from, to);
        try {
            TrajectoryActionBuilder b = drive.actionBuilder(from);
            if (wps.length == 0) {
                b = b.strafeToLinearHeading(to.position, to.heading.toDouble(), vel, accel);
            } else {
                b = b.setTangent(angle(from.position, wps[0]));
                for (int i = 0; i < wps.length; i++) {
                    Vector2d next = i + 1 < wps.length ? wps[i + 1] : to.position;
                    double h = i == wps.length - 1 ? to.heading.toDouble() : from.heading.toDouble();
                    b = b.splineToLinearHeading(new Pose2d(wps[i], h), angle(wps[i], next), vel, accel);
                }
                b = b.splineToLinearHeading(to, angle(wps[wps.length - 1], to.position), vel, accel);
            }
            return b.build();
        } catch (Exception splineFailed) {
            // Never let a path-builder exception end the auto: straight strafes always build.
            TrajectoryActionBuilder b = drive.actionBuilder(from);
            for (Vector2d w : wps) b = b.strafeToLinearHeading(w, to.heading.toDouble(), vel, accel);
            return b.strafeToLinearHeading(to.position, to.heading.toDouble(), vel, accel).build();
        }
    }

    /** Conservative drive-time estimate for {@link #route}, SECONDS. */
    public double estimateRoute(Pose2d from, Pose2d to) {
        Vector2d[] wps = waypoints(from, to);
        double len = pathLength(from.position, wps, to.position);
        double turn = Math.abs(wrap(to.heading.toDouble() - from.heading.toDouble()));
        return len / ROUTE_SPEED_IN_S + ROUTE_OVERHEAD_S + turn * ROUTE_TURN_S_PER_RAD
                + wps.length * ROUTE_WAYPOINT_S;
    }

    /**
     * Corridor waypoints for a route: none if the straight line is clear, otherwise the shortest of
     * [near corner], [far corner] or [near corner, far corner] whose every segment misses the HIVE.
     */
    private Vector2d[] waypoints(Pose2d from, Pose2d to) {
        Vector2d a = from.position, b = to.position;
        if (!hitsHive(a, b)) return new Vector2d[0];
        double sA = Math.signum(a.x), sB = Math.signum(b.x);
        if (sA == 0) sA = -sB;
        if (sB == 0) sB = -sA;
        Vector2d w1 = new Vector2d(sA * CORRIDOR_X, CORRIDOR_Y), w2 = new Vector2d(sB * CORRIDOR_X, CORRIDOR_Y);
        Vector2d[][] options = {{w1}, {w2}, {w1, w2}};
        Vector2d[] best = options[2];
        double bestLen = Double.MAX_VALUE;
        for (Vector2d[] o : options) {
            boolean clear = !hitsHive(a, o[0]) && !hitsHive(o[o.length - 1], b);
            for (int i = 0; i + 1 < o.length; i++) clear &= !hitsHive(o[i], o[i + 1]);
            double len = pathLength(a, o, b);
            if (clear && len < bestLen) {
                best = o;
                bestLen = len;
            }
        }
        return best;
    }

    private static double pathLength(Vector2d a, Vector2d[] wps, Vector2d b) {
        double len = 0;
        Vector2d prev = a;
        for (Vector2d w : wps) {
            len += dist(prev, w);
            prev = w;
        }
        return len + dist(prev, b);
    }

    /** Does the straight segment pass through the HIVE keep-out box? (Liang-Barsky clip.) */
    private static boolean hitsHive(Vector2d a, Vector2d b) {
        double t0 = 0, t1 = 1, dx = b.x - a.x, dy = b.y - a.y;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {a.x + KEEPOUT_X, KEEPOUT_X - a.x, a.y + KEEPOUT_Y, KEEPOUT_Y - a.y};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            } else {
                double r = q[i] / p[i];
                if (p[i] < 0) t0 = Math.max(t0, r);
                else t1 = Math.min(t1, r);
            }
        }
        return t0 < t1;
    }

    private boolean near(Pose2d a, Pose2d b) {
        return dist(a.position, b.position) < NEAR_IN
                && Math.abs(Math.toDegrees(wrap(a.heading.toDouble() - b.heading.toDouble()))) < NEAR_DEG;
    }

    private static double dist(Vector2d a, Vector2d b) {
        return Math.hypot(b.x - a.x, b.y - a.y);
    }

    private static double angle(Vector2d a, Vector2d b) {
        return Math.atan2(b.y - a.y, b.x - a.x);
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static String fmt(Pose2d p) {
        return String.format("(%.0f, %.0f, %.0f deg)", p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
    }
}
