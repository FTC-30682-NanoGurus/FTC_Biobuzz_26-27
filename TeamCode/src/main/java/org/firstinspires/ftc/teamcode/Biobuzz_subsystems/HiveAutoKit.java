package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
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
 * robot somewhere else than that leg was built to start from. Runtime routes ({@link #route}) keep
 * the whole robot clear of the HIVE (see the Routes section), going round below it when needed.
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
    /** Extra estimate per corner of a runtime route, SECONDS (the strafes stop at each corner). */
    public static double ROUTE_WAYPOINT_S = 0.55;
    /** Added to a route's exact planned time for the follower settling at the end, SECONDS. */
    public static double ROUTE_SETTLE_S = 0.2;
    /** How often the watchdog re-measures the exact park time once it is close, SECONDS. */
    public static double WATCHDOG_REFINE_S = 0.25;
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
    /**
     * If a tip is visibly under way when the alliance wait runs out (the LEFT CELL seen moving, or
     * upright but not yet at rest), wait up to this much longer for it to settle, SECONDS. A TIP
     * takes 2.5-4.8 s, so a tip seen late needs most of that.
     */
    public static double ALLIANCE_SETTLE_EXTRA_S = 3.0;
    /** Longest a shot waits for a HIVE that is part-way through a tip to come to rest, SECONDS. */
    public static double TIP_WAIT_MAX_S = 3.5;
    /** No movement seen for this long counts as the HIVE being still, SECONDS. */
    public static double HIVE_STILL_S = 0.5;
    /**
     * After our own volley, how much longer to watch the CELL for the start of its tip before
     * deciding it did not tip, SECONDS. A tip starts slowly, so this is spent only when the CELL
     * is still sitting at rest.
     */
    public static double OWN_TIP_GRACE_S = 1.0;
    /** A leg's prebuilt path is used only if the robot is this close to where it starts, INCHES / DEG. */
    public static double NEAR_IN = 8.0;
    public static double NEAR_DEG = 25.0;
    /**
     * HIVE frame half-sizes, field INCHES (frame centred on the origin). The robot's own size comes
     * from BiobuzzRobotConstants.robotWidthIn / robotLengthIn.
     */
    public static double HIVE_HALF_X = 19.5;
    public static double HIVE_HALF_Y = 24.7;
    /** Allowance for path-following error on top of the robot's size, INCHES. */
    public static double TRACKING_MARGIN_IN = 3.0;
    /**
     * The two corridor corners below the HIVE sit this far outside the clearance zone, INCHES. Small,
     * because every candidate route is checked against the real path (RouteOptimizer).
     */
    public static double WAYPOINT_SLACK_IN = 1.0;
    /** Spacing of the samples used to check a built path, INCHES. */
    public static double CHECK_STEP_IN = 0.5;
    /** When the watchdog fires while the robot is moving, it brakes first for at most this long, SECONDS. */
    public static double WATCHDOG_STOP_S = 0.4;
    /** ... and stops braking early once slower than this, IN/S. */
    public static double WATCHDOG_STOP_SPEED_IN_S = 4.0;
    /** One volley. Same as the original autos. */
    public static double TRANSFER_S = 1.0;

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
    private double matchTimeOffsetS = 0;

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
        matchTimeOffsetS = 0;
        clock.reset();
        log("START");
    }

    /** Sets the match clock to {@code seconds} from now on (for previewing decisions off the field). */
    public void setMatchTime(double seconds) {
        matchTimeOffsetS = seconds;
        clock.reset();
    }

    public double t() {
        return matchTimeOffsetS + clock.seconds();
    }

    public double remaining() {
        return AUTO_LENGTH_S - t();
    }

    /** True if {@code seconds} of work still leaves time to park from {@code endPose}. */
    public boolean fits(double seconds, Pose2d endPose) {
        return fitsAt(t(), seconds, endPose);
    }

    /** {@link #fits} as it would answer at match time {@code matchTimeS}. */
    public boolean fitsAt(double matchTimeS, double seconds, Pose2d endPose) {
        return AUTO_LENGTH_S - matchTimeS >= seconds + estimateRoute(endPose, parkPose) + PARK_MARGIN_S;
    }

    /** Latest match time at which {@link #fits} still says yes, SECONDS. */
    public double latestStart(double seconds, Pose2d endPose) {
        return AUTO_LENGTH_S - (seconds + estimateRoute(endPose, parkPose) + PARK_MARGIN_S);
    }

    public Pose2d parkPose() {
        return parkPose;
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
            stowRamp();
            log("parking");
        });
    }

    /** Also stows the FLOWER ramp, in case a leg back from a FLOWER was cut short. */
    public Action transfer() {
        return new SequentialAction(instant(this::stowRamp),
                intake.feedPollen(TRANSFER_S));
    }

    private void stowRamp() {
        if (intake != null) intake.setFlowerRamp(BiobuzzRobotConstants.flowerRampStowedPos);
    }

    /**
     * Runs {@code leg} away from a FLOWER with the ramp still down, and stows the ramp once the robot
     * is {@link BiobuzzRobotConstants#flowerRampClearIn} from where the leg started (or when the leg
     * ends, if sooner), so it never swings up into the FLOWER. Pair it with a leg that backs
     * straight out first ({@link #departFlower}, {@link #routeFromFlower}).
     */
    public Action leaveFlower(Action leg) {
        return new Action() {
            private Vector2d start = null;
            private boolean stowed = false;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (start == null) start = drive.pose.position;
                boolean running = leg.run(p);
                if (!stowed && (!running || dist(drive.pose.position, start)
                        >= BiobuzzRobotConstants.flowerRampClearIn - RAMP_CLEAR_TOLERANCE_IN)) {
                    stowRamp();
                    stowed = true;
                }
                return running;
            }

            @Override
            public void preview(@NonNull Canvas c) {
                leg.preview(c);
            }
        };
    }

    /** Slack on {@link BiobuzzRobotConstants#flowerRampClearIn} for the follower ending a little short, INCHES. */
    public static double RAMP_CLEAR_TOLERANCE_IN = 0.5;

    /** The point {@link BiobuzzRobotConstants#flowerRampClearIn} straight back from {@code pose}, opposite its heading. */
    public static Vector2d flowerDepart(Pose2d pose) {
        double h = pose.heading.toDouble(), d = BiobuzzRobotConstants.flowerRampClearIn;
        return new Vector2d(pose.position.x - d * Math.cos(h), pose.position.y - d * Math.sin(h));
    }

    /**
     * Starts a prebuilt path out of a FLOWER: straight back to {@link #flowerDepart} at the intake
     * heading, still moving, so the next spline carries on from it without stopping.
     */
    public static TrajectoryActionBuilder departFlower(TrajectoryActionBuilder b, Pose2d flower,
                                                       VelConstraint vel, AccelConstraint accel) {
        double back = flower.heading.toDouble() + Math.PI;
        return b.setTangent(back).splineToConstantHeading(flowerDepart(flower), back, vel, accel);
    }

    public TrajectoryActionBuilder departFlower(TrajectoryActionBuilder b, Pose2d flower) {
        return departFlower(b, flower, vel, accel);
    }

    /** {@link #route} that first backs straight out of the FLOWER the robot is at ({@link #flowerDepart}). */
    public Action routeFromFlower(Pose2d from, Pose2d to) {
        return routeFromFlower(from, to, robotEnv());
    }

    public Action routeFromFlower(Pose2d from, Pose2d to, RouteOptimizer.Env env) {
        Vector2d depart = flowerDepart(from);
        RouteOptimizer.Result r = RouteOptimizer.search(from, depart, to, null, plan(depart, to.position), env,
                vel, accel, preparedPattern(from, to), RouteOptimizer.MAX_SEARCH_S);
        log(String.format("route from FLOWER to %s: %s %.2f s (%d tried)", fmt(to), r.pattern, r.seconds, r.built));
        lastPattern = r.pattern;
        return r.action;
    }

    /** {@link #prepare} for a {@link #routeFromFlower}. */
    public void prepareFromFlower(Pose2d from, Pose2d to) {
        prepareFromFlower(from, to, robotEnv());
    }

    public void prepareFromFlower(Pose2d from, Pose2d to, RouteOptimizer.Env env) {
        Vector2d depart = flowerDepart(from);
        RouteOptimizer.Result r = RouteOptimizer.search(from, depart, to, null, plan(depart, to.position), env,
                vel, accel, null, Double.MAX_VALUE);
        prepared.add(new Object[]{from, to, r.pattern, r.seconds});
    }

    /**
     * {@link #leg} away from a FLOWER: the ramp stays down until clear ({@link #leaveFlower}), and a
     * rebuilt leg is a {@link #routeFromFlower}, so it backs straight out too.
     */
    public Action fromFlowerLeg(Pose2d expectedStart, Action prebuilt, Pose2d target) {
        return leaveFlower(branch(() -> {
            if (near(drive.pose, expectedStart)) return prebuilt;
            log("rebuilding leg from FLOWER to " + fmt(target));
            return routeFromFlower(drive.pose, target);
        }));
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
        return shotPoint(side, GATE_FRESH_S);
    }

    /**
     * {@link #shotPoint(Side)} accepting HIVE evidence up to {@code freshS} old. For a shot whose
     * CELL was seen seconds earlier and cannot have tipped since without a TIPPING / down event
     * being seen (those are newer evidence and still win), so no time is spent re-confirming it.
     */
    public Action shotPoint(Side side, double freshS) {
        return new SequentialAction(waitForShot(freshS), branch(() -> decideShot(side, true, freshS)));
    }

    /**
     * Waits for evidence it does not already have (up to {@link #GATE_MAX_WAIT_S}) and, if the HIVE
     * is part-way through a tip, for it to come to rest (up to {@link #TIP_WAIT_MAX_S}, never past
     * the park budget). Launching while the HIVE swings can stop the TIP.
     */
    public Action waitForShot(double freshS) {
        return new Action() {
            private long start = -1;
            private boolean told = false;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (start < 0) start = System.nanoTime();
                double elapsed = (System.nanoTime() - start) / 1e9;
                ShotWait w = shotWait(freshS);
                if (w == ShotWait.HIVE_MOVING) {
                    if (!told) log("HIVE moving - waiting for it to settle");
                    told = true;
                    return elapsed < TIP_WAIT_MAX_S && fits(TRANSFER_S, drive.pose);
                }
                if (w == ShotWait.READY) return false;
                return elapsed < GATE_MAX_WAIT_S;
            }
        };
    }

    /**
     * What {@link #waitForShot} is waiting on right now: READY (no camera frames, or fresh evidence),
     * HIVE_MOVING (waits up to {@link #TIP_WAIT_MAX_S}) or NO_EVIDENCE (waits up to {@link #GATE_MAX_WAIT_S}).
     */
    public enum ShotWait { READY, HIVE_MOVING, NO_EVIDENCE }

    public ShotWait shotWait(double freshS) {
        if (!hive.cameraHealthy()) return ShotWait.READY;
        if (hive.hiveMoving(HIVE_STILL_S)) return ShotWait.HIVE_MOVING;
        if (hive.upCell(freshS) != null) return ShotWait.READY;
        return ShotWait.NO_EVIDENCE;
    }

    /**
     * After our own volley into {@code side}: ends as soon as that CELL is seen starting to tip, or
     * is no longer settled up, or after maxS. Costs time only while the CELL sits still at rest.
     */
    public Action waitForTipStart(Side side, double maxS) {
        return waitUntil(() -> !hive.cameraHealthy() || hive.tipping(side, HiveWatcher.TIP_MAX_S)
                || !hive.settledUp(side, 0.4), maxS);
    }

    public enum TipResult { TIPPED, NOT_TIPPED, UNKNOWN }

    /**
     * Waits at the LEFT shooting pose for the alliance's tip: ends as soon as the LEFT CELL is up and
     * at rest, or after maxWaitS - plus up to {@link #ALLIANCE_SETTLE_EXTRA_S} while a tip is
     * visibly under way (the LEFT CELL seen moving, or upright but not yet at rest).
     */
    public Action waitForAllianceTip(double maxWaitS) {
        return new SequentialAction(
                waitUntil(() -> hive.settledUp(Side.LEFT, 0.4), maxWaitS),
                waitUntil(() -> hive.settledUp(Side.LEFT, 0.4) || !hive.tipActivity(Side.LEFT, HIVE_STILL_S),
                        ALLIANCE_SETTLE_EXTRA_S));
    }

    /**
     * After {@link #waitForAllianceTip}: TIPPED if the LEFT CELL is seen up; NOT_TIPPED if the RIGHT
     * CELL is seen up or the near LEFT CELL stayed unseen for {@link #ALLIANCE_ABSENCE_S} while the
     * camera was working and the robot sat facing it; UNKNOWN otherwise (e.g. no camera frames).
     */
    public TipResult allianceTipResult() {
        if (hive.settledUp(Side.LEFT, 0.6)) return TipResult.TIPPED;
        if (hive.settledUp(Side.RIGHT, 0.6) || (hive.downByAbsence(Side.LEFT, ALLIANCE_ABSENCE_S)
                && !hive.tipActivity(Side.LEFT, 1.0))) {
            return TipResult.NOT_TIPPED;
        }
        return TipResult.UNKNOWN;
    }

    /** What a shot at {@code side} decides once its wait is over (see {@link #shotPoint(Side)}). */
    public enum ShotChoice { SHOOT_UP, SHOOT_NO_VIEW, HOLD_SWINGING, HOLD_DOWN_AGAIN, CROSS, HOLD_NO_TIME }

    /**
     * The shot decision from the robot at {@code from} at match time {@code matchTimeS}. The autos
     * call it with the robot's pose and the match clock; it is public so it can be previewed.
     */
    public ShotChoice shotChoice(Side side, boolean mayCross, double freshS, Pose2d from, double matchTimeS) {
        if (hive.cameraHealthy() && hive.moving(side, HIVE_STILL_S)) return ShotChoice.HOLD_SWINGING;
        Side up = hive.upCell(freshS);
        if (up == side.other()) {
            if (!mayCross) return ShotChoice.HOLD_DOWN_AGAIN;
            Pose2d dest = shotPose(side.other());
            double need = estimateRoute(from, dest) + GATE_MAX_WAIT_S + TRANSFER_S;
            return fitsAt(matchTimeS, need, dest) ? ShotChoice.CROSS : ShotChoice.HOLD_NO_TIME;
        }
        return up == side ? ShotChoice.SHOOT_UP : ShotChoice.SHOOT_NO_VIEW;
    }

    /** The log line the autos write for a shot decision. */
    public static String describeShot(Side side, ShotChoice c) {
        switch (c) {
            case HOLD_SWINGING:
                return side + " CELL still swinging - holding";
            case HOLD_DOWN_AGAIN:
                return side + " CELL down again after crossing - holding";
            case CROSS:
                return side + " CELL is DOWN - crossing to shoot " + side.other();
            case HOLD_NO_TIME:
                return side + " CELL is DOWN, no time to cross - holding";
            case SHOOT_UP:
                return side + " CELL up - shooting";
            default:
                return "no HIVE view - shooting " + side + " as planned";
        }
    }

    private Action decideShot(Side side, boolean mayCross, double freshS) {
        ShotChoice c = shotChoice(side, mayCross, freshS, drive.pose, t());
        log(describeShot(side, c));
        if (c == ShotChoice.CROSS) {
            Pose2d dest = shotPose(side.other());
            return new SequentialAction(
                    route(drive.pose, dest),
                    waitForShot(GATE_FRESH_S),
                    branch(() -> decideShot(side.other(), false, GATE_FRESH_S)));
        }
        if (c == ShotChoice.SHOOT_UP || c == ShotChoice.SHOOT_NO_VIEW) return transfer();
        return new NullAction();
    }

    // =============================================================================================
    // Park watchdog
    // =============================================================================================

    /**
     * Runs {@code main}, but abandons it the moment the time left only just covers driving to park
     * from where the robot is (plus {@link #PARK_MARGIN_S}). Then stops every mechanism, brakes (if it was moving, so the park
     * route starts from where the robot really stopped rather than where it was a coast ago) and
     * parks from the robot's actual pose. The planned park (after {@link #beginPark}) is never
     * interrupted.
     */
    public Action guardPark(Action main) {
        return new Action() {
            private Action park;
            private boolean braking = false;
            private final ElapsedTime brakeTimer = new ElapsedTime();
            private Pose2d lastPose;
            private long lastNanos;
            private double speed = 0;
            private double exactS = 0;
            private final ElapsedTime exactTimer = new ElapsedTime();
            private boolean haveExact = false;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (park != null) return park.run(p);
                if (braking) {
                    PoseVelocity2d v = drive.updatePoseEstimate();
                    if (Math.hypot(v.linearVel.x, v.linearVel.y) < WATCHDOG_STOP_SPEED_IN_S
                            || brakeTimer.seconds() >= WATCHDOG_STOP_S) {
                        park = route(drive.pose, parkPose);
                    }
                    return true;
                }
                trackSpeed();
                boolean moving = speed > WATCHDOG_STOP_SPEED_IN_S;
                double extra = PARK_MARGIN_S + (moving ? WATCHDOG_STOP_S : 0);
                if (!parking && remaining() <= estimateRoute(drive.pose, parkPose) + extra
                        && remaining() <= exactParkS() + extra) {
                    log("WATCHDOG: out of time - parking now");
                    intake.stopAllFeeding();
                    parking = true;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    if (moving) {
                        braking = true;
                        brakeTimer.reset();
                        return true;
                    }
                    park = route(drive.pose, parkPose);
                    return park.run(p);
                }
                return main.run(p);
            }

            /**
             * The rough estimate is cheap but deliberately pessimistic. Only once it says "park now"
             * is a real park route built and its planned time used (re-measured every
             * WATCHDOG_REFINE_S), so the auto is not cut short by the pessimism. The plain strafe
             * route is timed (one quick build): the optimised one actually driven is never slower.
             */
            private double exactParkS() {
                if (!haveExact || exactTimer.seconds() >= WATCHDOG_REFINE_S) {
                    double prepared = preparedSeconds(drive.pose, parkPose);
                    exactS = (Double.isNaN(prepared) ? motionTime(safeRoute(drive.pose, parkPose, robotEnv())) : prepared)
                            + ROUTE_SETTLE_S;
                    haveExact = true;
                    exactTimer.reset();
                }
                return exactS;
            }

            private void trackSpeed() {
                long now = System.nanoTime();
                if (lastPose == null) {
                    lastPose = drive.pose;
                    lastNanos = now;
                    return;
                }
                double dt = (now - lastNanos) / 1e9;
                if (dt < 0.02) return;
                speed = 0.5 * speed + 0.5 * dist(lastPose.position, drive.pose.position) / dt;
                lastPose = drive.pose;
                lastNanos = now;
            }
        };
    }

    /** HIVE watcher update and telemetry; runs alongside everything else for the whole auto. */
    public Action background() {
        return p -> {
            hive.update(drive.pose);
            for (String e : hive.drainEvents()) log(e);
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
    //
    // Collision rule: the robot is a 15 x 18 in rectangle that may point any way while it drives, so
    // runtime routes keep its CENTRE at least robotRadius() + TRACKING_MARGIN_IN from the HIVE frame
    // (robotRadius = half the diagonal). plan() picks the corner points: straight lines between them
    // are checked exactly, using two corridor corners below the HIVE (red side only, never y > 0).
    // RouteOptimizer then decides HOW to drive through them - stop-and-strafe, or one smooth path
    // driven forward/backward along the travel direction - by timing each candidate with the Road
    // Runner profile and sampling it against the same clearance rule, the field walls and our
    // FLOWERS. A robot that starts inside the clearance zone (pushed, or the watchdog fired next to
    // the HIVE) first backs straight away from the nearest HIVE face.

    /** Half the robot's diagonal: the farthest any part of the robot reaches from its centre, INCHES. */
    public static double robotRadius() {
        return 0.5 * Math.hypot(BiobuzzRobotConstants.robotWidthIn, BiobuzzRobotConstants.robotLengthIn);
    }

    /** Distance the robot CENTRE must keep from the HIVE frame on runtime routes, INCHES. */
    public static double requiredClearance() {
        return robotRadius() + TRACKING_MARGIN_IN;
    }

    private static Vector2d[] corridorCorners() {
        double c = requiredClearance() + WAYPOINT_SLACK_IN;
        return new Vector2d[]{new Vector2d(-(HIVE_HALF_X + c), -(HIVE_HALF_Y + c)),
                new Vector2d(HIVE_HALF_X + c, -(HIVE_HALF_Y + c))};
    }

    /** Straight-segment plan from a to b: the intermediate points, whether it starts with an escape. */
    public static final class Plan {
        public final List<Vector2d> points;
        public final boolean escaped, safe;

        Plan(List<Vector2d> points, boolean escaped, boolean safe) {
            this.points = points;
            this.escaped = escaped;
            this.safe = safe;
        }
    }

    /**
     * Shortest plan (path length plus a penalty per corner) whose every straight segment keeps the
     * robot centre {@link #requiredClearance()} from the HIVE frame.
     */
    public static Plan plan(Vector2d a, Vector2d b) {
        List<Vector2d> best = bestThroughCorners(a, b);
        if (best != null) return new Plan(avoidFlowers(a, best, b), false, true);
        Vector2d e = escapePoint(a);
        best = bestThroughCorners(e, b);
        boolean safe = best != null;
        if (best == null) best = closestCornerChain(e, b);
        List<Vector2d> pts = new ArrayList<>();
        pts.add(e);
        pts.addAll(avoidFlowers(e, best, b));
        return new Plan(pts, true, safe);
    }

    /** Robot centre distance kept from a FLOWER centre when passing it, INCHES (any heading). */
    public static double flowerPassIn() {
        return RouteOptimizer.FLOWER_RADIUS_IN + robotRadius() + RouteOptimizer.OBSTACLE_GAP_IN;
    }

    /**
     * Where a straight piece would pass a FLOWER closer than {@link #flowerPassIn()} (and neither of
     * its ends is already that close - e.g. the FLOWER intake pose), adds a point beside the FLOWER,
     * on the side toward the field centre, provided the new pieces still clear the HIVE.
     */
    private static List<Vector2d> avoidFlowers(Vector2d a, List<Vector2d> corners, Vector2d b) {
        List<Vector2d> all = new ArrayList<>();
        all.add(a);
        all.addAll(corners);
        all.add(b);
        double zone = flowerPassIn(), req = requiredClearance();
        boolean[] used = new boolean[RouteOptimizer.FLOWERS.length];
        for (int i = 0; i + 1 < all.size(); i++) {
            Vector2d p = all.get(i), q = all.get(i + 1);
            for (int k = 0; k < RouteOptimizer.FLOWERS.length; k++) {
                double[] f = RouteOptimizer.FLOWERS[k];
                Vector2d fc = new Vector2d(f[0], f[1]);
                if (used[k] || dist(p, fc) <= zone || dist(q, fc) <= zone || pointToSegment(fc, p, q) >= zone) continue;
                double n = Math.hypot(fc.x, fc.y);
                Vector2d side = new Vector2d(fc.x - fc.x / n * (zone + 1), fc.y - fc.y / n * (zone + 1));
                if (segmentClearance(p, side) < req || segmentClearance(side, q) < req) continue;
                all.add(i + 1, side);
                used[k] = true;
                break;
            }
        }
        return new ArrayList<>(all.subList(1, all.size() - 1));
    }

    private static List<Vector2d> bestThroughCorners(Vector2d a, Vector2d b) {
        Vector2d[] w = corridorCorners();
        Vector2d[][] options = {{}, {w[0]}, {w[1]}, {w[0], w[1]}, {w[1], w[0]}};
        double req = requiredClearance();
        double penalty = ROUTE_WAYPOINT_S * ROUTE_SPEED_IN_S;
        List<Vector2d> best = null;
        double bestCost = Double.MAX_VALUE;
        for (Vector2d[] o : options) {
            Vector2d prev = a;
            boolean clear = true;
            double cost = o.length * penalty;
            for (int i = 0; i <= o.length && clear; i++) {
                Vector2d next = i < o.length ? o[i] : b;
                clear = segmentClearance(prev, next) >= req;
                cost += dist(prev, next);
                prev = next;
            }
            if (clear && cost < bestCost) {
                bestCost = cost;
                best = java.util.Arrays.asList(o);
            }
        }
        return best;
    }

    /** Only used when the target itself is too close to the HIVE: go round to the corner nearest it. */
    private static List<Vector2d> closestCornerChain(Vector2d e, Vector2d b) {
        Vector2d[] w = corridorCorners();
        Vector2d first = dist(e, w[0]) < dist(e, w[1]) ? w[0] : w[1];
        Vector2d last = dist(b, w[0]) < dist(b, w[1]) ? w[0] : w[1];
        List<Vector2d> out = new ArrayList<>();
        out.add(first);
        if (last != first) out.add(last);
        return out;
    }

    /**
     * Straight out from the nearest HIVE face (left, right or bottom) to the corridor line. Moving
     * along a face's outward normal never brings the robot closer to the frame.
     */
    private static Vector2d escapePoint(Vector2d a) {
        double c = requiredClearance() + WAYPOINT_SLACK_IN;
        double ex = Math.abs(a.x) - HIVE_HALF_X, ey = Math.abs(a.y) - HIVE_HALF_Y;
        if (ex > ey) return new Vector2d((a.x < 0 ? -1 : 1) * (HIVE_HALF_X + c), a.y);
        return new Vector2d(a.x, -(HIVE_HALF_Y + c));
    }

    private final List<Object[]> prepared = new ArrayList<>();
    /** How the last runtime route is driven (telemetry / testing). */
    public String lastPattern = "";

    /** The robot's own builder, sampler and timer for {@link RouteOptimizer}. */
    private RouteOptimizer.Env robotEnv() {
        return new RouteOptimizer.Env() {
            @Override
            public TrajectoryActionBuilder builder(Pose2d start) {
                return drive.actionBuilder(start);
            }

            @Override
            public List<Pose2d> poses(Action a) {
                return pathPoses(a);
            }

            @Override
            public double seconds(Action a) {
                return motionTime(a);
            }
        };
    }

    /**
     * The FASTEST safe route from {@code from} to {@code to} (see {@link RouteOptimizer}): driven
     * forward/backward along the travel direction where that is quicker, turning while moving, not
     * stopping at corners - whichever the Road Runner profile says is quickest - and never closer to
     * the HIVE than {@link #requiredClearance()}.
     */
    public Action route(Pose2d from, Pose2d to) {
        return route(from, to, robotEnv());
    }

    /** {@link #route(Pose2d, Pose2d)} with the builder/sampler/timer supplied (offline testing). */
    public Action route(Pose2d from, Pose2d to, RouteOptimizer.Env env) {
        Plan plan = plan(from.position, to.position);
        if (!plan.safe) log("WARNING: target " + fmt(to) + " is too close to the HIVE");
        RouteOptimizer.Result r = RouteOptimizer.search(from, to, plan, env, vel, accel,
                preparedPattern(from, to), RouteOptimizer.MAX_SEARCH_S);
        log(String.format("route to %s: %s %.2f s (%d tried)", fmt(to), r.pattern, r.seconds, r.built));
        lastPattern = r.pattern;
        return r.action;
    }

    /** The plain stop-at-each-corner strafe route: never faster than {@link #route}, so a safe upper bound. */
    public Action safeRoute(Pose2d from, Pose2d to, RouteOptimizer.Env env) {
        Plan plan = plan(from.position, to.position);
        Vector2d escape = plan.escaped ? plan.points.get(0) : null;
        List<Vector2d> corners = plan.escaped ? plan.points.subList(1, plan.points.size()) : plan.points;
        return RouteOptimizer.build("stops", from, to, escape, corners, env, vel, accel);
    }

    /**
     * Searches every candidate for a route the auto is likely to need (call in INIT, no time limit)
     * and remembers the winner, which a runtime {@link #route} from near the same start then tries
     * first.
     */
    public void prepare(Pose2d from, Pose2d to) {
        prepare(from, to, robotEnv());
    }

    public void prepare(Pose2d from, Pose2d to, RouteOptimizer.Env env) {
        RouteOptimizer.Result r = RouteOptimizer.search(from, to, plan(from.position, to.position), env, vel, accel,
                null, Double.MAX_VALUE);
        prepared.add(new Object[]{from, to, r.pattern, r.seconds});
    }

    // ---------------------------------------------------------------------------------------------
    // FLOWER approach: always head-on, in one continuous motion
    // ---------------------------------------------------------------------------------------------

    /**
     * The pose {@link BiobuzzRobotConstants#flowerApproachIn} straight back from a FLOWER intake
     * pose, at the intake heading - where the straight run into the FLOWER begins.
     */
    public static Pose2d flowerStaging(Pose2d flower) {
        double h = flower.heading.toDouble(), d = BiobuzzRobotConstants.flowerApproachIn;
        return new Pose2d(flower.position.x - d * Math.cos(h), flower.position.y - d * Math.sin(h), h);
    }

    /**
     * Ends a prebuilt path at a FLOWER: to the staging pose already facing the FLOWER and moving
     * straight at it, then straight forward onto the intake pose, in the same trajectory (no stop).
     */
    public TrajectoryActionBuilder approachFlower(TrajectoryActionBuilder b, Pose2d flower) {
        return approachFlower(b, flower, vel, accel);
    }

    public static TrajectoryActionBuilder approachFlower(TrajectoryActionBuilder b, Pose2d flower,
                                                         VelConstraint vel, AccelConstraint accel) {
        double h = flower.heading.toDouble();
        return b.splineToSplineHeading(flowerStaging(flower), h, vel, accel)
                .splineToConstantHeading(flower.position, h, vel, accel);
    }

    /** The fastest safe runtime route to a FLOWER intake pose, ending with the head-on approach. */
    public Action flowerRoute(Pose2d from, Pose2d flower) {
        return flowerRoute(from, flower, robotEnv());
    }

    public Action flowerRoute(Pose2d from, Pose2d flower, RouteOptimizer.Env env) {
        Pose2d staging = flowerStaging(flower);
        RouteOptimizer.Result r = RouteOptimizer.search(from, staging, flower.position,
                plan(from.position, staging.position), env, vel, accel, preparedPattern(from, flower),
                RouteOptimizer.MAX_SEARCH_S);
        log(String.format("FLOWER route to %s: %s %.2f s (%d tried)", fmt(flower), r.pattern, r.seconds, r.built));
        lastPattern = r.pattern;
        return r.action;
    }

    /** {@link #prepare} for a {@link #flowerRoute}. */
    public void prepareFlower(Pose2d from, Pose2d flower) {
        prepareFlower(from, flower, robotEnv());
    }

    public void prepareFlower(Pose2d from, Pose2d flower, RouteOptimizer.Env env) {
        Pose2d staging = flowerStaging(flower);
        RouteOptimizer.Result r = RouteOptimizer.search(from, staging, flower.position,
                plan(from.position, staging.position), env, vel, accel, null, Double.MAX_VALUE);
        prepared.add(new Object[]{from, flower, r.pattern, r.seconds});
    }

    /**
     * {@link #leg} for a FLOWER: if the robot is not where {@code prebuilt} starts, the rebuilt leg
     * lowers the ramp at once and drives a {@link #flowerRoute}, then runs {@code thenIfRebuilt}.
     */
    public Action flowerLeg(Pose2d expectedStart, Action prebuilt, Pose2d flower, Action thenIfRebuilt) {
        return branch(() -> {
            if (near(drive.pose, expectedStart)) return prebuilt;
            log("rebuilding FLOWER leg to " + fmt(flower));
            Action approach = new ParallelAction(intake.deployFlowerRamp(), flowerRoute(drive.pose, flower));
            return thenIfRebuilt == null ? approach : new SequentialAction(approach, thenIfRebuilt);
        });
    }

    /** Prepares the routes every HIVE auto may need: between the shot poses and from them to park. */
    public void prepareDefaults() {
        prepare(leftShot, rightShot);
        prepare(rightShot, leftShot);
        prepare(leftShot, parkPose);
        prepare(rightShot, parkPose);
    }

    /** Start within this of a prepared route's start lets its exact time stand in for an estimate. */
    public static double PREPARED_TIME_TOL_IN = 3.0;
    public static double PREPARED_TIME_TOL_DEG = 10.0;

    /** Exact planned time of a prepared route from (about) here to {@code to}, or NaN if none. */
    private double preparedSeconds(Pose2d from, Pose2d to) {
        for (Object[] e : prepared) {
            Pose2d f = (Pose2d) e[0], t = (Pose2d) e[1];
            if (dist(t.position, to.position) < 1.0
                    && Math.abs(Math.toDegrees(wrap(t.heading.toDouble() - to.heading.toDouble()))) < 3.0
                    && dist(from.position, f.position) < PREPARED_TIME_TOL_IN
                    && Math.abs(Math.toDegrees(wrap(from.heading.toDouble() - f.heading.toDouble()))) < PREPARED_TIME_TOL_DEG) {
                return (Double) e[3];
            }
        }
        return Double.NaN;
    }

    private String preparedPattern(Pose2d from, Pose2d to) {
        for (Object[] e : prepared) {
            Pose2d f = (Pose2d) e[0], t = (Pose2d) e[1];
            if (dist(t.position, to.position) < 1.0
                    && Math.abs(Math.toDegrees(wrap(t.heading.toDouble() - to.heading.toDouble()))) < 3.0
                    && near(from, f)) {
                return (String) e[2];
            }
        }
        return null;
    }

    /** Planned motion time of a built Road Runner action (its trajectories and turns), SECONDS. */
    public static double motionTime(Action a) {
        double t = 0;
        if (a instanceof SequentialAction) {
            for (Action c : ((SequentialAction) a).getInitialActions()) t += motionTime(c);
        } else if (a instanceof ParallelAction) {
            for (Action c : ((ParallelAction) a).getInitialActions()) t = Math.max(t, motionTime(c));
        } else if (a instanceof MecanumDrive.FollowTrajectoryAction) {
            t = ((MecanumDrive.FollowTrajectoryAction) a).timeTrajectory.duration;
        } else if (a instanceof MecanumDrive.FollowTrajectoryActionFast) {
            t = ((MecanumDrive.FollowTrajectoryActionFast) a).timeTrajectory.duration;
        } else if (a instanceof MecanumDrive.TurnAction) {
            t = ((MecanumDrive.TurnAction) a).turn.duration;
        } else if (a instanceof SleepAction) {
            t = ((SleepAction) a).getDt();
        }
        return t;
    }

    /**
     * Rollers start this long into a wall-set leg (the leg starting at a shooting pose): 0.4 s before
     * the first GARDEN pollen is reached (2.52 s), SECONDS.
     */
    public static double WALL_COLLECT_START_S = 2.1;
    /** ...and never run for less than this, SECONDS (through the last pollen at 2.93 s plus time to pull it in). */
    public static double WALL_COLLECT_MIN_S = 2.5;

    /**
     * Runs a whole wall-set leg ({@code drive}: no intake markers of its own) with the rollers on
     * from {@link #WALL_COLLECT_START_S} until the drive ends, but for at least
     * {@link #WALL_COLLECT_MIN_S}. Collecting therefore never holds the robot up once it has
     * arrived, and the rollers get every second the drive gives them.
     */
    public Action withWallCollect(Action drive) {
        double run = Math.max(WALL_COLLECT_MIN_S, motionTime(drive) - WALL_COLLECT_START_S);
        return new ParallelAction(drive,
                new SequentialAction(new SleepAction(WALL_COLLECT_START_S), intake.collectToGate(run)));
    }

    /**
     * Drive-time estimate for {@link #route}, SECONDS: the exact planned time (+ settle) of a route
     * prepared in INIT from about the same start, otherwise a conservative geometric estimate.
     */
    public double estimateRoute(Pose2d from, Pose2d to) {
        double rough = roughEstimate(from, to);
        double exact = preparedSeconds(from, to);
        return Double.isNaN(exact) ? rough : Math.min(rough, exact + ROUTE_SETTLE_S);
    }

    /** Geometry-only estimate, deliberately pessimistic (never below the real drive). */
    private double roughEstimate(Pose2d from, Pose2d to) {
        Plan plan = plan(from.position, to.position);
        double len = 0;
        Vector2d prev = from.position;
        for (Vector2d w : plan.points) {
            len += dist(prev, w);
            prev = w;
        }
        len += dist(prev, to.position);
        double turn = Math.abs(wrap(to.heading.toDouble() - from.heading.toDouble()));
        return len / ROUTE_SPEED_IN_S + ROUTE_OVERHEAD_S + turn * ROUTE_TURN_S_PER_RAD
                + plan.points.size() * ROUTE_WAYPOINT_S;
    }

    // ---------------------------------------------------------------------------------------------
    // Prebuilt path check (run in INIT)
    // ---------------------------------------------------------------------------------------------

    private String closestName = null;
    private double closestGap = Double.MAX_VALUE;
    private final List<String> clearanceWarnings = new ArrayList<>();

    /**
     * Samples a prebuilt path and measures the true gap between the robot's 15 x 18 in footprint (at
     * its real heading) and the HIVE frame. Gaps under {@link #TRACKING_MARGIN_IN} are reported.
     */
    public void checkPrebuilt(String name, Action path) {
        double min = Double.MAX_VALUE;
        for (Pose2d p : pathPoses(path)) min = Math.min(min, footprintGap(p));
        if (min == Double.MAX_VALUE) return;
        if (min < closestGap) {
            closestGap = min;
            closestName = name;
        }
        if (min < TRACKING_MARGIN_IN) {
            String w = String.format("WARNING %s comes within %.1f in of the HIVE", name, Math.max(0, min));
            clearanceWarnings.add(w);
            log(w);
        }
    }

    public String clearanceSummary() {
        if (closestName == null) return "no paths checked";
        if (!clearanceWarnings.isEmpty()) return String.join("\n", clearanceWarnings);
        return String.format("all paths clear (closest %.1f in, %s)", closestGap, closestName);
    }

    /** Every pose along the trajectories (and turns) inside a built Road Runner action. */
    public static List<Pose2d> pathPoses(Action a) {
        List<Pose2d> out = new ArrayList<>();
        collectPoses(a, out);
        return out;
    }

    private static void collectPoses(Action a, List<Pose2d> out) {
        if (a instanceof SequentialAction) {
            for (Action c : ((SequentialAction) a).getInitialActions()) collectPoses(c, out);
        } else if (a instanceof ParallelAction) {
            for (Action c : ((ParallelAction) a).getInitialActions()) collectPoses(c, out);
        } else if (a instanceof MecanumDrive.FollowTrajectoryAction) {
            samplePath(((MecanumDrive.FollowTrajectoryAction) a).timeTrajectory.path, out);
        } else if (a instanceof MecanumDrive.FollowTrajectoryActionFast) {
            samplePath(((MecanumDrive.FollowTrajectoryActionFast) a).timeTrajectory.path, out);
        } else if (a instanceof MecanumDrive.TurnAction) {
            TimeTurn t = ((MecanumDrive.TurnAction) a).turn;
            for (double s = 0; s < t.duration; s += 0.02) out.add(t.get(s).value());
            out.add(t.get(t.duration).value());
        }
    }

    public static void samplePath(PosePath path, List<Pose2d> out) {
        double len = path.length();
        for (double s = 0; s < len; s += CHECK_STEP_IN) out.add(path.get(s, 1).value());
        out.add(path.get(len, 1).value());
    }

    // ---------------------------------------------------------------------------------------------
    // Geometry against the HIVE frame (axis-aligned box centred on the origin)
    // ---------------------------------------------------------------------------------------------

    /** Distance from a point to the HIVE frame, 0 inside it. */
    public static double pointClearance(Vector2d p) {
        double dx = Math.max(0, Math.abs(p.x) - HIVE_HALF_X), dy = Math.max(0, Math.abs(p.y) - HIVE_HALF_Y);
        return Math.hypot(dx, dy);
    }

    /** Smallest distance from any point of segment ab to the HIVE frame, 0 if it crosses it. */
    public static double segmentClearance(Vector2d a, Vector2d b) {
        if (segmentHitsFrame(a, b)) return 0;
        double d = Math.min(pointClearance(a), pointClearance(b));
        for (Vector2d c : frameCorners()) d = Math.min(d, pointToSegment(c, a, b));
        return d;
    }

    /** True gap between the robot's footprint at pose p and the HIVE frame, 0 if they overlap. */
    public static double footprintGap(Pose2d p) {
        Vector2d[] robot = robotCorners(p);
        Vector2d[] frame = frameCorners();
        if (overlaps(robot, frame, p.heading.toDouble())) return 0;
        double d = Double.MAX_VALUE;
        for (Vector2d c : robot) d = Math.min(d, pointClearance(c));
        for (Vector2d c : frame) {
            for (int i = 0; i < 4; i++) d = Math.min(d, pointToSegment(c, robot[i], robot[(i + 1) % 4]));
        }
        return d;
    }

    private static Vector2d[] frameCorners() {
        return new Vector2d[]{new Vector2d(-HIVE_HALF_X, -HIVE_HALF_Y), new Vector2d(HIVE_HALF_X, -HIVE_HALF_Y),
                new Vector2d(HIVE_HALF_X, HIVE_HALF_Y), new Vector2d(-HIVE_HALF_X, HIVE_HALF_Y)};
    }

    /** Footprint corners: length along the heading, width across it. */
    private static Vector2d[] robotCorners(Pose2d p) {
        double h = p.heading.toDouble(), c = Math.cos(h), s = Math.sin(h);
        double l = BiobuzzRobotConstants.robotLengthIn / 2, w = BiobuzzRobotConstants.robotWidthIn / 2;
        double[][] local = {{l, w}, {-l, w}, {-l, -w}, {l, -w}};
        Vector2d[] out = new Vector2d[4];
        for (int i = 0; i < 4; i++) {
            out[i] = new Vector2d(p.position.x + local[i][0] * c - local[i][1] * s,
                    p.position.y + local[i][0] * s + local[i][1] * c);
        }
        return out;
    }

    /** Separating-axis test between the robot rectangle and the frame. */
    private static boolean overlaps(Vector2d[] robot, Vector2d[] frame, double heading) {
        double[][] axes = {{1, 0}, {0, 1}, {Math.cos(heading), Math.sin(heading)},
                {-Math.sin(heading), Math.cos(heading)}};
        for (double[] ax : axes) {
            double r0 = Double.MAX_VALUE, r1 = -Double.MAX_VALUE, f0 = Double.MAX_VALUE, f1 = -Double.MAX_VALUE;
            for (Vector2d v : robot) {
                double d = v.x * ax[0] + v.y * ax[1];
                r0 = Math.min(r0, d);
                r1 = Math.max(r1, d);
            }
            for (Vector2d v : frame) {
                double d = v.x * ax[0] + v.y * ax[1];
                f0 = Math.min(f0, d);
                f1 = Math.max(f1, d);
            }
            if (r1 < f0 || f1 < r0) return false;
        }
        return true;
    }

    private static double pointToSegment(Vector2d p, Vector2d a, Vector2d b) {
        double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        double t = len2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
        return Math.hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
    }

    /** Does segment ab cross the HIVE frame? (Liang-Barsky clip.) */
    private static boolean segmentHitsFrame(Vector2d a, Vector2d b) {
        double t0 = 0, t1 = 1, dx = b.x - a.x, dy = b.y - a.y;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {a.x + HIVE_HALF_X, HIVE_HALF_X - a.x, a.y + HIVE_HALF_Y, HIVE_HALF_Y - a.y};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            } else {
                double r = q[i] / p[i];
                if (p[i] < 0) t0 = Math.max(t0, r);
                else t1 = Math.min(t1, r);
            }
        }
        return t0 <= t1;
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
