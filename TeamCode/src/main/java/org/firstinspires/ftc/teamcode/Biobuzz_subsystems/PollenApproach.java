package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Drives the robot to the pile of pollen that {@link PollenCamera} has confirmed, and stops with
 * the intake pointed at it.
 *
 * <h2>How it drives</h2>
 * The pile is a fixed point on the field, so the move is a RoadRunner trajectory built from the
 * live pose to a standoff point short of that point, with the final heading aimed at the pile.
 * Using the existing {@code MecanumDrive.actionBuilder} rather than a hand-rolled P controller is
 * the whole reason this is short: it inherits the tuned feedforward, the holonomic controller and
 * the motion profile that every auto in this codebase already relies on, so it accelerates and
 * stops like the rest of the robot instead of like a new, separately-tuned thing.
 *
 * <h2>Non-blocking</h2>
 * {@link #update} runs exactly one {@code Action.run()} per opmode loop and returns. The teleop
 * loop keeps turning the whole time, which is what lets the driver take the robot back instantly -
 * a blocking {@code Actions.runBlocking()} would hold the robot hostage until the path finished,
 * and in teleop that is a way to lose a match.
 *
 * <h2>Re-planning</h2>
 * The first estimate is made from as far away as the robot can see, which is where the range
 * estimate is worst. As it closes in, the same pile is measured from closer and better, so the
 * plan is rebuilt whenever the confirmed target has moved more than {@link #REPLAN_TOL_IN} - up to
 * {@link #MAX_REPLANS} times, and never inside {@link #REPLAN_LOCKOUT_IN}. Both limits exist for
 * the same reason: close in, the pile is at the bottom edge of the frame and about to leave it
 * entirely, so late corrections are the least trustworthy ones, and a robot that keeps re-planning
 * on them will circle its target instead of arriving at it.
 *
 * <h2>Aborts</h2>
 * Driver stick or trigger movement, the cancel button, the timeout, or the opmode ending. Every
 * exit path goes through {@link #finish}, so there is exactly one place that stops the motors and
 * hands the drivetrain back in a sane state.
 */
@Config
public class PollenApproach {

    /** Stop this far short of the pile CENTROID, measured to the robot's centre of rotation. */
    public static double STANDOFF_IN = 11.0;
    /**
     * Added to the heading that points at the pile. 0 aims the robot's FRONT at it; use 180 if the
     * intake is on the back of the robot.
     */
    public static double APPROACH_HEADING_OFFSET_DEG = 0.0;

    /** Translational speed cap for the approach, in/s. Deliberately below the auto speeds. */
    public static double APPROACH_VEL_IN_S = 25.0;
    /** Turn constraints used when the robot only needs to pivot, rad/s and rad/s^2. */
    public static double TURN_VEL = 1.6;
    public static double TURN_ACCEL = 2.5;

    /** Refuse to start beyond this range - the far end of the estimate is the untrustworthy end. */
    public static double MAX_START_RANGE_IN = 66.0;
    /** Below this the robot is already there; the run becomes a pure aim, or nothing at all. */
    public static double MIN_MOVE_IN = 2.0;
    /** Refuse to start above this speed, in/s: the vision estimate lags the pose while moving. */
    public static double MAX_START_SPEED_IN_S = 6.0;
    /** Heading error under which "aim at the pile" counts as already satisfied, degrees. */
    public static double HEADING_TOL_DEG = 4.0;

    /** Hard limit on one run. */
    public static double TIMEOUT_S = 5.0;
    /** Any stick or trigger past this hands control straight back to the driver. */
    public static double ABORT_INPUT = 0.20;

    public static boolean ALLOW_REPLAN = true;
    public static double REPLAN_TOL_IN = 4.0;
    public static double REPLAN_MIN_INTERVAL_S = 0.5;
    public static double REPLAN_LOCKOUT_IN = 16.0;
    public static int MAX_REPLANS = 3;

    /**
     * Re-zero the drive's heading references when the run ends.
     *
     * Leave this ON. Heading hold latched its setpoint before the run and the run then rotated the
     * robot, so without the reset the hold immediately tries to spin the robot back to where it
     * was pointing when the driver last let go of the turn stick.
     *
     * NOTE the side effect: this is the same call the Back button makes, so with FIELD_CENTRIC on
     * it also re-zeroes the field-centric frame - "forward" becomes wherever the robot ended up
     * facing, exactly as if the driver had pressed Back.
     */
    public static boolean RESET_HEADING_REFS = true;

    /** Send the trajectory overlay to the dashboard field view while a run is active. */
    public static boolean DASHBOARD_OVERLAY = true;

    public enum State { IDLE, DRIVING, AIMING, ARRIVED, ABORTED }

    private final MecaTank mecaTank;
    private final PollenCamera camera;
    private final Telemetry telemetry;

    private State state = State.IDLE;
    private String status = "idle";
    private Action action = null;

    private Vector2d plannedTarget = null;   // pile centre this plan was built for, field inches
    private Vector2d goalPoint = null;       // where the robot is actually going, field inches
    private int replans = 0;

    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime planTimer = new ElapsedTime();

    public PollenApproach(MecaTank mecaTank, PollenCamera camera, Telemetry telemetry) {
        this.mecaTank = mecaTank;
        this.camera = camera;
        this.telemetry = telemetry;
    }

    // ===========================================================================================

    public State getState() {
        return state;
    }

    public boolean isRunning() {
        return state == State.DRIVING || state == State.AIMING;
    }

    public String getStatus() {
        return status;
    }

    /**
     * Begins a run at the currently confirmed pile.
     *
     * @return true if a run started. On false, {@link #getStatus()} says why not - every rejection
     *         is reported rather than silently doing nothing, because a button that sometimes does
     *         nothing and never says why is untunable in the pit.
     */
    public boolean start() {
        if (isRunning()) {
            status = "already running";
            return false;
        }
        if (!camera.isAvailable()) {
            state = State.ABORTED;
            status = "no camera";
            return false;
        }
        if (!camera.isTargetReady()) {
            state = State.ABORTED;
            status = "no confirmed pollen";
            return false;
        }
        if (mecaTank.getMeasuredVelocity() > MAX_START_SPEED_IN_S) {
            state = State.ABORTED;
            status = "moving too fast to aim";
            return false;
        }

        Pose2d pose = mecaTank.getPoseEstimate();
        Vector2d target = camera.getFieldTarget();
        double range = Math.hypot(target.x - pose.position.x, target.y - pose.position.y);
        if (range > MAX_START_RANGE_IN) {
            state = State.ABORTED;
            status = String.format("pollen too far (%.0f in)", range);
            return false;
        }

        replans = 0;
        runTimer.reset();
        // plan() reports its own outcome through state/status on every failure path, so there is
        // nothing to overwrite here - doing so would replace a specific reason with a generic one.
        return plan(pose, target);
    }

    /**
     * One step of the run. Call every loop while {@link #isRunning()}.
     *
     * While this is running the caller must NOT also drive the motors, and does not need to call
     * {@code updatePoseEstimate()} - RoadRunner's follower already does that inside the action.
     *
     * @param driverInput largest absolute stick/trigger value this loop, for the abort test
     * @param cancel      cancel button
     * @param opModeActive false ends the run immediately
     */
    public void update(double driverInput, boolean cancel, boolean opModeActive) {
        if (!isRunning()) return;

        if (!opModeActive) { finish(State.ABORTED, "opmode ended"); return; }
        if (cancel) { finish(State.ABORTED, "cancelled"); return; }
        if (Math.abs(driverInput) > ABORT_INPUT) { finish(State.ABORTED, "driver took over"); return; }
        if (runTimer.seconds() > TIMEOUT_S) { finish(State.ABORTED, "timeout"); return; }

        maybeReplan();
        // A re-plan can legitimately END the run - if the corrected target turns out to be inside
        // the standoff and already faced, there is nothing left to drive. Re-test before running.
        if (!isRunning() || action == null) return;

        TelemetryPacket packet = new TelemetryPacket();
        boolean alive;
        try {
            alive = action.run(packet);
        } catch (Exception e) {
            finish(State.ABORTED, "follower error: " + e.getMessage());
            return;
        }
        if (DASHBOARD_OVERLAY) FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (!alive) {
            finish(State.ARRIVED, state == State.AIMING ? "aimed" : "arrived");
        }
    }

    /** Ends a run from outside, e.g. when another subsystem takes the drivetrain. */
    public void cancel(String reason) {
        if (isRunning()) finish(State.ABORTED, reason);
    }

    // ===========================================================================================

    /**
     * Builds the trajectory to the current best guess.
     *
     * The goal is placed on the line from the robot to the pile, STANDOFF_IN short of it, so the
     * robot ends up nose-on with the pile just in front of the intake instead of underneath the
     * chassis. Heading is the bearing to the pile plus the mounting offset.
     */
    private boolean plan(Pose2d pose, Vector2d target) {
        double dx = target.x - pose.position.x;
        double dy = target.y - pose.position.y;
        double range = Math.hypot(dx, dy);

        if (range < 1e-6) {
            finishNoAction("already on top of the pollen");
            return false;
        }

        double bearing = Math.atan2(dy, dx);
        double finalHeading = bearing + Math.toRadians(APPROACH_HEADING_OFFSET_DEG);
        double travel = range - STANDOFF_IN;

        plannedTarget = target;
        planTimer.reset();

        if (travel < MIN_MOVE_IN) {
            // Already inside the standoff. Do not build a translation - a zero or near-zero length
            // path is not something RoadRunner's profile generator can express, and forcing one
            // through produces either an exception or a jerk. Turn to face it instead, or, if it
            // is already faced, declare the run finished.
            double err = wrap(finalHeading - pose.heading.toDouble());
            if (Math.abs(err) < Math.toRadians(HEADING_TOL_DEG)) {
                finishNoAction("already in position");
                return false;
            }
            action = mecaTank.drive.actionBuilder(pose)
                    .turnTo(finalHeading, new TurnConstraints(TURN_VEL, -TURN_ACCEL, TURN_ACCEL))
                    .build();
            goalPoint = pose.position;
            state = State.AIMING;
            status = String.format("aiming %.0f deg", Math.toDegrees(err));
            return true;
        }

        double ux = dx / range, uy = dy / range;
        goalPoint = new Vector2d(pose.position.x + ux * travel, pose.position.y + uy * travel);

        try {
            action = mecaTank.drive.actionBuilder(pose)
                    .strafeToLinearHeading(goalPoint, finalHeading,
                            new TranslationalVelConstraint(Math.max(2.0, APPROACH_VEL_IN_S)))
                    .build();
        } catch (Exception e) {
            finishNoAction("could not build path: " + e.getMessage());
            return false;
        }

        state = State.DRIVING;
        status = String.format("driving %.0f in to n=%d", travel, camera.getCount());
        return true;
    }

    /**
     * Rebuilds the plan if the camera's confirmed pile has moved meaningfully and it is still
     * early enough in the run for a correction to be worth more than the discontinuity it costs.
     */
    private void maybeReplan() {
        if (!ALLOW_REPLAN || state != State.DRIVING) return;
        if (replans >= MAX_REPLANS) return;
        if (planTimer.seconds() < REPLAN_MIN_INTERVAL_S) return;
        if (!camera.isTargetReady()) return;

        Pose2d pose = mecaTank.getPoseEstimate();
        Vector2d target = camera.getFieldTarget();
        if (target == null || plannedTarget == null) return;

        double moved = Math.hypot(target.x - plannedTarget.x, target.y - plannedTarget.y);
        if (moved < REPLAN_TOL_IN) return;

        double range = Math.hypot(target.x - pose.position.x, target.y - pose.position.y);
        if (range < REPLAN_LOCKOUT_IN) return;

        int keep = replans + 1;
        if (plan(pose, target)) {
            replans = keep;
            status = status + " (replan " + replans + ")";
        }
    }

    /** Single exit path: stop the wheels, hand the drivetrain back, record why. */
    private void finish(State endState, String reason) {
        mecaTank.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        releaseDrivetrain();
        action = null;
        state = endState;
        status = reason;
        camera.reset();   // force a fresh confirmation before the next run
    }

    /** Same, for rejections that never built an action and so never touched the motors. */
    private void finishNoAction(String reason) {
        releaseDrivetrain();
        action = null;
        state = State.ARRIVED;
        status = reason;
        camera.reset();
    }

    /**
     * Puts MecaTank's manual-drive path back in a state the driver can be handed.
     *
     * The RoadRunner follower writes motor powers directly, behind the back of
     * setDrivePowersSmooth()'s acceleration limiter, so the limiter's stored "previous command" is
     * whatever the driver last asked for - possibly seconds ago and at full stick. Left alone, the
     * first driver command after a run would be slewed from that stale value. One zeroed call with
     * override=true bypasses the limiter and stores zero, which is the truth.
     */
    private void releaseDrivetrain() {
        if (RESET_HEADING_REFS) mecaTank.resetDriveHeading();
        mecaTank.setDrivePowersSmooth(0, 0, 0, 0, false, true);
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    public void telemetry() {
        telemetry.addData("Approach", "%s - %s", state, status);
        if (goalPoint != null) {
            telemetry.addData("Approach goal", "%.1f, %.1f", goalPoint.x, goalPoint.y);
        }
        if (isRunning()) {
            telemetry.addData("Approach t", "%.1f s  replans %d", runTimer.seconds(), replans);
        }
    }
}
