package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.Alliance;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.HiveSide;

import java.util.Collections;
import java.util.List;

/**
 * The aiming brain: turns odometry plus camera detections into a turret angle, every loop, whether
 * or not a tag happens to be visible right now.
 *
 * <h2>The strategy, and why it is built this way</h2>
 * The obvious approach - "point the turret wherever the camera currently says the tag is" - fails
 * the moment the tag leaves the frame, which on a fixed chassis-mounted camera happens constantly
 * as the robot drives and turns. The turret would snap back to centre, then lurch when the tag
 * reappeared.
 *
 * So this controller does something different. It uses vision to answer a question that has a
 * STABLE answer - "where is the CELL opening, in FIELD coordinates?" - and then aims by pure
 * geometry from the live odometry pose to that coordinate. A CELL does not move between hive tips,
 * so that field coordinate stays valid for seconds at a time, and the turret keeps tracking
 * smoothly through every occlusion. Vision then sharpens the aim whenever a tag IS visible.
 *
 * <h2>TWO HIVES PER ALLIANCE</h2>
 * Each alliance has two hives, roughly 24 in apart, both near the field centreline. This controller
 * therefore keeps TWO independent filtered estimates, one per hive, and picks between them - it
 * never averages them, because the midpoint of two hives is a wall.
 *
 * Both estimates start SEEDED AT CONFIRMED COORDINATES from {@link TurretConstants}, not at rough
 * guesses. That is a meaningful change in how much the blind feedforward can be trusted: the turret
 * now points accurately at a chosen hive from odometry alone, before the camera has ever seen it
 * and straight through arbitrarily long occlusions. Vision's job shrinks to correcting the last
 * degree or two, which is the part a monocular camera is actually good at.
 *
 * <h2>Why the camera is never allowed to touch the robot's pose</h2>
 * In BioBuzz the tags ride on the HIVE CELLs, which move, and FIRST states they are not usable for
 * field localization. The data flow here is strictly one-way: odometry -> aiming. Odometry owns the
 * global pose and nothing in this file writes to it.
 *
 * <h2>The steps, in order</h2>
 * <ol>
 *   <li><b>Timestamped pose history.</b> Every loop the current pose is pushed into a ring buffer
 *       with a timestamp. When a detection arrives it is paired with the pose from when the SHUTTER
 *       OPENED, not the current one. Skipping this is the single most common way to get a target
 *       estimate that smears: at 30 in/s with 80 ms of camera latency, using the current pose puts
 *       the target 2.4 in from where it really is, and worse while turning.</li>
 *   <li><b>Back-project each detection into a field coordinate.</b> Camera frame -> robot frame
 *       (through the camera mount pose) -> field frame (through the back-dated robot pose).</li>
 *   <li><b>Associate it to one of the two hives</b>, by cluster name when the name-to-hive mapping
 *       has been filled in, otherwise by whichever hive estimate it lands nearest to.</li>
 *   <li><b>Filter that hive only.</b> A low-pass average, because the opening is quasi-static and
 *       averaging is what beats down the noisy monocular range axis. An innovation gate catches a
 *       genuine move - a hive tip - and resets rather than sliding the estimate across the field.
 *       The estimate freezes entirely while that hive is mid-tip.</li>
 *   <li><b>Choose the active hive</b> - see {@link #chooseActiveHive}.</li>
 *   <li><b>Aim from live odometry geometry</b> to the active hive's estimate, every loop, visible
 *       or not. This is the primary command.</li>
 *   <li><b>Trim with the live bearing</b> of the ACTIVE hive's detection when there is one. A
 *       monocular camera measures ANGLE well and RANGE badly, so the bearing is the part worth
 *       trusting frame-to-frame. It is latency-compensated for how far the robot has yawed since
 *       capture, capped, and low-passed, so one bad frame cannot swing the turret.</li>
 * </ol>
 *
 * <h2>Never fire on feedforward alone</h2>
 * {@link #readyToFire()} requires a live, fresh, alliance-matched, scorable, non-transitioning
 * detection OF THE ACTIVE HIVE. The estimates are good enough to keep the turret TRACKING through a
 * blind gap; they are not good enough to prove the opening is still facing up, and a flipped cell
 * looks identical to a scorable one from odometry.
 */
public class TurretAimController {

    /** Where the current aim point came from. Telemetry, and a sanity check while debugging. */
    public enum AimSource {
        /** A tag on the active hive is visible right now and its bearing is trimming the command. */
        LIVE_BEARING,
        /** Flying on the active hive's filtered estimate; no tag visible this instant. */
        ESTIMATE,
        /** The active hive's estimate has never been corrected by vision - still on its confirmed
         *  constant. Accurate, unlike the old rough seed, but unverified this match. */
        SEED,
        /** Coarse last resort: no alliance was ever selected, so one was guessed from field half. */
        FIELD_HALF,
        /** Nothing to aim at at all - turret held where it is. */
        NONE
    }

    /**
     * One hive's filtered field-coordinate estimate.
     *
     * Deliberately a small mutable holder rather than a record: it is updated in place every loop
     * and there are exactly two of them for the life of the opmode, so allocating a new one per
     * frame would be pure garbage for no benefit.
     */
    public static final class HiveEstimate {
        public final HiveSide side;
        /** Filtered opening position, FIELD INCHES. */
        double x, y;
        /** False while still sitting on the confirmed constant, i.e. vision has never corrected it. */
        boolean correctedByVision = false;
        /** When a detection last updated this estimate. */
        long updatedNanos = 0L;
        /** True when the last update snapped through the innovation gate rather than smoothing. */
        boolean justReset = false;
        /** The detection associated with this hive THIS loop, or null. Not owned; just referenced. */
        TurretTarget liveTarget = null;

        HiveEstimate(HiveSide side) {
            this.side = side;
        }

        /** Filtered opening position, FIELD INCHES. */
        public double getX() {
            return x;
        }

        /** Filtered opening position, FIELD INCHES. */
        public double getY() {
            return y;
        }

        /** Which hive this is. */
        public HiveSide getSide() {
            return side;
        }

        /** True once vision has corrected this estimate; false while still on the constant. */
        public boolean isCorrectedByVision() {
            return correctedByVision;
        }

        /** The detection associated with this hive this loop, or null. */
        public TurretTarget getLiveTarget() {
            return liveTarget;
        }

        /** True when this hive has a fresh, scorable, non-transitioning detection right now. */
        public boolean isLive() {
            return hasLiveFix();
        }

        /** Seconds since a detection last updated this estimate, or NaN if never. */
        public double getAgeSeconds() {
            return ageSeconds();
        }

        /** Seconds since a detection last updated this estimate, or NaN if never. */
        double ageSeconds() {
            if (!correctedByVision) return Double.NaN;
            return (System.nanoTime() - updatedNanos) / 1e9;
        }

        /** True when this hive has a usable detection right now. */
        boolean hasLiveFix() {
            return liveTarget != null
                    && liveTarget.scorable
                    && !liveTarget.transitioning
                    && liveTarget.ageSeconds() <= TurretConstants.BEARING_MAX_LATENCY_S;
        }
    }

    private final Turret turret;
    private final Telemetry telemetry;

    private Alliance alliance;

    // ---- Step 1: pose history ring buffer --------------------------------------------------------

    private final long[] histNanos = new long[TurretConstants.POSE_HISTORY_SIZE];
    private final double[] histX = new double[TurretConstants.POSE_HISTORY_SIZE];
    private final double[] histY = new double[TurretConstants.POSE_HISTORY_SIZE];
    private final double[] histHeading = new double[TurretConstants.POSE_HISTORY_SIZE];
    private int histWrite = 0;
    private int histCount = 0;

    /** True when the last detection's capture time fell outside the buffer. Telemetry only. */
    private boolean lastLookupMissed = false;

    // ---- The two hive estimates ------------------------------------------------------------------

    private final HiveEstimate hiveLeft = new HiveEstimate(HiveSide.LEFT);
    private final HiveEstimate hiveRight = new HiveEstimate(HiveSide.RIGHT);

    /** Which hive the turret is currently committed to. */
    private HiveSide activeSide = HiveSide.RIGHT;

    /** Driver override. Null means automatic selection. */
    private HiveSide overrideSide = null;

    /** Hysteresis state: the hive that has been winning, and since when. */
    private HiveSide pendingSide = null;
    private long pendingSinceNanos = 0L;

    // ---- Command state ---------------------------------------------------------------------------

    private double commandedAngleDeg = 0.0;
    private double unclampedCommandDeg = 0.0;
    private double aimErrorDeg = Double.NaN;
    private AimSource aimSource = AimSource.NONE;

    private double trimFilteredDeg = 0.0;

    // Encoder-path PID state.
    private double pidLastErrorDeg = 0.0;
    private double pidIntegralDegS = 0.0;
    private long pidLastNanos = 0L;

    private boolean needsRobotRotation = false;
    private double desiredRobotHeadingRad = 0.0;

    private double targetDistanceIn = Double.NaN;

    /** Field coordinate actually aimed at this loop, after any lead. Telemetry only. */
    private double aimPointX = 0.0;
    private double aimPointY = 0.0;

    /**
     * @param turret    the hardware layer this controller commands
     * @param alliance  the alliance locked in during init; may be null until selected
     * @param telemetry optional, for warnings only
     */
    public TurretAimController(Turret turret, Alliance alliance, Telemetry telemetry) {
        this.turret = turret;
        this.telemetry = telemetry;
        setAlliance(alliance);
    }

    /**
     * Sets the alliance and RE-SEEDS both hive estimates to that alliance's confirmed coordinates.
     * Call during init, before start.
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        seedEstimates();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /**
     * Resets both hive estimates to the confirmed constants for the current alliance and marks them
     * as not yet corrected by vision.
     *
     * Note what this does NOT do: it does not blank them. Because the constants are surveyed
     * opening centres rather than rough guesses, a freshly seeded estimate is still accurate enough
     * to aim and shoot on, so losing vision is a loss of precision rather than a loss of capability.
     */
    public void seedEstimates() {
        if (alliance == Alliance.BLUE) {
            hiveLeft.x = TurretConstants.BLUE_LEFT_X_IN;
            hiveLeft.y = TurretConstants.BLUE_LEFT_Y_IN;
            hiveRight.x = TurretConstants.BLUE_RIGHT_X_IN;
            hiveRight.y = TurretConstants.BLUE_RIGHT_Y_IN;
        } else {
            // RED, and also the null-alliance case: red's coordinates are as good a placeholder as
            // any, and the FIELD_HALF fallback below will pick a side for a null alliance anyway.
            hiveLeft.x = TurretConstants.RED_LEFT_X_IN;
            hiveLeft.y = TurretConstants.RED_LEFT_Y_IN;
            hiveRight.x = TurretConstants.RED_RIGHT_X_IN;
            hiveRight.y = TurretConstants.RED_RIGHT_Y_IN;
        }
        hiveLeft.correctedByVision = false;
        hiveRight.correctedByVision = false;
        hiveLeft.liveTarget = null;
        hiveRight.liveTarget = null;
        pendingSide = null;
    }

    /** Alias kept for callers written against the single-target version. */
    public void clearEstimate() {
        seedEstimates();
    }

    // =============================================================================================
    // Driver override
    // =============================================================================================

    /**
     * Forces the turret onto one hive, ignoring automatic selection.
     *
     * @param side the hive to commit to, or null to hand control back to automatic selection
     */
    public void setHiveOverride(HiveSide side) {
        this.overrideSide = side;
        if (side != null) {
            activeSide = side;
            pendingSide = null;
        }
    }

    /** The current manual override, or null when selection is automatic. */
    public HiveSide getHiveOverride() {
        return overrideSide;
    }

    /** Clears the manual override. */
    public void clearHiveOverride() {
        this.overrideSide = null;
        this.pendingSide = null;
    }

    // =============================================================================================
    // Main entry point
    // =============================================================================================

    /** Single-target convenience overload. Prefer the list form so both hives stay fed. */
    public void update(Pose2d robotPose, double robotAngVelRadPerSec, TurretTarget target) {
        update(robotPose, robotAngVelRadPerSec, 0.0, 0.0,
                (target == null) ? Collections.<TurretTarget>emptyList()
                                 : Collections.singletonList(target));
    }

    /**
     * Runs one control iteration. Call every loop, whether or not anything is visible.
     *
     * @param robotPose            current field pose from the odometry localizer (inches, radians)
     * @param robotAngVelRadPerSec chassis angular velocity, RADIANS/SECOND, CCW-positive, from the
     *                             same localizer update. Used to latency-compensate the camera
     *                             bearing; pass 0 if unavailable and the trim simply will not be
     *                             corrected for rotation.
     * @param targets              every valid alliance cluster this loop, from
     *                             {@link TurretVision#getAllianceTargets}. May be empty; may hold
     *                             one entry per hive.
     */
    public void update(Pose2d robotPose, double robotAngVelRadPerSec, List<TurretTarget> targets) {
        update(robotPose, robotAngVelRadPerSec, 0.0, 0.0, targets);
    }

    /**
     * Runs one control iteration, with the chassis velocity supplied so the shot can be LED while
     * the robot is moving.
     *
     * The lead is only applied when {@link TurretConstants#PROJECTILE_FLIGHT_TIME_S} is non-zero,
     * which it is not until someone measures it. A guessed flight time makes aiming worse, not
     * better, so the default is off.
     *
     * @param robotVelFieldXInPerSec chassis velocity along FIELD x, INCHES/SECOND
     * @param robotVelFieldYInPerSec chassis velocity along FIELD y, INCHES/SECOND
     */
    public void update(Pose2d robotPose,
                       double robotAngVelRadPerSec,
                       double robotVelFieldXInPerSec,
                       double robotVelFieldYInPerSec,
                       List<TurretTarget> targets) {

        final long now = System.nanoTime();

        if (robotPose == null) {
            // No pose means no geometry. Hold the turret where it is rather than swinging it.
            aimSource = AimSource.NONE;
            aimErrorDeg = Double.NaN;
            return;
        }

        final double robotX = robotPose.position.x;
        final double robotY = robotPose.position.y;
        final double robotHeadingRad = robotPose.heading.toDouble();

        // ---- Step 1: record this pose, timestamped ----------------------------------------------
        pushPose(now, robotX, robotY, robotHeadingRad);

        // ---- Steps 2-4: associate each detection to a hive and filter that hive -----------------
        hiveLeft.liveTarget = null;
        hiveRight.liveTarget = null;
        hiveLeft.justReset = false;
        hiveRight.justReset = false;

        if (targets != null) {
            for (int i = 0; i < targets.size(); i++) {
                TurretTarget t = targets.get(i);
                if (t == null || t.alliance != alliance) continue;

                // Back-project first: the field coordinate is needed both to associate the
                // detection and to update the estimate, so it is computed once.
                double[] measured = backProject(t, robotX, robotY, robotHeadingRad);
                if (measured == null) continue;

                HiveEstimate hive = associate(t, measured[0], measured[1]);

                // Two detections can map to the same hive if the name mapping is empty and both
                // land near it. Keep the better-covered one, whose pose solution is stronger.
                if (hive.liveTarget == null || t.percentFound > hive.liveTarget.percentFound) {
                    hive.liveTarget = t;
                }

                // A TRANSITIONING target is deliberately NOT ingested: the hive is rotating, so its
                // pose solution describes a cell that is halfway to somewhere else. Freezing the
                // estimate keeps the turret on the last known-good opening until the tip finishes.
                if (t.usableForEstimate()) {
                    applyMeasurement(hive, measured[0], measured[1], now);
                }
            }
        }

        // ---- Step 5: choose which hive to shoot at ----------------------------------------------
        HiveEstimate active = chooseActiveHive(robotX, robotY, now);

        // ---- Step 6: aim from live odometry geometry --------------------------------------------
        double aimX = active.x;
        double aimY = active.y;

        // Optional lead for shooting on the move: aim at where the target will be RELATIVE TO THE
        // ROBOT once the ball arrives. The target is static, so the whole correction is the robot's
        // own displacement during the flight, subtracted from the target position.
        double leadT = TurretConstants.PROJECTILE_FLIGHT_TIME_S;
        if (leadT > 0.0) {
            aimX -= robotVelFieldXInPerSec * leadT;
            aimY -= robotVelFieldYInPerSec * leadT;
        }
        aimPointX = aimX;
        aimPointY = aimY;

        double fieldBearingRad = Math.atan2(aimY - robotY, aimX - robotX);
        double geometryAngleDeg = normalizeDeg(
                Math.toDegrees(fieldBearingRad - robotHeadingRad) - TurretConstants.TURRET_MOUNT_OFFSET_DEG);

        // Aim source, before the live-bearing trim possibly upgrades it.
        if (alliance == null) {
            aimSource = AimSource.FIELD_HALF;
        } else if (active.correctedByVision) {
            aimSource = AimSource.ESTIMATE;
        } else {
            aimSource = AimSource.SEED;
        }

        // Distance for the launch velocity table, from the ACTIVE hive.
        targetDistanceIn = computeTargetDistance(active, robotX, robotY, aimX, aimY);

        // ---- Step 7: trim with the ACTIVE hive's live bearing -----------------------------------
        TurretTarget liveTarget = active.liveTarget;
        boolean liveBearingUsable = active.hasLiveFix();

        double desiredFromBearingDeg = Double.NaN;

        if (liveBearingUsable) {
            // Latency compensation. The bearing was measured when the shutter opened; the robot has
            // yawed by angVel * dt since. A target that was 10 degrees left of the optical axis is
            // only 4 degrees left after the robot has turned 6 degrees to the left, hence the
            // subtraction.
            double dt = liveTarget.ageSeconds();
            double bearingNowDeg = liveTarget.bearingDeg - Math.toDegrees(robotAngVelRadPerSec * dt);

            // The bearing is measured from the CAMERA's optical axis; the camera sits
            // CAMERA_MOUNT_YAW_DEG from robot forward, so adding it puts the bearing in the robot
            // frame. Subtracting the turret mount offset then puts it in the turret's own frame.
            double bearingRobotDeg = bearingNowDeg + TurretConstants.CAMERA_MOUNT_YAW_DEG;
            desiredFromBearingDeg = normalizeDeg(bearingRobotDeg - TurretConstants.TURRET_MOUNT_OFFSET_DEG);

            double correctionDeg;
            double measured = turret.getMeasuredAngleDeg();

            if (!Double.isNaN(measured)) {
                // CLOSED LOOP. The encoder reports where the turret truly is, so drive that error
                // to zero with a real PID. This is the only configuration in which servo lag, slop
                // and the gearbox backlash are actually corrected rather than assumed away.
                double errorDeg = normalizeDeg(desiredFromBearingDeg - measured);
                double dtPid = (pidLastNanos == 0L) ? 0.02 : (now - pidLastNanos) / 1e9;
                if (dtPid <= 0.0 || dtPid > 0.5) dtPid = 0.02; // first loop, or a stall
                pidLastNanos = now;

                pidIntegralDegS += errorDeg * dtPid;
                double iTerm = TurretConstants.TURRET_KI * pidIntegralDegS;
                if (iTerm > TurretConstants.TURRET_MAX_I_DEG) {
                    iTerm = TurretConstants.TURRET_MAX_I_DEG;
                    pidIntegralDegS = (TurretConstants.TURRET_KI == 0.0) ? 0.0
                            : TurretConstants.TURRET_MAX_I_DEG / TurretConstants.TURRET_KI;
                } else if (iTerm < -TurretConstants.TURRET_MAX_I_DEG) {
                    iTerm = -TurretConstants.TURRET_MAX_I_DEG;
                    pidIntegralDegS = (TurretConstants.TURRET_KI == 0.0) ? 0.0
                            : -TurretConstants.TURRET_MAX_I_DEG / TurretConstants.TURRET_KI;
                }

                double dTerm = TurretConstants.TURRET_KD * (errorDeg - pidLastErrorDeg) / dtPid;
                pidLastErrorDeg = errorDeg;

                correctionDeg = TurretConstants.TURRET_KP * errorDeg + iTerm + dTerm;
            } else {
                // OPEN LOOP. There is no measurement of where the turret went, so the only honest
                // correction is the difference between what the camera says and what the geometry
                // says, applied at a fraction of full strength. The gain is capped deliberately:
                // now that the hive coordinates are surveyed, geometry is the MORE trustworthy of
                // the two signals and vision is the adjustment, not the other way round.
                double errorDeg = normalizeDeg(desiredFromBearingDeg - geometryAngleDeg);
                correctionDeg = TurretConstants.BEARING_TRIM_GAIN * errorDeg;
                pidIntegralDegS = 0.0;
                pidLastErrorDeg = 0.0;
                pidLastNanos = now;
            }

            // Cap before filtering, so one wild frame cannot load the filter with a huge value.
            correctionDeg = clamp(correctionDeg,
                    -TurretConstants.BEARING_TRIM_MAX_DEG, TurretConstants.BEARING_TRIM_MAX_DEG);

            trimFilteredDeg = TurretConstants.BEARING_TRIM_ALPHA * correctionDeg
                    + (1.0 - TurretConstants.BEARING_TRIM_ALPHA) * trimFilteredDeg;

            aimSource = AimSource.LIVE_BEARING;
        } else {
            // No usable bearing: bleed the trim away instead of holding a stale correction, which
            // would otherwise bias the geometry command indefinitely after the tag disappeared.
            trimFilteredDeg *= (1.0 - TurretConstants.BEARING_TRIM_ALPHA);
            pidIntegralDegS = 0.0;
            pidLastNanos = now;
        }

        // ---- Command, limits, deadband -----------------------------------------------------------
        unclampedCommandDeg = normalizeDeg(geometryAngleDeg + trimFilteredDeg);

        // Outside the turret's travel the chassis has to help. Report the heading the robot would
        // need so that the turret's mechanical zero points at the target - the driver or an auto
        // routine can then rotate to it.
        needsRobotRotation = unclampedCommandDeg > TurretConstants.TURRET_MAX_ANGLE_DEG
                || unclampedCommandDeg < TurretConstants.TURRET_MIN_ANGLE_DEG;
        desiredRobotHeadingRad = normalizeRad(
                fieldBearingRad - Math.toRadians(TurretConstants.TURRET_MOUNT_OFFSET_DEG));

        double clampedCommandDeg = Turret.clampAngle(unclampedCommandDeg);

        // Deadband: inside this the turret is not re-commanded at all. A servo fed a stream of
        // sub-degree corrections buzzes, draws current and heats up without ever moving usefully.
        if (Math.abs(clampedCommandDeg - commandedAngleDeg) >= TurretConstants.AIM_DEADBAND_DEG) {
            commandedAngleDeg = turret.setOutputAngleDeg(clampedCommandDeg);
        }

        // ---- Aim error ---------------------------------------------------------------------------
        // When a tag is visible, the honest error is vision's: how far the turret is from where the
        // camera says the target actually is. Without a tag the best available number is how far
        // the turret is from the commanded angle, which is zero in open loop but correctly non-zero
        // whenever a travel limit is stopping the turret from reaching the command.
        double turretNow = turret.getAngleDeg();
        if (liveBearingUsable && !Double.isNaN(desiredFromBearingDeg)) {
            aimErrorDeg = normalizeDeg(desiredFromBearingDeg - turretNow);
        } else {
            aimErrorDeg = normalizeDeg(unclampedCommandDeg - turretNow);
        }
    }

    // =============================================================================================
    // Hive association and selection
    // =============================================================================================

    /**
     * Decides which hive a detection belongs to.
     *
     * <h3>By name when possible</h3>
     * If the name-to-hive mapping in {@link TurretConstants} has been filled in and this cluster's
     * name matches one of the two entries for the active alliance, that is definitive and is used.
     * Name matching does not degrade with distance, which matters because the hives are only ~24 in
     * apart and monocular range noise grows with range.
     *
     * <h3>By proximity otherwise</h3>
     * With the mapping empty, the detection is assigned to whichever hive estimate its
     * back-projected field coordinate lands nearest to. This is reliable in practice because both
     * estimates START at surveyed coordinates 24 in apart, so a detection has to be more than 12 in
     * wrong before it is mis-assigned - and a detection that wrong would be caught by the
     * innovation gate anyway.
     */
    private HiveEstimate associate(TurretTarget t, double measX, double measY) {
        String leftName = (alliance == Alliance.BLUE)
                ? TurretConstants.BLUE_LEFT_CLUSTER_NAME : TurretConstants.RED_LEFT_CLUSTER_NAME;
        String rightName = (alliance == Alliance.BLUE)
                ? TurretConstants.BLUE_RIGHT_CLUSTER_NAME : TurretConstants.RED_RIGHT_CLUSTER_NAME;

        if (nameMatches(t.name, leftName)) return hiveLeft;
        if (nameMatches(t.name, rightName)) return hiveRight;

        double dLeft = Math.hypot(measX - hiveLeft.x, measY - hiveLeft.y);
        double dRight = Math.hypot(measX - hiveRight.x, measY - hiveRight.y);
        return (dLeft <= dRight) ? hiveLeft : hiveRight;
    }

    /** Case-insensitive, whitespace-tolerant match. An empty mapping entry never matches. */
    private static boolean nameMatches(String detected, String configured) {
        if (configured == null || configured.trim().isEmpty()) return false;
        if (detected == null) return false;
        return detected.trim().equalsIgnoreCase(configured.trim());
    }

    /**
     * Picks which hive the turret commits to this loop.
     *
     * <p>Order of precedence:</p>
     * <ol>
     *   <li><b>Driver override</b> ({@link #setHiveOverride}) wins outright, with no hysteresis. A
     *       driver pressing a button wants the turret to move now.</li>
     *   <li><b>Nearest hive with a live scorable fix.</b> This is the real selector. Preferring a
     *       hive we can actually SEE over one we merely believe in means the shot is taken with a
     *       verified opening, and among the visible ones, nearest is the easier shot.</li>
     *   <li><b>Nearest by estimate</b> when nothing is visible, so the turret keeps tracking
     *       through occlusion instead of parking.</li>
     * </ol>
     *
     * <p>Hysteresis guards every automatic switch. A challenger must be closer than the incumbent
     * by {@link TurretConstants#HIVE_SWITCH_HYSTERESIS_IN} AND stay that way for
     * {@link TurretConstants#HIVE_SWITCH_DWELL_S}. Without this a robot parked midway between two
     * hives 24 in apart would flip the turret back and forth on range noise alone, and each flip
     * costs a full servo sweep and a re-spin-up.</p>
     *
     * <p>The FIELD-HALF rule is NOT used here. It is a coarse last resort for a missing alliance
     * only, and it is close to useless for choosing between two hives that both sit near the field
     * centreline about 24 in apart.</p>
     */
    private HiveEstimate chooseActiveHive(double robotX, double robotY, long now) {
        if (overrideSide != null) {
            activeSide = overrideSide;
            return hiveOf(activeSide);
        }

        double dLeft = Math.hypot(hiveLeft.x - robotX, hiveLeft.y - robotY);
        double dRight = Math.hypot(hiveRight.x - robotX, hiveRight.y - robotY);

        boolean leftLive = hiveLeft.hasLiveFix();
        boolean rightLive = hiveRight.hasLiveFix();

        HiveSide preferred;
        if (leftLive && !rightLive) {
            preferred = HiveSide.LEFT;
        } else if (rightLive && !leftLive) {
            preferred = HiveSide.RIGHT;
        } else {
            // Both visible, or neither: fall through to nearest.
            preferred = (dLeft <= dRight) ? HiveSide.LEFT : HiveSide.RIGHT;
        }

        if (preferred == activeSide) {
            pendingSide = null;
            return hiveOf(activeSide);
        }

        // A challenger has to be clearly better AND consistently better.
        double dActive = (activeSide == HiveSide.LEFT) ? dLeft : dRight;
        double dChallenger = (preferred == HiveSide.LEFT) ? dLeft : dRight;

        boolean seesOnlyChallenger =
                (preferred == HiveSide.LEFT && leftLive && !rightLive)
                || (preferred == HiveSide.RIGHT && rightLive && !leftLive);

        // Visibility overrules distance: if the active hive is not visible and the other one is,
        // the distance margin is not required, only the dwell. Holding a turret on a hive we cannot
        // see while a perfectly good one is in frame is the wrong trade.
        boolean marginMet = seesOnlyChallenger
                || (dChallenger < dActive - TurretConstants.HIVE_SWITCH_HYSTERESIS_IN);

        if (!marginMet) {
            pendingSide = null;
            return hiveOf(activeSide);
        }

        if (pendingSide != preferred) {
            pendingSide = preferred;
            pendingSinceNanos = now;
            return hiveOf(activeSide);
        }

        if ((now - pendingSinceNanos) / 1e9 >= TurretConstants.HIVE_SWITCH_DWELL_S) {
            activeSide = preferred;
            pendingSide = null;
        }
        return hiveOf(activeSide);
    }

    private HiveEstimate hiveOf(HiveSide side) {
        return (side == HiveSide.LEFT) ? hiveLeft : hiveRight;
    }

    // =============================================================================================
    // Step 2: back-projection, and step 4: per-hive filtering
    // =============================================================================================

    /**
     * Turns one detection into a FIELD coordinate.
     *
     * @return {fieldX, fieldY} in inches, or null when it could not be computed
     */
    private double[] backProject(TurretTarget target,
                                 double currentX, double currentY, double currentHeading) {

        // Pair the detection with the pose from when the SHUTTER OPENED. See the class comment.
        double[] poseAtCapture = lookupPose(target.frameAcquisitionNanoTime);
        double px, py, ph;
        if (poseAtCapture == null) {
            lastLookupMissed = true;
            px = currentX;
            py = currentY;
            ph = currentHeading;
        } else {
            lastLookupMissed = false;
            px = poseAtCapture[0];
            py = poseAtCapture[1];
            ph = poseAtCapture[2];
        }

        // ---- Camera frame -> robot frame ---------------------------------------------------------
        //
        // ftcPose axes are +x RIGHT, +y FORWARD, +z UP. The robot frame is +x FORWARD, +y LEFT,
        // +z UP. Re-labelling the axes is therefore the first step, before any rotation:
        double pFwd  =  target.camY;
        double pLeft = -target.camX;
        double pUp   =  target.camZ;

        // Now rotate by the camera's mount orientation, R = Rz(yaw) * Ry(pitch) * Rx(roll), about
        // the robot's up, left and forward axes respectively. Yaw dominates - an error in it
        // rotates the whole target estimate about the robot, so at 96 in a 2-degree yaw error is
        // roughly 3.4 in of lateral error in the estimated target position.
        double yaw   = Math.toRadians(TurretConstants.CAMERA_MOUNT_YAW_DEG);
        double pitch = Math.toRadians(TurretConstants.CAMERA_MOUNT_PITCH_DEG);
        double roll  = Math.toRadians(TurretConstants.CAMERA_MOUNT_ROLL_DEG);

        // Rx(roll): about the forward axis.
        double r1Fwd  = pFwd;
        double r1Left = pLeft * Math.cos(roll) - pUp * Math.sin(roll);
        double r1Up   = pLeft * Math.sin(roll) + pUp * Math.cos(roll);

        // Ry(pitch): about the left axis; positive pitch tilts the forward axis UP.
        double r2Fwd  =  r1Fwd * Math.cos(pitch) + r1Up * Math.sin(pitch);
        double r2Left =  r1Left;
        double r2Up   = -r1Fwd * Math.sin(pitch) + r1Up * Math.cos(pitch);

        // Rz(yaw): about the up axis, CCW-positive.
        double rFwd  = r2Fwd * Math.cos(yaw) - r2Left * Math.sin(yaw);
        double rLeft = r2Fwd * Math.sin(yaw) + r2Left * Math.cos(yaw);
        // rUp is r2Up; unused because the turret aims in azimuth only, but kept in the comment so
        // the transform reads as the complete 3D rotation it is.

        // Translate by where the lens sits on the robot.
        double robotFrameX = TurretConstants.CAMERA_OFFSET_X_IN + rFwd;
        double robotFrameY = TurretConstants.CAMERA_OFFSET_Y_IN + rLeft;

        // ---- Robot frame -> field frame, using the BACK-DATED pose ------------------------------
        double cosH = Math.cos(ph), sinH = Math.sin(ph);
        double measX = px + robotFrameX * cosH - robotFrameY * sinH;
        double measY = py + robotFrameX * sinH + robotFrameY * cosH;

        return new double[]{measX, measY};
    }

    /**
     * Folds one measured field coordinate into a single hive's filtered estimate.
     *
     * Note the asymmetry with the old single-target version: an estimate is ALWAYS valid here,
     * because it starts seeded at a surveyed coordinate. So there is no "first measurement" branch
     * that adopts a reading wholesale - every measurement is either smoothed in or gated out, and
     * a single bad first frame can no longer capture the estimate.
     */
    private void applyMeasurement(HiveEstimate hive, double measX, double measY, long nowNanos) {
        double jump = Math.hypot(measX - hive.x, measY - hive.y);

        if (jump > TurretConstants.TARGET_INNOVATION_GATE_IN) {
            // INNOVATION GATE. This is not noise - it is a different place. Smoothing across it
            // would walk the estimate slowly through the empty field between the old position and
            // the new one, aiming at nothing at all for a second or more. Snap instead.
            //
            // With surveyed seeds this also acts as a sanity check on vision: a detection that
            // lands more than the gate away from a KNOWN hive position is more likely a bad pose
            // solution than a moved hive, and snapping keeps it from poisoning the average.
            hive.x = measX;
            hive.y = measY;
            hive.justReset = true;
        } else {
            // Heavy low-pass. The opening is quasi-static between tips, so averaging many frames is
            // close to free and it is what beats down the monocular range noise, which is where
            // nearly all the error lives.
            double a = TurretConstants.TARGET_ESTIMATE_ALPHA;
            hive.x = a * measX + (1.0 - a) * hive.x;
            hive.y = a * measY + (1.0 - a) * hive.y;
        }
        hive.correctedByVision = true;
        hive.updatedNanos = nowNanos;
    }

    // =============================================================================================
    // Step 1: pose history
    // =============================================================================================

    private void pushPose(long nanos, double x, double y, double heading) {
        histNanos[histWrite] = nanos;
        histX[histWrite] = x;
        histY[histWrite] = y;
        histHeading[histWrite] = heading;
        histWrite = (histWrite + 1) % TurretConstants.POSE_HISTORY_SIZE;
        if (histCount < TurretConstants.POSE_HISTORY_SIZE) histCount++;
    }

    /**
     * Finds the recorded pose closest in time to {@code captureNanos}.
     *
     * @return {x, y, heading} or null when the capture time falls outside the buffer's span, in
     *         which case the caller falls back to the current pose and flags it.
     */
    private double[] lookupPose(long captureNanos) {
        if (histCount == 0) return null;

        long maxLookbackNanos = (long) (TurretConstants.POSE_HISTORY_MAX_LOOKBACK_S * 1e9);
        if (System.nanoTime() - captureNanos > maxLookbackNanos) return null;

        int bestIndex = -1;
        long bestDelta = Long.MAX_VALUE;
        for (int i = 0; i < histCount; i++) {
            long delta = Math.abs(histNanos[i] - captureNanos);
            if (delta < bestDelta) {
                bestDelta = delta;
                bestIndex = i;
            }
        }
        if (bestIndex < 0) return null;
        return new double[]{histX[bestIndex], histY[bestIndex], histHeading[bestIndex]};
    }

    // =============================================================================================
    // Distance
    // =============================================================================================

    /**
     * Horizontal distance to the ACTIVE hive, INCHES - the number the launch velocity table is
     * indexed by.
     *
     * <h3>Why this is not simply ftcPose.range</h3>
     * {@code range} is the SLANT distance from the lens to the tag, which includes the height
     * difference between the camera and the goal. The odometry fallback below is a GROUND distance.
     * Indexing one lookup table with two different quantities would make the shot velocity jump
     * every time a tag came into or out of view, which is exactly the moment consistency matters
     * most. Both branches therefore return the horizontal distance: from the camera, that is
     * hypot of the forward and lateral components, dropping the vertical one.
     */
    private double computeTargetDistance(HiveEstimate active,
                                         double robotX, double robotY,
                                         double aimX, double aimY) {
        if (active.hasLiveFix()) {
            return Math.hypot(active.liveTarget.camX, active.liveTarget.camY);
        }
        // Always available now: the estimate is seeded at a surveyed coordinate, so there is no
        // "distance unknown" state once an alliance has been selected.
        return Math.hypot(aimX - robotX, aimY - robotY);
    }

    /**
     * Horizontal distance to the active hive, INCHES, for the launch velocity lookup.
     *
     * @return the live measurement when that hive is visible, otherwise the odometry distance to
     *         its filtered estimate. NaN only before the first {@link #update} call.
     */
    public double getTargetDistanceInches() {
        return targetDistanceIn;
    }

    // =============================================================================================
    // Firing gate and status
    // =============================================================================================

    /**
     * True when it is worth committing a ball.
     *
     * Requires ALL of: a live detection OF THE ACTIVE HIVE this loop, matching the selected
     * alliance, scorable (opening up), not mid-tip, fresh enough that its bearing still means
     * something, an aim error inside {@link TurretConstants#AIM_READY_TOLERANCE_DEG}, and a turret
     * that is not pinned against a travel limit.
     *
     * The live-detection requirement is the important one, and surveyed coordinates do NOT remove
     * it. Knowing exactly where a hive is says nothing about whether its opening is currently
     * facing up, and a flipped cell is a wall that looks identical from odometry.
     */
    public boolean readyToFire() {
        HiveEstimate active = hiveOf(activeSide);
        TurretTarget t = active.liveTarget;
        if (t == null) return false;
        if (t.alliance != alliance) return false;
        if (!t.scorable) return false;
        if (t.transitioning) return false;
        if (t.ageSeconds() > TurretConstants.BEARING_MAX_LATENCY_S) return false;
        if (Double.isNaN(aimErrorDeg)) return false;
        if (Math.abs(aimErrorDeg) > TurretConstants.AIM_READY_TOLERANCE_DEG) return false;
        if (turret.isAtHardStop()) return false;
        return true;
    }

    /**
     * Why {@link #readyToFire()} is false, in plain words, or "" when it is true.
     * Put this on telemetry - "blocked" with no reason is the hardest thing to debug on a field.
     */
    public String getNotReadyReason() {
        HiveEstimate active = hiveOf(activeSide);
        TurretTarget t = active.liveTarget;
        if (t == null) return "active hive " + activeSide + " not visible";
        if (t.alliance != alliance) return "target is other alliance";
        if (!t.scorable) return "cell flipped (roll " + Math.round(t.rollDeg) + ")";
        if (t.transitioning) return "hive transitioning";
        if (t.ageSeconds() > TurretConstants.BEARING_MAX_LATENCY_S) {
            return String.format("detection stale (%.0f ms)", t.ageSeconds() * 1000.0);
        }
        if (Double.isNaN(aimErrorDeg)) return "aim error unknown";
        if (Math.abs(aimErrorDeg) > TurretConstants.AIM_READY_TOLERANCE_DEG) {
            return String.format("aim off by %.1f deg", aimErrorDeg);
        }
        if (turret.isAtHardStop()) return "turret at hard stop";
        return "";
    }

    /**
     * True when the required turret angle is outside its travel and the CHASSIS must rotate for the
     * shot to be possible. Pair with {@link #getDesiredRobotHeadingRad()}.
     */
    public boolean needsRobotRotation() {
        return needsRobotRotation;
    }

    /**
     * The field heading, RADIANS, that would put the target inside the turret's travel - specifically
     * the heading at which the turret's mechanical zero points straight at the target.
     */
    public double getDesiredRobotHeadingRad() {
        return desiredRobotHeadingRad;
    }

    /** Same thing in degrees, for telemetry. */
    public double getDesiredRobotHeadingDeg() {
        return Math.toDegrees(desiredRobotHeadingRad);
    }

    // =============================================================================================
    // Getters for telemetry and the overlay
    // =============================================================================================

    /** The hive the turret is currently committed to. */
    public HiveSide getActiveHiveSide() {
        return activeSide;
    }

    /** The active hive's full estimate: side, field x/y, age, live detection. */
    public HiveEstimate getActiveTarget() {
        return hiveOf(activeSide);
    }

    /** The other hive, for telemetry and for deciding whether an override is worth offering. */
    public HiveEstimate getInactiveTarget() {
        return hiveOf((activeSide == HiveSide.LEFT) ? HiveSide.RIGHT : HiveSide.LEFT);
    }

    /** A hive by side. */
    public HiveEstimate getHive(HiveSide side) {
        return hiveOf(side);
    }

    /** Active hive's estimated field X, INCHES. */
    public double getTargetEstimateX() {
        return hiveOf(activeSide).x;
    }

    /** Active hive's estimated field Y, INCHES. */
    public double getTargetEstimateY() {
        return hiveOf(activeSide).y;
    }

    /** Seconds since vision last corrected the active hive, or NaN if it never has. */
    public double getTargetEstimateAgeS() {
        return hiveOf(activeSide).ageSeconds();
    }

    /** True once vision has corrected the active hive's estimate at least once. */
    public boolean hasTargetEstimate() {
        return hiveOf(activeSide).correctedByVision;
    }

    /**
     * True when the active hive's estimate has not been refreshed recently.
     *
     * Far less alarming than it used to be: a stale estimate now means "flying on surveyed
     * coordinates", which is accurate, rather than "flying on a rough guess".
     */
    public boolean isTargetEstimateStale() {
        double age = hiveOf(activeSide).ageSeconds();
        return Double.isNaN(age) || age > TurretConstants.TARGET_ESTIMATE_MAX_AGE_S;
    }

    /** True when the last ingest snapped the active hive through the innovation gate. */
    public boolean didEstimateJustReset() {
        return hiveOf(activeSide).justReset;
    }

    /** True when a detection's capture time could not be matched to a recorded pose. */
    public boolean didPoseLookupMiss() {
        return lastLookupMissed;
    }

    /** The active hive's live detection this loop, or null. */
    public TurretTarget getLastTarget() {
        return hiveOf(activeSide).liveTarget;
    }

    /** Where the aim command came from this loop. */
    public AimSource getAimSource() {
        return aimSource;
    }

    /** Last angle commanded to the turret, DEGREES, after clamping. */
    public double getCommandedAngleDeg() {
        return commandedAngleDeg;
    }

    /** The angle the geometry asked for before clamping, DEGREES. Differs when at a travel limit. */
    public double getUnclampedCommandDeg() {
        return unclampedCommandDeg;
    }

    /** Signed aim error, DEGREES, or NaN when unknown. See the note in {@link #update}. */
    public double getAimErrorDeg() {
        return aimErrorDeg;
    }

    /** Field point actually aimed at this loop (after any lead), X, INCHES. */
    public double getAimPointX() {
        return aimPointX;
    }

    /** Field point actually aimed at this loop (after any lead), Y, INCHES. */
    public double getAimPointY() {
        return aimPointY;
    }

    /**
     * Pushes this loop's aim state into the crosshair overlay, using the ACTIVE hive's detection.
     *
     * The turret bearing handed over is in the ROBOT frame, which is the turret's own angle plus
     * the mount offset - the overlay works in robot-frame angles because that is what it can
     * compare against the camera's mount yaw.
     */
    public void updateOverlay(TurretAimOverlay overlay) {
        if (overlay == null) return;
        double turretRobotFrameDeg = turret.getAngleDeg() + TurretConstants.TURRET_MOUNT_OFFSET_DEG;
        overlay.setAimState(turretRobotFrameDeg, aimErrorDeg);

        TurretTarget t = hiveOf(activeSide).liveTarget;
        if (t != null && t.alliance == alliance && t.scorable) {
            overlay.setTarget(t.camX, t.camY, t.camZ);
        } else {
            overlay.clearTarget();
        }
    }

    /** Adds the full aim picture to telemetry. */
    public void telemetry() {
        if (telemetry == null) return;
        HiveEstimate active = hiveOf(activeSide);
        HiveEstimate other = getInactiveTarget();

        telemetry.addData("Active hive", "%s%s", activeSide,
                (overrideSide != null) ? " (DRIVER OVERRIDE)" : "");
        telemetry.addData("Aim source", aimSource);
        telemetry.addData("Aim error (deg)", Double.isNaN(aimErrorDeg) ? "--"
                : String.format("%.2f", aimErrorDeg));
        telemetry.addData("Turret cmd (deg)", "%.1f (raw %.1f)", commandedAngleDeg, unclampedCommandDeg);

        telemetry.addData(" hive " + active.side, "(%.1f, %.1f) %s%s",
                active.x, active.y,
                active.correctedByVision ? String.format("age %.2fs", active.ageSeconds()) : "SEED",
                active.hasLiveFix() ? " LIVE" : "");
        telemetry.addData(" hive " + other.side, "(%.1f, %.1f) %s%s",
                other.x, other.y,
                other.correctedByVision ? String.format("age %.2fs", other.ageSeconds()) : "SEED",
                other.hasLiveFix() ? " LIVE" : "");
        if (pendingSide != null) {
            telemetry.addData(" switch pending", "%s in %.2fs", pendingSide,
                    Math.max(0.0, TurretConstants.HIVE_SWITCH_DWELL_S
                            - (System.nanoTime() - pendingSinceNanos) / 1e9));
        }

        telemetry.addData("Target distance (in)", Double.isNaN(targetDistanceIn) ? "--"
                : String.format("%.1f", targetDistanceIn));
        telemetry.addData("Needs robot rotation", needsRobotRotation
                ? String.format("YES -> heading %.1f deg", getDesiredRobotHeadingDeg()) : "no");
    }

    // =============================================================================================
    // Math helpers
    // =============================================================================================

    /** Wraps an angle in DEGREES into [-180, 180). */
    public static double normalizeDeg(double deg) {
        double d = (deg + 180.0) % 360.0;
        if (d < 0) d += 360.0;
        return d - 180.0;
    }

    /** Wraps an angle in RADIANS into [-pi, pi). */
    public static double normalizeRad(double rad) {
        double r = (rad + Math.PI) % (2.0 * Math.PI);
        if (r < 0) r += 2.0 * Math.PI;
        return r - Math.PI;
    }

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

    private static Alliance otherAlliance(Alliance a) {
        return (a == Alliance.RED) ? Alliance.BLUE : Alliance.RED;
    }
}
