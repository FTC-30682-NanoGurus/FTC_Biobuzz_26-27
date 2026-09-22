package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.Alliance;

/**
 * One immutable snapshot of a BioBuzz HIVE CELL cluster, as seen in a single camera frame.
 *
 * <h2>Why this type exists instead of passing the SDK's detection around</h2>
 * An {@code AprilTagClusterDetection} is a live object owned by the vision thread, and reading it
 * from the opmode thread a few milliseconds later can hand you a half-updated pose. Everything the
 * aim controller needs is copied out once, under the vision lock, into this final-fields object,
 * which is then safe to hold for as long as you like.
 *
 * <h2>Why it carries BOTH a bearing and a raw camera-frame position</h2>
 * Those two are used for different jobs and have very different error characteristics:
 * <ul>
 *   <li>{@link #bearingDeg} is an ANGLE, and monocular angle measurements are good - it comes
 *       almost directly from where the tag corners landed on the sensor. This is what sharpens the
 *       aim when the tag is visible.</li>
 *   <li>{@link #camX}/{@link #camY}/{@link #camZ} include the RANGE axis, which a single camera
 *       estimates from apparent tag size and therefore estimates badly. These are used to
 *       back-project the target into a FIELD coordinate, where heavy filtering over many frames
 *       averages that range noise down.</li>
 * </ul>
 *
 * <h2>Why the capture timestamp is mandatory</h2>
 * {@link #frameAcquisitionNanoTime} is when the SHUTTER opened, not when the opmode read the
 * result. The gap is tens of milliseconds, and the robot moves during it. Every consumer of this
 * object must pair it with the robot pose at THAT time, not the current one.
 */
public final class TurretTarget {

    /** Cluster name straight from {@code metadata.name}, e.g. the authoritative alliance source. */
    public final String name;

    /** Alliance this cluster belongs to, decoded from the first character of {@link #name}. */
    public final Alliance alliance;

    /**
     * Horizontal angle from the CAMERA's optical axis to the cluster centre, DEGREES,
     * CCW-positive (target to the LEFT of the optical axis is positive), as the SDK reports it.
     * Accurate. This is the measurement the aim trim leans on.
     */
    public final double bearingDeg;

    /**
     * Straight-line distance from the camera to the cluster centre, INCHES.
     * Noisy - monocular range from a 3.25 in tag is the weakest number in the whole detection.
     */
    public final double rangeIn;

    /**
     * Cluster centre in the CAMERA frame, INCHES, in the SDK's {@code ftcPose} convention:
     * +x RIGHT, +y FORWARD (out of the lens), +z UP.
     * NOTE this is NOT the OpenCV optical convention (+x right, +y DOWN, +z forward); anything
     * projecting these to pixels must convert first.
     */
    public final double camX;
    public final double camY;
    public final double camZ;

    /**
     * Cluster roll, DEGREES. This is the tip state of the HIVE CELL: the opening faces up, and the
     * cell is therefore scorable, while {@code abs(roll) < 90}.
     */
    public final double rollDeg;

    /**
     * Fraction of the cluster's four tags currently decoded, normalised to PERCENT (0-100).
     * Higher coverage means a better-conditioned pose solution, so this doubles as a quality score.
     */
    public final double percentFound;

    /** True when {@code abs(rollDeg)} is inside {@link TurretConstants#SCORABLE_MAX_ROLL_DEG}. */
    public final boolean scorable;

    /**
     * True when the hive appears to be MID-TIP: roll is near the +-90 flip point, or coverage just
     * collapsed between frames. While this is set, the aim controller freezes its target estimate
     * and refuses to fire, because a rotating cell has a meaningless pose and a closing opening.
     */
    public final boolean transitioning;

    /**
     * {@code System.nanoTime()}-comparable timestamp of when this frame was CAPTURED.
     * Copied from the SDK detection; used to look up the robot pose at capture time.
     */
    public final long frameAcquisitionNanoTime;

    public TurretTarget(String name,
                        Alliance alliance,
                        double bearingDeg,
                        double rangeIn,
                        double camX,
                        double camY,
                        double camZ,
                        double rollDeg,
                        double percentFound,
                        boolean scorable,
                        boolean transitioning,
                        long frameAcquisitionNanoTime) {
        this.name = name;
        this.alliance = alliance;
        this.bearingDeg = bearingDeg;
        this.rangeIn = rangeIn;
        this.camX = camX;
        this.camY = camY;
        this.camZ = camZ;
        this.rollDeg = rollDeg;
        this.percentFound = percentFound;
        this.scorable = scorable;
        this.transitioning = transitioning;
        this.frameAcquisitionNanoTime = frameAcquisitionNanoTime;
    }

    /**
     * Age of this detection right now, SECONDS, measured from when the shutter opened.
     * Used to decide whether the live bearing is still fresh enough to trim with.
     */
    public double ageSeconds() {
        return (System.nanoTime() - frameAcquisitionNanoTime) / 1e9;
    }

    /**
     * True when this target is worth building a field-coordinate estimate from: it is scorable and
     * the hive is not mid-tip. Coverage and alliance are already enforced by the selector that
     * produced this object.
     */
    public boolean usableForEstimate() {
        return scorable && !transitioning;
    }

    @Override
    public String toString() {
        return String.format("%s %s bear=%.1fdeg range=%.1fin roll=%.1fdeg cov=%.0f%% %s%s",
                name, alliance, bearingDeg, rangeIn, rollDeg, percentFound,
                scorable ? "SCORABLE" : "FLIPPED",
                transitioning ? " TRANSITIONING" : "");
    }
}
