package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import java.util.Map;
import java.util.TreeMap;

/**
 * Distance -> flywheel velocity lookup with linear interpolation between measured points.
 *
 * This is the SHOT velocity, which varies with range. It has nothing to do with how fast the
 * turret rotates.
 *
 * <h2>Why a TreeMap and not a plain HashMap</h2>
 * A bare map only answers distances you happened to measure. The robot shoots from wherever it is
 * standing, so the useful question is always "what velocity for 61.4 inches", which no map has an
 * entry for. A {@link TreeMap} answers that in two calls - {@link TreeMap#floorEntry} and
 * {@link TreeMap#ceilingEntry} give the measured points either side, and the answer is the
 * straight line between them.
 *
 * <h2>Why it clamps instead of extrapolating</h2>
 * Outside the measured range the curve is unknown, and a shooter's distance/velocity relationship
 * is not linear - it flattens as drag grows. Extrapolating past the last measured point therefore
 * produces a confident, wrong, and usually far-too-high number, which throws the ball over the
 * field. Clamping to the nearest measured end value is wrong too, but wrong in a bounded, visible
 * way: shots simply fall short beyond the tuned range, which is obvious on the field and tells you
 * to go measure another row.
 *
 * It never throws, because this is called every loop from teleop with whatever distance the aim
 * controller currently believes, including garbage distances during startup.
 *
 * <h2>FTCLib note</h2>
 * If the team later adds FTCLib, {@code com.arcrobotics.ftclib.util.InterpLUT} does the same job.
 * This implementation is dependency-free on purpose so nothing new has to be added to Gradle.
 */
public class LaunchVelocityTable {

    /** distance (INCHES) -> flywheel velocity (TICKS/SECOND), sorted by distance. */
    private final TreeMap<Double, Double> table = new TreeMap<>();

    /**
     * Builds the table pre-seeded with placeholder rows.
     *
     * EVERY ROW BELOW IS A PLACEHOLDER AND MUST BE MEASURED ON THE REAL SHOOTER.
     * They are all the same value on purpose: a flat table makes it immediately obvious on
     * telemetry that nothing has been tuned yet, whereas a plausible-looking made-up curve would
     * hide that fact until the robot missed every shot at competition.
     *
     * How to fill them in: park the robot at each distance, sweep the flywheel velocity until the
     * ball consistently lands in the CELL, and record the velocity that works. Add more rows
     * wherever the curve bends - near-range rows matter more than far ones because the velocity
     * changes fastest there.
     */
    public LaunchVelocityTable() {
        // TODO-TUNE: distance (in) -> flywheel velocity (ticks/s). Measure all four on the robot.
        add(24.0, 1000.0);   // TODO-TUNE close range
        add(48.0, 1000.0);   // TODO-TUNE mid range
        add(72.0, 1000.0);   // TODO-TUNE long range
        add(96.0, 1000.0);   // TODO-TUNE max range
    }

    /**
     * Adds or replaces one measured point.
     *
     * @param distanceInches horizontal distance to the CELL, INCHES
     * @param velocityTps    flywheel velocity that scores from there, TICKS/SECOND
     */
    public void add(double distanceInches, double velocityTps) {
        table.put(distanceInches, velocityTps);
    }

    /** Removes every row. Used by the tuning opmode when rebuilding a table from scratch. */
    public void clear() {
        table.clear();
    }

    /**
     * The flywheel velocity to shoot from this distance, TICKS/SECOND.
     *
     * @param distanceInches horizontal distance to the CELL, INCHES. NaN and infinite values are
     *                       tolerated and return the nearest end of the table.
     * @return interpolated velocity, clamped to the table's end values outside the measured range.
     *         Returns 0 only when the table is empty, which stops the flywheel rather than
     *         guessing.
     */
    public double getVelocity(double distanceInches) {
        if (table.isEmpty()) return 0.0;

        // A NaN distance (no target estimate yet) must not propagate into a motor command.
        if (Double.isNaN(distanceInches)) return table.firstEntry().getValue();

        Map.Entry<Double, Double> floor = table.floorEntry(distanceInches);
        Map.Entry<Double, Double> ceiling = table.ceilingEntry(distanceInches);

        // Below the first measured point: clamp to it. See the class comment on why not extrapolate.
        if (floor == null) return ceiling.getValue();
        // Above the last measured point: clamp to it.
        if (ceiling == null) return floor.getValue();
        // Exactly on a measured point (both lookups landed on the same entry).
        if (floor.getKey().doubleValue() == ceiling.getKey().doubleValue()) return floor.getValue();

        double span = ceiling.getKey() - floor.getKey();
        double t = (distanceInches - floor.getKey()) / span;
        return floor.getValue() + t * (ceiling.getValue() - floor.getValue());
    }

    /** The measured distances currently in the table, INCHES, ascending. Telemetry only. */
    public String describe() {
        if (table.isEmpty()) return "EMPTY";
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<Double, Double> e : table.entrySet()) {
            if (sb.length() > 0) sb.append("  ");
            sb.append(String.format("%.0fin=%.0f", e.getKey(), e.getValue()));
        }
        return sb.toString();
    }

    /** Number of measured points. */
    public int size() {
        return table.size();
    }
}
