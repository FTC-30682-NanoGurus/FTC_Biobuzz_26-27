package org.firstinspires.ftc.teamcode.hivemind.grid;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * HiveMind Component A &mdash; a thread-safe, layered occupancy model of the 144" x 144"
 * FTC field.
 *
 * <h2>Layers</h2>
 * The grid keeps two independent cost layers that are composited with a per-cell maximum:
 * <ul>
 *   <li>{@link Layer#STATIC} &mdash; permanent field geometry. Walls, the goal structure,
 *       anything bolted down. Written once during init.</li>
 *   <li>{@link Layer#DYNAMIC} &mdash; things that move. Opposing robots seen by a distance
 *       sensor or camera. Rewritten many times per second.</li>
 * </ul>
 * Separating them is what makes {@link #clearDynamicObstacles()} a safe operation: dropping
 * every robot you saw last cycle cannot accidentally erase the field walls, so there is no
 * "rebuild the static map from scratch" step in the hot loop.
 *
 * <h2>Cost field, not a bitmap</h2>
 * Cells hold a cost in {@code [0, 255]} rather than a boolean. Stamping an obstacle marks a
 * hard core at {@link GridSnapshot#BLOCKED} and surrounds it with a soft-cost band that
 * decays linearly to zero over {@link #getSoftClearanceInches()}. A* adds that soft cost to
 * its step cost, so a path that has room to spare beats a path that scrapes the corner of
 * the submersible &mdash; unless the safe route is much longer, in which case the planner
 * still takes the tight one. A pure boolean grid cannot express that tradeoff at all; it
 * only knows "legal" and "illegal", and so it always shaves the corner as tightly as the
 * inflation radius permits.
 *
 * <h2>Inflation, and the guarantee it buys</h2>
 * The pathfinder treats the robot as a single point, which is only valid if obstacles are
 * grown by the robot's radius first (a Minkowski sum). Every stamp therefore inflates by
 * {@code obstacleRadius + robotRadius + cellCircumradius}. That last term &mdash; half a
 * cell diagonal &mdash; upgrades the usual "the cell's center is clear" approximation into
 * a real guarantee:
 *
 * <blockquote>If a cell is not {@code BLOCKED}, then <b>no point anywhere inside that
 * cell</b> is within {@code robotRadius} of an obstacle.</blockquote>
 *
 * Without it, a cell whose center sits just outside the radius but whose corner pokes into
 * the obstacle would be reported traversable, and the robot would clip it.
 *
 * <h2>Concurrency</h2>
 * Mutators are {@code synchronized}, so writers serialize against each other. Readers do
 * not take the lock at all: {@link #snapshot()} publishes an immutable {@link GridSnapshot}
 * through a {@code volatile} field, and the A* worker searches that frozen copy. A sensor
 * update never stalls a search in progress, and a search can never see a torn map.
 *
 * <p>Mutations invalidate the published snapshot rather than rebuilding it, so stamping
 * fifty obstacles in a row costs one composite pass, not fifty &mdash; the rebuild happens
 * lazily on the next {@link #snapshot()} call.
 */
public class FieldGrid {
    /** Edge length of a standard FTC field, in inches. */
    public static final double FTC_FIELD_SIZE_INCHES = 144.0;

    /** Default cell edge length. 2" gives a 73x73 matrix, which A* clears in a few ms. */
    public static final double DEFAULT_RESOLUTION_INCHES = 2.0;

    /**
     * Default robot radius used for obstacle inflation, in inches.
     *
     * <p>9" is the <i>inscribed</i> radius of a legal 18" robot &mdash; half its width. The
     * circumscribed radius is 12.73", which is what you would need to be safe at every
     * possible heading, but inflating that much makes a legitimate 24" field gap look
     * impassable. 9" is the standard compromise; raise it toward 12.73 if your paths cut
     * corners too aggressively, and pass your real robot's half-width if it isn't 18".
     */
    public static final double DEFAULT_ROBOT_RADIUS_INCHES = 9.0;

    /** Default width of the soft-cost band beyond the hard inflation radius, in inches. */
    public static final double DEFAULT_SOFT_CLEARANCE_INCHES = 6.0;

    /** Default soft cost immediately outside the hard radius, decaying to 0 at the band edge. */
    public static final int DEFAULT_MAX_SOFT_COST = 60;

    /** Which cost layer a mutation writes to. */
    public enum Layer {
        /** Permanent field geometry. Survives {@link FieldGrid#clearDynamicObstacles()}. */
        STATIC,
        /** Transient obstacles such as other robots. Cleared every sensor cycle. */
        DYNAMIC
    }

    private final GridGeometry geometry;
    private final double robotRadiusInches;
    private final double softClearanceInches;
    private final int maxSoftCost;
    private final double quantizationMargin;

    private final byte[] staticLayer;
    private final byte[] dynamicLayer;

    /** Last composited snapshot, or null when a mutation has invalidated it. */
    private volatile GridSnapshot published;
    private long version;

    /** 2" cells, 9" robot radius, 6" soft clearance. */
    public FieldGrid() {
        this(DEFAULT_RESOLUTION_INCHES, DEFAULT_ROBOT_RADIUS_INCHES);
    }

    /** Custom cell size with the default robot radius. */
    public FieldGrid(double resolutionInches) {
        this(resolutionInches, DEFAULT_ROBOT_RADIUS_INCHES);
    }

    /** Custom cell size and robot radius, with the default soft-cost band. */
    public FieldGrid(double resolutionInches, double robotRadiusInches) {
        this(resolutionInches, robotRadiusInches, FTC_FIELD_SIZE_INCHES,
                DEFAULT_SOFT_CLEARANCE_INCHES, DEFAULT_MAX_SOFT_COST);
    }

    /**
     * Full control over the model.
     *
     * @param resolutionInches    edge length of one square cell; smaller is more precise and slower
     * @param robotRadiusInches   how far obstacles are grown so the planner can treat the robot
     *                            as a point; see {@link #DEFAULT_ROBOT_RADIUS_INCHES}
     * @param fieldSizeInches     edge length of the square field
     * @param softClearanceInches width of the decaying soft-cost band outside the hard radius;
     *                            0 disables soft costs entirely
     * @param maxSoftCost         soft cost immediately outside the hard radius, in {@code [1, 254]}
     */
    public FieldGrid(double resolutionInches,
                     double robotRadiusInches,
                     double fieldSizeInches,
                     double softClearanceInches,
                     int maxSoftCost) {
        if (robotRadiusInches < 0.0) {
            throw new IllegalArgumentException("robot radius must be >= 0, got " + robotRadiusInches);
        }
        if (softClearanceInches < 0.0) {
            throw new IllegalArgumentException("soft clearance must be >= 0, got " + softClearanceInches);
        }
        if (maxSoftCost < 1 || maxSoftCost > GridSnapshot.BLOCKED - 1) {
            throw new IllegalArgumentException(
                    "max soft cost must be in [1, " + (GridSnapshot.BLOCKED - 1) + "], got " + maxSoftCost);
        }

        this.geometry = new GridGeometry(resolutionInches, fieldSizeInches);
        this.robotRadiusInches = robotRadiusInches;
        this.softClearanceInches = softClearanceInches;
        this.maxSoftCost = maxSoftCost;
        this.quantizationMargin = geometry.cellCircumradius();

        this.staticLayer = new byte[geometry.cellCount()];
        this.dynamicLayer = new byte[geometry.cellCount()];
        this.version = 0L;
        this.published = new GridSnapshot(geometry, new byte[geometry.cellCount()], 0L, maxSoftCost);
    }

    // ---------------------------------------------------------------- geometry

    public GridGeometry geometry() {
        return geometry;
    }

    public int rows() {
        return geometry.rows;
    }

    public int cols() {
        return geometry.cols;
    }

    public double getResolutionInches() {
        return geometry.resolutionInches;
    }

    public double getRobotRadiusInches() {
        return robotRadiusInches;
    }

    public double getSoftClearanceInches() {
        return softClearanceInches;
    }

    /** Soft cost stamped immediately outside an obstacle's hard radius. */
    public int getMaxSoftCost() {
        return maxSoftCost;
    }

    /** Converts continuous field inches to a matrix index. Off-field points clamp to the edge. */
    public GridPoint fieldToGrid(double xInches, double yInches) {
        return geometry.fieldToGrid(xInches, yInches);
    }

    public GridPoint fieldToGrid(Vector2d position) {
        return geometry.fieldToGrid(position);
    }

    public GridPoint fieldToGrid(Pose2d pose) {
        return geometry.fieldToGrid(pose);
    }

    /** Converts a matrix index back to the field position at the center of that cell. */
    public Vector2d gridToField(GridPoint point) {
        return geometry.gridToField(point);
    }

    public Vector2d gridToField(int row, int col) {
        return geometry.gridToField(row, col);
    }

    // ---------------------------------------------------------------- reading

    /**
     * Returns the current immutable view of the map. Cheap: a volatile read on the common
     * path, and one composite pass only when a mutation has happened since the last call.
     *
     * <p>A pathfinder should call this <b>once</b> and hold the result for the whole search.
     */
    public GridSnapshot snapshot() {
        GridSnapshot current = published;
        if (current != null) return current;
        return rebuild();
    }

    /** Revision number of the map as it currently stands. */
    public long currentVersion() {
        return snapshot().version;
    }

    /** Convenience read-through to {@link GridSnapshot#isBlocked(GridPoint)}. */
    public boolean isBlocked(GridPoint point) {
        return snapshot().isBlocked(point);
    }

    public boolean isBlocked(int row, int col) {
        return snapshot().isBlocked(row, col);
    }

    public boolean isBlocked(Vector2d position) {
        return snapshot().isBlocked(position);
    }

    /** Convenience read-through to {@link GridSnapshot#costAt(GridPoint)}. */
    public int costAt(GridPoint point) {
        return snapshot().costAt(point);
    }

    public int costAt(int row, int col) {
        return snapshot().costAt(row, col);
    }

    private synchronized GridSnapshot rebuild() {
        GridSnapshot current = published;
        if (current != null) return current; // another thread rebuilt while we waited

        byte[] merged = new byte[staticLayer.length];
        for (int i = 0; i < merged.length; i++) {
            int s = staticLayer[i] & 0xFF;
            int d = dynamicLayer[i] & 0xFF;
            merged[i] = (byte) (s >= d ? s : d);
        }
        GridSnapshot next = new GridSnapshot(geometry, merged, ++version, maxSoftCost);
        published = next;
        return next;
    }

    /** Called from inside a synchronized mutator to force the next read to recomposite. */
    private void invalidate() {
        published = null;
    }

    // ---------------------------------------------------------------- mutation

    /**
     * Stamps a circular obstacle onto the {@link Layer#DYNAMIC} layer.
     *
     * <p>This is the signature the HiveMind spec mandates. It targets the dynamic layer
     * because that is the mid-match case &mdash; a robot you just saw, which should vanish
     * on the next {@link #clearDynamicObstacles()}. For permanent field geometry that must
     * survive clearing, use {@link #setStaticObstacle}.
     *
     * @param x             obstacle center, field inches
     * @param y             obstacle center, field inches
     * @param radiusInches  physical radius of the obstacle, <i>before</i> robot inflation
     */
    public synchronized void setObstacle(double x, double y, double radiusInches) {
        setObstacle(x, y, radiusInches, Layer.DYNAMIC);
    }

    /** Stamps a circular obstacle onto the {@link Layer#STATIC} layer. */
    public synchronized void setStaticObstacle(double x, double y, double radiusInches) {
        setObstacle(x, y, radiusInches, Layer.STATIC);
    }

    /**
     * Stamps a circular obstacle onto an explicit layer, inflating it by the robot radius
     * plus the cell quantization margin and surrounding it with the soft-cost band.
     */
    public synchronized void setObstacle(final double x, final double y,
                                         double radiusInches, Layer layer) {
        if (radiusInches < 0.0) {
            throw new IllegalArgumentException("obstacle radius must be >= 0, got " + radiusInches);
        }
        double hardR = radiusInches + robotRadiusInches + quantizationMargin;
        double softR = hardR + softClearanceInches;

        stamp(x - softR, y - softR, x + softR, y + softR, hardR, softR, layerArray(layer),
                new ShapeDistance() {
                    @Override
                    public double distanceTo(double px, double py) {
                        double dx = px - x;
                        double dy = py - y;
                        return Math.sqrt(dx * dx + dy * dy);
                    }
                });
        invalidate();
    }

    /** Field-space overload of {@link #setObstacle(double, double, double, Layer)}. */
    public synchronized void setObstacle(Vector2d center, double radiusInches, Layer layer) {
        setObstacle(center.x, center.y, radiusInches, layer);
    }

    /**
     * Stamps an axis-aligned rectangular obstacle, which is what most FTC field elements
     * actually are. The inflated footprint is a correct rounded rectangle (the Minkowski
     * sum of the rectangle and the robot disc), not a padded box, so corners stay reachable.
     *
     * @param minX left edge, field inches
     * @param minY bottom edge, field inches
     * @param maxX right edge, field inches
     * @param maxY top edge, field inches
     */
    public synchronized void setObstacleRect(double minX, double minY,
                                             double maxX, double maxY, Layer layer) {
        final double loX = Math.min(minX, maxX);
        final double hiX = Math.max(minX, maxX);
        final double loY = Math.min(minY, maxY);
        final double hiY = Math.max(minY, maxY);

        double hardR = robotRadiusInches + quantizationMargin;
        double softR = hardR + softClearanceInches;

        stamp(loX - softR, loY - softR, hiX + softR, hiY + softR, hardR, softR, layerArray(layer),
                new ShapeDistance() {
                    @Override
                    public double distanceTo(double px, double py) {
                        double dx = Math.max(Math.max(loX - px, px - hiX), 0.0);
                        double dy = Math.max(Math.max(loY - py, py - hiY), 0.0);
                        return Math.sqrt(dx * dx + dy * dy);
                    }
                });
        invalidate();
    }

    /**
     * Blocks everything <i>outside</i> the given rectangle. Use it to keep the planner
     * inside the field, or to fence the robot into one half during an alliance-specific
     * routine.
     */
    public synchronized void blockOutside(double minX, double minY,
                                          double maxX, double maxY, Layer layer) {
        double loX = Math.min(minX, maxX);
        double hiX = Math.max(minX, maxX);
        double loY = Math.min(minY, maxY);
        double hiY = Math.max(minY, maxY);

        byte[] target = layerArray(layer);
        for (int row = 0; row < geometry.rows; row++) {
            double cy = geometry.fieldY(row);
            for (int col = 0; col < geometry.cols; col++) {
                double cx = geometry.fieldX(col);
                if (cx < loX || cx > hiX || cy < loY || cy > hiY) {
                    raise(target, geometry.index(row, col), GridSnapshot.BLOCKED);
                }
            }
        }
        invalidate();
    }

    /**
     * Blocks the band around the field perimeter that the robot's center can never occupy.
     * Call this once at init so A* stops routing paths that scrape the wall.
     */
    public synchronized void addPerimeterWalls(Layer layer) {
        double inset = robotRadiusInches + quantizationMargin;
        double limit = geometry.halfFieldInches - inset;
        blockOutside(-limit, -limit, limit, limit, layer);
    }

    /** Erases every obstacle on the given layer. */
    public synchronized void clear(Layer layer) {
        java.util.Arrays.fill(layerArray(layer), (byte) 0);
        invalidate();
    }

    /**
     * Erases the transient layer, leaving permanent field geometry intact. Cheap enough to
     * call at the top of every sensor cycle.
     */
    public synchronized void clearDynamicObstacles() {
        clear(Layer.DYNAMIC);
    }

    /** Erases both layers, returning the grid to a blank field. */
    public synchronized void clearAll() {
        java.util.Arrays.fill(staticLayer, (byte) 0);
        java.util.Arrays.fill(dynamicLayer, (byte) 0);
        invalidate();
    }

    // ---------------------------------------------------------------- internals

    /** Distance from a field point to the surface of some shape; 0 when inside it. */
    private interface ShapeDistance {
        double distanceTo(double xInches, double yInches);
    }

    /**
     * Rasterizes a shape's cost profile into a layer over the cells its soft radius reaches.
     * Costs combine with a maximum, so overlapping obstacles never cancel each other out.
     */
    private void stamp(double minX, double minY, double maxX, double maxY,
                       double hardR, double softR, byte[] target, ShapeDistance shape) {
        // Skip shapes whose entire influence lies off the field.
        double edge = geometry.halfFieldInches + geometry.resolutionInches;
        if (maxX < -edge || minX > edge || maxY < -edge || minY > edge) {
            return;
        }

        // fieldToGrid clamps, so widen by a cell to be sure the whole footprint is covered.
        int rowStart = Math.max(0, geometry.rowFor(maxY) - 1);
        int rowEnd = Math.min(geometry.rows - 1, geometry.rowFor(minY) + 1);
        int colStart = Math.max(0, geometry.colFor(minX) - 1);
        int colEnd = Math.min(geometry.cols - 1, geometry.colFor(maxX) + 1);

        for (int row = rowStart; row <= rowEnd; row++) {
            double cy = geometry.fieldY(row);
            int rowBase = row * geometry.cols;
            for (int col = colStart; col <= colEnd; col++) {
                double d = shape.distanceTo(geometry.fieldX(col), cy);
                int cost = costForDistance(d, hardR, softR);
                if (cost > GridSnapshot.FREE) {
                    raise(target, rowBase + col, cost);
                }
            }
        }
    }

    /**
     * The cost profile: impassable inside the hard radius, then a linear ramp from
     * {@code maxSoftCost} down to free across the soft clearance band.
     */
    private int costForDistance(double distance, double hardR, double softR) {
        if (distance <= hardR) return GridSnapshot.BLOCKED;
        if (distance >= softR) return GridSnapshot.FREE;
        double t = (softR - distance) / (softR - hardR); // 1 at the hard edge, 0 at the soft edge
        int cost = (int) Math.round(maxSoftCost * t);
        if (cost < 1) cost = 1;
        if (cost > maxSoftCost) cost = maxSoftCost;
        return cost;
    }

    /** Raises a cell to {@code cost} if it is not already at least that expensive. */
    private static void raise(byte[] layer, int index, int cost) {
        if ((layer[index] & 0xFF) < cost) {
            layer[index] = (byte) cost;
        }
    }

    private byte[] layerArray(Layer layer) {
        return layer == Layer.STATIC ? staticLayer : dynamicLayer;
    }

    @Override
    public String toString() {
        return "FieldGrid(" + geometry + ", robotRadius=" + robotRadiusInches + "\")";
    }
}
