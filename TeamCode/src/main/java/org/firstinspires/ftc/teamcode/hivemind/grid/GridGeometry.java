package org.firstinspires.ftc.teamcode.hivemind.grid;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * The immutable coordinate system shared by a {@link FieldGrid} and every
 * {@link GridSnapshot} it publishes.
 *
 * <p>Factoring the mapping out into its own object is deliberate: the mutable grid and
 * the lock-free snapshots handed to the pathfinder thread can never disagree about where
 * a cell lives, because they hold a reference to the same geometry instance.
 *
 * <h2>Why an odd number of cells</h2>
 * The grid is sized to {@code 2 * ceil(halfField / resolution) + 1} cells per axis, which
 * is always <b>odd</b>. That places a single cell exactly on the field origin, so field
 * {@code (0, 0)} maps to the true center of the matrix and
 * {@code gridToField(fieldToGrid(v))} snaps to the nearest cell <i>center</i> rather than
 * drifting by half a cell. With a 144" field at the default 2" resolution this is a
 * 73 x 73 matrix whose center cell is {@code (36, 36)}.
 *
 * <p>Cell {@code (row, col)} is centered at
 * {@code x = (col - centerCol) * resolution}, {@code y = (centerRow - row) * resolution},
 * and covers a {@code resolution}-square around that point. The outermost cells are
 * therefore centered exactly on the field walls and hang half a cell outside them, which
 * is harmless: the perimeter is inflated by the robot radius anyway (see
 * {@link FieldGrid#addPerimeterWalls}).
 *
 * <p>Immutable and safe to share across threads.
 */
public final class GridGeometry {
    /** Edge length of the square field, in inches. */
    public final double fieldSizeInches;
    /** Edge length of one square cell, in inches. */
    public final double resolutionInches;
    /** Half the field edge length; the field spans {@code [-halfField, +halfField]} on both axes. */
    public final double halfFieldInches;

    /** Matrix height. Always odd. */
    public final int rows;
    /** Matrix width. Always odd. */
    public final int cols;

    /** Row index of the cell centered on field y = 0. */
    public final int centerRow;
    /** Column index of the cell centered on field x = 0. */
    public final int centerCol;

    public GridGeometry(double resolutionInches, double fieldSizeInches) {
        if (!(resolutionInches > 0.0)) {
            throw new IllegalArgumentException("resolution must be > 0, got " + resolutionInches);
        }
        if (!(fieldSizeInches > 0.0)) {
            throw new IllegalArgumentException("field size must be > 0, got " + fieldSizeInches);
        }

        this.resolutionInches = resolutionInches;
        this.fieldSizeInches = fieldSizeInches;
        this.halfFieldInches = fieldSizeInches / 2.0;

        int halfCells = (int) Math.ceil(halfFieldInches / resolutionInches);
        this.rows = 2 * halfCells + 1;
        this.cols = this.rows;
        this.centerRow = halfCells;
        this.centerCol = halfCells;
    }

    /** Total number of cells, i.e. the length of a flat cost array. */
    public int cellCount() {
        return rows * cols;
    }

    /** Row-major flat index for the cell. No bounds checking. */
    public int index(int row, int col) {
        return row * cols + col;
    }

    /** Row component of a flat index. */
    public int rowOf(int index) {
        return index / cols;
    }

    /** Column component of a flat index. */
    public int colOf(int index) {
        return index % cols;
    }

    public boolean inBounds(int row, int col) {
        return row >= 0 && row < rows && col >= 0 && col < cols;
    }

    public boolean inBounds(GridPoint point) {
        return inBounds(point.row, point.col);
    }

    /**
     * Converts continuous field inches to the matrix index of the containing cell.
     * Coordinates outside the field are clamped to the nearest edge cell rather than
     * throwing, so a bad odometry reading degrades instead of crashing an OpMode.
     */
    public GridPoint fieldToGrid(double xInches, double yInches) {
        return new GridPoint(rowFor(yInches), colFor(xInches));
    }

    public GridPoint fieldToGrid(Vector2d position) {
        return fieldToGrid(position.x, position.y);
    }

    /** Convenience overload; the heading is ignored because the grid is a point-robot model. */
    public GridPoint fieldToGrid(Pose2d pose) {
        return fieldToGrid(pose.position.x, pose.position.y);
    }

    /** Clamped column index for a field x coordinate. */
    public int colFor(double xInches) {
        if (Double.isNaN(xInches)) return centerCol;
        // Clamp in double space: an absurd odometry reading would overflow an int cast.
        return clamp(centerCol + Math.floor(xInches / resolutionInches + 0.5), cols - 1);
    }

    /** Clamped row index for a field y coordinate. Remember that row grows as y shrinks. */
    public int rowFor(double yInches) {
        if (Double.isNaN(yInches)) return centerRow;
        return clamp(centerRow - Math.floor(yInches / resolutionInches + 0.5), rows - 1);
    }

    /**
     * Converts a matrix index back to the continuous field position at the cell's center.
     * This is the exact inverse of {@link #fieldToGrid}: for any in-bounds cell,
     * {@code fieldToGrid(gridToField(cell)).equals(cell)}.
     */
    public Vector2d gridToField(int row, int col) {
        return new Vector2d(fieldX(col), fieldY(row));
    }

    public Vector2d gridToField(GridPoint point) {
        return gridToField(point.row, point.col);
    }

    /** Field x coordinate of the center of the given column. */
    public double fieldX(int col) {
        return (col - centerCol) * resolutionInches;
    }

    /** Field y coordinate of the center of the given row. */
    public double fieldY(int row) {
        return (centerRow - row) * resolutionInches;
    }

    /** True if the point lies within the physical field bounds (before any clamping). */
    public boolean withinField(double xInches, double yInches) {
        return Math.abs(xInches) <= halfFieldInches && Math.abs(yInches) <= halfFieldInches;
    }

    /**
     * Half the diagonal of one cell. Adding this to an obstacle radius guarantees that
     * every cell left unblocked is <i>entirely</i> outside the obstacle, rather than
     * merely having its center outside.
     */
    public double cellCircumradius() {
        return resolutionInches * Math.sqrt(2.0) / 2.0;
    }

    private static int clamp(double v, int hi) {
        if (v < 0.0) return 0;
        if (v > hi) return hi;
        return (int) v;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GridGeometry)) return false;
        GridGeometry other = (GridGeometry) o;
        return Double.compare(resolutionInches, other.resolutionInches) == 0
                && Double.compare(fieldSizeInches, other.fieldSizeInches) == 0;
    }

    @Override
    public int hashCode() {
        return 31 * Double.hashCode(resolutionInches) + Double.hashCode(fieldSizeInches);
    }

    @Override
    public String toString() {
        return "GridGeometry(" + rows + "x" + cols + " cells, "
                + resolutionInches + "\" each, field " + fieldSizeInches + "\")";
    }
}
