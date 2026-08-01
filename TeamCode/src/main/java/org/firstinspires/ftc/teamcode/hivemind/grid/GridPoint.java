package org.firstinspires.ftc.teamcode.hivemind.grid;

/**
 * An immutable index into the {@link FieldGrid} matrix.
 *
 * <p>By convention throughout HiveMind:
 * <ul>
 *   <li>{@code col} increases with field <b>+x</b> (left to right).</li>
 *   <li>{@code row} increases with field <b>-y</b> (top to bottom), so row 0 is the
 *       <i>maximum</i> y edge of the field. This is the usual "matrix printed on a
 *       screen looks like the field seen from above" convention, and it is what
 *       {@link GridSnapshot#toAscii()} relies on.</li>
 * </ul>
 *
 * <p>Instances are immutable and safe to share across threads.
 */
public final class GridPoint {
    public final int row;
    public final int col;

    public GridPoint(int row, int col) {
        this.row = row;
        this.col = col;
    }

    /** Number of orthogonal-only steps between the two cells. */
    public int manhattanTo(GridPoint other) {
        return Math.abs(row - other.row) + Math.abs(col - other.col);
    }

    /** Number of 8-way steps between the two cells (Chebyshev distance). */
    public int chebyshevTo(GridPoint other) {
        return Math.max(Math.abs(row - other.row), Math.abs(col - other.col));
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GridPoint)) return false;
        GridPoint other = (GridPoint) o;
        return row == other.row && col == other.col;
    }

    @Override
    public int hashCode() {
        return 31 * row + col;
    }

    @Override
    public String toString() {
        return "GridPoint(row=" + row + ", col=" + col + ")";
    }
}
