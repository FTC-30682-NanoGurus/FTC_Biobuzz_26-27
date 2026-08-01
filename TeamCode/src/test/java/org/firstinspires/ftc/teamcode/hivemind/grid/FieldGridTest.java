package org.firstinspires.ftc.teamcode.hivemind.grid;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Milestone 1 verification for HiveMind Component A.
 *
 * <p>Runs on the JVM, no robot or Android device required: {@code ./gradlew :TeamCode:test}
 */
public class FieldGridTest {

    private static final double EPS = 1e-9;

    // ------------------------------------------------------------ dimensions

    @Test
    public void defaultGridIsOddSizedSoOriginGetsItsOwnCell() {
        FieldGrid grid = new FieldGrid();
        assertEquals(73, grid.rows());
        assertEquals(73, grid.cols());
        assertEquals(1, grid.rows() % 2);
        assertEquals(2.0, grid.getResolutionInches(), EPS);
    }

    @Test
    public void gridStaysOddAtEveryResolution() {
        double[] resolutions = {1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0};
        for (double res : resolutions) {
            FieldGrid grid = new FieldGrid(res);
            assertEquals("odd row count at " + res + "\"", 1, grid.rows() % 2);
            assertEquals("square matrix at " + res + "\"", grid.rows(), grid.cols());

            // The matrix must cover the whole field.
            double covered = grid.geometry().centerCol * res;
            assertTrue("grid too small at " + res + "\" (covers " + covered + "\")",
                    covered >= grid.geometry().halfFieldInches - EPS);
        }
        assertEquals(37, new FieldGrid(4.0).rows());
        assertEquals(145, new FieldGrid(1.0).rows());
    }

    // ------------------------------------------------------- coordinate mapping

    @Test
    public void originMapsToTheCenterOfTheMatrix() {
        FieldGrid grid = new FieldGrid();
        GridPoint center = grid.fieldToGrid(0.0, 0.0);

        assertEquals(36, center.row);
        assertEquals(36, center.col);
        assertEquals(grid.rows() / 2, center.row);
        assertEquals(grid.cols() / 2, center.col);

        Vector2d back = grid.gridToField(center);
        assertEquals(0.0, back.x, EPS);
        assertEquals(0.0, back.y, EPS);
    }

    /** gridToField -> fieldToGrid must be the exact identity for every cell in the matrix. */
    @Test
    public void gridToFieldRoundTripsExactlyForEveryCell() {
        for (double res : new double[]{1.0, 2.0, 3.0, 4.0}) {
            FieldGrid grid = new FieldGrid(res);
            for (int row = 0; row < grid.rows(); row++) {
                for (int col = 0; col < grid.cols(); col++) {
                    GridPoint original = new GridPoint(row, col);
                    GridPoint round = grid.fieldToGrid(grid.gridToField(original));
                    assertEquals("round trip at " + res + "\" " + original, original, round);
                }
            }
        }
    }

    /** fieldToGrid -> gridToField must snap to the nearest cell center, never further. */
    @Test
    public void fieldToGridRoundTripsWithinHalfACell() {
        FieldGrid grid = new FieldGrid(2.0);
        double half = grid.getResolutionInches() / 2.0;

        for (double x = -72.0; x <= 72.0; x += 0.37) {
            for (double y = -72.0; y <= 72.0; y += 4.1) {
                Vector2d snapped = grid.gridToField(grid.fieldToGrid(x, y));
                assertTrue("x drift at (" + x + "," + y + ")", Math.abs(snapped.x - x) <= half + EPS);
                assertTrue("y drift at (" + x + "," + y + ")", Math.abs(snapped.y - y) <= half + EPS);
            }
        }
    }

    @Test
    public void allFourQuadrantsLandInTheCorrectCorner() {
        FieldGrid grid = new FieldGrid();
        int center = 36;

        // Quadrant I: +x, +y -> right of center, above center (smaller row).
        GridPoint q1 = grid.fieldToGrid(24.0, 24.0);
        assertEquals(48, q1.col);
        assertEquals(24, q1.row);

        // Quadrant II: -x, +y
        GridPoint q2 = grid.fieldToGrid(-24.0, 24.0);
        assertEquals(24, q2.col);
        assertEquals(24, q2.row);

        // Quadrant III: -x, -y
        GridPoint q3 = grid.fieldToGrid(-24.0, -24.0);
        assertEquals(24, q3.col);
        assertEquals(48, q3.row);

        // Quadrant IV: +x, -y
        GridPoint q4 = grid.fieldToGrid(24.0, -24.0);
        assertEquals(48, q4.col);
        assertEquals(48, q4.row);

        // Sign relationships, stated independently of the exact numbers above.
        assertTrue(q1.col > center && q1.row < center);
        assertTrue(q2.col < center && q2.row < center);
        assertTrue(q3.col < center && q3.row > center);
        assertTrue(q4.col > center && q4.row > center);

        // ...and each maps back into its own quadrant.
        assertQuadrant(grid.gridToField(q1), 1, 1);
        assertQuadrant(grid.gridToField(q2), -1, 1);
        assertQuadrant(grid.gridToField(q3), -1, -1);
        assertQuadrant(grid.gridToField(q4), 1, -1);
    }

    private static void assertQuadrant(Vector2d v, int signX, int signY) {
        assertTrue("x sign of " + v, Math.signum(v.x) == signX);
        assertTrue("y sign of " + v, Math.signum(v.y) == signY);
    }

    @Test
    public void rowIncreasesAsYDecreases() {
        FieldGrid grid = new FieldGrid();
        assertEquals(0, grid.fieldToGrid(0.0, 72.0).row);   // top of the field
        assertEquals(72, grid.fieldToGrid(0.0, -72.0).row); // bottom of the field
        assertTrue(grid.fieldToGrid(0.0, 30.0).row < grid.fieldToGrid(0.0, -30.0).row);
    }

    @Test
    public void fieldCornersMapToMatrixCorners() {
        FieldGrid grid = new FieldGrid();
        assertEquals(new GridPoint(0, 0), grid.fieldToGrid(-72.0, 72.0));
        assertEquals(new GridPoint(0, 72), grid.fieldToGrid(72.0, 72.0));
        assertEquals(new GridPoint(72, 0), grid.fieldToGrid(-72.0, -72.0));
        assertEquals(new GridPoint(72, 72), grid.fieldToGrid(72.0, -72.0));
    }

    @Test
    public void outOfBoundsCoordinatesClampInsteadOfThrowing() {
        FieldGrid grid = new FieldGrid();
        assertEquals(new GridPoint(0, 72), grid.fieldToGrid(500.0, 500.0));
        assertEquals(new GridPoint(72, 0), grid.fieldToGrid(-500.0, -500.0));
        assertEquals(new GridPoint(36, 72), grid.fieldToGrid(Double.MAX_VALUE, 0.0));
    }

    @Test
    public void poseOverloadIgnoresHeading() {
        FieldGrid grid = new FieldGrid();
        GridPoint fromPose = grid.fieldToGrid(new Pose2d(24.0, -12.0, Math.toRadians(137.0)));
        assertEquals(grid.fieldToGrid(24.0, -12.0), fromPose);
    }

    // ---------------------------------------------------------- obstacle stamping

    @Test
    public void obstacleBlocksItsOwnCellAndTheInflationRadius() {
        FieldGrid grid = new FieldGrid(2.0, 9.0);
        grid.setObstacle(24.0, 24.0, 4.0);
        GridSnapshot snap = grid.snapshot();

        assertTrue(snap.isBlocked(grid.fieldToGrid(24.0, 24.0)));
        // 4" obstacle + 9" robot: a point 10" away must still be blocked.
        assertTrue(snap.isBlocked(grid.fieldToGrid(34.0, 24.0)));
        // ...but 30" away is clear even of soft cost.
        assertEquals(GridSnapshot.FREE, snap.costAt(grid.fieldToGrid(54.0, 24.0)));
    }

    /**
     * The core safety property: an unblocked cell must be entirely outside the obstacle's
     * inflated footprint, not merely have its center outside.
     */
    @Test
    public void unblockedCellsAreEntirelyOutsideTheRobotRadius() {
        double robotRadius = 9.0;
        double obstacleRadius = 5.0;
        double ox = 13.0;
        double oy = -27.0;

        FieldGrid grid = new FieldGrid(2.0, robotRadius);
        grid.setObstacle(ox, oy, obstacleRadius);
        GridSnapshot snap = grid.snapshot();

        double half = grid.getResolutionInches() / 2.0;
        double forbidden = obstacleRadius + robotRadius;

        for (int row = 0; row < snap.rows(); row++) {
            for (int col = 0; col < snap.cols(); col++) {
                if (snap.isBlocked(row, col)) continue;

                // Closest point of this cell's square to the obstacle center.
                Vector2d c = snap.geometry.gridToField(row, col);
                double dx = Math.max(Math.max(c.x - half - ox, ox - (c.x + half)), 0.0);
                double dy = Math.max(Math.max(c.y - half - oy, oy - (c.y + half)), 0.0);
                double closest = Math.sqrt(dx * dx + dy * dy);

                assertTrue("cell (" + row + "," + col + ") is reported free but its nearest"
                                + " corner is only " + closest + "\" from the obstacle center",
                        closest >= forbidden - EPS);
            }
        }
    }

    @Test
    public void softCostDecaysMonotonicallyWithDistance() {
        FieldGrid grid = new FieldGrid(2.0, 6.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 12.0, 100);
        grid.setObstacle(0.0, 0.0, 0.0);
        GridSnapshot snap = grid.snapshot();

        int previous = Integer.MAX_VALUE;
        boolean sawSoft = false;
        boolean sawFree = false;

        // Walk outward along +x from the origin, one cell at a time.
        for (int col = snap.geometry.centerCol; col < snap.cols(); col++) {
            int cost = snap.costAt(snap.geometry.centerRow, col);
            assertTrue("cost must never increase with distance", cost <= previous);
            previous = cost;
            if (cost > GridSnapshot.FREE && cost < GridSnapshot.BLOCKED) sawSoft = true;
            if (cost == GridSnapshot.FREE) sawFree = true;
        }
        assertTrue("expected a soft-cost band outside the hard radius", sawSoft);
        assertTrue("expected the soft band to decay to free", sawFree);
    }

    @Test
    public void softClearanceOfZeroProducesOnlyHardObstacles() {
        FieldGrid grid = new FieldGrid(2.0, 4.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 0.0, 1);
        grid.setObstacle(0.0, 0.0, 2.0);
        GridSnapshot snap = grid.snapshot();

        for (int row = 0; row < snap.rows(); row++) {
            for (int col = 0; col < snap.cols(); col++) {
                int cost = snap.costAt(row, col);
                assertTrue("cost " + cost + " should be FREE or BLOCKED only",
                        cost == GridSnapshot.FREE || cost == GridSnapshot.BLOCKED);
            }
        }
    }

    @Test
    public void rectangleInflationHasRoundedCornersNotSquareOnes() {
        FieldGrid grid = new FieldGrid(1.0, 8.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 0.1, 1);
        grid.setObstacleRect(-10.0, -10.0, 10.0, 10.0, FieldGrid.Layer.STATIC);
        GridSnapshot snap = grid.snapshot();

        // Straight out from an edge: inside the 8" inflation, so blocked.
        assertTrue(snap.isBlocked(grid.fieldToGrid(16.0, 0.0)));
        // Diagonally off the corner by the same amount per axis: 8.49" away, still blocked.
        assertTrue(snap.isBlocked(grid.fieldToGrid(16.0, 16.0)));
        // A square (padded-box) inflation would blindly block this corner; a correct
        // rounded-rect one leaves it open, because it is 11.3" from the rectangle.
        assertFalse("corner should be reachable - inflation must be a rounded rect",
                snap.isBlocked(grid.fieldToGrid(18.0, 18.0)));
    }

    /**
     * A later, weaker stamp must not lower a cell that an earlier one made expensive.
     * Probing the two obstacles at equal distance would pass even if stamping simply
     * overwrote, so the second obstacle here is deliberately far enough away to write a
     * strictly smaller cost.
     */
    @Test
    public void aWeakerOverlappingStampNeverLowersACell() {
        FieldGrid grid = new FieldGrid(2.0, 0.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 10.0, 50);
        GridPoint probe = grid.fieldToGrid(2.0, 0.0);

        grid.setObstacle(0.0, 0.0, 0.0);       // 2" away: expensive
        int strong = grid.snapshot().costAt(probe);

        grid.setObstacle(10.0, 0.0, 0.0);      // 8" away: would write a much smaller cost
        int after = grid.snapshot().costAt(probe);

        assertTrue("setup: first stamp should be expensive", strong > 30);
        assertEquals("overlapping stamps must combine with max, not overwrite", strong, after);
    }

    @Test
    public void softCostsCombineAcrossLayersWithMax() {
        FieldGrid grid = new FieldGrid(2.0, 0.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 10.0, 50);
        GridPoint probe = grid.fieldToGrid(2.0, 0.0);

        grid.setStaticObstacle(0.0, 0.0, 0.0);     // near, on STATIC
        int staticOnly = grid.snapshot().costAt(probe);
        grid.setObstacle(10.0, 0.0, 0.0);          // far, on DYNAMIC
        assertEquals("composite must take the max across layers",
                staticOnly, grid.snapshot().costAt(probe));

        grid.clear(FieldGrid.Layer.STATIC);
        int dynamicOnly = grid.snapshot().costAt(probe);
        assertTrue("the weaker dynamic cost should surface once static is gone",
                dynamicOnly > GridSnapshot.FREE && dynamicOnly < staticOnly);
    }

    @Test
    public void snapshotCarriesTheSoftCostScaleForThePlanner() {
        assertEquals(50, new FieldGrid(2.0, 9.0, 144.0, 6.0, 50).snapshot().maxSoftCost);
        assertEquals(FieldGrid.DEFAULT_MAX_SOFT_COST, new FieldGrid().snapshot().maxSoftCost);

        FieldGrid grid = new FieldGrid(2.0, 0.0, 144.0, 20.0, 77);
        grid.setObstacle(0.0, 0.0, 0.0);
        GridSnapshot snap = grid.snapshot();
        for (int row = 0; row < snap.rows(); row++) {
            for (int col = 0; col < snap.cols(); col++) {
                int cost = snap.costAt(row, col);
                assertTrue("soft cost " + cost + " exceeds the advertised maximum",
                        cost == GridSnapshot.BLOCKED || cost <= snap.maxSoftCost);
            }
        }
    }

    @Test
    public void obstaclesEntirelyOffFieldAreIgnored() {
        FieldGrid grid = new FieldGrid();
        grid.setObstacle(500.0, 500.0, 4.0);
        assertEquals(0, grid.snapshot().blockedCellCount());
    }

    @Test
    public void perimeterWallsBlockTheEdgesAndLeaveTheMiddleOpen() {
        FieldGrid grid = new FieldGrid(2.0, 9.0);
        grid.addPerimeterWalls(FieldGrid.Layer.STATIC);
        GridSnapshot snap = grid.snapshot();

        assertTrue(snap.isBlocked(grid.fieldToGrid(-72.0, 0.0)));
        assertTrue(snap.isBlocked(grid.fieldToGrid(0.0, 72.0)));
        assertTrue(snap.isBlocked(grid.fieldToGrid(70.0, 70.0)));
        assertFalse(snap.isBlocked(grid.fieldToGrid(0.0, 0.0)));
        assertFalse(snap.isBlocked(grid.fieldToGrid(40.0, -40.0)));
    }

    @Test
    public void blockOutsideFencesTheRobotIntoARegion() {
        FieldGrid grid = new FieldGrid();
        grid.blockOutside(-72.0, -72.0, 0.0, 72.0, FieldGrid.Layer.STATIC);
        GridSnapshot snap = grid.snapshot();

        assertFalse(snap.isBlocked(grid.fieldToGrid(-40.0, 10.0)));
        assertTrue(snap.isBlocked(grid.fieldToGrid(40.0, 10.0)));
    }

    // ------------------------------------------------------------------ layers

    @Test
    public void clearingDynamicObstaclesLeavesStaticGeometryIntact() {
        FieldGrid grid = new FieldGrid();
        grid.setStaticObstacle(-30.0, -30.0, 6.0);
        grid.setObstacle(30.0, 30.0, 6.0); // dynamic by default

        GridPoint staticCell = grid.fieldToGrid(-30.0, -30.0);
        GridPoint dynamicCell = grid.fieldToGrid(30.0, 30.0);

        assertTrue(grid.isBlocked(staticCell));
        assertTrue(grid.isBlocked(dynamicCell));

        grid.clearDynamicObstacles();

        assertTrue("static obstacle must survive a dynamic clear", grid.isBlocked(staticCell));
        assertFalse("dynamic obstacle must be gone", grid.isBlocked(dynamicCell));
    }

    @Test
    public void theSpecMandatedSetObstacleTargetsTheDynamicLayer() {
        FieldGrid grid = new FieldGrid();
        grid.setObstacle(10.0, 10.0, 5.0);
        assertTrue(grid.isBlocked(grid.fieldToGrid(10.0, 10.0)));
        grid.clearDynamicObstacles();
        assertFalse(grid.isBlocked(grid.fieldToGrid(10.0, 10.0)));
    }

    @Test
    public void clearAllEmptiesBothLayers() {
        FieldGrid grid = new FieldGrid();
        grid.setStaticObstacle(-20.0, 0.0, 5.0);
        grid.setObstacle(20.0, 0.0, 5.0);
        assertTrue(grid.snapshot().blockedCellCount() > 0);

        grid.clearAll();
        assertEquals(0, grid.snapshot().blockedCellCount());
    }

    @Test
    public void overlappingLayersStayBlockedUntilBothAreCleared() {
        FieldGrid grid = new FieldGrid();
        GridPoint spot = grid.fieldToGrid(0.0, 0.0);
        grid.setStaticObstacle(0.0, 0.0, 4.0);
        grid.setObstacle(0.0, 0.0, 4.0);

        grid.clearDynamicObstacles();
        assertTrue("static layer still covers this cell", grid.isBlocked(spot));

        grid.clear(FieldGrid.Layer.STATIC);
        assertFalse(grid.isBlocked(spot));
    }

    // --------------------------------------------------------------- snapshots

    @Test
    public void anOldSnapshotNeverChangesUnderneathItsReader() {
        FieldGrid grid = new FieldGrid();
        grid.setStaticObstacle(0.0, 0.0, 4.0);

        GridSnapshot before = grid.snapshot();
        int blockedBefore = before.blockedCellCount();
        long versionBefore = before.version;

        grid.setObstacle(40.0, 40.0, 10.0);
        GridSnapshot after = grid.snapshot();

        assertEquals("held snapshot must be frozen", blockedBefore, before.blockedCellCount());
        assertEquals(versionBefore, before.version);
        assertTrue("new snapshot must be a later version", after.version > before.version);
        assertTrue(after.blockedCellCount() > blockedBefore);
    }

    @Test
    public void repeatedReadsWithoutMutationReturnTheSameInstance() {
        FieldGrid grid = new FieldGrid();
        GridSnapshot a = grid.snapshot();
        GridSnapshot b = grid.snapshot();
        assertTrue("clean reads must not recomposite", a == b);
    }

    /** Fifty stamps should cost one composite pass, not fifty. */
    @Test
    public void batchedMutationsProduceASingleNewVersion() {
        FieldGrid grid = new FieldGrid();
        long start = grid.snapshot().version;

        for (int i = 0; i < 50; i++) {
            grid.setObstacle(i - 25.0, 0.0, 1.0);
        }

        assertEquals(start + 1, grid.snapshot().version);
    }

    @Test
    public void versionAdvancesOncePerReadAfterMutation() {
        FieldGrid grid = new FieldGrid();
        long v0 = grid.snapshot().version;
        grid.setObstacle(0.0, 0.0, 2.0);
        long v1 = grid.snapshot().version;
        grid.clearDynamicObstacles();
        long v2 = grid.snapshot().version;

        assertEquals(v0 + 1, v1);
        assertEquals(v1 + 1, v2);
    }

    // --------------------------------------------------------------- queries

    @Test
    public void lineOfSightIsClearAcrossAnEmptyField() {
        GridSnapshot snap = new FieldGrid().snapshot();
        assertTrue(snap.hasLineOfSight(new Vector2d(-60.0, -60.0), new Vector2d(60.0, 60.0)));
        assertTrue(snap.hasLineOfSight(new Vector2d(-60.0, 0.0), new Vector2d(60.0, 0.0)));
        assertTrue(snap.hasLineOfSight(new Vector2d(0.0, 0.0), new Vector2d(0.0, 0.0)));
    }

    @Test
    public void lineOfSightIsBrokenByABarrier() {
        FieldGrid grid = new FieldGrid();
        // A wall spanning y = -40..40 at x = 0.
        grid.setObstacleRect(-1.0, -40.0, 1.0, 40.0, FieldGrid.Layer.STATIC);
        GridSnapshot snap = grid.snapshot();

        assertFalse("straight through the wall",
                snap.hasLineOfSight(new Vector2d(-40.0, 0.0), new Vector2d(40.0, 0.0)));
        assertTrue("around the end of the wall",
                snap.hasLineOfSight(new Vector2d(-40.0, 60.0), new Vector2d(40.0, 60.0)));
    }

    @Test
    public void lineOfSightRefusesToSqueezeThroughADiagonalGap() {
        FieldGrid grid = pointGrid();
        GridGeometry geo = grid.geometry();
        int r = geo.centerRow;
        int c = geo.centerCol;

        // Block the two cells flanking the diagonal step from (r,c) to (r+1,c+1).
        blockCell(grid, r, c + 1);
        blockCell(grid, r + 1, c);
        GridSnapshot snap = grid.snapshot();

        assertTrue("setup: flanking cells blocked", snap.isBlocked(r, c + 1));
        assertTrue("setup: flanking cells blocked", snap.isBlocked(r + 1, c));
        assertFalse("setup: endpoints free", snap.isBlocked(r, c));
        assertFalse("setup: endpoints free", snap.isBlocked(r + 1, c + 1));

        assertFalse("a zero-width diagonal gap is not drivable",
                snap.hasLineOfSight(new GridPoint(r, c), new GridPoint(r + 1, c + 1)));
    }

    @Test
    public void lineOfSightAllowsGrazingASingleCorner() {
        FieldGrid grid = pointGrid();
        GridGeometry geo = grid.geometry();
        int r = geo.centerRow;
        int c = geo.centerCol;

        blockCell(grid, r, c + 1); // only one flank blocked
        GridSnapshot snap = grid.snapshot();

        assertTrue("one open flank is enough room",
                snap.hasLineOfSight(new GridPoint(r, c), new GridPoint(r + 1, c + 1)));
    }

    @Test
    public void lineOfSightIsFalseWhenEitherEndpointIsBlocked() {
        FieldGrid grid = new FieldGrid();
        grid.setStaticObstacle(30.0, 30.0, 2.0);
        GridSnapshot snap = grid.snapshot();

        assertFalse(snap.hasLineOfSight(new Vector2d(0.0, 0.0), new Vector2d(30.0, 30.0)));
        assertFalse(snap.hasLineOfSight(new Vector2d(30.0, 30.0), new Vector2d(0.0, 0.0)));
    }

    /**
     * The safety property the whole smoother rests on: line of sight must never report a
     * segment clear when it actually crosses a blocked cell. Checked against a densely
     * sampled ground truth over a cluttered field.
     *
     * <p>This is a regression test. A Bresenham walk visits only {@code max(|dr|,|dc|)}
     * cells and so skips the intermediate cell on every non-diagonal step; measured this
     * way it wrongly cleared ~0.7% of segments. Only a supercover traversal passes.
     */
    @Test
    public void lineOfSightNeverClearsASegmentThatCrossesAnObstacle() {
        GridSnapshot snap = clutteredField();
        Random rnd = new Random(20260731L);

        int unsafe = 0;
        int checked = 0;
        String firstFailure = null;

        for (int i = 0; i < 40000; i++) {
            GridPoint a = new GridPoint(rnd.nextInt(snap.rows()), rnd.nextInt(snap.cols()));
            GridPoint b = new GridPoint(rnd.nextInt(snap.rows()), rnd.nextInt(snap.cols()));
            if (snap.isBlocked(a) || snap.isBlocked(b)) continue;
            checked++;

            if (snap.hasLineOfSight(a, b) && !denselySampledLineIsClear(snap, a, b)) {
                unsafe++;
                if (firstFailure == null) firstFailure = a + " -> " + b;
            }
        }

        assertTrue("sanity: expected a meaningful number of open pairs", checked > 10000);
        assertEquals("line of sight cleared " + unsafe + " segments that cross an obstacle;"
                + " first was " + firstFailure, 0, unsafe);
    }

    /** The cells a segment crosses depend on its geometry, not on which end you start from. */
    @Test
    public void lineOfSightIsSymmetric() {
        GridSnapshot snap = clutteredField();
        Random rnd = new Random(99L);

        for (int i = 0; i < 50000; i++) {
            GridPoint a = new GridPoint(rnd.nextInt(snap.rows()), rnd.nextInt(snap.cols()));
            GridPoint b = new GridPoint(rnd.nextInt(snap.rows()), rnd.nextInt(snap.cols()));
            assertEquals("asymmetric for " + a + " <-> " + b,
                    snap.hasLineOfSight(a, b), snap.hasLineOfSight(b, a));
        }
    }

    /**
     * A shallow-angle segment crosses cells that a Bresenham walk skips entirely. One
     * blocked cell placed in that gap is enough to tell the two apart.
     */
    @Test
    public void lineOfSightSeesCellsAShallowAngleWalkWouldSkip() {
        FieldGrid grid = pointGrid();
        GridGeometry geo = grid.geometry();
        int r = geo.centerRow;
        int c = geo.centerCol;

        // Segment from (r,c) to (r+1,c+2): it passes through (r,c+1) AND (r+1,c+1).
        // Bresenham visits only one of the two.
        blockCell(grid, r + 1, c + 1);
        GridSnapshot snap = grid.snapshot();

        assertFalse("endpoint cells must be open for a meaningful test", snap.isBlocked(r, c));
        assertFalse(snap.isBlocked(r + 1, c + 2));
        assertFalse("the other intermediate cell is open", snap.isBlocked(r, c + 1));

        assertFalse("segment crosses the blocked cell and must not be reported clear",
                snap.hasLineOfSight(new GridPoint(r, c), new GridPoint(r + 1, c + 2)));
    }

    @Test
    public void nearestFreeEscapesAnObstacleTheRobotIsStandingIn() {
        FieldGrid grid = new FieldGrid();
        grid.setObstacle(0.0, 0.0, 10.0);
        GridSnapshot snap = grid.snapshot();

        GridPoint stuck = grid.fieldToGrid(0.0, 0.0);
        assertTrue(snap.isBlocked(stuck));

        GridPoint escape = snap.nearestFree(stuck, -1.0);
        assertNotNull("must find a way out", escape);
        assertFalse(snap.isBlocked(escape));

        // 10" obstacle + 9" robot + quantization margin gives a 20.4" inflated radius, so
        // the escape must clear that, and BFS must not wander far past it.
        Vector2d where = snap.geometry.gridToField(escape);
        double distance = Math.sqrt(where.x * where.x + where.y * where.y);
        assertTrue("escape is still inside the inflated radius: " + distance,
                distance > 20.4);
        assertTrue("escape wandered too far from the robot: " + distance,
                distance < 32.0);
    }

    /**
     * Regression test: the escape must be the true Euclidean nearest open cell. A plain
     * ring-order (Chebyshev) search returns whatever the first ring happens to contain,
     * which on a round obstacle pops out diagonally and measured up to 16% too far.
     */
    @Test
    public void nearestFreeIsExactlyTheEuclideanNearestOpenCell() {
        Random rnd = new Random(7L);

        for (int trial = 0; trial < 300; trial++) {
            FieldGrid grid = new FieldGrid(2.0, 9.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 0.0, 1);
            double ox = rnd.nextDouble() * 80.0 - 40.0;
            double oy = rnd.nextDouble() * 80.0 - 40.0;
            grid.setObstacle(ox, oy, rnd.nextDouble() * 20.0);

            GridSnapshot snap = grid.snapshot();
            GridPoint origin = grid.fieldToGrid(ox, oy);
            if (!snap.isBlocked(origin)) continue;

            GridPoint got = snap.nearestFree(origin, -1.0);
            assertNotNull("trial " + trial + ": no escape found", got);

            long bestSq = Long.MAX_VALUE;
            for (int r = 0; r < snap.rows(); r++) {
                for (int c = 0; c < snap.cols(); c++) {
                    if (snap.isBlocked(r, c)) continue;
                    long dr = r - origin.row;
                    long dc = c - origin.col;
                    bestSq = Math.min(bestSq, dr * dr + dc * dc);
                }
            }
            long gotSq = (long) (got.row - origin.row) * (got.row - origin.row)
                    + (long) (got.col - origin.col) * (got.col - origin.col);

            assertEquals("trial " + trial + " (obstacle at " + ox + "," + oy + ")"
                    + " returned a cell further than the true nearest", bestSq, gotSq);
        }
    }

    @Test
    public void nearestFreeReturnsTheCellItselfWhenAlreadyClear() {
        GridSnapshot snap = new FieldGrid().snapshot();
        GridPoint here = snap.geometry.fieldToGrid(20.0, -20.0);
        assertEquals(here, snap.nearestFree(here, 12.0));
    }

    @Test
    public void nearestFreeRespectsItsSearchLimitAndGivesUp() {
        FieldGrid grid = new FieldGrid();
        grid.setObstacle(0.0, 0.0, 30.0);
        GridSnapshot snap = grid.snapshot();

        GridPoint stuck = grid.fieldToGrid(0.0, 0.0);
        assertNull("should give up rather than search the whole field",
                snap.nearestFree(stuck, 6.0));
        assertNotNull(snap.nearestFree(stuck, 60.0));
    }

    @Test
    public void nearestFreeReturnsNullOnAFullyBlockedField() {
        FieldGrid grid = new FieldGrid();
        // No cell center falls inside this sliver (centers sit on even inches), so the
        // "keep only what is inside" region is empty and the whole matrix is blocked.
        grid.blockOutside(0.5, 0.5, 1.5, 1.5, FieldGrid.Layer.STATIC);
        GridSnapshot snap = grid.snapshot();
        assertEquals(snap.rows() * snap.cols(), snap.blockedCellCount());
        assertNull(snap.nearestFree(grid.fieldToGrid(0.0, 0.0), -1.0));
    }

    @Test
    public void asciiRenderingHasOneLinePerRow() {
        FieldGrid grid = new FieldGrid(4.0);
        grid.setStaticObstacle(0.0, 0.0, 6.0);
        String[] lines = grid.snapshot().toAscii().split("\n");

        assertEquals(grid.rows() + 1, lines.length); // header + one line per row
        for (int i = 1; i < lines.length; i++) {
            assertEquals("row " + (i - 1) + " width", grid.cols(), lines[i].length());
        }
        assertTrue("blocked cells should render", lines[1 + grid.rows() / 2].contains("#"));
    }

    // ------------------------------------------------------------ concurrency

    /**
     * Milestone 4 leans on this: a sensor thread hammering the map must never let a reader
     * observe a torn or inconsistent view, and neither side may block the other.
     */
    @Test
    public void concurrentWritersNeverExposeATornMapToReaders() throws Exception {
        final FieldGrid grid = new FieldGrid();
        grid.setStaticObstacle(-50.0, -50.0, 6.0);
        final GridPoint staticCell = grid.fieldToGrid(-50.0, -50.0);

        final int writers = 4;
        final int readers = 4;
        final AtomicBoolean stop = new AtomicBoolean(false);
        final AtomicReference<Throwable> failure = new AtomicReference<Throwable>(null);
        final CountDownLatch ready = new CountDownLatch(writers + readers);
        final CountDownLatch go = new CountDownLatch(1);
        List<Thread> threads = new ArrayList<Thread>();

        for (int w = 0; w < writers; w++) {
            final int id = w;
            threads.add(new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        ready.countDown();
                        go.await();
                        int i = 0;
                        while (!stop.get()) {
                            double x = ((i + id * 7) % 100) - 50.0;
                            grid.setObstacle(x, 20.0, 5.0);
                            if (i % 16 == 0) grid.clearDynamicObstacles();
                            i++;
                        }
                    } catch (Throwable t) {
                        failure.compareAndSet(null, t);
                    }
                }
            }, "hivemind-writer-" + w));
        }

        for (int r = 0; r < readers; r++) {
            threads.add(new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        ready.countDown();
                        go.await();
                        long lastVersion = -1L;
                        while (!stop.get()) {
                            GridSnapshot snap = grid.snapshot();

                            // Versions handed to a reader must never run backwards.
                            if (snap.version < lastVersion) {
                                throw new AssertionError("version went backwards: "
                                        + snap.version + " after " + lastVersion);
                            }
                            lastVersion = snap.version;

                            // Static geometry is never touched by the writers, so every
                            // snapshot must still contain it. A torn composite would drop it.
                            if (!snap.isBlocked(staticCell)) {
                                throw new AssertionError("static obstacle missing from v" + snap.version);
                            }

                            // Every cost must be a legal value.
                            for (int idx = 0; idx < snap.rows() * snap.cols(); idx += 37) {
                                int cost = snap.costAtIndex(idx);
                                if (cost < 0 || cost > 255) {
                                    throw new AssertionError("illegal cost " + cost);
                                }
                            }

                            // A held snapshot must be stable across repeated reads.
                            int first = snap.blockedCellCount();
                            if (first != snap.blockedCellCount()) {
                                throw new AssertionError("snapshot mutated while held");
                            }
                        }
                    } catch (Throwable t) {
                        failure.compareAndSet(null, t);
                    }
                }
            }, "hivemind-reader-" + r));
        }

        for (Thread t : threads) t.start();
        assertTrue("threads failed to start", ready.await(5, TimeUnit.SECONDS));
        go.countDown();
        Thread.sleep(750);
        stop.set(true);
        for (Thread t : threads) {
            t.join(5000);
            if (t.isAlive()) fail("thread " + t.getName() + " did not finish - possible deadlock");
        }

        if (failure.get() != null) {
            throw new AssertionError("concurrent access failed", failure.get());
        }
    }

    @Test
    public void readsStayFastEnoughForTheTwentyMillisecondBudget() {
        FieldGrid grid = new FieldGrid();
        grid.addPerimeterWalls(FieldGrid.Layer.STATIC);
        grid.setStaticObstacle(0.0, 0.0, 12.0);
        GridSnapshot snap = grid.snapshot();

        int cells = snap.rows() * snap.cols();
        long start = System.nanoTime();
        int sink = 0;
        for (int pass = 0; pass < 200; pass++) {
            for (int i = 0; i < cells; i++) {
                sink += snap.costAtIndex(i);
            }
        }
        long elapsedMs = (System.nanoTime() - start) / 1_000_000L;

        assertTrue("sanity", sink != 0);
        // 200 full sweeps of the map is far more work than one A* search does.
        assertTrue("cost lookups too slow: " + elapsedMs + "ms for 200 full sweeps",
                elapsedMs < 1000);
    }

    // ------------------------------------------------------------ validation

    @Test
    public void invalidConstructorArgumentsAreRejected() {
        assertThrows(IllegalArgumentException.class, new Runnable() {
            public void run() { new FieldGrid(0.0); }
        });
        assertThrows(IllegalArgumentException.class, new Runnable() {
            public void run() { new FieldGrid(-2.0); }
        });
        assertThrows(IllegalArgumentException.class, new Runnable() {
            public void run() { new FieldGrid(2.0, -1.0); }
        });
        assertThrows(IllegalArgumentException.class, new Runnable() {
            public void run() { new FieldGrid(2.0, 9.0, 144.0, 6.0, 0); }
        });
        assertThrows(IllegalArgumentException.class, new Runnable() {
            public void run() { new FieldGrid(2.0, 9.0, 144.0, 6.0, 255); }
        });
    }

    @Test
    public void negativeObstacleRadiusIsRejected() {
        final FieldGrid grid = new FieldGrid();
        assertThrows(IllegalArgumentException.class, new Runnable() {
            public void run() { grid.setObstacle(0.0, 0.0, -1.0); }
        });
    }

    // ------------------------------------------------------------ test helpers

    /** A grid with no robot inflation and no soft band: one stamp marks exactly one cell. */
    private static FieldGrid pointGrid() {
        return new FieldGrid(2.0, 0.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 0.0, 1);
    }

    /** A repeatable messy field: 25 scattered obstacles, roughly a quarter of it blocked. */
    private static GridSnapshot clutteredField() {
        Random rnd = new Random(42L);
        FieldGrid grid = new FieldGrid(2.0, 4.0, FieldGrid.FTC_FIELD_SIZE_INCHES, 4.0, 50);
        for (int i = 0; i < 25; i++) {
            grid.setStaticObstacle(rnd.nextDouble() * 120.0 - 60.0,
                    rnd.nextDouble() * 120.0 - 60.0,
                    rnd.nextDouble() * 6.0);
        }
        return grid.snapshot();
    }

    /**
     * Ground truth for line of sight: walk the continuous segment at 20 samples per cell
     * and report whether every sample lands in an open cell. Slow, but derived from the
     * segment's geometry rather than from the traversal being tested.
     *
     * <p>Samples that land exactly on a grid corner are skipped. Such a point lies in no
     * cell's <i>interior</i> &mdash; it is shared by four cells at once, and which one
     * {@code fieldToGrid} rounds it into is a tie-break convention rather than a fact about
     * where the segment goes. Counting those would test the rounding rule instead of the
     * traversal. Corner crossings are covered directly and deliberately by
     * {@link #lineOfSightRefusesToSqueezeThroughADiagonalGap} and
     * {@link #lineOfSightAllowsGrazingASingleCorner}.
     */
    private static boolean denselySampledLineIsClear(GridSnapshot snap, GridPoint a, GridPoint b) {
        Vector2d pa = snap.geometry.gridToField(a);
        Vector2d pb = snap.geometry.gridToField(b);
        double dx = pb.x - pa.x;
        double dy = pb.y - pa.y;
        double res = snap.resolutionInches();
        double length = Math.sqrt(dx * dx + dy * dy);
        int samples = Math.max(1, (int) (length / (res / 20.0)));

        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;
            double px = pa.x + dx * t;
            double py = pa.y + dy * t;

            double u = px / res + 0.5;
            double v = py / res + 0.5;
            boolean onVerticalEdge = Math.abs(u - Math.rint(u)) < 1e-9;
            boolean onHorizontalEdge = Math.abs(v - Math.rint(v)) < 1e-9;
            if (onVerticalEdge && onHorizontalEdge) continue; // exact corner, no interior

            if (snap.isBlocked(snap.geometry.fieldToGrid(px, py))) {
                return false;
            }
        }
        return true;
    }

    private static void blockCell(FieldGrid grid, int row, int col) {
        Vector2d center = grid.gridToField(row, col);
        grid.setObstacle(center.x, center.y, 0.0, FieldGrid.Layer.STATIC);
    }

    private static void assertThrows(Class<? extends Throwable> expected, Runnable action) {
        try {
            action.run();
        } catch (Throwable actual) {
            if (expected.isInstance(actual)) return;
            throw new AssertionError("expected " + expected.getSimpleName()
                    + " but got " + actual, actual);
        }
        fail("expected " + expected.getSimpleName() + " but nothing was thrown");
    }
}
