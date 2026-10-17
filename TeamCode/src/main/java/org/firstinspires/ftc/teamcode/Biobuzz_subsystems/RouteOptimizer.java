package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Picks the FASTEST safe way to drive a runtime route, by building several candidates and letting
 * Road Runner's own time-optimal profile (with the auto's tuned velocity / acceleration limits)
 * time each one.
 *
 * <h2>Why this beats strafing</h2>
 * On this drivetrain strafing is expensive: the lateral multiplier is ~1.27, so the same wheel speed
 * gives ~27% less speed sideways, and diagonal travel plus turning eats the acceleration budget.
 * Turning in place is slow too (180 deg/s, 180 deg/s^2). Fast teams therefore drive along the
 * direction of travel (forward or backward), turn WHILE moving, and never stop at corners. The
 * candidates below are exactly those options; the profiler decides which is quickest here.
 *
 * <h2>Candidates</h2>
 * All go through the same safe corner points chosen by {@link HiveAutoKit#plan}:
 * <ul>
 *   <li><b>stops</b>: straight strafes, stopping at each corner (the old behaviour; always valid,
 *       used as the fallback).</li>
 *   <li><b>smooth</b>: one continuous spline through the corners (tangent along the corner's
 *       bisector, so it bows OUTWARD, away from the HIVE), with the heading at each corner chosen
 *       from: the target heading, aligned with the incoming / outgoing direction (forward or
 *       backward), or unchanged (move first, turn later - for starts next to a wall).</li>
 *   <li><b>smooth+mid</b>: the same, plus a pose in the middle of every long straight with the
 *       heading ALIGNED to the travel direction - the robot turns early, drives straight forward or
 *       backward along the long part, and turns again near the end.</li>
 *   <li><b>direct aligned</b>: no corners needed - drive straight with an aligned midpoint, or
 *       keep the start heading to the midpoint and turn on the second half.</li>
 * </ul>
 * Every candidate is sampled and must keep the robot centre {@link HiveAutoKit#requiredClearance()}
 * from the HIVE, the whole footprint inside the field walls, and clear of our two FLOWERS (never
 * worse than the start/end poses themselves, which may be right next to a wall or FLOWER).
 *
 * Runtime searches are time-capped ({@link #MAX_SEARCH_S}); INIT can prepare routes with no cap
 * (HiveAutoKit.prepare) and the winning pattern is tried first at runtime.
 *
 * <h2>FLOWER approach</h2>
 * With an {@code approach} point, {@code to} is a staging pose facing that point: every smooth
 * candidate arrives there already travelling along the staging heading and carries straight on to
 * the approach point in the same trajectory, so the robot drives head-on into the FLOWER without
 * stopping ("stops" still stops at the staging pose).
 *
 * With a {@code depart} point the route first drives straight back from {@code from} to it at
 * constant heading (out of a FLOWER, ramp still down), then carries on from it without stopping;
 * every pattern is also tried with a stop there (suffix "!"), which can turn more freely.
 */
public final class RouteOptimizer {

    /** Longest a runtime search may spend building candidates, SECONDS (always builds 2+). */
    public static double MAX_SEARCH_S = 0.08;
    /** Straights longer than this get an aligned midpoint in "smooth+mid", INCHES. */
    public static double LONG_STRAIGHT_IN = 40.0;
    /** Gap kept from the field walls and FLOWERS (unless an end pose is already closer), INCHES. */
    public static double OBSTACLE_GAP_IN = 1.0;
    /** Our two FLOWERS in the alliance frame (the wall-mounted ones on our side), INCHES. */
    public static double[][] FLOWERS = {{-70, -24}, {24, -70}};
    public static double FLOWER_RADIUS_IN = 4.5;
    /** Field half-size, INCHES. */
    public static double FIELD_HALF_IN = 72.0;

    private static final String[] CHOICES = {"T", "Fi", "Ri", "Fo", "Ro", "H"};

    private RouteOptimizer() {
    }

    /** What the search needs from the robot (real) or the offline test harness. */
    public interface Env {
        TrajectoryActionBuilder builder(Pose2d start);

        List<Pose2d> poses(Action a);

        double seconds(Action a);
    }

    public static final class Result {
        public final Action action;
        public final String pattern;
        public final double seconds;
        public final int built;

        Result(Action action, String pattern, double seconds, int built) {
            this.action = action;
            this.pattern = pattern;
            this.seconds = seconds;
            this.built = built;
        }
    }

    /** Every pattern for a route with {@code corners} corner points, best-first by experience. */
    public static List<String> patterns(int corners) {
        List<String> out = new ArrayList<>();
        int combos = (int) Math.pow(CHOICES.length, corners);
        for (String fam : new String[]{"M", "S"}) {
            for (int c = 0; c < combos; c++) {
                StringBuilder sb = new StringBuilder(fam).append(':');
                int cc = c;
                for (int i = 0; i < corners; i++) {
                    if (i > 0) sb.append(',');
                    sb.append(CHOICES[cc % CHOICES.length]);
                    cc /= CHOICES.length;
                }
                out.add(sb.toString());
            }
        }
        if (corners == 0) {
            out.add("D:F");
            out.add("D:R");
            out.add("D:H");
        }
        return out;
    }

    /**
     * Builds "stops" first (the safe fallback) and {@code preferred} (if any), then other patterns
     * until maxSearchS has passed, and returns the fastest valid one.
     */
    public static Result search(Pose2d from, Pose2d to, HiveAutoKit.Plan plan, Env env,
                                VelConstraint vel, AccelConstraint accel, String preferred, double maxSearchS) {
        return search(from, to, null, plan, env, vel, accel, preferred, maxSearchS);
    }

    /** {@link #search} ending with a straight run from {@code to} on to {@code approach} (null = none). */
    public static Result search(Pose2d from, Pose2d to, Vector2d approach, HiveAutoKit.Plan plan, Env env,
                                VelConstraint vel, AccelConstraint accel, String preferred, double maxSearchS) {
        return search(from, null, to, approach, plan, env, vel, accel, preferred, maxSearchS);
    }

    /**
     * {@link #search} starting with a straight run back from {@code from} to {@code depart} and
     * ending with one from {@code to} on to {@code approach} (either null = none). With a depart
     * point, {@code plan} runs from it.
     */
    public static Result search(Pose2d from, Vector2d depart, Pose2d to, Vector2d approach, HiveAutoKit.Plan plan, Env env,
                                VelConstraint vel, AccelConstraint accel, String preferred, double maxSearchS) {
        long start = System.nanoTime();
        Pose2d end = approach == null ? to : new Pose2d(approach, to.heading);
        Vector2d escape = plan.escaped ? plan.points.get(0) : null;
        List<Vector2d> corners = plan.escaped ? plan.points.subList(1, plan.points.size()) : plan.points;

        Action stops = build("stops", from, depart, to, escape, corners, approach, env, vel, accel);
        Result best = new Result(stops, "stops", env.seconds(stops), 1);
        double bestMargin = margin(env.poses(stops), from, end, escape);
        double stopsSeconds = best.seconds;
        int built = 1;

        List<String> order = new ArrayList<>();
        if (preferred != null && !preferred.equals("stops")) order.add(preferred);
        for (String p : patterns(corners.size())) {
            if (!p.equals(preferred)) order.add(p);
            if (depart != null && !(p + "!").equals(preferred)) order.add(p + "!");
        }

        for (String p : order) {
            if (built >= 2 && (System.nanoTime() - start) / 1e9 > maxSearchS) break;
            try {
                Action a = build(p, from, depart, to, escape, corners, approach, env, vel, accel);
                built++;
                double s = env.seconds(a);
                boolean bestOk = bestMargin >= -1e-6;
                if (bestOk && s >= best.seconds) continue;
                // Never slower than "stops": the park watchdog times "stops" as its upper bound.
                if (s > stopsSeconds) continue;
                double m = margin(env.poses(a), from, end, escape);
                boolean ok = m >= -1e-6;
                // Fastest of the valid ones; if none is valid yet, the one that stays clearest.
                if ((ok && (!bestOk || s < best.seconds)) || (!ok && !bestOk && m > bestMargin)) {
                    best = new Result(a, p, s, built);
                    bestMargin = m;
                }
            } catch (Exception badPattern) {
                built++;
            }
        }
        return new Result(best.action, best.pattern, best.seconds, built);
    }

    /** Builds one pattern (see the class comment). */
    public static Action build(String pattern, Pose2d from, Pose2d to, Vector2d escape, List<Vector2d> corners,
                               Env env, VelConstraint vel, AccelConstraint accel) {
        return build(pattern, from, to, escape, corners, null, env, vel, accel);
    }

    /** {@link #build} ending with a straight run from {@code to} on to {@code approach} (null = none). */
    public static Action build(String pattern, Pose2d from, Pose2d to, Vector2d escape, List<Vector2d> corners,
                               Vector2d approach, Env env, VelConstraint vel, AccelConstraint accel) {
        return build(pattern, from, null, to, escape, corners, approach, env, vel, accel);
    }

    /** {@link #build} with a straight run back from {@code from} to {@code depart} first (null = none). */
    public static Action build(String pattern, Pose2d from, Vector2d depart, Pose2d to, Vector2d escape, List<Vector2d> corners,
                               Vector2d approach, Env env, VelConstraint vel, AccelConstraint accel) {
        double h0 = from.heading.toDouble(), hE = to.heading.toDouble();
        TrajectoryActionBuilder b = env.builder(from);
        Vector2d first = from.position;
        boolean moving = false;
        boolean stopAtDepart = pattern.endsWith("!");
        if (stopAtDepart) pattern = pattern.substring(0, pattern.length() - 1);
        if (depart != null) {
            if (stopAtDepart) {
                b = b.strafeToConstantHeading(depart, vel, accel);
            } else {
                double back = angle(from.position, depart);
                b = b.setTangent(back).splineToConstantHeading(depart, back, vel, accel);
                moving = true;
            }
            first = depart;
        }
        if (escape != null) {
            moving = false;
            b = b.strafeToLinearHeading(escape, h0, vel, accel);
            first = escape;
        }
        List<Vector2d> all = new ArrayList<>();
        all.add(first);
        all.addAll(corners);
        all.add(to.position);
        int n = corners.size();

        if (pattern.equals("stops")) {
            for (int i = 1; i <= n; i++) b = b.strafeToLinearHeading(all.get(i), near(hE, h0), vel, accel);
            b = b.strafeToLinearHeading(to.position, near(hE, h0), vel, accel);
            if (approach != null) b = b.strafeToLinearHeading(approach, near(hE, h0), vel, accel);
            return b.build();
        }

        String fam = pattern.substring(0, 1);
        String[] ch = pattern.length() > 2 ? pattern.substring(2).split(",") : new String[0];
        if (fam.equals("D")) {
            double d = angle(first, to.position);
            double hm = ch[0].equals("H") ? h0 : near(d + (ch[0].equals("R") ? Math.PI : 0), h0);
            Vector2d mid = new Vector2d((first.x + to.position.x) / 2, (first.y + to.position.y) / 2);
            if (!moving) b = b.setTangent(d);
            b = b.splineToSplineHeading(new Pose2d(mid, hm), d, vel, accel)
                    .splineToSplineHeading(new Pose2d(to.position, near(hE, hm)), approach == null ? d : hE, vel, accel);
            if (approach != null) b = b.splineToConstantHeading(approach, hE, vel, accel);
            return b.build();
        }

        double[] hs = new double[n + 2];
        hs[0] = h0;
        for (int i = 1; i <= n; i++) {
            double din = angle(all.get(i - 1), all.get(i)), dout = angle(all.get(i), all.get(i + 1));
            String c = ch[i - 1];
            double target = c.equals("T") ? hE : c.equals("Fi") ? din : c.equals("Ri") ? din + Math.PI
                    : c.equals("Fo") ? dout : c.equals("Ro") ? dout + Math.PI : hs[i - 1];
            hs[i] = near(target, hs[i - 1]);
        }
        hs[n + 1] = near(hE, hs[n]);

        if (!moving) b = b.setTangent(angle(all.get(0), all.get(1)));
        for (int i = 1; i <= n + 1; i++) {
            Vector2d p = all.get(i), q = all.get(i - 1);
            double d = angle(q, p);
            if (fam.equals("M") && dist(p, q) > LONG_STRAIGHT_IN) {
                double f = near(d, hs[i - 1]), r = near(d + Math.PI, hs[i - 1]);
                double hm = turnCost(hs[i - 1], f, hs[i]) <= turnCost(hs[i - 1], r, hs[i]) ? f : r;
                b = b.splineToSplineHeading(new Pose2d(new Vector2d((p.x + q.x) / 2, (p.y + q.y) / 2), hm), d, vel, accel);
                hs[i] = near(hs[i], hm);
            }
            double tangent = i <= n ? d + wrap(angle(p, all.get(i + 1)) - d) / 2 : approach == null ? d : hE;
            b = b.splineToSplineHeading(new Pose2d(p, hs[i]), tangent, vel, accel);
        }
        if (approach != null) b = b.splineToConstantHeading(approach, hE, vel, accel);
        return b.build();
    }

    /** True when {@link #margin} is not negative. */
    public static boolean valid(List<Pose2d> poses, Pose2d from, Pose2d to, Vector2d escape) {
        return margin(poses, from, to, escape) >= -1e-6;
    }

    /**
     * Worst margin along the path, INCHES (negative = breaks a rule): HIVE clearance everywhere off
     * the escape move; walls and FLOWERS no closer than {@link #OBSTACLE_GAP_IN}, or than the end
     * poses already are.
     */
    public static double margin(List<Pose2d> poses, Pose2d from, Pose2d to, Vector2d escape) {
        if (poses.isEmpty()) return Double.NEGATIVE_INFINITY;
        double req = HiveAutoKit.requiredClearance();
        double wallAllow = Math.min(OBSTACLE_GAP_IN, Math.min(wallGap(from), wallGap(to)));
        double flowerAllow = Math.min(OBSTACLE_GAP_IN, Math.min(flowerGap(from), flowerGap(to)));
        double worst = Double.MAX_VALUE;
        for (Pose2d p : poses) {
            boolean onEscape = escape != null && pointToSegment(p.position, from.position, escape) < 0.5;
            if (!onEscape) worst = Math.min(worst, HiveAutoKit.pointClearance(p.position) - req);
            worst = Math.min(worst, wallGap(p) - wallAllow);
            worst = Math.min(worst, flowerGap(p) - flowerAllow);
        }
        return worst;
    }

    /** Smallest gap between the robot footprint and the field walls, INCHES (negative = through). */
    public static double wallGap(Pose2d p) {
        double worst = Double.MAX_VALUE;
        for (Vector2d c : corners(p)) {
            worst = Math.min(worst, FIELD_HALF_IN - Math.max(Math.abs(c.x), Math.abs(c.y)));
        }
        return worst;
    }

    /** Smallest gap between the robot footprint and our FLOWERS (as circles), INCHES. */
    public static double flowerGap(Pose2d p) {
        Vector2d[] c = corners(p);
        double worst = Double.MAX_VALUE;
        for (double[] f : FLOWERS) {
            Vector2d fc = new Vector2d(f[0], f[1]);
            double d = Double.MAX_VALUE;
            for (int i = 0; i < 4; i++) d = Math.min(d, pointToSegment(fc, c[i], c[(i + 1) % 4]));
            if (insideRect(fc, c)) d = 0;
            worst = Math.min(worst, d - FLOWER_RADIUS_IN);
        }
        return worst;
    }

    private static Vector2d[] corners(Pose2d p) {
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

    private static boolean insideRect(Vector2d p, Vector2d[] c) {
        boolean pos = false, neg = false;
        for (int i = 0; i < 4; i++) {
            Vector2d a = c[i], b = c[(i + 1) % 4];
            double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            if (cross > 0) pos = true;
            if (cross < 0) neg = true;
        }
        return !(pos && neg);
    }

    private static double turnCost(double a, double mid, double b) {
        return Math.abs(mid - a) + Math.abs(wrap(b - mid));
    }

    private static double pointToSegment(Vector2d p, Vector2d a, Vector2d b) {
        double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        double t = len2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
        return Math.hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
    }

    /** {@code target} unwrapped to the nearest equivalent angle to {@code prev}. */
    private static double near(double target, double prev) {
        return prev + wrap(target - prev);
    }

    private static double angle(Vector2d a, Vector2d b) {
        return Math.atan2(b.y - a.y, b.x - a.x);
    }

    private static double dist(Vector2d a, Vector2d b) {
        return Math.hypot(b.x - a.x, b.y - a.y);
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
