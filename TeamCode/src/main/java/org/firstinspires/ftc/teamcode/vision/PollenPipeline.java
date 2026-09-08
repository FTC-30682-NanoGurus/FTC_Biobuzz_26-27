package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Finds pollen (spherical game elements) and reports the biggest PILE of them, in inches relative
 * to the robot.
 *
 * <h2>Pipeline</h2>
 * <ol>
 *   <li>Optional downscale, blur, RGB -&gt; HSV, colour threshold, morphological open then close.</li>
 *   <li>External contours. Each survivor is projected to the floor through
 *       {@link PollenGeometry} and gets an estimated BALL COUNT, not just a yes/no.</li>
 *   <li>Single-link clustering of those floor points, in INCHES.</li>
 *   <li>The cluster with the best score is published for the drive code to aim at.</li>
 * </ol>
 *
 * <h2>Merged blobs are the whole problem</h2>
 * "The largest cluster of pollen" is by definition balls that are touching, and touching balls
 * share one contour. A pipeline that filters on circularity throws that contour away - it rejects
 * exactly the thing it was asked to find, and reports the loose ball off in the corner instead.
 *
 * So each contour is compared against the area a SINGLE ball would have at that spot on the floor
 * ({@link PollenGeometry.Model#expectedRadiusPx}), and the ratio decides how it is treated:
 * <ul>
 *   <li>near 1 - one ball. It must also look round, so circularity and circle-fill are enforced.</li>
 *   <li>well above 1 - several balls fused together. Circularity is dropped (a clump is not round
 *       and never will be) and SOLIDITY is enforced instead, which still rejects the stringy,
 *       ragged blobs that lighting artefacts produce. It counts as {@code round(ratio)} balls at
 *       its centroid.</li>
 *   <li>well below 1 - too small to be pollen at that range. Discarded.</li>
 * </ul>
 * The count is an estimate. A pile two balls deep reads as fewer balls than it holds, because the
 * back row is hidden. That is fine for the job: it only has to rank piles against each other, and
 * occlusion hurts every candidate the same way.
 *
 * <h2>Clustering in inches, not pixels</h2>
 * Two balls 8" apart at the far end of the field are a handful of pixels apart; the same two balls
 * at the robot's feet are hundreds. Any pixel-space distance threshold therefore means a different
 * real distance in every part of the image, and will happily merge the entire far half of the
 * field into one "cluster". Clustering after projection removes the perspective from the problem
 * entirely, and {@link #CLUSTER_LINK_IN} then means one honest thing everywhere in frame.
 *
 * <h2>Threading</h2>
 * {@link #processFrame} runs on the VisionPortal thread. Results are published as one immutable
 * {@link Result} through a volatile field, so the opmode thread can never see a half-written
 * detection list. Read it with {@link #getResult()}; it is never null.
 */
@Config
public class PollenPipeline implements VisionProcessor {

    // =======================================================================================
    // COLOUR - green DECODE ARTIFACT. Tune these FIRST; nothing downstream fixes a bad mask.
    // OpenCV hue is 0..179, not 0..359.
    //
    // The band is 50 hue units wide, running from yellow-green at 35 to cyan-green at 85. That
    // width is forgiving of white-balance drift; it is also the first thing to narrow (try 40..80)
    // if the mask picks up yellow or teal field elements. Saturation is far more selective than
    // hue, so if the mask view (DPAD RIGHT) lights up on things that are not artifacts, raise
    // S_LOW before touching the hue bounds. Magenta, for the purple artifact, was H 130..165.
    //
    // A NOTE ON COLOUR SPACE. FIRST does not tune its own ARTIFACT_GREEN swatch in HSV - the
    // predefined ColorRanges in the SDK's ColorBlobLocatorProcessor are all YCrCb (verified by
    // decompiling ColorRange from the Vision AAR: GREEN is YCrCb Y 32..255, Cr 0..120, Cb 0..133).
    // YCrCb puts all the brightness in one channel it can then ignore entirely, which handles the
    // artifact's glossy specular highlight better than HSV does, because a highlight collapses
    // saturation but barely moves chroma. This pipeline stays in HSV because that is what the
    // rest of this codebase uses and what these numbers are expressed in - but if the mask keeps
    // punching a hole through the middle of every ball under bright light, switching this one
    // threshold to YCrCb is the fix, not more HSV tuning.
    // =======================================================================================
    public static double H_LOW = 35;
    public static double H_HIGH = 85;
    public static double S_LOW = 50;
    public static double S_HIGH = 255;
    public static double V_LOW = 50;
    public static double V_HIGH = 255;

    // =======================================================================================
    // IMAGE PROCESSING
    // =======================================================================================
    /**
     * Process at this fraction of the incoming frame. 0.5 is ~4x cheaper and still plenty.
     *
     * Left at 0.5 deliberately: a 640x480 stream downscales to 320x240, which is the resolution
     * FIRST's own DECODE artifact sample runs at. That means BLUR_PX, CLOSE_PX and OPEN_PX below can be
     * taken straight from FIRST's published numbers with no rescaling.
     */
    public static double PROC_SCALE = 0.5;
    /** Gaussian blur kernel, odd, in PROCESSING pixels. 0 or 1 disables. FIRST also uses 5. */
    public static int BLUR_PX = 5;
    /**
     * Morphological kernel, odd, in PROCESSING pixels. 0 or 1 disables.
     *
     * This one matters more than it looks. A DECODE artifact is not a solid ball - it is a hollow
     * Gopher ResisDent polyethylene ball with LARGE wiffle-style cutouts, so the raw colour mask
     * of one artifact is a lattice with big dark gaps, not a disc. Glossy plastic adds a specular
     * highlight that punches a second, desaturated hole through the middle. Both have to be sealed
     * before the shape gates below mean anything: unsealed, a single ball fragments into several
     * sub-threshold blobs and is thrown away entirely.
     *
     * Raised from 5 to 11 for exactly this. FIRST's artifact sample uses 5 to 15 pixels at
     * 320x240 - the same processing resolution this runs at - and 11 sits mid-range, big enough
     * for the wide holes of a close ball without dissolving a distant one. Lower it toward 7 if
     * balls 4 inches apart are being welded into one blob; raise it toward 15 if single balls are
     * coming apart at close range.
     */
    public static int CLOSE_PX = 11;
    /**
     * Despeckle kernel, odd, in PROCESSING pixels, applied AFTER the close. 0 or 1 disables.
     *
     * Deliberately much smaller than CLOSE_PX rather than the same kernel used twice. Opening with
     * an 11 px kernel would erode 5 px off every side, and an artifact at the far end of
     * MAX_RANGE_IN is only about 20 processing pixels across - the open would eat it. 3 px kills
     * single-pixel colour noise and leaves everything real intact.
     */
    public static int OPEN_PX = 3;
    /**
     * Contours smaller than this (in FULL-RES pixels) are noise.
     *
     * 200 full-res = 50 pixels at the 320x240 processing scale, which is FIRST's own floor for
     * artifact blobs. MIN_SIZE_RATIO below is the gate that actually does the work; this one only
     * exists to throw out specks before the more expensive per-contour maths runs.
     */
    public static double MIN_AREA_PX = 200;

    // =======================================================================================
    // SHAPE GATES
    // =======================================================================================
    /**
     * 4*pi*area/perimeter^2. 1.0 is a perfect circle. Applied to SINGLE balls only.
     *
     * 0.6 is the same threshold FIRST uses to pick a DECODE artifact out of a frame in its own
     * round-blob sample, which is a useful independent confirmation: it is low enough for the
     * ragged outline the edge cutouts leave, high enough to reject a shadow. It is only valid
     * AFTER the close in CLOSE_PX has sealed the holes - measured on a raw lattice mask the same
     * ball scores far below this, because every hole that breaks the silhouette adds perimeter,
     * and perimeter is squared in the denominator.
     */
    public static double MIN_CIRCULARITY = 0.60;
    /**
     * area / area of the minimum enclosing circle. Applied to SINGLE balls only.
     *
     * Kept loose at 0.55. A properly sealed artifact fills its enclosing circle at around 0.85, so
     * this is not the gate doing the rejecting - it is the backstop for a ball whose cutouts
     * happened to land on the silhouette edge and took a bite out of it.
     */
    public static double MIN_CIRCLE_FILL = 0.55;
    /** area / convex hull area. Applied to MERGED blobs, where circularity is meaningless. */
    public static double MIN_SOLIDITY = 0.72;
    /**
     * MASTER SWITCH for the geometry-based size test. OFF by default, and that default matters.
     *
     * When on, a blob is measured against the size a ball SHOULD be at the floor position the
     * camera model projects it to, and rejected if it does not match. That is a genuinely good
     * test - and it is only as good as CAM_HEIGHT_IN and CAM_PITCH_DEG, which start life as
     * placeholders. With those wrong, the test throws away real balls in whole regions of the
     * frame while passing everything in others, which looks exactly like "it only detects near
     * the top edge".
     *
     * Worse, it is circular: the bring-up procedure calibrates the mount by reading the range of
     * a DETECTED ball, so a geometry test that blocks detection also blocks the calibration that
     * would fix it. Detection must not depend on the number you are trying to measure.
     *
     * So: leave this OFF until the range readout agrees with a tape measure at two distances.
     * Then turn it on - it is what makes the merged-blob ball counts trustworthy.
     */
    public static boolean SIZE_GATE = false;
    /** Blobs smaller than expected/this are discarded. Only applied when SIZE_GATE is on. */
    public static double MIN_SIZE_RATIO = 0.45;
    /** Blobs larger than expected*this are discarded. Only applied when SIZE_GATE is on. */
    public static double MAX_SIZE_RATIO = 20.0;
    /** Absolute upper area bound, FULL-RES pixels. Geometry-free, so always applied. */
    public static double MAX_AREA_PX = 90000;
    /** Merged-blob count ceiling while SIZE_GATE is off and the counts cannot be trusted. */
    public static int UNCALIBRATED_MAX_COUNT = 4;
    /** At or above this many single-ball areas, a blob is treated as several merged balls. */
    public static double MERGE_RATIO = 1.65;
    /** Ceiling on the estimated count from one blob. Stops a lighting flare scoring 40. */
    public static int MAX_MERGED_COUNT = 12;

    // =======================================================================================
    // RANGE GATES, inches from the robot's centre
    // =======================================================================================
    /**
     * Lower range bound, inches from the robot's centre. Zero by default - deliberately no floor.
     *
     * This used to be 4.0, which is another silent veto in disguise: with a low, steeply aimed
     * camera the whole lower half of the frame projects to under four inches, and every ball there
     * vanishes with no explanation. Nothing downstream needs a floor - MAX_RANGE_IN is the bound
     * that protects against the near-horizon nonsense, and the approach has its own MIN_MOVE_IN.
     */
    public static double MIN_RANGE_IN = 0.0;
    /** Beyond this the pitch-angle error in the range estimate is larger than it is worth. */
    public static double MAX_RANGE_IN = 78.0;
    /** Ignore anything further off the robot's nose than this, degrees. 180 = full field of view. */
    public static double MAX_ABS_BEARING_DEG = 90.0;

    // =======================================================================================
    // CLUSTERING
    // =======================================================================================
    /**
     * Two detections join the same cluster when their floor points are within this, inches.
     *
     * 9.8 in is exactly TWO artifact diameters (2 x 4.9). Two artifacts in contact have centres
     * one diameter apart, so this links balls that are touching or with roughly one ball's gap
     * between them, and nothing further. Expressing it as a multiple of the real element is what
     * keeps it meaningful: raise it much past this and single-link chaining will start welding
     * separate piles - and eventually the whole field - into one cluster whose centroid points at
     * bare floor. Watch 'spread' on telemetry to catch that happening.
     */
    public static double CLUSTER_LINK_IN = 9.8;
    /** Score = estimated ball count - this * range. Breaks ties toward the nearer pile. */
    public static double RANGE_PENALTY_PER_IN = 0.02;
    /** A cluster must hold at least this many estimated balls to be reported at all. */
    public static int MIN_CLUSTER_COUNT = 1;

    // =======================================================================================
    // OVERLAY
    // =======================================================================================
    public static boolean DRAW_OVERLAY = true;
    /** Replace the preview with the raw colour mask. The fastest way to tune the HSV band. */
    public static boolean DRAW_MASK = false;

    // ---------------------------------------------------------------------------------------

    /** One accepted blob: where it is on the floor, and how many balls it is worth. */
    public static final class Detection {
        public final double x, y;          // robot frame, inches
        public final double range, bearingDeg;
        public final int count;            // estimated balls in this blob
        public final boolean merged;
        public final double uPx, vPx, rPx; // full-res pixel centroid and radius, for the overlay

        Detection(double x, double y, int count, boolean merged,
                  double uPx, double vPx, double rPx) {
            this.x = x;
            this.y = y;
            this.count = count;
            this.merged = merged;
            this.uPx = uPx;
            this.vPx = vPx;
            this.rPx = rPx;
            this.range = Math.hypot(x, y);
            this.bearingDeg = Math.toDegrees(Math.atan2(y, x));
        }
    }

    /** A pile of pollen. {@code x, y} is the count-weighted centroid, in robot inches. */
    public static final class Cluster {
        public final double x, y;
        public final int count;        // estimated balls
        public final int blobs;        // contours that fed it
        public final double spread;    // furthest member from the centroid, inches
        public final double range, bearingDeg;
        public final double score;

        Cluster(double x, double y, int count, int blobs, double spread, double score) {
            this.x = x;
            this.y = y;
            this.count = count;
            this.blobs = blobs;
            this.spread = spread;
            this.score = score;
            this.range = Math.hypot(x, y);
            this.bearingDeg = Math.toDegrees(Math.atan2(y, x));
        }
    }

    // Why a contour was thrown away. Counted per frame and reported, because a detector that
    // silently discards things is untunable - every failure looks identical from outside.
    public static final int REJ_AREA = 0;
    public static final int REJ_SHAPE = 1;
    public static final int REJ_PROJECTION = 2;
    public static final int REJ_RANGE = 3;
    public static final int REJ_BEARING = 4;
    public static final int REJ_SIZE = 5;
    private static final int REJ_COUNT = 6;
    private static final String[] REJ_NAMES = {"area", "shape", "horizon", "range", "bearing", "size"};

    /** An immutable snapshot of one frame. Safely published; never partially visible. */
    public static final class Result {
        public final long frameId;
        public final long timestampNs;
        public final List<Detection> detections;
        public final List<Cluster> clusters;
        /** Best pile this frame, or null if nothing qualified. */
        public final Cluster best;
        public final double processMs;
        /** Contours discarded this frame, indexed by REJ_*. */
        public final int[] rejected;
        /** Contours that passed MIN_AREA_PX and so were actually considered. */
        public final int examined;

        Result(long frameId, long timestampNs, List<Detection> d, List<Cluster> c,
               Cluster best, double processMs, int[] rejected, int examined) {
            this.frameId = frameId;
            this.timestampNs = timestampNs;
            this.detections = Collections.unmodifiableList(d);
            this.clusters = Collections.unmodifiableList(c);
            this.best = best;
            this.processMs = processMs;
            this.rejected = rejected;
            this.examined = examined;
        }

        /** "3 range, 1 shape", or "-" when nothing was thrown away. Empty string never returned. */
        public String rejectionSummary() {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < rejected.length && i < REJ_NAMES.length; i++) {
                if (rejected[i] <= 0) continue;
                if (sb.length() > 0) sb.append(", ");
                sb.append(rejected[i]).append(' ').append(REJ_NAMES[i]);
            }
            return sb.length() == 0 ? "-" : sb.toString();
        }
    }

    private static final Result EMPTY =
            new Result(0, 0, new ArrayList<Detection>(), new ArrayList<Cluster>(), null, 0,
                    new int[REJ_COUNT], 0);

    private volatile Result result = EMPTY;
    private long frameCounter = 0;

    // Scratch buffers. Allocated once and reused: a fresh Mat per frame on a Control Hub is a
    // guaranteed way to spend the whole match in garbage collection.
    private final Mat small = new Mat();
    private final Mat blurred = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat maskExtra = new Mat();
    private final Mat hierarchy = new Mat();
    private Mat kernel = null;
    private int kernelSize = -1;
    private Mat openKernel = null;
    private int openKernelSize = -1;

    /** Latest published frame. Never null. */
    public Result getResult() {
        return result;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Nothing to do. The camera model is rebuilt per frame from PollenGeometry so that a
        // dashboard tweak to the mounting angle takes effect immediately.
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long t0 = System.nanoTime();
        if (frame == null || frame.empty()) return null;

        final int fullW = frame.width(), fullH = frame.height();
        final PollenGeometry.Model model = PollenGeometry.model(fullW, fullH);

        // 1. Downscale. Everything below works in processing pixels, then multiplies back up by
        //    invScale so that every published number is in FULL-RES pixels.
        double scale = clamp(PROC_SCALE, 0.2, 1.0);
        Mat work;
        if (scale < 0.99) {
            Imgproc.resize(frame, small, new Size(Math.max(2, fullW * scale),
                                                 Math.max(2, fullH * scale)),
                    0, 0, Imgproc.INTER_AREA);
            work = small;
        } else {
            work = frame;
        }
        double invScale = fullW / (double) work.width();

        // 2. Blur then threshold.
        int blur = oddOrZero(BLUR_PX);
        if (blur > 1) {
            Imgproc.GaussianBlur(work, blurred, new Size(blur, blur), 0);
            Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        } else {
            Imgproc.cvtColor(work, hsv, Imgproc.COLOR_RGB2HSV);
        }
        buildMask(hsv, mask);

        // 3. CLOSE first (seal the cutouts and the specular highlight), THEN open (despeckle).
        //
        //    This order is the opposite of the usual despeckle-first habit, and it is the order
        //    FIRST switched its own DECODE artifact sample to - "Dilate then Erode ... improves
        //    detection of a DECODE Artifact (smoothes edges of its large holes)".
        //
        //    The reason is specific to this game element. An artifact's colour mask is a lattice:
        //    a ring of green with big dark cutouts through it, plus a washed-out specular blob in
        //    the middle. The webs of green between those holes are THIN. Opening first erodes the
        //    thin webs away before anything has sealed the holes, so the ball disintegrates into
        //    fragments that each fail MIN_AREA_PX, and a ball in plain view is detected as nothing
        //    at all. Closing first fuses the lattice into one solid disc; opening it afterwards
        //    then removes real speckle without a ball to destroy.
        int close = oddOrZero(CLOSE_PX);
        if (close > 1) {
            if (kernel == null || kernelSize != close) {
                kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(close, close));
                kernelSize = close;
            }
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        }
        int open = oddOrZero(OPEN_PX);
        if (open > 1) {
            if (openKernel == null || openKernelSize != open) {
                openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(open, open));
                openKernelSize = open;
            }
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel);
        }

        // 4. Contours -> floor points with a ball count.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int[] rej = new int[REJ_COUNT];
        List<Detection> detections = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            Detection d = evaluateContour(contour, model, invScale, rej);
            if (d != null) detections.add(d);
            contour.release();
        }
        int examined = detections.size();
        for (int r : rej) examined += r;

        // 5. Cluster and score.
        List<Cluster> clusters = cluster(detections);
        Cluster best = null;
        for (Cluster c : clusters) {
            if (c.count < MIN_CLUSTER_COUNT) continue;
            if (best == null || c.score > best.score) best = c;
        }

        double ms = (System.nanoTime() - t0) / 1e6;
        frameCounter++;
        Result r = new Result(frameCounter, System.nanoTime(), detections, clusters, best, ms,
                rej, examined);
        result = r;

        if (DRAW_MASK) {
            // Straight to the preview: the single most useful view when the HSV band is wrong.
            Imgproc.cvtColor(mask, maskExtra, Imgproc.COLOR_GRAY2RGB);
            if (maskExtra.size().equals(frame.size())) {
                maskExtra.copyTo(frame);
            } else {
                Imgproc.resize(maskExtra, maskExtra, frame.size(), 0, 0, Imgproc.INTER_NEAREST);
                maskExtra.copyTo(frame);
            }
        } else if (DRAW_OVERLAY) {
            drawOverlay(frame, model, r);
        }
        return null;
    }

    /**
     * Builds the colour mask, handling a band that wraps past hue 179.
     *
     * Wrapping is not a nicety - red sits at both ends of the hue circle, so if pollen ever turns
     * out to be red, {@code H_LOW = 170, H_HIGH = 10} is the only way to express it, and a naive
     * inRange() on those two numbers returns an empty mask with no error.
     */
    private void buildMask(Mat hsvIn, Mat out) {
        double lo = clamp(H_LOW, 0, 179), hi = clamp(H_HIGH, 0, 179);
        if (lo <= hi) {
            Core.inRange(hsvIn, new Scalar(lo, S_LOW, V_LOW), new Scalar(hi, S_HIGH, V_HIGH), out);
        } else {
            Core.inRange(hsvIn, new Scalar(lo, S_LOW, V_LOW), new Scalar(179, S_HIGH, V_HIGH), out);
            Core.inRange(hsvIn, new Scalar(0, S_LOW, V_LOW), new Scalar(hi, S_HIGH, V_HIGH), maskExtra);
            Core.bitwise_or(out, maskExtra, out);
        }
    }

    /**
     * One contour -> a Detection, or null if it fails a gate. Every rejection is tallied into
     * {@code rej} so the frame can report what it threw away and why.
     *
     * ORDER MATTERS HERE. The geometry-free tests run first - absolute size, then shape - so that
     * a blob is classified as a ball or not a ball WITHOUT reference to the camera mount. Only
     * then is it projected onto the floor to be placed. That ordering is the fix for detection
     * that worked in one band of the frame and nowhere else: the camera model decides WHERE a ball
     * is, never WHETHER it is one, unless SIZE_GATE is explicitly turned on after calibration.
     */
    private Detection evaluateContour(MatOfPoint contour, PollenGeometry.Model model,
                                      double invScale, int[] rej) {
        double areaSmall = Imgproc.contourArea(contour);
        if (areaSmall <= 0) return null;

        // To full-res units. Area scales with the square of a length ratio.
        double area = areaSmall * invScale * invScale;
        if (area < MIN_AREA_PX || area > MAX_AREA_PX) { rej[REJ_AREA]++; return null; }

        org.opencv.imgproc.Moments m = Imgproc.moments(contour);
        if (m.m00 <= 0) { rej[REJ_AREA]++; return null; }
        double u = (m.m10 / m.m00) * invScale;
        double v = (m.m01 / m.m00) * invScale;

        // ---- geometry-free shape analysis -----------------------------------------------------
        MatOfPoint2f c2f = new MatOfPoint2f(contour.toArray());
        Point circleCentre = new Point();
        float[] circleRadius = new float[1];
        Imgproc.minEnclosingCircle(c2f, circleCentre, circleRadius);
        double rPx = circleRadius[0] * invScale;

        double perimSmall = Imgproc.arcLength(c2f, true);
        double circularity = perimSmall <= 1e-6
                ? 0 : 4.0 * Math.PI * areaSmall / (perimSmall * perimSmall);
        double circleArea = Math.PI * rPx * rPx;
        double fill = circleArea <= 0 ? 0 : area / circleArea;

        // Round enough to be one ball? Decided by shape alone, at any distance.
        boolean round = circularity >= MIN_CIRCULARITY && fill >= MIN_CIRCLE_FILL;

        double solidity = round ? 1.0 : solidity(c2f, areaSmall);
        c2f.release();

        // A blob that is neither round nor a solid clump is not pollen - it is a shadow edge or a
        // reflection off something green. This is the one shape rejection that always applies.
        if (!round && solidity < MIN_SOLIDITY) { rej[REJ_SHAPE]++; return null; }

        // ---- place it on the floor ------------------------------------------------------------
        double[] ground = model.pollenGroundPoint(u, v);
        if (ground == null) { rej[REJ_PROJECTION]++; return null; }

        double x = ground[0], y = ground[1];
        double range = Math.hypot(x, y);
        if (range < MIN_RANGE_IN || range > MAX_RANGE_IN) { rej[REJ_RANGE]++; return null; }
        if (Math.abs(Math.toDegrees(Math.atan2(y, x))) > MAX_ABS_BEARING_DEG) {
            rej[REJ_BEARING]++;
            return null;
        }

        // ---- ball count -----------------------------------------------------------------------
        // ratio is how many single balls would fit in this blob's area, given where the camera
        // model thinks the blob is. It is the only count estimator available, so it is used even
        // when the geometry is not trusted - but then its output is capped hard, because an
        // uncalibrated mount can inflate it without limit near the top of the frame and hand one
        // bogus blob a score no real pile could beat.
        double expectedR = model.expectedRadiusPx(x, y);
        double ratio = (expectedR > 0.5) ? area / (Math.PI * expectedR * expectedR) : 1.0;

        if (SIZE_GATE && expectedR > 0.5 && (ratio < MIN_SIZE_RATIO || ratio > MAX_SIZE_RATIO)) {
            rej[REJ_SIZE]++;
            return null;
        }

        boolean merged = !round;
        int count = 1;
        if (merged) {
            count = (int) Math.round(ratio);
            if (count < 2) count = 2;
            int cap = SIZE_GATE ? MAX_MERGED_COUNT : Math.min(MAX_MERGED_COUNT, UNCALIBRATED_MAX_COUNT);
            if (count > cap) count = cap;
        }

        return new Detection(x, y, count, merged, u, v, rPx);
    }

    private double solidity(MatOfPoint2f c2f, double areaSmall) {
        MatOfPoint pts = new MatOfPoint();
        c2f.convertTo(pts, org.opencv.core.CvType.CV_32S);
        org.opencv.core.MatOfInt hullIdx = new org.opencv.core.MatOfInt();
        Imgproc.convexHull(pts, hullIdx);

        int[] idx = hullIdx.toArray();
        Point[] all = pts.toArray();
        Point[] hull = new Point[idx.length];
        for (int i = 0; i < idx.length; i++) hull[i] = all[idx[i]];

        MatOfPoint hullMat = new MatOfPoint(hull);
        double hullArea = Imgproc.contourArea(hullMat);

        pts.release();
        hullIdx.release();
        hullMat.release();
        return hullArea <= 1e-9 ? 0 : areaSmall / hullArea;
    }

    /**
     * Single-link clustering by union-find, on floor points, in inches.
     *
     * Single-link means a chain of balls each within CLUSTER_LINK_IN of the next is one cluster
     * even if the two ends are far apart. That is the right behaviour here - a spilled line of
     * pollen IS one thing to drive at - but it is also why CLUSTER_LINK_IN must not be set much
     * above a ball diameter or two, or the whole field chains into a single cluster whose centroid
     * points at empty floor. {@code spread} on the result is there to catch exactly that.
     *
     * O(n^2), with n the number of accepted blobs - a handful. Not worth a spatial index.
     */
    private List<Cluster> cluster(List<Detection> ds) {
        List<Cluster> out = new ArrayList<>();
        int n = ds.size();
        if (n == 0) return out;

        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;

        double link = Math.max(0.1, CLUSTER_LINK_IN);
        double link2 = link * link;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double dx = ds.get(i).x - ds.get(j).x;
                double dy = ds.get(i).y - ds.get(j).y;
                if (dx * dx + dy * dy <= link2) union(parent, i, j);
            }
        }

        // Accumulate per root. Centroid is weighted by estimated ball count, so a merged blob of
        // five pulls the aim point five times harder than a lone ball beside it - which is what
        // "drive to the pollen" means.
        java.util.HashMap<Integer, double[]> acc = new java.util.HashMap<>();
        for (int i = 0; i < n; i++) {
            int r = find(parent, i);
            double[] a = acc.get(r);
            if (a == null) {
                a = new double[4];   // sumW*x, sumW*y, sumW, blobs
                acc.put(r, a);
            }
            Detection d = ds.get(i);
            a[0] += d.x * d.count;
            a[1] += d.y * d.count;
            a[2] += d.count;
            a[3] += 1;
        }

        for (java.util.Map.Entry<Integer, double[]> e : acc.entrySet()) {
            double[] a = e.getValue();
            if (a[2] <= 0) continue;
            double cx = a[0] / a[2], cy = a[1] / a[2];

            double spread = 0;
            int root = e.getKey();
            for (int i = 0; i < n; i++) {
                if (find(parent, i) != root) continue;
                spread = Math.max(spread, Math.hypot(ds.get(i).x - cx, ds.get(i).y - cy));
            }

            int count = (int) Math.round(a[2]);
            double score = count - RANGE_PENALTY_PER_IN * Math.hypot(cx, cy);
            out.add(new Cluster(cx, cy, count, (int) a[3], spread, score));
        }
        return out;
    }

    private static int find(int[] p, int i) {
        while (p[i] != i) {
            p[i] = p[p[i]];
            i = p[i];
        }
        return i;
    }

    private static void union(int[] p, int a, int b) {
        int ra = find(p, a), rb = find(p, b);
        if (ra != rb) p[rb] = ra;
    }

    /**
     * Annotations are drawn onto the frame Mat rather than through onDrawFrame(Canvas) so that the
     * FTC Dashboard camera stream carries them too, not just the driver station preview.
     */
    private void drawOverlay(Mat frame, PollenGeometry.Model model, Result r) {
        Scalar single = new Scalar(80, 200, 255);
        Scalar mergedC = new Scalar(255, 140, 0);
        Scalar bestC = new Scalar(60, 255, 60);

        for (Detection d : r.detections) {
            Scalar col = d.merged ? mergedC : single;
            Imgproc.circle(frame, new Point(d.uPx, d.vPx), (int) Math.max(3, d.rPx), col, 2);
            Imgproc.putText(frame, (d.merged ? "x" : "") + d.count,
                    new Point(d.uPx - 8, d.vPx - d.rPx - 6),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.45, col, 1);
        }

        if (r.best != null) {
            double[] px = model.projectToPixel(r.best.x, r.best.y, PollenGeometry.ballRadiusIn());
            if (px != null) {
                Point c = new Point(px[0], px[1]);
                Imgproc.drawMarker(frame, c, bestC, Imgproc.MARKER_CROSS, 26, 2);
                Imgproc.putText(frame,
                        String.format(java.util.Locale.US, "n=%d  %.1fin  %.0fdeg",
                                r.best.count, r.best.range, r.best.bearingDeg),
                        new Point(Math.max(2, px[0] - 70), Math.max(14, px[1] - 20)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, bestC, 2);
            }
        }

        Imgproc.putText(frame,
                String.format(java.util.Locale.US, "blobs %d  clusters %d  %.1fms",
                        r.detections.size(), r.clusters.size(), r.processMs),
                new Point(6, 18), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(230, 230, 230), 1);

        // Second line only when something was discarded. This is the line that turns "it does not
        // detect" into a specific gate to go and change.
        if (r.examined > r.detections.size()) {
            Imgproc.putText(frame, "dropped: " + r.rejectionSummary(),
                    new Point(6, 36), Imgproc.FONT_HERSHEY_SIMPLEX, 0.45,
                    new Scalar(255, 170, 90), 1);
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Deliberately empty - see drawOverlay().
    }

    private static int oddOrZero(int k) {
        if (k <= 1) return 0;
        return (k % 2 == 0) ? k + 1 : k;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
