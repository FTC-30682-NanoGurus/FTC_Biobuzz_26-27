package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

/**
 * The pinhole camera model that turns a pollen pixel into a point on the floor, in inches,
 * relative to the robot.
 *
 * <h2>Frames</h2>
 * ROBOT frame is RoadRunner's: <b>+X forward, +Y left, +Z up</b>, origin at the centre of the
 * robot on the floor. It is the same frame {@code MecanumDrive.pose} lives in, so a point produced
 * here can be rotated into field coordinates with nothing but the current heading.
 *
 * CAMERA frame is OpenCV's: x right, y down, z out of the lens.
 *
 * <h2>Why a ground-plane intersection instead of apparent size</h2>
 * Range from apparent radius, {@code range = f * D / (2 * r_px)}, divides by a measured pixel
 * radius. That radius is the least reliable number in the whole pipeline - it moves with exposure,
 * with the blob threshold, and it is meaningless the moment two pollen balls touch and merge into
 * one contour. Ray-plane intersection instead uses only the pixel POSITION, which is stable, and
 * one fact that is true by construction: a pollen ball resting on the field has its centre exactly
 * one ball-radius above the floor. So the ray through the blob centroid is intersected with the
 * plane {@code z = POLLEN_DIAMETER_IN / 2} rather than with the floor itself.
 *
 * The consequence to be aware of: this model is only valid for pollen ON THE FLOOR. A ball held by
 * another robot, sitting on a field element, or in flight will be reported at the wrong range -
 * further away than it is, because the ray has to travel further to fall to ball-centre height.
 *
 * <h2>Accuracy</h2>
 * Range error is dominated by CAM_PITCH_DEG, and it grows roughly with the square of range: a 1
 * degree pitch error is about an inch at 30", and about four at 60". Measure the tilt with a phone
 * inclinometer against the camera's mounting face rather than guessing it, and prefer a lower
 * MAX_RANGE_IN in {@link PollenPipeline} over trusting a distant estimate.
 *
 * <h2>Defaults</h2>
 * The intrinsics are FIRST's published Logitech C920 calibration at 640x480 and are scaled here for
 * any other stream size. They are good enough to drive to a pile; they are not a substitute for
 * calibrating your own copy of the camera if you want the last inch.
 *
 * EVERY extrinsic below is a placeholder that MUST be measured on the real robot.
 */
@Config
public final class PollenGeometry {
    private PollenGeometry() {}

    // ---------------------------------------------------------------------------------------
    // Intrinsics. FIRST's C920 calibration, 640x480. Scaled automatically for other resolutions.
    // ---------------------------------------------------------------------------------------
    public static double FX = 622.001;
    public static double FY = 622.001;
    public static double CX = 319.803;
    public static double CY = 241.251;
    /** Resolution the four numbers above were measured at. Do not change unless you recalibrate. */
    public static int CALIB_WIDTH = 640;
    public static int CALIB_HEIGHT = 480;

    // ---------------------------------------------------------------------------------------
    // Extrinsics - MEASURE THESE. All in inches / degrees, robot frame.
    // ---------------------------------------------------------------------------------------
    /** Height of the LENS above the floor. */
    public static double CAM_HEIGHT_IN = 9.0;
    /** Lens position ahead of the robot's centre of rotation. Negative if it is behind. */
    public static double CAM_FORWARD_IN = 6.0;
    /** Lens position to the LEFT of centre. Negative for right of centre. */
    public static double CAM_LEFT_IN = 0.0;
    /** Downward tilt of the optical axis. 0 = dead level, 90 = staring at the floor. */
    public static double CAM_PITCH_DEG = 25.0;
    /** Rotation of the camera about vertical. Positive = aimed to the robot's LEFT. */
    public static double CAM_YAW_DEG = 0.0;

    /**
     * Diameter of one ARTIFACT. Sets both the projection plane and the expected blob size.
     *
     * FIRST specifies 4.9 in +/- 0.25 in (12.45 cm +/- 0.65 cm) measured AT THE MOLD SEAM, per
     * DECODE Team Update 09. The seam is the widest point, which is exactly the dimension the
     * silhouette shows and therefore the right one to use here - "5 inch ball" is the marketing
     * round number, not the spec.
     *
     * The +/- 0.25 in tolerance is real and worth knowing: it is +/- 5%, which flows straight
     * through into a +/- 5% range error and a +/- 10% error in the estimated ball count of a
     * merged blob (count goes as area, so as the square). MERGE_RATIO in PollenPipeline is set
     * loose enough to absorb it.
     */
    public static double POLLEN_DIAMETER_IN = 4.9;

    public static double ballRadiusIn() {
        return POLLEN_DIAMETER_IN / 2.0;
    }

    /** Snapshot of the model above, built for one frame size. Cheap - rebuild it per frame. */
    public static Model model(int frameWidth, int frameHeight) {
        return new Model(frameWidth, frameHeight);
    }

    /**
     * An immutable copy of the camera model, resolved for one frame size.
     *
     * Built fresh each frame so that a dashboard edit to any static above takes effect on the very
     * next frame instead of at the next opmode start. It is a dozen trig calls; that is nothing
     * next to the colour threshold that follows it.
     */
    public static final class Model {
        public final int width, height;
        public final double fx, fy, cx, cy;

        /** Camera basis vectors expressed in the ROBOT frame. */
        public final double fwdX, fwdY, fwdZ;     // optical axis
        public final double rightX, rightY, rightZ;
        public final double downX, downY, downZ;

        public final double camX, camY, camZ;
        public final double ballR;

        private Model(int frameWidth, int frameHeight) {
            this.width = Math.max(1, frameWidth);
            this.height = Math.max(1, frameHeight);

            // Scale the calibration to the stream we actually got. Both axes scale independently
            // so a non-4:3 stream is still handled correctly.
            double sx = this.width / (double) Math.max(1, CALIB_WIDTH);
            double sy = this.height / (double) Math.max(1, CALIB_HEIGHT);
            this.fx = FX * sx;
            this.fy = FY * sy;
            this.cx = CX * sx;
            this.cy = CY * sy;

            double p = Math.toRadians(CAM_PITCH_DEG);
            double y = Math.toRadians(CAM_YAW_DEG);
            double cp = Math.cos(p), sp = Math.sin(p);
            double cy_ = Math.cos(y), sy_ = Math.sin(y);

            // Pitch down about the camera's right axis, then yaw about robot +Z. At zero pitch and
            // zero yaw this gives forward = +X, right = -Y, down = -Z, which is the camera bolted
            // on facing straight ahead.
            fwdX = cp * cy_;   fwdY = cp * sy_;   fwdZ = -sp;
            downX = -sp * cy_; downY = -sp * sy_; downZ = -cp;
            rightX = sy_;      rightY = -cy_;     rightZ = 0.0;

            camX = CAM_FORWARD_IN;
            camY = CAM_LEFT_IN;
            camZ = CAM_HEIGHT_IN;
            ballR = ballRadiusIn();
        }

        /**
         * Where the ray through pixel (u, v) crosses the horizontal plane {@code z = planeZ}.
         *
         * @return {x, y} in robot inches, or null if the ray never gets there - it points at or
         *         above the horizon, or the plane is above the lens. Returning null rather than a
         *         huge number is deliberate: a pixel a few rows above the horizon would otherwise
         *         produce a plausible-looking 400 inch reading that no range gate would catch as
         *         obviously wrong.
         */
        public double[] rayToPlane(double u, double v, double planeZ) {
            double a = (u - cx) / fx;   // along camera right
            double b = (v - cy) / fy;   // along camera down

            double dx = a * rightX + b * downX + fwdX;
            double dy = a * rightY + b * downY + fwdY;
            double dz = a * rightZ + b * downZ + fwdZ;

            if (dz >= -1e-9) return null;              // level or looking up
            double t = (planeZ - camZ) / dz;
            if (!(t > 0) || !Double.isFinite(t)) return null;

            return new double[]{camX + t * dx, camY + t * dy};
        }

        /** Ground position of a pollen ball whose blob centroid landed on pixel (u, v). */
        public double[] pollenGroundPoint(double u, double v) {
            return rayToPlane(u, v, ballR);
        }

        /** Depth of a robot-frame point along the optical axis. Negative means behind the lens. */
        public double depthOf(double x, double y, double z) {
            return (x - camX) * fwdX + (y - camY) * fwdY + (z - camZ) * fwdZ;
        }

        /**
         * Pixel radius a single pollen ball SHOULD have if its centre sits at robot (x, y).
         *
         * This is the reference the pipeline measures blobs against: at or near it the blob is one
         * ball, well above it the blob is several balls that merged into one contour. Doing the
         * comparison against a per-location expectation instead of one global pixel threshold is
         * what lets a single close ball and a distant clump of five be told apart at all - in raw
         * pixels they are the same size.
         *
         * @return radius in pixels, or -1 if the point is not in front of the lens.
         */
        public double expectedRadiusPx(double x, double y) {
            double z = depthOf(x, y, ballR);
            if (z <= 1e-6) return -1;
            return fx * ballR / z;
        }

        /** Robot-frame point back to a pixel, for overlay drawing. Null if behind the lens. */
        public double[] projectToPixel(double x, double y, double z) {
            double ex = x - camX, ey = y - camY, ez = z - camZ;
            double zc = ex * fwdX + ey * fwdY + ez * fwdZ;
            if (zc <= 1e-6) return null;
            double xc = ex * rightX + ey * rightY + ez * rightZ;
            double yc = ex * downX + ey * downY + ez * downZ;
            return new double[]{cx + fx * xc / zc, cy + fy * yc / zc};
        }
    }
}
