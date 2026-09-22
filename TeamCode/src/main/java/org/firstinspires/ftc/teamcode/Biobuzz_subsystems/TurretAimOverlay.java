package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Draws a live crosshair on the camera stream showing where the TURRET is currently pointed, plus
 * a reticle on the centre of the HIVE CELL opening the vision has locked onto.
 *
 * <h2>Why the crosshair has to be computed rather than painted at a fixed spot</h2>
 * The C920 is bolted to the CHASSIS and the turret rotates independently of it. A static crosshair
 * would therefore show where the CAMERA looks, which is the one thing you do not need to check.
 * What matters is whether the TURRET is pointed at the opening, so this processor takes the
 * turret's current bearing (measured from the encoder when one exists, otherwise the commanded
 * angle) and projects that ray through the camera's own intrinsics to find which image column it
 * would cross. When the turret is aimed correctly, the vertical crosshair line lands on the target
 * reticle. That coincidence IS "the turret is aiming at the centre of the open area".
 *
 * <h2>Why there is no work in processFrame</h2>
 * {@link #processFrame(Mat, long)} runs on the vision thread for every frame and shares that
 * thread with the AprilTag decoder. Anything done here is taken straight out of the detection
 * frame rate, so this processor does nothing there at all and draws only in
 * {@link #onDrawFrame}, which runs on the preview thread.
 *
 * <h2>Threading</h2>
 * The setters are called from the opmode loop; {@code onDrawFrame} runs on the preview thread.
 * Every shared field is {@code volatile} and every draw reads each field exactly once into a local,
 * so a frame can never be drawn from a half-updated state.
 */
public class TurretAimOverlay implements VisionProcessor {

    // ---- Image geometry, learned at init --------------------------------------------------------

    private volatile int imageWidth = TurretConstants.STREAM_WIDTH;
    private volatile int imageHeight = TurretConstants.STREAM_HEIGHT;

    // ---- Aim state, pushed in from the opmode loop ----------------------------------------------

    /** Where the turret actually points, DEGREES in the ROBOT frame, CCW-positive. */
    private volatile double turretAimBearingRobotDeg = 0.0;

    /** Aim error, DEGREES, used only to colour the crosshair. */
    private volatile double aimErrorDeg = Double.NaN;

    /** True when a valid, scorable, alliance-matched cluster is being tracked right now. */
    private volatile boolean targetValid = false;

    /** Latest target position in the CAMERA frame, INCHES, in ftcPose axes (+x right, +y fwd, +z up). */
    private volatile double targetCamX = 0.0;
    private volatile double targetCamY = 0.0;
    private volatile double targetCamZ = 0.0;

    // ---- Camera model, pushed in from constants --------------------------------------------------

    private volatile double fx = TurretConstants.CAMERA_FX;
    private volatile double fy = TurretConstants.CAMERA_FY;
    private volatile double cx = TurretConstants.CAMERA_CX;
    private volatile double cy = TurretConstants.CAMERA_CY;
    private volatile double cameraMountYawDeg = TurretConstants.CAMERA_MOUNT_YAW_DEG;

    // ---- Paints, allocated once ------------------------------------------------------------------

    private final Paint crosshairPaint = new Paint();
    private final Paint targetPaint = new Paint();
    private final Paint textPaint = new Paint();

    public TurretAimOverlay() {
        crosshairPaint.setAntiAlias(true);
        crosshairPaint.setStyle(Paint.Style.STROKE);
        crosshairPaint.setStrokeWidth(4);

        targetPaint.setAntiAlias(true);
        targetPaint.setStyle(Paint.Style.STROKE);
        targetPaint.setStrokeWidth(3);
        targetPaint.setColor(Color.CYAN);

        textPaint.setAntiAlias(true);
        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(24);
    }

    // =============================================================================================
    // VisionProcessor
    // =============================================================================================

    /** Stores the stream dimensions. No allocation, no OpenCV work. */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.imageWidth = width;
        this.imageHeight = height;
    }

    /**
     * Deliberately a no-op. See the class comment: this processor shares the vision thread with the
     * AprilTag decoder, so it spends none of it.
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    /**
     * Draws the turret crosshair and the target reticle.
     *
     * @param scaleBmpPxToCanvasPx multiply every IMAGE pixel coordinate by this before drawing. The
     *                             canvas is the size of the preview on screen, not 640x480, so
     *                             skipping this puts the crosshair in the wrong place on the
     *                             Driver Station and in the right place in a screenshot.
     */
    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {

        // Snapshot every shared field once, so the whole frame is drawn from one consistent state.
        final boolean valid = targetValid;
        final double aimDeg = turretAimBearingRobotDeg;
        final double errDeg = aimErrorDeg;
        final double camYaw = cameraMountYawDeg;
        final double lfx = fx, lfy = fy, lcx = cx, lcy = cy;
        final double tx = targetCamX, ty = targetCamY, tz = targetCamZ;
        final int w = imageWidth, h = imageHeight;

        // -----------------------------------------------------------------------------------------
        // TURRET CROSSHAIR - which image column the turret's aim ray crosses.
        //
        // The turret bearing is measured from ROBOT forward; the camera's optical axis sits
        // CAMERA_MOUNT_YAW_DEG away from robot forward, so the angle between the aim ray and the
        // optical axis is the difference of the two. A ray at angle a from the optical axis meets
        // the image plane at column cx + fx*tan(a) - the standard pinhole projection.
        //
        // The vertical line is the meaningful one: this turret rotates in azimuth only, so its
        // aim has no elevation to show.
        //
        // This ignores the parallax between the turret pivot and the camera lens - the two are a
        // few inches apart, which at goal distance is a small fraction of a degree. If you ever
        // want it exact, project the aim ray at the measured target range instead: take the point
        // (r*sin(a), r*cos(a)) in the turret frame, translate it by the pivot-to-camera offset,
        // then project that point.
        // -----------------------------------------------------------------------------------------
        double bearingRelCamRad = Math.toRadians(aimDeg - camYaw);

        // A turret pointed near or behind 90 degrees off the optical axis has no image column at
        // all (tan blows up), so clamp well short of the singularity and let it pin to the edge.
        final double MAX_PROJECTABLE_RAD = Math.toRadians(80.0);
        boolean aimOffScreen = Math.abs(bearingRelCamRad) > MAX_PROJECTABLE_RAD;
        if (bearingRelCamRad > MAX_PROJECTABLE_RAD) bearingRelCamRad = MAX_PROJECTABLE_RAD;
        if (bearingRelCamRad < -MAX_PROJECTABLE_RAD) bearingRelCamRad = -MAX_PROJECTABLE_RAD;

        // Image +u runs RIGHT while turret bearing is CCW-positive (LEFT), hence the minus sign.
        double uAim = lcx - lfx * Math.tan(bearingRelCamRad);

        // -----------------------------------------------------------------------------------------
        // TARGET RETICLE - the centre of the CELL opening.
        //
        // The cluster's ftcPose origin already sits at the centre of the opening, so it only has to
        // be projected to pixels. It must first be converted out of the SDK's ftcPose axes
        // (+x right, +y forward, +z up) into the OpenCV optical axes the pinhole model assumes
        // (+x right, +y DOWN, +z forward). That conversion is the reason this is not simply
        // u = cx + fx*(x/z) on the raw ftcPose values - doing that would divide by the HEIGHT of
        // the target instead of its DISTANCE and put the reticle somewhere meaningless.
        // -----------------------------------------------------------------------------------------
        double uTarget = Double.NaN;
        double vTarget = Double.NaN;
        if (valid) {
            double optX = tx;    // right stays right
            double optY = -tz;   // up becomes down
            double optZ = ty;    // forward becomes the optical axis
            if (optZ > 1e-3) {   // behind or level with the lens cannot be projected
                uTarget = lcx + lfx * (optX / optZ);
                vTarget = lcy + lfy * (optY / optZ);
            }
        }

        // -----------------------------------------------------------------------------------------
        // Colour: green = aimed and ready, yellow = target seen but not lined up,
        //         grey = no valid target (flying on the filtered estimate or the seed).
        // -----------------------------------------------------------------------------------------
        int color;
        if (!valid) {
            color = Color.GRAY;
        } else if (!Double.isNaN(errDeg) && Math.abs(errDeg) <= TurretConstants.AIM_READY_TOLERANCE_DEG) {
            color = Color.GREEN;
        } else {
            color = Color.YELLOW;
        }
        crosshairPaint.setColor(color);

        // The crosshair's horizontal line sits on the target's row when there is one, so that
        // "lines cross on the reticle" reads as aimed; otherwise it falls back to image centre.
        double vCross = (!Double.isNaN(vTarget)) ? vTarget : (h / 2.0);

        float sx = scaleBmpPxToCanvasPx;

        // Vertical line: the actual aim indication.
        canvas.drawLine((float) (uAim * sx), 0f,
                        (float) (uAim * sx), (float) (h * sx), crosshairPaint);

        // Horizontal line: context only, spans the frame.
        canvas.drawLine(0f, (float) (vCross * sx),
                        (float) (w * sx), (float) (vCross * sx), crosshairPaint);

        // Target reticle.
        if (!Double.isNaN(uTarget) && !Double.isNaN(vTarget)) {
            canvas.drawCircle((float) (uTarget * sx), (float) (vTarget * sx), 18f * sx, targetPaint);
            canvas.drawCircle((float) (uTarget * sx), (float) (vTarget * sx), 3f * sx, targetPaint);
        }

        // Numeric readout, because a crosshair alone cannot tell you HOW far off you are.
        String label = String.format("turret %.1fdeg  err %s%s",
                aimDeg,
                Double.isNaN(errDeg) ? "--" : String.format("%.1fdeg", errDeg),
                aimOffScreen ? "  [AIM OFF-FRAME]" : "");
        canvas.drawText(label, 8f * sx, 28f * sx, textPaint);
    }

    // =============================================================================================
    // Thread-safe setters, called from the opmode loop
    // =============================================================================================

    /**
     * Pushes the turret's current aim and its error, for the crosshair position and colour.
     *
     * @param turretBearingRobotDeg where the turret actually points, DEGREES in the robot frame.
     *                              Pass the MEASURED encoder angle when an encoder exists, so the
     *                              crosshair shows reality; pass the commanded angle otherwise.
     * @param errorDeg              signed aim error, DEGREES, or NaN when it is unknown.
     */
    public void setAimState(double turretBearingRobotDeg, double errorDeg) {
        this.turretAimBearingRobotDeg = turretBearingRobotDeg;
        this.aimErrorDeg = errorDeg;
    }

    /**
     * Pushes the latest valid target's position in the CAMERA frame, INCHES, ftcPose axes.
     * Call {@link #clearTarget()} instead when nothing valid is being tracked.
     */
    public void setTarget(double camX, double camY, double camZ) {
        this.targetCamX = camX;
        this.targetCamY = camY;
        this.targetCamZ = camZ;
        this.targetValid = true;
    }

    /** Marks that there is no valid target this loop; the crosshair goes grey and the reticle off. */
    public void clearTarget() {
        this.targetValid = false;
    }

    /**
     * Pushes the camera model. Call once at init from {@link TurretConstants}; it is a setter
     * rather than a direct read so the overlay stays testable and so dashboard edits to the
     * intrinsics take effect without restarting the stream.
     *
     * @param fx,fy focal lengths, PIXELS
     * @param cx,cy principal point, PIXELS
     * @param cameraMountYawDeg camera yaw relative to robot forward, DEGREES, CCW-positive
     */
    public void setCameraModel(double fx, double fy, double cx, double cy, double cameraMountYawDeg) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.cameraMountYawDeg = cameraMountYawDeg;
    }
}
