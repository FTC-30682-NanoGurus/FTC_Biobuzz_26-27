package org.firstinspires.ftc.teamcode.library;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * A generic, future-proof sensor fusion engine capable of combining any two data streams.
 * Contains nested strategies for advanced state estimation, filtering, and logic.
 *
 * @param <T> The data type of the continuous/fast sensor.
 * @param <U> The data type of the absolute/slow sensor.
 * @param <R> The output result type of the fusion.
 */
public class NGSensorFusion<T, U, R> {

    private final Supplier<T> continuousSensor;
    private final Supplier<U> absoluteSensor;
    private final FusionStrategy<T, U, R> strategy;
    private final ElapsedTime timer;
    private double lastTime;

    /**
     * Defines the algorithmic contract for fusing two independent data streams.
     */
    public interface FusionStrategy<T, U, R> {

        /**
         * Fuses two data streams into a single computed result.
         *
         * @param continuousData The high-frequency data (can be null if unavailable).
         * @param absoluteData   The low-frequency or absolute data (can be null if unavailable).
         * @param dt             The time delta in seconds since the last update.
         * @return The fused output state.
         */
        R fuse(T continuousData, U absoluteData, double dt);
    }

    /**
     * Initializes the fusion engine with target sensors and a specific fusion algorithm.
     *
     * @param continuousSensor A method reference supplying the primary, fast data stream.
     * @param absoluteSensor   A method reference supplying the secondary, slow data stream.
     * @param strategy         The specific mathematical or logical strategy to apply.
     */
    public NGSensorFusion(Supplier<T> continuousSensor, Supplier<U> absoluteSensor, FusionStrategy<T, U, R> strategy) {
        this.continuousSensor = continuousSensor;
        this.absoluteSensor = absoluteSensor;
        this.strategy = strategy;
        this.timer = new ElapsedTime();
        this.lastTime = timer.seconds();
    }

    /**
     * Fetches the latest data from both sensors, calculates the time delta,
     * and passes them through the selected fusion strategy.
     *
     * @return The updated fused result based on the generic type R.
     */
    public R update() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        this.lastTime = currentTime;

        T dataA = continuousSensor.get();
        U dataB = absoluteSensor.get();

        return strategy.fuse(dataA, dataB, dt);
    }

    /**
     * A robust pose estimation strategy utilizing a Discrete Kalman Filter with
     * Mahalanobis distance glitch rejection and dynamic covariance scaling.
     */
    public static class KinematicPoseFilter implements FusionStrategy<Pose2d, Pose2d, Pose2d> {

        private Pose2d fusedPose;
        private double pX = 0.0;
        private double pY = 0.0;
        private double pH = 0.0;
        private final double maxMahalanobisSq;

        /**
         * Initializes the kinematic filter with a starting position and outlier threshold.
         *
         * @param startPose          The initial known position of the robot.
         * @param rejectionThreshold The chi-square threshold for Mahalanobis glitch rejection (e.g., 11.34).
         */
        public KinematicPoseFilter(Pose2d startPose, double rejectionThreshold) {
            this.fusedPose = startPose;
            this.maxMahalanobisSq = rejectionThreshold;
        }

        /**
         * Predicts the state using odometry and corrects it utilizing camera data
         * if the camera data passes statistical outlier rejection.
         */
        @Override
        public Pose2d fuse(Pose2d odoPose, Pose2d camPose, double dt) {
            if (odoPose != null) {
                fusedPose = odoPose;
                pX += 0.5 * dt;
                pY += 0.5 * dt;
                pH += 0.1 * dt;
            }

            if (camPose != null) {
                double rX = 2.0;
                double rY = 2.0;
                double rH = 0.5;

                double resX = camPose.position.x - fusedPose.position.x;
                double resY = camPose.position.y - fusedPose.position.y;
                double resH = camPose.heading.toDouble() - fusedPose.heading.toDouble();

                while (resH > Math.PI) resH -= 2 * Math.PI;
                while (resH < -Math.PI) resH += 2 * Math.PI;

                double mDistSq = (resX * resX) / (pX + rX) + (resY * resY) / (pY + rY) + (resH * resH) / (pH + rH);

                if (mDistSq < maxMahalanobisSq || pX > 10.0) {
                    double kX = pX / (pX + rX);
                    double kY = pY / (pY + rY);
                    double kH = pH / (pH + rH);

                    fusedPose = new Pose2d(
                            fusedPose.position.x + (kX * resX),
                            fusedPose.position.y + (kY * resY),
                            fusedPose.heading.toDouble() + (kH * resH)
                    );

                    pX *= (1.0 - kX);
                    pY *= (1.0 - kY);
                    pH *= (1.0 - kH);
                }
            }
            return fusedPose;
        }
    }

    /**
     * A validation strategy that cross-references a vision system against a physical
     * distance sensor to reject false-positive object detections.
     */
    public static class TargetValidator implements FusionStrategy<Double, Double, Boolean> {

        private final double maxDiscrepancy;

        /**
         * Initializes the validator with a tolerance threshold.
         *
         * @param maxDiscrepancy The maximum allowable difference in inches before rejecting the target.
         */
        public TargetValidator(double maxDiscrepancy) {
            this.maxDiscrepancy = maxDiscrepancy;
        }

        /**
         * Compares the vision-estimated distance against the laser-measured distance.
         */
        @Override
        public Boolean fuse(Double visionDist, Double laserDist, double dt) {
            if (visionDist == null) return false;
            if (laserDist == null) return true;

            return Math.abs(visionDist - laserDist) <= maxDiscrepancy;
        }
    }

    /**
     * Analyzes physical intake conditions by combining spatial occupancy
     * (Beam Break) with spectral data (Color Sensor) to determine game piece state.
     */
    public static class IntakeAnalyzer implements FusionStrategy<Boolean, int[], String> {

        /**
         * Evaluates boolean presence against RGB arrays to return a specific piece state.
         */
        @Override
        public String fuse(Boolean isBeamBroken, int[] rgbColor, double dt) {
            if (isBeamBroken == null || !isBeamBroken) {
                return "EMPTY";
            }

            if (rgbColor == null || rgbColor.length < 3) {
                return "UNKNOWN_JAM";
            }

            int r = rgbColor[0];
            int g = rgbColor[1];
            int b = rgbColor[2];

            if (r > g && r > b) return "RED_SAMPLE";
            if (b > r && b > g) return "BLUE_SAMPLE";
            if (r > b && g > b && Math.abs(r - g) < 20) return "YELLOW_SAMPLE";

            return "UNKNOWN_JAM";
        }
    }
}