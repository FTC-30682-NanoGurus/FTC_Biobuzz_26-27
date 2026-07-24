package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;
import java.util.TreeMap;

public class TargetingComputer {
    private static final double BASE_CONVERGENCE_SPEED = 0.15;
    private static final double HEADING_CONVERGENCE_SPEED = 0.10;
    private static final double MAX_TRUST_VELOCITY = 20.0;
    private static final double MAX_JUMP_THRESHOLD = 4.0;
    public Pose2d fusedPose;
    private ElapsedTime lastCameraUpdate;
    private double currentConfidence = 0.0;
    private final TreeMap<Double, ShotData> ballisticTable = new TreeMap<>();
    private final double targetGoalX;
    private final double targetGoalY;

    // Container for rows of data
    public static class ShotData {
        public final double velocity;
        public final double hoodPosition;

        public ShotData(double velocity, double hoodPosition) {
            this.velocity = velocity;
            this.hoodPosition = hoodPosition;
        }
    }

    /**
     * Confidence decay rate, in confidence units per SECOND. Negative = legacy behaviour
     * (a flat -2.0 every time update() is called with no camera pose).
     *
     * The legacy decay is per-LOOP, which silently couples the fusion filter to loop rate: the
     * faster the opmode loops, the faster confidence bleeds off between the ~200 ms camera
     * samples. Once confidence drops under 10 the filter flips into "snap straight to the camera
     * pose" mode (posGain = 1.0 below), so simply making a loop faster can change how the robot
     * localises. Opmodes that have been sped up should call setTimeBasedDecay() so the effective
     * rate stays put.
     */
    private double decayPerSecond = -1.0;
    private final ElapsedTime decayTimer = new ElapsedTime();

    /**
     * @param perSecond confidence lost per second with no camera fix. To match the old behaviour,
     *                  pass (2.0 * your measured pre-optimisation loop rate in Hz) - e.g. a loop
     *                  that ran at 50 Hz was losing 100.0 per second.
     */
    public void setTimeBasedDecay(double perSecond) {
        this.decayPerSecond = perSecond;
        this.decayTimer.reset();
    }

    public TargetingComputer(Pose2d startPose, double goalX, double goalY) {
        this.fusedPose = startPose;
        this.targetGoalX = goalX;
        this.targetGoalY = goalY;
        this.lastCameraUpdate = new ElapsedTime();

        // BALLISTICS TABLE (Distance, ShotData)
        ballisticTable.put(24.0, new ShotData(1000.0, 1.0)); // Point Blank
        ballisticTable.put(35.0, new ShotData(1000.0, 0.85)); // Close-Close //1070
        ballisticTable.put(52.0, new ShotData(1040.0, 0.72)); // Close-Mid //1100
        ballisticTable.put(80.0, new ShotData(1130.0, 0.67)); // Close-Far
        ballisticTable.put(120.0, new ShotData(800, 0.6)); // Far Zone

        /*ballisticTable.put(24.0, new ShotData(1150.0, 1.0)); // Point Blank
        ballisticTable.put(35.0, new ShotData(1250.0, 1.0)); // Close-Close
        ballisticTable.put(52.0, new ShotData(1300.0, 1.0)); // Close-Mid
        ballisticTable.put(72.0, new ShotData(1350.0, 0.9)); // Close-Far
        ballisticTable.put(120.0, new ShotData(1450.0, 0.5)); // Far Zone*/
    }

    public Pose2d update(Pose2d odoPose, Pose2d camPose, double robotVel) {

        double dt = decayTimer.seconds();
        decayTimer.reset();

        double newX = odoPose.position.x;
        double newY = odoPose.position.y;
        double newH = odoPose.heading.toDouble();

        if (camPose != null) {
            lastCameraUpdate.reset();

            double distError = Math.hypot(camPose.position.x - newX, camPose.position.y - newY);

            double errorH = camPose.heading.toDouble() - newH;
            while (errorH > Math.PI) errorH -= 2 * Math.PI;
            while (errorH < -Math.PI) errorH += 2 * Math.PI;

            boolean isGlitch;

            if (currentConfidence < 10.0) {
                isGlitch = false;
            } else {
                isGlitch = distError > MAX_JUMP_THRESHOLD;
            }

            double velocityTrust = 1.0 - Range.clip(Math.abs(robotVel) / MAX_TRUST_VELOCITY, 0, 1);

            if (!isGlitch) {
                double posGain = BASE_CONVERGENCE_SPEED * velocityTrust;
                double rotGain = HEADING_CONVERGENCE_SPEED * velocityTrust;

                if (currentConfidence < 10.0) {
                    posGain = 1.0;
                    rotGain = 1.0;
                }

                newX = newX + (camPose.position.x - newX) * posGain;
                newY = newY + (camPose.position.y - newY) * posGain;

                newH = newH + (errorH * rotGain);

                currentConfidence = 100.0 * velocityTrust;
            }
        } else {
            // Decay confidence if tag is lost
            if (decayPerSecond > 0) {
                currentConfidence -= decayPerSecond * dt;
            } else {
                currentConfidence -= 2.0;
            }
        }

        currentConfidence = Range.clip(currentConfidence, 0, 100);

        fusedPose = new Pose2d(newX, newY, newH);

        return fusedPose;
    }

    public ShotData getShooterSolution() {
        double distance = getDistanceToGoal();

        Map.Entry<Double, ShotData> floor = ballisticTable.floorEntry(distance);
        Map.Entry<Double, ShotData> ceiling = ballisticTable.ceilingEntry(distance);

        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();

        double ratio = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());

        double interpVel = floor.getValue().velocity + ratio * (ceiling.getValue().velocity - floor.getValue().velocity);
        double interpHood = floor.getValue().hoodPosition + ratio * (ceiling.getValue().hoodPosition - floor.getValue().hoodPosition);

        return new ShotData(interpVel, interpHood);
    }

    public double getDistanceToGoal() {
        return Math.hypot(targetGoalX - fusedPose.position.x, targetGoalY - fusedPose.position.y);
    }

    public double getConfidence() { return currentConfidence; }
}
