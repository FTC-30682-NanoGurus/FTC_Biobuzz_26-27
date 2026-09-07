package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;

/**
 * Mecanum drive test opmode, plus REV 2m border protection.
 *
 * CONTROLS - stick and trigger mapping and signs are UNCHANGED from the original
 *   Left stick Y ........ left side  (tank)
 *   Right stick Y ....... right side (tank)
 *   Left trigger ........ strafe left   - now COMBINES with the sticks instead of overriding them
 *   Right trigger ....... strafe right  - now COMBINES with the sticks instead of overriding them
 *   Left bumper (hold) .. precision mode, 35%
 *   Right bumper (hold).. override - bypasses the accel limiter and traction control
 *   Y ................... toggle field centric      (default OFF - see sign note)
 *   X ................... toggle heading hold       (default OFF - see sign note)
 *   Back ................ reset the heading reference
 *   B ................... toggle border protection  (default ON)
 *
 *
 * SIGN WARNING: field centric, heading hold and traction control default to OFF because the
 * correct rotation sign cannot be confirmed without driving the robot. Enable them one at a time
 * from the dashboard. For heading hold, lift the wheels off the ground and twist the chassis by
 * hand: the wheels should spin so as to UNDO the twist. If they fight to continue it, flip
 * MecaTank.HEADING_HOLD_SIGN to -1.
 *
 *
 * BORDER PROTECTION
 * -----------------
 * A REV 2m distance sensor watches the gap to the field wall. Inside BORDER_THRESHOLD_IN the
 * gamepad DRIVING inputs are dropped entirely and the opmode drives itself away from the wall
 * until it has put SAFE_RETREAT_IN of measured travel between the robot and the border. The
 * feature toggles (Y / X / A / Back) keep working while locked out - only translation is taken
 * away, so the driver can still reconfigure the drive while the retreat runs.
 *
 * SENSOR_STRAFES picks the retreat axis and must match how the sensor is physically mounted:
 *   false - sensor points FORWARD, the robot retreats BACKWARD
 *   true  - sensor points RIGHT,   the robot strafes LEFT
 * The retreat is expressed as synthetic gamepad values fed through the SAME
 * setDrivePowersSmooth() path the driver uses, so it inherits every sign convention already
 * validated for the sticks and triggers - there is no second set of signs to get wrong. Backing
 * up is a positive stick value because the shaper negates stick Y; strafing left is a positive
 * left trigger, matching the control map above.
 *
 * FAIL-SAFE DIRECTION: a timed-out, non-finite or out-of-range read is treated as FAR, not near.
 * A dead sensor therefore leaves the driver in full control instead of pinning the robot in a
 * permanent retreat. The trade is deliberate - a test opmode that locks out the driver on a
 * broken I2C line is worse than one that simply stops protecting.
 *
 * B only suppresses the RETREAT. The sensor keeps being polled and the gap keeps being reported
 * on telemetry while protection is off, so the same opmode doubles as a way to read wall
 * distances by hand while driving normally.
 *
 * The sensor is read directly rather than through the Distance subsystem: that wrapper's filter
 * starts at 0.0, and a 0.0 inch reading is below every sane threshold, so the first loops would
 * trip the lockout before the first real sample landed.
 */

@Config
@TeleOp
public class driveTestingBorderAvoid extends LinearOpMode{
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    MecaTank mecaTank;
    BulkRead bulkRead;

    public static double TELEMETRY_INTERVAL_MS = 200.0;

    // --- border protection tuning -------------------------------------------------------------
    /** Master switch. B toggles it at runtime. */
    public static boolean BORDER_PROTECT = true;
    /** Sensor mounting: false = forward facing (back up), true = right facing (strafe left). */
    public static boolean SENSOR_STRAFES = false;
    /** Lock the driver out below this gap, inches. */
    public static double BORDER_THRESHOLD_IN = 8.0;
    /** Hysteresis: the gap must exceed threshold + this before the retreat is considered done. */
    public static double BORDER_RELEASE_MARGIN_IN = 3.0;
    /** How far the robot must actually travel before handing control back, inches (odometry). */
    public static double SAFE_RETREAT_IN = 6.0;
    /** Synthetic stick / trigger magnitude used for the retreat, 0..1. */
    public static double RETREAT_INPUT = 0.35;
    /** Hard stop on a retreat, seconds - covers a wall the robot cannot get away from. */
    public static double RETREAT_TIMEOUT_S = 2.0;
    /** After a retreat ends, the driver gets this long before protection can re-trigger. */
    public static double RETREAT_COOLDOWN_S = 0.5;
    /** I2C poll period. The 2m sensor is a real transaction, not part of the bulk read. */
    public static double BORDER_POLL_MS = 50.0;
    /** Consecutive close reads required to trip. Stops one spurious short read killing control. */
    public static int BORDER_TRIP_COUNT = 2;
    /** Reads at or above this are treated as no target in view, inches. */
    public static double BORDER_MAX_VALID_IN = 78.0;

    private Rev2mDistanceSensor borderSensor;
    private double borderDist = Double.POSITIVE_INFINITY;   // last accepted read, inches
    private boolean borderSensorOk = false;                 // last read produced a usable number
    private int closeReadStreak = 0;
    private boolean retreating = false;
    private Pose2d retreatStartPose = null;
    private double retreatTravelled = 0;
    private final ElapsedTime borderPollTimer = new ElapsedTime();
    private final ElapsedTime retreatTimer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private String retreatEndReason = "-";

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevA, prevB;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mecaTank = new MecaTank(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        // MecaTank's internal MecanumDrive forces every hub to AUTO, so take MANUAL back and let
        // one clearCache() per loop define the read cycle.
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();

        // This opmode drives the pose estimate itself, and only when a feature actually needs it.
        mecaTank.setAutoPoseUpdate(false);

        // A missing sensor must not stop the drive test - fall back to protection off.
        try {
            borderSensor = (Rev2mDistanceSensor)
                    hardwareMap.get(DistanceSensor.class, RobotConstants.distance);
        } catch (Exception e) {
            borderSensor = null;
            BORDER_PROTECT = false;
            telemetry.addLine("WARNING: no '" + RobotConstants.distance
                    + "' sensor - border protection disabled");
        }

        telemetry.addLine("MECANUM DRIVE TEST + BORDER PROTECTION");
        telemetry.addLine("LB = precision   RB = override");
        telemetry.addLine("Y = field centric   X = heading hold   Back = reset heading");
        telemetry.addLine("A = translation hold (straight strafe while turning)");
        telemetry.addLine("B = border protection   (" + (SENSOR_STRAFES ? "strafes left" : "backs up") + ")");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetryTimer.reset();
        borderPollTimer.reset();
        cooldownTimer.reset();
        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;

        while(!isStopRequested()&& opModeIsActive()) {
            bulkRead.clearCache();

            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // 1. Border sensor, throttled. Runs before the pose block so that a retreat starting
            //    this loop still gets its start pose from a fresh estimate.
            pollBorderSensor();
            boolean tooClose = BORDER_PROTECT && borderSensorOk && closeReadStreak >= BORDER_TRIP_COUNT;
            if (!BORDER_PROTECT && retreating) {
                retreating = false;                       // toggled off mid-retreat
                retreatEndReason = "cancelled";
            }
            if (tooClose && !retreating && cooldownTimer.seconds() >= RETREAT_COOLDOWN_S) {
                retreating = true;
                retreatStartPose = null;                  // captured just below, after the update
                retreatTravelled = 0;
                retreatTimer.reset();
                retreatEndReason = "-";
            }

            // Heading hold, field centric and traction control all read the pose estimate. Skip
            // the update entirely when none of them are on - it costs two IMU reads. A retreat
            // needs it too: measured travel is what says the robot is a SAFE distance away rather
            // than merely a timed guess.
            if (mecaTank.smoothDriveNeedsPose() || retreating) {
                mecaTank.updatePoseEstimate();
            }

            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            if (gamepad1.a && !prevA) MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            if (gamepad1.b && !prevB) BORDER_PROTECT = !BORDER_PROTECT;
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevBack = gamepad1.back;

            if (retreating) {
                if (retreatStartPose == null) retreatStartPose = mecaTank.getPoseEstimate();

                Pose2d now = mecaTank.getPoseEstimate();
                retreatTravelled = Math.hypot(now.position.x - retreatStartPose.position.x,
                                              now.position.y - retreatStartPose.position.y);

                boolean farEnough = retreatTravelled >= SAFE_RETREAT_IN
                        && borderDist >= BORDER_THRESHOLD_IN + BORDER_RELEASE_MARGIN_IN;

                if (farEnough || retreatTimer.seconds() >= RETREAT_TIMEOUT_S) {
                    // The cooldown runs even after a timeout, so a robot wedged against the wall
                    // hands control back periodically instead of pinning the driver out forever.
                    retreating = false;
                    retreatEndReason = farEnough ? "clear" : "timeout";
                    cooldownTimer.reset();
                    closeReadStreak = 0;
                    mecaTank.setDrivePowersSmooth(0, 0, 0, 0, false, false);
                } else {
                    // GAMEPAD DRIVING IS DISABLED HERE - every value below is synthetic. Same
                    // call the driver's inputs go through, so the signs need no separate proof.
                    double r = Math.max(0.0, Math.min(1.0, RETREAT_INPUT));
                    if (SENSOR_STRAFES) {
                        // Sensor faces right -> strafe LEFT, i.e. the left trigger.
                        mecaTank.setDrivePowersSmooth(0, 0, r, 0, false, true);
                    } else {
                        // Sensor faces forward -> BACK UP. Positive stick Y is reverse, because
                        // the input shaper negates stick Y before it becomes forward.
                        mecaTank.setDrivePowersSmooth(r, r, 0, 0, false, true);
                    }
                }
            } else {
                // ALL FOUR WHEELS REVERSED, relative to what this opmode used to do.
                //
                // Done by dropping the four negations rather than by setDirection(REVERSE) on the
                // motors, because the two are equivalent for driving but NOT for odometry: negating
                // all of L, R and S negates forward, turn and strafe, which negates all four wheel
                // outputs - exactly what flipping every motor would do. Flipping the motors instead
                // would also invert the odometry, because RawEncoder.applyDirection() multiplies the
                // encoder count by DcMotorEx.getDirection(), and TwoDeadWheelLocalizer reads the pods
                // through the fl and fr MOTOR ports. That would break localisation and every auto.
                //
                // This also brings the opmode in line with the rest of the codebase: every other
                // opmode already passes these four values un-negated.
                mecaTank.setDrivePowersSmooth(
                        gamepad1.left_stick_y, gamepad1.right_stick_y,
                        gamepad1.left_trigger, gamepad1.right_trigger,
                        gamepad1.left_bumper, gamepad1.right_bumper);
            }

            double now = System.nanoTime();
            double loopMs = (now - lastLoopTime) / 1e6;
            lastLoopTime = now;
            if (loopMs > loopMsMax) loopMsMax = loopMs;

            if (sendTelemetry) {
                telemetry.addData("Precision", gamepad1.left_bumper);
                telemetry.addData("Override", gamepad1.right_bumper);
                telemetry.addData("Field centric", MecaTank.FIELD_CENTRIC);
                telemetry.addData("Heading hold", MecaTank.HEADING_HOLD);
                telemetry.addData("Translation hold", MecaTank.TRANSLATION_HOLD);
                telemetry.addData("Traction control", MecaTank.TRACTION_CONTROL);
                telemetry.addData("Border protect", BORDER_PROTECT ? "on" : "off (reading only)");
                telemetry.addData("Border dist (in)", borderSensorOk ? borderDist : Double.NaN);
                telemetry.addData("Border sensor ok", borderSensorOk);
                telemetry.addData("Border close streak", closeReadStreak);
                // "Driver locked out" is the one line to read when the sticks feel dead.
                telemetry.addData("Driver locked out", retreating);
                telemetry.addData("Retreat axis", SENSOR_STRAFES ? "strafe left" : "backward");
                telemetry.addData("Retreat travelled (in)", retreatTravelled);
                telemetry.addData("Retreat ended", retreatEndReason);
                mecaTank.smoothDriveTelemetry();
                telemetry.addData("Loop Time (ms)", loopMs);
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }
    }

    /**
     * Reads the REV 2m sensor at most once every BORDER_POLL_MS and updates borderDist plus the
     * consecutive-close-read streak.
     *
     * Polling is deliberately NOT gated on BORDER_PROTECT: the distance stays on telemetry with
     * the retreat toggled off, which is what makes B usable as a "watch the number, keep the
     * sticks" mode for finding the right BORDER_THRESHOLD_IN by hand. Only the RETREAT is
     * suppressed, by the BORDER_PROTECT test on tooClose in the main loop. The cost is that the
     * I2C transaction keeps running while protection is off - it is bounded by BORDER_POLL_MS and
     * shows up in Loop Time either way.
     *
     * The streak keeps counting while protection is off, so re-arming with B inside the threshold
     * retreats on the spot rather than waiting for BORDER_TRIP_COUNT fresh samples. That is the
     * safe direction: B is asking for protection back, and the robot is already too close.
     *
     * Anything the sensor cannot vouch for - a timeout, a NaN, or a reading past
     * BORDER_MAX_VALID_IN (the 2m part reports its out-of-range value as a huge number) - clears
     * the streak and is reported as FAR. Between polls the previous verdict simply persists, so
     * the streak is a count of samples, not of loops.
     */
    private void pollBorderSensor() {
        if (borderSensor == null) {
            borderSensorOk = false;
            closeReadStreak = 0;
            return;
        }
        if (borderPollTimer.milliseconds() < BORDER_POLL_MS) return;
        borderPollTimer.reset();

        double d = borderSensor.getDistance(DistanceUnit.INCH);
        boolean valid = Double.isFinite(d) && d > 0 && d < BORDER_MAX_VALID_IN
                && !borderSensor.didTimeoutOccur();

        borderSensorOk = valid;
        if (!valid) {
            borderDist = Double.POSITIVE_INFINITY;   // fail toward "far": driver keeps control
            closeReadStreak = 0;
            return;
        }

        borderDist = d;
        if (d < BORDER_THRESHOLD_IN) {
            if (closeReadStreak < BORDER_TRIP_COUNT) closeReadStreak++;
        } else {
            closeReadStreak = 0;
        }
    }
}
