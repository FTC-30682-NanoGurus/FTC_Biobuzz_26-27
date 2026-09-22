package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;

/**
 * Mecanum drive test opmode.
 *
 * CONTROLS - ARCADE mapping, the conventional field-centric layout
 *   Left stick X ........ strafe   (translation)
 *   Left stick Y ........ forward  (translation)
 *   Right stick X ....... turn
 *   Triggers ............ unused for driving now; the left stick handles strafe
 *   Left bumper (hold) .. precision mode, 35%
 *   Right bumper (hold).. override - bypasses the accel limiter and traction control
 *   Y ................... toggle field centric      (now default ON - see below)
 *   X ................... toggle heading hold       (default OFF - see sign note)
 *   Back ................ reset the heading reference
 *   Options ............. reset the heading reference (same action, thumb-reachable)
 *
 *
 * INPUT MAPPING CHANGED: this opmode used to be TANK (left stick Y drove the left side, right
 * stick Y the right side, triggers strafed). It is now ARCADE, because field-centric driving with
 * a tank stick pair is close to unusable - "forward" stops being a direction the two sticks can
 * express once the frame is rotating under them. The tank entry point still exists on MecaTank,
 * unchanged, for anything already tuned against it.
 *
 * FIELD CENTRIC IS NOW THE DEFAULT. It is switched on at init below rather than left to the Y
 * toggle, so the driver's stick directions are field-relative from the moment the opmode starts.
 *
 * The transform itself is NOT duplicated here. MecaTank.smoothDriveCore() already rotates the
 * translation vector by the negative of the heading, sourcing that heading from the tuned localizer
 * (drive.pose.heading) - see MecaTank step 3a. Re-applying the same rotation in this opmode would
 * rotate the sticks TWICE and the robot would drive off at an angle that changed as it turned.
 *
 * "Forward" is defined by MecaTank.fieldCentricRef, not by the raw odometry heading, so resetting
 * it re-zeroes the DRIVER's reference only and leaves odometry x/y and the pose estimate untouched.
 * That is what Back and Options both do, via mecaTank.resetDriveHeading().
 *
 * SIGN WARNING - STILL UNVALIDATED, READ BEFORE THE FIRST DRIVE. Field centric, heading hold and
 * traction control were all written with a rotation sign that has never been confirmed on the real
 * robot. Field centric is now on by default, so check IT first: put the robot on blocks, drive the
 * left stick forward, then rotate the chassis 90 degrees by hand. The commanded wheel direction
 * should stay pointing the same way in the ROOM. If it instead swings the wrong way, flip
 * MecaTank.FIELD_CENTRIC_SIGN to -1 from the dashboard. For heading hold, lift the wheels and twist
 * the chassis by hand: the wheels should spin so as to UNDO the twist. If they fight to continue
 * it, flip MecaTank.HEADING_HOLD_SIGN to -1.
 *
 * The arcade mapping adds two more unvalidated signs. On blocks: push the left stick RIGHT and the
 * robot should strafe right - if not, flip MecaTank.ARCADE_STRAFE_SIGN. Push the right stick RIGHT
 * and the robot should turn clockwise seen from above - if not, flip MecaTank.ARCADE_TURN_SIGN.
 * Check these two BEFORE field centric, because a wrong translation sign will make the
 * field-centric check above look broken when it is fine.
 */

@Config
@TeleOp
public class driveTesting extends LinearOpMode{
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    MecaTank mecaTank;
    BulkRead bulkRead;

    public static double TELEMETRY_INTERVAL_MS = 200.0;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevA, prevOptions;

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

        // FIELD CENTRIC ON BY DEFAULT. This is the whole of the change - the rotation is already
        // implemented inside MecaTank.smoothDriveCore(), which reads drive.pose.heading from
        // the tuned localizer every loop. Switching the flag on here also makes
        // smoothDriveNeedsPose() return true, which is what causes updatePoseEstimate() to run each
        // loop below, so the heading the transform uses is never a loop stale.
        //
        // Y still toggles it off for back-to-back comparison while validating the sign.
        MecaTank.FIELD_CENTRIC = true;

        // Define "forward" as wherever the robot is pointing when the opmode starts. Without this
        // the reference is whatever a previous opmode left in the static, which is a confusing way
        // to start a match.
        mecaTank.updatePoseEstimate();
        mecaTank.resetDriveHeading();

        telemetry.addLine("MECANUM DRIVE TEST - FIELD CENTRIC");
        telemetry.addLine("LB = precision   RB = override");
        telemetry.addLine("Y = field centric (ON)   X = heading hold");
        telemetry.addLine("Back / Options = reset heading reference");
        telemetry.addLine("A = translation hold (straight strafe while turning)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetryTimer.reset();
        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;

        while(!isStopRequested()&& opModeIsActive()) {
            bulkRead.clearCache();

            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // Heading hold, field centric and traction control all read the pose estimate. Skip
            // the update entirely when none of them are on - it costs two IMU reads.
            if (mecaTank.smoothDriveNeedsPose()) {
                mecaTank.updatePoseEstimate();
            }

            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            if (gamepad1.a && !prevA) MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            // Heading reset on either button. Re-zeroes the DRIVER's field-centric reference
            // (MecaTank.fieldCentricRef) so the driver can redefine "forward"; odometry x/y and the
            // pose estimate are not touched, so autos and the turret's geometry stay valid.
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();
            if (gamepad1.options && !prevOptions) mecaTank.resetDriveHeading();
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevA = gamepad1.a;
            prevBack = gamepad1.back;
            prevOptions = gamepad1.options;

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
            // ARCADE mapping: LEFT stick translates (x = strafe, y = forward), RIGHT stick turns.
            // This is the conventional field-centric layout. Everything downstream of the sticks is
            // the same code the tank mapping used - see MecaTank.smoothDriveCore() - so the curve,
            // the field-centric rotation, traction control and the accel limiter are unchanged.
            //
            // The triggers no longer strafe; the left stick's X axis does. They are free now.
            mecaTank.setDrivePowersSmoothArcade(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper, gamepad1.right_bumper);

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
                mecaTank.smoothDriveTelemetry();
                telemetry.addData("Loop Time (ms)", loopMs);
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }
    }
}
