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
 *
 *
 * SIGN WARNING: field centric, heading hold and traction control default to OFF because the
 * correct rotation sign cannot be confirmed without driving the robot. Enable them one at a time
 * from the dashboard. For heading hold, lift the wheels off the ground and twist the chassis by
 * hand: the wheels should spin so as to UNDO the twist. If they fight to continue it, flip
 * MecaTank.HEADING_HOLD_SIGN to -1.
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
    private boolean prevY, prevX, prevBack, prevA;

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

        telemetry.addLine("MECANUM DRIVE TEST");
        telemetry.addLine("LB = precision   RB = override");
        telemetry.addLine("Y = field centric   X = heading hold   Back = reset heading");
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
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();
            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevA = gamepad1.a;
            prevBack = gamepad1.back;

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
