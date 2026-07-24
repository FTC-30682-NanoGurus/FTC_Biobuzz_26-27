package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.subsystems.ArmClaw;

/**
 * Driver / test opmode for the TETRIX channel arm + 180 degree servo claw.
 *
 *  ARM
 *   Left stick Y ........ manual arm power (push up = raise). Gravity-compensated, so the arm
 *                         feels weightless. Release and it holds wherever you left it.
 *   D-pad Up / Down ..... fine trim. Ramps the target while held instead of stepping per loop.
 *   Y ................... preset HIGH
 *   B ................... preset MID
 *   X ................... preset FLOOR (intake)
 *   D-pad Left .......... preset STOW (home)
 *
 *  CLAW
 *   A ................... toggle open / closed
 *   Left bumper ......... open   (0.00)
 *   Right bumper ........ close  (0.22)
 *   Right trigger ....... analog squeeze - overrides the buttons while held past the deadband,
 *                         so you can feather the grip on a delicate game piece.
 *
 *  UTILITY
 *   Right stick button .. HOLD TO KILL the arm motor. The arm sags while held; releasing
 *                         re-holds it exactly where it ended up. Use this the first time you run
 *                         a new arm, with a hand under it.
 *   Back ................ re-zero the arm encoder HERE (declare this position home).
 *                         Only do this with the arm physically at START_ANGLE_DEG.
 *
 * Loop cost is kept in line with the rest of the codebase: one bulk cache clear, one ArmClaw
 * update (which does exactly one encoder read), and telemetry throttled to 10 Hz with addData
 * gated by the same flag, since FTC telemetry only auto-clears on update().
 */
@Config
@TeleOp(name = "Arm + Claw TeleOp (Test)", group = "testing")
public class ArmClawTeleOp extends LinearOpMode {

    /**
     * Preset arm angles in DEGREES FROM HORIZONTAL - the same reference as ArmClaw.START_ANGLE_DEG.
     * These are placeholders that sit inside the default soft limits; set them to whatever your
     * build actually needs (dashboard-tunable, so you can find them live).
     */
    public static double STOW_DEG = 0.0;
    public static double FLOOR_DEG = -10.0;
    public static double MID_DEG = 45.0;
    public static double HIGH_DEG = 90.0;

    /** Trim rate for the d-pad, in encoder ticks per second held. */
    public static double TRIM_TICKS_PER_SEC = 250.0;

    /** Right trigger has to move past this before it takes the claw off the buttons. */
    public static double CLAW_TRIGGER_DEADBAND = 0.05;

    public static double TELEMETRY_INTERVAL_MS = 100.0;

    private ArmClaw armClaw;
    private BulkRead bulkRead;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime trimTimer = new ElapsedTime();

    // Rising-edge state. Held locally rather than relying on SDK button helpers so the behaviour
    // is identical across SDK versions.
    private boolean prevA, prevX, prevB, prevY, prevDpadLeft, prevBack, prevLb, prevRb;
    private boolean triggerHadClaw = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armClaw = new ArmClaw(hardwareMap, telemetry);
        bulkRead = new BulkRead(hardwareMap);

        // No MecanumDrive is constructed here, so nothing overrides BulkRead's MANUAL mode and
        // clearCache() below is what actually drives the read cycle.

        telemetry.addLine("ARM + CLAW TEST");
        telemetry.addLine();
        telemetry.addLine("Place the arm at START_ANGLE_DEG before pressing START.");
        telemetry.addLine("init() zeroes the encoder there.");
        telemetry.addLine();
        telemetry.addLine("Stick Y = manual   D-pad U/D = trim");
        telemetry.addLine("Y/B/X = HIGH/MID/FLOOR   D-pad L = stow");
        telemetry.addLine("A = toggle claw   LB/RB = open/close   RT = squeeze");
        telemetry.addLine("R-stick button = HOLD TO KILL   Back = re-zero");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Zero the encoder and capture the starting hold position.
        armClaw.init();

        telemetryTimer.reset();
        trimTimer.reset();

        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;

        while (opModeIsActive() && !isStopRequested()) {
            bulkRead.clearCache();

            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            double trimDt = trimTimer.seconds();
            trimTimer.reset();
            if (trimDt > 0.1) trimDt = 0.1;

            // ----------------------------------------------------------------------------- kill
            armClaw.setEnabled(!gamepad1.right_stick_button);

            // ------------------------------------------------------------------------------ arm
            // Stick is negated: pushing the stick forward reads negative, and forward should raise.
            armClaw.setManualPower(-gamepad1.left_stick_y);

            // Trim only makes sense when the driver is not already holding the stick.
            if (!armClaw.isManual()) {
                if (gamepad1.dpad_up) {
                    armClaw.nudgeTicks(TRIM_TICKS_PER_SEC * trimDt);
                } else if (gamepad1.dpad_down) {
                    armClaw.nudgeTicks(-TRIM_TICKS_PER_SEC * trimDt);
                }
            }

            if (gamepad1.y && !prevY) armClaw.setTargetDegrees(HIGH_DEG);
            if (gamepad1.b && !prevB) armClaw.setTargetDegrees(MID_DEG);
            if (gamepad1.x && !prevX) armClaw.setTargetDegrees(FLOOR_DEG);
            if (gamepad1.dpad_left && !prevDpadLeft) armClaw.setTargetDegrees(STOW_DEG);

            // ----------------------------------------------------------------------------- claw
            double squeeze = gamepad1.right_trigger;
            if (squeeze > CLAW_TRIGGER_DEADBAND) {
                // Analog grip: trigger travel maps onto open -> closed.
                armClaw.setClawPosition(ArmClaw.CLAW_OPEN
                        + squeeze * (ArmClaw.CLAW_CLOSED - ArmClaw.CLAW_OPEN));
                triggerHadClaw = true;
            } else {
                if (triggerHadClaw) {
                    // Trigger just released - park the claw open rather than leaving it mid-travel.
                    armClaw.openClaw();
                    triggerHadClaw = false;
                }
                if (gamepad1.a && !prevA) armClaw.toggleClaw();
                if (gamepad1.left_bumper && !prevLb) armClaw.openClaw();
                if (gamepad1.right_bumper && !prevRb) armClaw.closeClaw();
            }

            // -------------------------------------------------------------------------- utility
            if (gamepad1.back && !prevBack) {
                // Re-declare "home" at the arm's current physical position.
                armClaw.init();
            }

            // ------------------------------------------------------------------- run the control
            armClaw.update();

            // ------------------------------------------------------------------- edge bookkeeping
            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevB = gamepad1.b;
            prevY = gamepad1.y;
            prevDpadLeft = gamepad1.dpad_left;
            prevBack = gamepad1.back;
            prevLb = gamepad1.left_bumper;
            prevRb = gamepad1.right_bumper;

            // ------------------------------------------------------------------------ telemetry
            double now = System.nanoTime();
            double loopMs = (now - lastLoopTime) / 1e6;
            lastLoopTime = now;
            if (loopMs > loopMsMax) loopMsMax = loopMs;

            if (sendTelemetry) {
                if (!armClaw.isEnabled()) telemetry.addLine(">>> ARM KILLED (holding R-stick) <<<");
                armClaw.telemetry();
                telemetry.addData("Loop Time (ms)", loopMs);
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }
    }
}
