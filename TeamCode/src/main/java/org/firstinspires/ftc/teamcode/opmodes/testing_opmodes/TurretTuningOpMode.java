package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;

/**
 * Bench opmode for the MOTOR turret. Put the robot on the floor (or a box), turret facing straight
 * FORWARD before INIT - the encoder is zeroed at INIT. Graph "angle", "target" and "power" on FTC
 * Dashboard while tuning.
 *
 * <h2>1. Direction and gearing</h2>
 * Hold the left stick right/left (open-loop power). Turning the turret to the robot's LEFT must make
 * the angle go UP - if it goes down, flip TurretConstants.TURRET_MOTOR_REVERSED. Turn it exactly
 * 90 degrees by eye: the angle should read ~90. If not, TURRET_GEAR_RATIO / TURRET_MOTOR_TICKS_PER_REV
 * are wrong.
 *
 * <h2>2. Friction (kS)</h2>
 * Hold RIGHT TRIGGER: the power ramps up slowly from 0 until the turret starts moving; the power at
 * that moment is printed as "kS found". Put a little less than that in TURRET_KS.
 *
 * <h2>3. Position steps</h2>
 * A = 0, dpad right/left = +/-STEP_DEG from the current target, B / X = the left / right limit
 * minus 5 degrees, Y = sweep back and forth between +/-SWEEP_DEG (tracking test). Watch the settle
 * time and overshoot; raise TURRET_KVEL / TURRET_POS_KP for snappier, lower TURRET_DECEL_DEG_S2 if it
 * overshoots.
 *
 * <h2>4. Travel limit</h2>
 * TurretConstants.TURRET_LIMIT_DEG is the travel each way for EVERY opmode. Check B / X stop short
 * of the hard stops and cables. BACK re-zeroes (turret must face forward).
 */
@Config
@TeleOp(name = "Turret Tuning", group = "turret")
public class TurretTuningOpMode extends LinearOpMode {

    public static double STEP_DEG = 45.0;
    public static double SWEEP_DEG = 90.0;
    /** Sweep speed for the tracking test, DEGREES/SECOND. */
    public static double SWEEP_RATE_DEG_S = 120.0;
    /** Open-loop power at full left stick. */
    public static double MANUAL_POWER = 0.4;
    /** kS search: power ramp, per second. */
    public static double KS_RAMP_PER_S = 0.08;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BulkRead bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();
        Turret turret = new Turret(hardwareMap, telemetry, true);

        telemetry.addLine("TURRET TUNING - turret must face FORWARD now (zeroed at INIT)");
        telemetry.addData("Ticks per degree", "%.3f", Turret.ticksPerDeg());
        telemetry.addData("Travel", "+-%.0f deg (TurretConstants.TURRET_LIMIT_DEG)", Turret.maxAngleDeg());
        telemetry.update();
        waitForStart();

        double target = 0.0, ksPower = 0.0, ksFound = Double.NaN;
        boolean sweeping = false, prevA = false, prevB = false, prevX = false, prevY = false;
        boolean prevL = false, prevR = false, prevBack = false;
        double sweepPhase = 0.0, sweepDir = 1.0;
        ElapsedTime loop = new ElapsedTime(), stepTimer = new ElapsedTime();
        double settleS = Double.NaN, peakOver = 0.0, stepFrom = 0.0;
        boolean settled = true;

        while (opModeIsActive()) {
            bulkRead.clearCache();
            double dt = Math.min(0.1, loop.seconds());
            loop.reset();

            double oldTarget = target;
            if (gamepad1.a && !prevA) { target = 0.0; sweeping = false; }
            if (gamepad1.b && !prevB) { target = Turret.maxAngleDeg() - 5.0; sweeping = false; }
            if (gamepad1.x && !prevX) { target = Turret.minAngleDeg() + 5.0; sweeping = false; }
            if (gamepad1.dpad_right && !prevR) { target = Turret.clampAngle(target + STEP_DEG); sweeping = false; }
            if (gamepad1.dpad_left && !prevL) { target = Turret.clampAngle(target - STEP_DEG); sweeping = false; }
            if (gamepad1.y && !prevY) { sweeping = !sweeping; sweepPhase = turret.getAngleDeg(); }
            if (gamepad1.back && !prevBack) { turret.rezero(); target = 0.0; sweeping = false; }
            prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x; prevY = gamepad1.y;
            prevL = gamepad1.dpad_left; prevR = gamepad1.dpad_right; prevBack = gamepad1.back;
            if (target != oldTarget) {
                stepTimer.reset();
                stepFrom = turret.getAngleDeg();
                settled = false;
                peakOver = 0.0;
                settleS = Double.NaN;
            }

            String mode;
            if (Math.abs(gamepad1.left_stick_x) > 0.05) {
                turret.setManualPower(-gamepad1.left_stick_x * MANUAL_POWER);
                target = turret.getAngleDeg();
                sweeping = false;
                mode = "MANUAL (left stick)";
            } else if (gamepad1.right_trigger > 0.5) {
                ksPower += KS_RAMP_PER_S * dt;
                turret.setManualPower(ksPower);
                if (Double.isNaN(ksFound) && Math.abs(turret.getVelocityDegS()) > 5.0) ksFound = ksPower;
                target = turret.getAngleDeg();
                mode = "kS SEARCH (right trigger)";
            } else {
                ksPower = 0.0;
                double ff = 0.0;
                if (sweeping) {
                    sweepPhase += sweepDir * SWEEP_RATE_DEG_S * dt;
                    if (sweepPhase > SWEEP_DEG) { sweepPhase = SWEEP_DEG; sweepDir = -1.0; }
                    if (sweepPhase < -SWEEP_DEG) { sweepPhase = -SWEEP_DEG; sweepDir = 1.0; }
                    target = sweepPhase;
                    ff = sweepDir * SWEEP_RATE_DEG_S;
                }
                turret.setTarget(target, ff);
                mode = sweeping ? "SWEEP (Y)" : "HOLD TARGET";
            }
            if (gamepad1.right_trigger <= 0.5) ksPower = 0.0;
            turret.update();

            if (!settled && !sweeping) {
                double dir = Math.signum(target - stepFrom);
                peakOver = Math.max(peakOver, dir * (turret.getAngleDeg() - target));
                if (turret.onTarget()) {
                    settled = true;
                    settleS = stepTimer.seconds();
                }
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("angle", "%.2f", turret.getAngleDeg());
            telemetry.addData("target", "%.2f", turret.getTargetDeg());
            telemetry.addData("velocity (deg/s)", "%.1f", turret.getVelocityDegS());
            telemetry.addData("error", "%.2f", turret.getTargetDeg() - turret.getAngleDeg());
            telemetry.addData("Last step", Double.isNaN(settleS) ? "settling..." : String.format("settled in %.2f s, overshoot %.1f deg", settleS, peakOver));
            telemetry.addData("kS found", Double.isNaN(ksFound) ? "hold RT" : String.format("%.3f (set TURRET_KS a bit below)", ksFound));
            telemetry.addData("Travel", "+-%.0f deg   at limit: %s", Turret.maxAngleDeg(), turret.isAtHardStop());
            turret.telemetry();
            telemetry.addLine("A=0  dpad L/R=step  B/X=limits  Y=sweep  LS=manual  RT=kS  BACK=rezero");
            telemetry.update();
        }
        turret.stop();
    }
}
