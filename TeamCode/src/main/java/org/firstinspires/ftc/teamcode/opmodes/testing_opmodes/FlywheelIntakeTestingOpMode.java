package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Intake;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;

/**
 * Bench opmode for tuning a FIXED (non-turret) flywheel shooter, with the intake rollers available
 * so the feed-and-shoot cycle can be exercised for real. NO DRIVETRAIN.
 *
 * <h2>Why a second, drive-free copy exists</h2>
 * This is {@link FlywheelTestingOpMode} with the drivetrain removed and nothing else changed. The
 * flywheel and intake logic is identical, line for line.
 *
 * It exists because the robot does not have an Expansion Hub yet, so there are not enough motor
 * ports for the four drive motors AND the shooter at the same time. Constructing MecaTank would
 * make {@code hardwareMap.get()} fail on drive motors that are not plugged in, which throws before
 * the opmode ever reaches the flywheel, so simply not touching the sticks would not have been
 * enough. Only two motors are claimed here: the flywheel and the intake rollers.
 *
 * Once the Expansion Hub is fitted, {@link FlywheelTestingOpMode} is the same tool with driving
 * added. Keep gains in sync between the two, or better, delete this one.
 *
 * <h2>What this is for</h2>
 * Every constant that actually affects {@code NGMotor.updateFlywheels()} is live-editable here from
 * the gamepad, and separately from FTC Dashboard. Change one, watch the velocity trace, keep what
 * works, then copy the final numbers into their permanent homes (listed at the bottom of this
 * comment).
 *
 * <h2>How NGMotor's flywheel loop actually works - read this before tuning</h2>
 * The loop is at {@code NGMotor.updateFlywheels()}, NGMotor.java:155. Per iteration it computes:
 *
 * <pre>
 *   smooth  = ALPHA*raw + (1-ALPHA)*previousSmooth          // velocity low-pass
 *   error   = target - smooth                                // ticks/second
 *   P_Term  = error * (error &gt; RECOVERY_THRESHOLD ? kP_Recovery : kP_Stable)
 *   I_Term  = kI * integral      // integral ZEROED whenever abs(error) &gt;= RECOVERY_THRESHOLD,
 *                                // and clamped so this term never exceeds +-0.1 power
 *   D_Term  = kD * d(error)/dt
 *   F_Term  = kF * target                                    // NOT kF*error - pure feedforward
 *   Load    = kLoad + kRamp*secondsFeeding                   // only while the feeder flag is true
 *   power   = (P+I+D+F+Load) * (12.0 / batteryVolts)         // voltage compensated
 * </pre>
 *
 * <h3>Five things about that loop that will bite you</h3>
 * <ol>
 *   <li><b>The P you pass to {@code setCustomVelocityPID} is ignored.</b> It is stored in
 *       {@code kP} and never read. The real gains are {@code kP_Recovery} and {@code kP_Stable},
 *       package-private fields at NGMotor.java:86-87, which is why this opmode reaches them through
 *       {@link NGMotorFlywheelTuner}.</li>
 *   <li><b>The recovery test is one-sided.</b> {@code error > RECOVERY_THRESHOLD}, not
 *       {@code abs(error)}. Overshoot is therefore only ever corrected by the weak
 *       {@code kP_Stable}, so a feedforward set slightly too HIGH settles very slowly, while one
 *       set slightly too LOW is pulled up briskly. Tune kF from below.</li>
 *   <li><b>The integral is zeroed, not frozen, outside the threshold band</b>, and its contribution
 *       is hard-clamped to +-0.1 power by {@code maxISum = 0.1/kI}. It exists to trim a small
 *       steady-state offset and nothing more. Raising kI does not raise its authority.</li>
 *   <li><b>{@code F_Term = kF * target}</b>, so kF is roughly (power needed to hold target) divided
 *       by (target). It does the bulk of the work; P only closes the remainder.</li>
 *   <li><b>{@code setCustomVelocityPID} resets the integral, the last error AND the loop timer.</b>
 *       Calling it every loop would keep the derivative dividing by a near-zero dt and stop the
 *       integral ever accumulating. This opmode calls it only when a gain actually changes, and
 *       does so at the END of the loop so the timer has a full cycle before the next read. Plain
 *       target-velocity changes are written straight to the public {@code targetVelocity} field,
 *       which avoids the reset entirely.</li>
 * </ol>
 *
 * <h2>CONTROLS</h2>
 *
 * Everything is on GAMEPAD 2, unchanged from the version with driving, so the two opmodes feel the
 * same and muscle memory carries over. Gamepad 1 is unused. On the Driver Station, bind a second
 * controller with Start + B.
 *
 * <pre>
 * GAMEPAD 2 - FLYWHEEL AND INTAKE
 *   Right bumper ........ intake IN
 *   Left bumper ......... intake STOP
 *   B ................... intake REVERSE
 *   Y ................... flywheel run/stop toggle
 *   Dpad up / down ...... target velocity +/- (ticks/s)
 *   X / A ............... select next / previous tunable constant
 *   Dpad right / left ... increase / decrease the selected constant
 *   Right trigger (hold)  coarse: 10x step        Left trigger (hold) fine: 0.1x step
 *   Start (hold) ........ simulate FEEDER ACTIVE, to tune kLoad and kRamp
 *   Back ................ re-apply gains and clear the integral
 * </pre>
 *
 * <h2>Where the tuned numbers go permanently</h2>
 * See the report delivered with this file. In short: kI/kD/kF go to the
 * {@code setCustomVelocityPID} call site that drives this flywheel, target velocity goes to
 * {@code BiobuzzRobotConstants.fixedShootingVel}, and kP_Recovery/kP_Stable/ALPHA/
 * RECOVERY_THRESHOLD have no other home than their field initialisers in NGMotor.java.
 */
@Config
@TeleOp(name = "Flywheel + Intake Testing (no drive)", group = "tuning")
public class FlywheelIntakeTestingOpMode extends LinearOpMode {

    // =============================================================================================
    // Dashboard-editable starting values
    //
    // These seed the gamepad-editable working copies below. Editing them on the dashboard mid-run
    // does nothing on purpose - the gamepad owns the values once the opmode starts, and having two
    // writers for one number is how you lose a tuning session.
    // =============================================================================================

    /** Starting target velocity, TICKS/SECOND. Seeded from the value the robot already uses. */
    public static double START_TARGET_VELOCITY = BiobuzzRobotConstants.fixedShootingVel;

    /** Starting gains. Seeded from the values Intake2_0.java:88 already runs the flywheels at. */
    //kF: 0.0007
    //kP stable: 0.005
    //kP recovery: 0.0320
    //Target vel: 1500
    public static double START_KI = 0;
    public static double START_KD = 0;
    public static double START_KF = 0.0007;

    /** Step sizes for the gamepad editor, in each constant's own units. */
    public static double TARGET_STEP = 25.0;

    /** Velocity band counted as "at speed", TICKS/SECOND. Drives the at-speed light and dip timer. */
    public static double AT_SPEED_TOLERANCE = 30.0;

    /** How far below target counts as a shot-induced dip, TICKS/SECOND. */
    public static double DIP_DETECT_TICKS = 40.0;

    /** Telemetry refresh interval, MILLISECONDS. */
    public static double TELEMETRY_INTERVAL_MS = 100.0;

    // =============================================================================================
    // Hub bulk reads
    //
    // NOT drive code. One clearCache() per loop makes every getVelocity()/getCurrentPosition() in
    // the loop come from a single hub round trip instead of one each, which is what keeps the
    // control loop fast enough for the derivative and integral terms to mean anything.
    // =============================================================================================
    BulkRead bulkRead;

    private final ElapsedTime telemetryTimer = new ElapsedTime();

    // =============================================================================================
    // Shooter and intake
    // =============================================================================================
    private NGMotor flywheel;
    private Intake intake;

    private double targetVelocity;
    private boolean flywheelRunning = false;

    /** Set whenever a gain changes, cleared once it has been pushed into NGMotor. */
    private boolean gainsDirty = true;

    // Working copies of every tunable. kI/kD/kF reach NGMotor via setCustomVelocityPID; the rest
    // reach it via NGMotorFlywheelTuner because NGMotor keeps them package-private.
    private double kI, kD, kF;
    private double kpRecovery, kpStable, alpha, recoveryThreshold, kLoad, kRamp;

    /** Index into {@link #PARAM_NAMES} of the constant the dpad currently edits. */
    private int selectedParam = 0;

    private static final String[] PARAM_NAMES = {
            "kF", "kP_Stable", "kP_Recovery", "kI", "kD",
            "RECOVERY_THRESHOLD", "ALPHA", "kLoad", "kRamp"
    };

    /**
     * Base step per parameter, in that parameter's own units. Chosen so one press is a visible but
     * not violent change: roughly 2-5% of a sane value for the gains, and a round number for the
     * two that are in ticks/second.
     */
    private static final double[] PARAM_STEPS = {
            0.00005,  // kF        - power per tick/s; sane range is ~0.0003-0.0009
            0.000025,   // kP_Stable - power per tick/s
            0.0005,    // kP_Recovery
            0.00005,    // kI
            0.00005,  // kD
            10.0,      // RECOVERY_THRESHOLD - ticks/s
            0.05,     // ALPHA     - unitless 0..1
            0.05,     // kLoad     - power
            0.1       // kRamp     - power per second
    };

    private boolean g2PrevY, g2PrevX, g2PrevA, g2PrevUp, g2PrevDown, g2PrevLeft, g2PrevRight, g2PrevBack;

    // Shot dip / recovery measurement.
    private boolean wasAtSpeed = false;
    private boolean inDip = false;
    private double dipMinVelocity = 0.0;
    private final ElapsedTime dipTimer = new ElapsedTime();
    private double lastDipDepth = Double.NaN;
    private double lastRecoveryMs = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Take MANUAL hub caching and let one clearCache() per loop define the read cycle.
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();

        // ---- SHOOTER AND INTAKE SETUP -----------------------------------------------------------
        // Config names resolved from existing code, not invented: BiobuzzRobotConstants.flywheels
        // is "flywheels" and is what Intake2_0 already drives. Intake reads
        // TurretConstants.INTAKE_MOTOR_NAME, which resolves to BiobuzzRobotConstants.rollers.
        flywheel = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.flywheels);
        // RUN_WITHOUT_ENCODER with BRAKE. The loop computes power itself rather than handing a
        // velocity to the controller's built-in PIDF, but getVelocity() still needs the encoder
        // plugged in - if velocity reads a flat zero while the wheel spins, that cable is why.
        flywheel.init();

        intake = new Intake(hardwareMap, telemetry);

        targetVelocity = START_TARGET_VELOCITY;
        kI = START_KI;
        kD = START_KD;
        kF = START_KF;
        kpRecovery = NGMotorFlywheelTuner.getRecoveryKp(flywheel);
        kpStable = NGMotorFlywheelTuner.getStableKp(flywheel);
        alpha = NGMotorFlywheelTuner.getAlpha(flywheel);
        recoveryThreshold = NGMotorFlywheelTuner.getRecoveryThreshold(flywheel);
        kLoad = NGMotor.kLoad;
        kRamp = NGMotor.kRamp;
//kF: 0.0007
//kP stable: 0.005
//kP recovery: 0.0320
//Target vel: 1500
        telemetry.addLine("FLYWHEEL + INTAKE TESTING (no drive)");
        telemetry.addLine("gp2: Y = run/stop   dpad up/dn = target");
        telemetry.addLine("gp2: X/A = select constant   dpad L/R = adjust");
        telemetry.addLine("gp2: RT = 10x step   LT = 0.1x step   Start = fake feeder");
        telemetry.addLine("gp2: RB = intake   LB = stop   B = reverse");
        telemetry.addLine("");
        telemetry.addLine("Flywheel starts STOPPED. Clear the shooter before pressing Y.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetryTimer.reset();

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();
            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // =========================================================================================
            // INTAKE - gamepad2 only
            // =========================================================================================
            if (gamepad2.right_bumper) intake.start();
            if (gamepad2.left_bumper) intake.stop();
            if (gamepad2.b) intake.reverse();

            // =========================================================================================
            // FLYWHEEL - gamepad2 only
            // =========================================================================================

            // ---- run / stop ----------------------------------------------------------------------
            if (gamepad2.y && !g2PrevY) {
                flywheelRunning = !flywheelRunning;
                if (!flywheelRunning) {
                    // Cut power rather than asking the velocity loop for zero, which would command
                    // a large negative power and brake the wheel hard against its own momentum.
                    flywheel.targetVelocity = 0.0;
                    flywheel.setAbsPower(0.0);
                    NGMotorFlywheelTuner.resetVelocityFilter(flywheel);
                    resetDipTracking();
                } else {
                    // Re-arm the gains on every spin-up so the run starts from a clean integrator.
                    gainsDirty = true;
                }
            }
            g2PrevY = gamepad2.y;

            // ---- target velocity -------------------------------------------------------------------
            // Written straight to the public field, NOT through setCustomVelocityPID, because that
            // method also wipes the integral and restarts the loop timer. Nudging the target while
            // watching the trace should not reset the controller underneath you.
            double targetStep = TARGET_STEP * stepMultiplier();
            if (gamepad2.dpad_up && !g2PrevUp) targetVelocity += targetStep;
            if (gamepad2.dpad_down && !g2PrevDown) targetVelocity -= targetStep;
            if (targetVelocity < 0) targetVelocity = 0;
            g2PrevUp = gamepad2.dpad_up;
            g2PrevDown = gamepad2.dpad_down;

            // ---- constant selection and adjustment --------------------------------------------------
            if (gamepad2.x && !g2PrevX) selectedParam = (selectedParam + 1) % PARAM_NAMES.length;
            if (gamepad2.a && !g2PrevA) {
                selectedParam = (selectedParam - 1 + PARAM_NAMES.length) % PARAM_NAMES.length;
            }
            g2PrevX = gamepad2.x;
            g2PrevA = gamepad2.a;

            if (gamepad2.dpad_right && !g2PrevRight) adjustSelected(+1);
            if (gamepad2.dpad_left && !g2PrevLeft) adjustSelected(-1);
            g2PrevRight = gamepad2.dpad_right;
            g2PrevLeft = gamepad2.dpad_left;

            if (gamepad2.back && !g2PrevBack) gainsDirty = true;
            g2PrevBack = gamepad2.back;

            // ---- push the non-PIDF constants -------------------------------------------------------
            // These are plain field writes with no side effects, so they can go every loop.
            NGMotorFlywheelTuner.setRecoveryKp(flywheel, kpRecovery);
            NGMotorFlywheelTuner.setStableKp(flywheel, kpStable);
            NGMotorFlywheelTuner.setAlpha(flywheel, alpha);
            NGMotorFlywheelTuner.setRecoveryThreshold(flywheel, recoveryThreshold);
            NGMotor.kLoad = kLoad;
            NGMotor.kRamp = kRamp;

            // ---- run the loop ------------------------------------------------------------------------
            // Start held simulates a ball going through, so kLoad and kRamp can be tuned without a
            // feeder mechanism existing yet.
            boolean simulatedFeeder = gamepad2.start;

            if (flywheelRunning) {
                flywheel.targetVelocity = targetVelocity;
                flywheel.updateFlywheels(simulatedFeeder);
            } else {
                flywheel.setAbsPower(0.0);
            }

            double rawVelocity = flywheel.getVelocity();
            trackDip(rawVelocity);

            // =========================================================================================
            // TELEMETRY
            // =========================================================================================
            if (sendTelemetry) {
                double error = targetVelocity - rawVelocity;
                boolean atSpeed = flywheelRunning && Math.abs(error) <= AT_SPEED_TOLERANCE;

                telemetry.addData("FLYWHEEL", flywheelRunning ? (atSpeed ? "AT SPEED" : "SPINNING")
                        : "STOPPED");
                telemetry.addData("target (ticks/s)", "%.0f", targetVelocity);
                telemetry.addData("raw velocity", "%.0f", rawVelocity);
                telemetry.addData("smoothed velocity", "%.0f",
                        NGMotorFlywheelTuner.getSmoothedVelocity(flywheel));
                telemetry.addData("error", "%.0f", error);
                telemetry.addData("motor power", "%.3f", flywheel.getPower());
                telemetry.addData("feeder sim", simulatedFeeder ? "ACTIVE (Start held)" : "off");

                telemetry.addLine("--- SHOT RECOVERY ---");
                telemetry.addData("last dip depth (ticks/s)", Double.isNaN(lastDipDepth) ? "--"
                        : String.format("%.0f", lastDipDepth));
                telemetry.addData("last recovery (ms)", Double.isNaN(lastRecoveryMs) ? "--"
                        : String.format("%.0f", lastRecoveryMs));

                telemetry.addLine("--- CONSTANTS (X/A select, dpad L/R adjust) ---");
                for (int i = 0; i < PARAM_NAMES.length; i++) {
                    telemetry.addData((i == selectedParam ? ">> " : "   ") + PARAM_NAMES[i],
                            formatParam(i));
                }
                telemetry.addData("step multiplier", "%.2fx", stepMultiplier());
                telemetry.addData("gains", gainsDirty ? "PENDING" : "applied");

                telemetry.addLine("--- INTAKE ---");
                intake.telemetry();

                telemetry.update();
                telemetryTimer.reset();
            }

            // ---- apply pending gain changes LAST -----------------------------------------------------
            // setCustomVelocityPID resets the integral, the last error and the loop timer. Doing it
            // here, after updateFlywheels() has already run, leaves a full loop period before the
            // next velocity read, so the derivative term is not handed a near-zero dt to divide by.
            // Expect one slightly lumpy cycle after any gain change - that is the reset, not the
            // new gain misbehaving.
            if (gainsDirty) {
                flywheel.setCustomVelocityPID(targetVelocity, kpStable, kI, kD, kF);
                gainsDirty = false;
            }
        }

        flywheel.setAbsPower(0.0);
        intake.stop();
    }

    // =============================================================================================
    // Helpers
    // =============================================================================================

    /**
     * Step scaling from the triggers: right for coarse, left for fine.
     * Coarse first when both are held, because that is the one you reach for deliberately.
     */
    private double stepMultiplier() {
        if (gamepad2.right_trigger > 0.5) return 10.0;
        if (gamepad2.left_trigger > 0.5) return 0.1;
        return 1.0;
    }

    /** Nudges the selected constant by one step in the given direction. */
    private void adjustSelected(int direction) {
        double step = PARAM_STEPS[selectedParam] * stepMultiplier() * direction;
        switch (selectedParam) {
            case 0: kF = Math.max(0.0, kF + step); gainsDirty = true; break;
            case 1: kpStable = Math.max(0.0, kpStable + step); break;
            case 2: kpRecovery = Math.max(0.0, kpRecovery + step); break;
            case 3: kI = Math.max(0.0, kI + step); gainsDirty = true; break;
            case 4: kD = Math.max(0.0, kD + step); gainsDirty = true; break;
            case 5: recoveryThreshold = Math.max(1.0, recoveryThreshold + step); break;
            // ALPHA outside 0..1 makes the low-pass diverge rather than filter.
            case 6: alpha = Math.min(1.0, Math.max(0.01, alpha + step)); break;
            case 7: kLoad = kLoad + step; break;
            case 8: kRamp = kRamp + step; break;
            default: break;
        }
    }

    /** Formats one constant for telemetry, with enough decimals to see a single step. */
    private String formatParam(int i) {
        switch (i) {
            case 0: return String.format("%.6f", kF);
            case 1: return String.format("%.5f", kpStable);
            case 2: return String.format("%.4f", kpRecovery);
            case 3: return String.format("%.4f", kI);
            case 4: return String.format("%.6f", kD);
            case 5: return String.format("%.0f", recoveryThreshold);
            case 6: return String.format("%.2f", alpha);
            case 7: return String.format("%.2f", kLoad);
            case 8: return String.format("%.2f", kRamp);
            default: return "?";
        }
    }

    /**
     * Measures how far a shot knocks the wheel down and how long it takes to come back.
     *
     * This is the number that actually matters for BioBuzz. Steady-state precision is easy; what
     * limits a cycle is how long the wheel spends below target after each artifact goes through,
     * because that is dead time between shots. Tune to minimise recovery time, not to flatten the
     * trace.
     */
    private void trackDip(double velocity) {
        if (!flywheelRunning || targetVelocity <= 0) {
            resetDipTracking();
            return;
        }

        boolean atSpeed = Math.abs(targetVelocity - velocity) <= AT_SPEED_TOLERANCE;

        if (!inDip) {
            // A dip only counts if the wheel had actually reached speed first, otherwise the
            // initial spin-up would be logged as a shot every single time.
            if (wasAtSpeed && velocity < targetVelocity - DIP_DETECT_TICKS) {
                inDip = true;
                dipMinVelocity = velocity;
                dipTimer.reset();
            }
        } else {
            if (velocity < dipMinVelocity) dipMinVelocity = velocity;
            if (atSpeed) {
                lastDipDepth = targetVelocity - dipMinVelocity;
                lastRecoveryMs = dipTimer.milliseconds();
                inDip = false;
            }
        }
        wasAtSpeed = atSpeed;
    }

    private void resetDipTracking() {
        inDip = false;
        wasAtSpeed = false;
        dipMinVelocity = 0.0;
    }
}
