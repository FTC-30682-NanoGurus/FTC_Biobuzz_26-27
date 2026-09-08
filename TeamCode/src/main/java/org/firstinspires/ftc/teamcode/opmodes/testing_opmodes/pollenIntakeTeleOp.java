package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.PollenApproach;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.PollenCamera;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.vision.PollenGeometry;
import org.firstinspires.ftc.teamcode.vision.PollenPipeline;

/**
 * Mecanum drive test opmode, plus one-button automatic drive to the largest pile of pollen.
 *
 * The manual half is {@link driveTesting} unchanged - same sticks, same triggers, same signs, same
 * feature toggles. Everything new hangs off the dpad and B, so a driver who already knows the
 * drive test already knows this opmode.
 *
 * CONTROLS
 *   Left stick Y ........ left side  (tank)
 *   Right stick Y ....... right side (tank)
 *   Left trigger ........ strafe left
 *   Right trigger ....... strafe right
 *   Left bumper (hold) .. precision mode, 35%
 *   Right bumper (hold).. override - bypasses the accel limiter and traction control
 *   Y ................... toggle field centric   (default OFF - see the sign note in MecaTank)
 *   X ................... toggle heading hold    (default OFF - see the sign note in MecaTank)
 *   A ................... toggle translation hold
 *   Back ................ reset the heading reference
 *
 *   DPAD UP ............. drive to the largest pollen pile     &lt;-- the new button
 *   B ................... cancel the automatic drive
 *   DPAD DOWN ........... toggle the camera stream on / off
 *   DPAD RIGHT .......... toggle the raw colour mask on the preview (for tuning HSV)
 *
 * ONE OF TWO THINGS OWNS THE DRIVETRAIN
 * -------------------------------------
 * Either the driver does, through MecaTank.setDrivePowersSmooth(), or the approach does, through
 * RoadRunner's follower. Never both: they write the same four motors by two different routes, and
 * interleaving them per loop would make the robot fight itself. The if/else in the main loop is
 * the entire arbitration, and touching the sticks is what moves the boundary back.
 *
 * POSE IS UPDATED EVERY LOOP HERE, unlike in driveTesting
 * ------------------------------------------------------
 * driveTesting skips updatePoseEstimate() when no drive feature needs it, because it costs an IMU
 * read. This opmode cannot: the camera converts each sighting into FIELD coordinates using the
 * pose that is current when the frame is read, so a stale pose puts the pollen in the wrong place
 * on the field, and the robot then drives confidently to the wrong spot. Expect a slightly longer
 * loop time here than in the plain drive test. While the approach is running the update is skipped
 * instead - RoadRunner's follower already does one inside the action, and a second is waste.
 *
 * WHAT THIS OPMODE DOES NOT DO
 * ----------------------------
 * It does not intake. It stops PollenApproach.STANDOFF_IN short of the pile with the intake side
 * facing it and hands the robot back. Running the intake is the next piece of work; the hook is
 * the ARRIVED state marked in the loop below.
 */
@Config
@TeleOp(name = "Pollen Intake TeleOp", group = "biobuzz")
public class pollenIntakeTeleOp extends LinearOpMode {

    public static double TELEMETRY_INTERVAL_MS = 200.0;
    /** Auto-approach can be disabled outright from the dashboard without editing the opmode. */
    public static boolean POLLEN_ENABLED = true;

    // ---- intake roller -----------------------------------------------------------------------
    /** Config name of the intake roller motor. */
    public static String INTAKE_MOTOR = DECODERobotConstants.rollers;
    /** Set true if the roller spins the wrong way. Cheaper than re-wiring in the pit. */
    public static boolean INTAKE_REVERSED = false;
    /** Power used by the hold-on toggle and by the auto-run. Triggers are proportional. */
    public static double INTAKE_POWER = 1.0;
    /** Eject power. Negative - it is the opposite direction from intake. */
    public static double EJECT_POWER = -0.8;
    /** Trigger travel below this counts as released. */
    public static double INTAKE_TRIGGER_DEADBAND = 0.10;
    /**
     * Spin the roller automatically when an auto-approach reaches the pile.
     *
     * OFF by default on purpose: a mechanism that starts itself the first time you press the
     * approach button is a surprise, and surprises near a moving robot are how fingers get caught.
     * Turn it on once the approach is stopping where you want it.
     */
    public static boolean AUTO_INTAKE_ON_ARRIVAL = false;
    /** How long the auto-run keeps the roller going, seconds. */
    public static double AUTO_INTAKE_S = 1.5;

    MecaTank mecaTank;
    BulkRead bulkRead;
    PollenCamera pollenCamera;
    PollenApproach pollenApproach;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevA, prevB;
    private boolean prevDpadUp, prevDpadDown, prevDpadRight;
    private boolean cameraOn = true;

    private DcMotor intakeMotor;
    private boolean intakeLatched = false;
    private boolean autoIntakeRunning = false;
    private double lastIntakePower = 0;
    private String intakeStatus = "not initialised";
    private boolean prevG2A, prevG1DpadLeft;
    private PollenApproach.State prevApproachState = PollenApproach.State.IDLE;
    private final ElapsedTime autoIntakeTimer = new ElapsedTime();

    // ---- init-time tuner -----------------------------------------------------------------
    /**
     * The values worth reaching without a laptop, in the order the bring-up procedure needs them.
     *
     * This exists because the dashboard is not always reachable in a pit, and because walking back
     * to a laptop between every 1 degree nudge of CAM_PITCH_DEG makes the range calibration in
     * stage 5 take an hour instead of five minutes. It is init-only: the drive control map after
     * START is left byte-for-byte identical to driveTesting, which is the whole point of that map.
     */
    private static final String[] TUNABLES = {
            "H_LOW", "H_HIGH", "S_LOW", "V_LOW",
            "CLOSE_PX", "CAM_PITCH_DEG", "CAM_HEIGHT_IN", "EXPOSURE_MS"
    };
    /** Multiplier applied to every step while the right bumper is held. */
    private static final double COARSE = 10.0;
    private int tuneIdx = 0;
    private boolean tPrevLeft, tPrevRight, tPrevUp, tPrevDown, tPrevX;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mecaTank = new MecaTank(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        // MecaTank's internal MecanumDrive forces every hub to AUTO, so take MANUAL back and let
        // one clearCache() per loop define the read cycle.
        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();

        // This opmode drives the pose estimate itself - see the header.
        mecaTank.setAutoPoseUpdate(false);

        // A missing roller must not take the drive and vision half of the opmode down with it -
        // same rule the camera follows. Everything else stays usable; only the roller is lost.
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(INTAKE_REVERSED
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeStatus = "ready";
        } catch (Exception e) {
            intakeMotor = null;
            intakeStatus = "NO MOTOR '" + INTAKE_MOTOR + "'";
        }

        pollenCamera = new PollenCamera(hardwareMap, telemetry);
        pollenCamera.init();
        pollenApproach = new PollenApproach(mecaTank, pollenCamera, telemetry);

        telemetry.addLine("MECANUM DRIVE + POLLEN AUTO APPROACH");
        telemetry.addLine("LB = precision   RB = override");
        telemetry.addLine("Y = field centric   X = heading hold   Back = reset heading");
        telemetry.addLine("A = translation hold");
        telemetry.addLine("DPAD UP = drive to pollen   B = cancel");
        telemetry.addLine("DPAD DOWN = camera on/off   DPAD RIGHT = mask view");
        telemetry.addLine("INTAKE: gp2 RT = in, gp2 LT = eject, gp2 A / gp1 DPAD LEFT = hold on");
        telemetry.addLine("Intake: " + intakeStatus);
        telemetry.addLine(pollenCamera.isAvailable()
                ? "Camera opened. MEASURE PollenGeometry before trusting the range."
                : "WARNING: camera failed to open - manual drive only");
        telemetry.update();

        // INIT LOOP, rather than a bare waitForStart(). Three things depend on it:
        //
        //  1. MANUAL EXPOSURE. It can only be applied once the portal actually reaches STREAMING,
        //     which is a moment after build() returns - so it is applied from the tracker, not
        //     from init(). With a bare waitForStart() nothing would call the tracker until after
        //     START, meaning the whole init preview would run on AUTO exposure and then visibly
        //     jump the instant the match began. Any HSV band tuned against that preview would be
        //     tuned against the wrong image.
        //
        //  2. LIVE TELEMETRY while aiming. Blob and cluster counts update as the camera is moved,
        //     which is what makes the camera aimable at all before START.
        //
        //  3. DRAW_MASK can be toggled from the dashboard and the result seen immediately. The
        //     DPAD RIGHT shortcut cannot work here - no loop is reading the gamepad yet.
        //
        // update() with no pose is the right call: it runs the exposure and the pipeline readout
        // but deliberately refuses to place a target on the field, because with the robot not yet
        // running there is no pose estimate worth trusting.
        while (opModeInInit()) {
            pollenCamera.update();
            initTuner();

            telemetry.addLine("INIT TUNING - no motors, safe to hold the camera");
            telemetry.addLine("DPAD L/R = pick value   DPAD U/D = change it");
            telemetry.addLine("RB (hold) = coarse x10   X = mask view");
            telemetry.addLine();
            telemetry.addData("> " + TUNABLES[tuneIdx], "%.2f   (step %.2f%s)",
                    tuneGet(tuneIdx), tuneStep(tuneIdx) * (gamepad1.right_bumper ? COARSE : 1),
                    gamepad1.right_bumper ? ", COARSE" : "");
            telemetry.addData("Mask view", PollenPipeline.DRAW_MASK);
            telemetry.addLine();
            telemetry.addData("HSV", "H %.0f-%.0f  S %.0f+  V %.0f+",
                    PollenPipeline.H_LOW, PollenPipeline.H_HIGH,
                    PollenPipeline.S_LOW, PollenPipeline.V_LOW);
            telemetry.addData("Mount", "%.2f in high, %.1f deg down",
                    PollenGeometry.CAM_HEIGHT_IN, PollenGeometry.CAM_PITCH_DEG);
            telemetry.addLine();
            pollenCamera.telemetry();
            telemetry.addLine();
            telemetry.addLine("Press START when detection is stable.");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            pollenCamera.close();
            return;
        }

        telemetryTimer.reset();
        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();

            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // ---- edge-detected buttons -------------------------------------------------------
            boolean dpadUp = gamepad1.dpad_up && !prevDpadUp;
            boolean dpadDown = gamepad1.dpad_down && !prevDpadDown;
            boolean dpadRight = gamepad1.dpad_right && !prevDpadRight;
            boolean bPressed = gamepad1.b && !prevB;

            if (gamepad1.y && !prevY) MecaTank.FIELD_CENTRIC = !MecaTank.FIELD_CENTRIC;
            if (gamepad1.x && !prevX) MecaTank.HEADING_HOLD = !MecaTank.HEADING_HOLD;
            if (gamepad1.a && !prevA) MecaTank.TRANSLATION_HOLD = !MecaTank.TRANSLATION_HOLD;
            if (gamepad1.back && !prevBack) mecaTank.resetDriveHeading();

            prevY = gamepad1.y;
            prevX = gamepad1.x;
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevBack = gamepad1.back;
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadRight = gamepad1.dpad_right;

            if (dpadDown) {
                cameraOn = !cameraOn;
                pollenCamera.setStreaming(cameraOn);
                if (!cameraOn) pollenApproach.cancel("camera stopped");
            }
            if (dpadRight) {
                PollenPipeline.DRAW_MASK = !PollenPipeline.DRAW_MASK;
            }

            // Largest magnitude the driver is asking for. One number, so the abort test in
            // PollenApproach does not have to know this opmode's control layout.
            double driverInput = Math.max(
                    Math.max(Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.right_stick_y)),
                    Math.max(gamepad1.left_trigger, gamepad1.right_trigger));

            // ---- drivetrain arbitration ------------------------------------------------------
            if (pollenApproach.isRunning()) {
                // The camera keeps tracking THROUGHOUT the approach, not just before it. That is
                // what makes PollenApproach's re-planning real: with the tracker frozen, the
                // confirmed target could never change and the re-plan test could never fire.
                //
                // The pose used here is one loop old - the follower refreshed it at the end of the
                // previous iteration - which at teleop loop rates is a fraction of an inch.
                pollenCamera.update(mecaTank.getPoseEstimate());

                // The follower calls updatePoseEstimate() itself, and it owns the motors.
                pollenApproach.update(driverInput, bPressed, opModeIsActive());
            } else {
                mecaTank.updatePoseEstimate();
                pollenCamera.update(mecaTank.getPoseEstimate());

                if (dpadUp && POLLEN_ENABLED) {
                    // start() reports its own refusal reason through getStatus(), which the
                    // telemetry block below already prints. Nothing to handle here.
                    //
                    // The first follower step runs in this same iteration so the motors are never
                    // left holding the driver's last command for a loop after the handover.
                    if (pollenApproach.start()) {
                        pollenApproach.update(driverInput, false, opModeIsActive());
                    }
                }

                if (!pollenApproach.isRunning()) {
                    // ALL FOUR WHEELS REVERSED relative to what this opmode used to do - the four
                    // values below are passed un-negated, matching driveTesting and the rest of
                    // the codebase. See the long note in driveTesting for why this is done here
                    // and not with setDirection(REVERSE): flipping the motors would also invert
                    // the odometry, which is read through the same motor ports.
                    mecaTank.setDrivePowersSmooth(
                            gamepad1.left_stick_y, gamepad1.right_stick_y,
                            gamepad1.left_trigger, gamepad1.right_trigger,
                            gamepad1.left_bumper, gamepad1.right_bumper);
                }
            }

            // The approach reaching ARRIVED is an EDGE, not a state to poll: it stays ARRIVED
            // until the next run starts, so testing the state directly would re-trigger the
            // auto-run every loop and the roller would never stop.
            PollenApproach.State approachState = pollenApproach.getState();
            boolean justArrived = approachState == PollenApproach.State.ARRIVED
                    && prevApproachState != PollenApproach.State.ARRIVED;
            prevApproachState = approachState;

            updateIntake(justArrived);

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
                telemetry.addData("Camera stream", cameraOn);
                telemetry.addData("Intake", "%s  power %.2f%s", intakeStatus, lastIntakePower,
                        intakeLatched ? "  [HELD]" : (autoIntakeRunning ? "  [AUTO]" : ""));
                telemetry.addLine();
                pollenCamera.telemetry();
                pollenApproach.telemetry();
                Pose2d p = mecaTank.getPoseEstimate();
                telemetry.addData("Pose", "%.1f, %.1f, %.0f deg",
                        p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
                telemetry.addLine();
                mecaTank.smoothDriveTelemetry();
                telemetry.addData("Loop Time (ms)", loopMs);
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }

        if (intakeMotor != null) intakeMotor.setPower(0);
        pollenApproach.cancel("opmode ended");
        pollenCamera.close();
    }

    /**
     * Intake roller, driven from gamepad 2 so that nothing here can collide with the driving map
     * on gamepad 1 - that map is deliberately identical to driveTesting.
     *
     *   gp2 right trigger .... intake, proportional
     *   gp2 left trigger ..... eject, proportional
     *   gp2 A ................ toggle hold-on at INTAKE_POWER
     *   gp1 DPAD LEFT ........ same toggle, for testing with a single gamepad
     *
     * Eject always wins. If the roller has jammed on a ball, the driver reaching for reverse
     * should not have to first remember to cancel a latch or wait out an auto-run.
     */
    private void updateIntake(boolean justArrived) {
        if (intakeMotor == null) return;

        boolean toggle = (gamepad2.a && !prevG2A) || (gamepad1.dpad_left && !prevG1DpadLeft);
        prevG2A = gamepad2.a;
        prevG1DpadLeft = gamepad1.dpad_left;
        if (toggle) intakeLatched = !intakeLatched;

        if (AUTO_INTAKE_ON_ARRIVAL && justArrived) {
            autoIntakeRunning = true;
            autoIntakeTimer.reset();
        }
        if (autoIntakeRunning && autoIntakeTimer.seconds() >= AUTO_INTAKE_S) {
            autoIntakeRunning = false;
        }

        double power;
        if (gamepad2.left_trigger > INTAKE_TRIGGER_DEADBAND) {
            power = EJECT_POWER * gamepad2.left_trigger;
            intakeLatched = false;      // eject cancels both automatic sources, so releasing the
            autoIntakeRunning = false;  // trigger leaves the roller stopped rather than re-intaking
        } else if (gamepad2.right_trigger > INTAKE_TRIGGER_DEADBAND) {
            power = INTAKE_POWER * gamepad2.right_trigger;
        } else if (intakeLatched || autoIntakeRunning) {
            power = INTAKE_POWER;
        } else {
            power = 0;
        }

        power = Math.max(-1.0, Math.min(1.0, power));
        intakeMotor.setPower(power);
        lastIntakePower = power;
    }

    // ===========================================================================================
    // Init-time tuner. Called only from the init loop, so none of it can move the robot.
    // ===========================================================================================

    private void initTuner() {
        boolean left = gamepad1.dpad_left && !tPrevLeft;
        boolean right = gamepad1.dpad_right && !tPrevRight;
        boolean up = gamepad1.dpad_up && !tPrevUp;
        boolean down = gamepad1.dpad_down && !tPrevDown;
        boolean x = gamepad1.x && !tPrevX;
        tPrevLeft = gamepad1.dpad_left;
        tPrevRight = gamepad1.dpad_right;
        tPrevUp = gamepad1.dpad_up;
        tPrevDown = gamepad1.dpad_down;
        tPrevX = gamepad1.x;

        if (left) tuneIdx = (tuneIdx + TUNABLES.length - 1) % TUNABLES.length;
        if (right) tuneIdx = (tuneIdx + 1) % TUNABLES.length;
        if (x) PollenPipeline.DRAW_MASK = !PollenPipeline.DRAW_MASK;

        if (up || down) {
            double step = tuneStep(tuneIdx) * (gamepad1.right_bumper ? COARSE : 1.0);
            tuneAdd(tuneIdx, up ? step : -step);
        }
    }

    private double tuneStep(int i) {
        switch (i) {
            case 0: case 1: return 1;      // hue, in OpenCV's 0..179 units
            case 2: case 3: return 5;      // saturation / value, 0..255
            case 4: return 2;              // CLOSE_PX - stepping by 2 keeps the kernel odd
            case 5: return 0.5;            // pitch, degrees
            case 6: return 0.25;           // height, inches
            default: return 1;             // exposure, ms
        }
    }

    private double tuneGet(int i) {
        switch (i) {
            case 0: return PollenPipeline.H_LOW;
            case 1: return PollenPipeline.H_HIGH;
            case 2: return PollenPipeline.S_LOW;
            case 3: return PollenPipeline.V_LOW;
            case 4: return PollenPipeline.CLOSE_PX;
            case 5: return PollenGeometry.CAM_PITCH_DEG;
            case 6: return PollenGeometry.CAM_HEIGHT_IN;
            default: return PollenCamera.EXPOSURE_MS;
        }
    }

    /**
     * Applies a delta, clamped to the range each value is actually defined over.
     *
     * The clamps are not decoration. A hue past 179 or a negative saturation silently produces an
     * empty mask with no error; a CAM_HEIGHT_IN at or below the ball radius makes the ground-plane
     * intersection undefined and every detection vanishes. Finding that out by feel, in a pit,
     * with the mask view on, is exactly the hour this tuner is meant to save.
     */
    private void tuneAdd(int i, double d) {
        switch (i) {
            case 0: PollenPipeline.H_LOW = clamp(PollenPipeline.H_LOW + d, 0, 179); break;
            case 1: PollenPipeline.H_HIGH = clamp(PollenPipeline.H_HIGH + d, 0, 179); break;
            case 2: PollenPipeline.S_LOW = clamp(PollenPipeline.S_LOW + d, 0, 255); break;
            case 3: PollenPipeline.V_LOW = clamp(PollenPipeline.V_LOW + d, 0, 255); break;
            case 4: PollenPipeline.CLOSE_PX = (int) clamp(PollenPipeline.CLOSE_PX + d, 1, 31); break;
            case 5: PollenGeometry.CAM_PITCH_DEG = clamp(PollenGeometry.CAM_PITCH_DEG + d, 0, 89); break;
            case 6: PollenGeometry.CAM_HEIGHT_IN =
                    clamp(PollenGeometry.CAM_HEIGHT_IN + d, PollenGeometry.ballRadiusIn() + 0.5, 40); break;
            default:
                PollenCamera.EXPOSURE_MS = (int) clamp(PollenCamera.EXPOSURE_MS + d, 1, 100);
                // One-shot by design, so an edit here is invisible until the apply is re-armed.
                pollenCamera.reapplyExposure();
                break;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
