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
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Intake;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.LaunchVelocityTable;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretAimController;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.Alliance;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretTarget;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.HiveSide;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretVision;

import java.util.List;

/**
 * The full teleop: drive testing, auto-aiming turret, shooting and intake in one opmode.
 *
 * <h2>Where the driving came from</h2>
 * Every line of the drivetrain setup and the gamepad1 bindings is copied VERBATIM from
 * {@code opmodes/testing_opmodes/driveTesting.java}. Nothing was renamed, simplified or
 * "improved", including the parts that file warns about itself - the un-negated stick values, the
 * defaults-off sign-sensitive features, and the manual bulk-read cycle. If the drive feels
 * different from driveTesting, that is a bug in this file.
 *
 * The one addition to the drive half is a single pose update in the TURRET section, explained at
 * the call site: the turret needs a fresh pose every loop, and driveTesting only updates the pose
 * when a drive feature happens to want it.
 *
 * INPUT MAPPING: driveTesting was converted from TANK to ARCADE (left stick translates, right
 * stick turns) and this file follows it exactly, so the two still drive identically.
 *
 * FIELD CENTRIC: driveTesting now enables it at init, and this file does exactly the same, so the
 * two drive identically. The rotation lives in MecaTank.smoothDriveCore() and is NOT
 * duplicated here - doing so would rotate the sticks twice. Read the sign warning in
 * driveTesting's header before the first drive; the rotation sign has never been validated.
 *
 * <h2>STRICT GAMEPAD SPLIT</h2>
 * <b>gamepad1 drives. gamepad2 shoots.</b> No driving control reads gamepad2 and no turret, intake
 * or launcher control reads gamepad1. The single exception is alliance selection, which is on
 * gamepad1 during INIT ONLY, before the drive loop exists - which is also why it cannot collide
 * with gamepad1.x's heading-hold toggle during play.
 *
 * <pre>
 * GAMEPAD 1 - DRIVING (copied from driveTesting, unchanged)
 *   Left stick X ........ strafe   (translation)
 *   Left stick Y ........ forward  (translation)
 *   Right stick X ....... turn
 *   Triggers ............ unused for driving; the left stick handles strafe
 *   Left bumper (hold) .. precision mode, 35%
 *   Right bumper (hold).. override - bypasses the accel limiter and traction control
 *   Y ................... toggle field centric (ON by default)
 *   X ................... toggle heading hold
 *   A ................... toggle translation hold
 *   Back ................ reset the heading reference
 *   Options ............. reset the heading reference (same action)
 *
 * GAMEPAD 1 - DURING INIT ONLY
 *   B ................... select RED alliance
 *   X ................... select BLUE alliance
 *   (locked in at start)
 *
 * GAMEPAD 2 - TURRET, LAUNCHER, INTAKE
 *   X ................... toggle auto-aim / manual turret jog
 *   Left stick X ........ manual turret jog (manual mode only)
 *   Dpad left ........... force the LEFT hive
 *   Dpad right .......... force the RIGHT hive
 *   Dpad down ........... back to automatic hive selection
 *   Right trigger (hold)  spin the flywheel up to the table velocity
 *   A ................... feed one ball - blocked unless aimed AND at velocity
 *   Right bumper ........ intake start
 *   Left bumper ......... intake stop
 *   B ................... intake reverse
 * </pre>
 */
@Config
@TeleOp(name = "Turret Drive + Shoot", group = "turret")
public class TurretDriveShootTeleOp extends LinearOpMode {

    // =============================================================================================
    // DRIVE SECTION - fields copied verbatim from driveTesting.java
    // =============================================================================================
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    MecaTank mecaTank;
    BulkRead bulkRead;

    public static double TELEMETRY_INTERVAL_MS = 200.0;

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevY, prevX, prevBack, prevA, prevOptions;

    // =============================================================================================
    // TURRET / SHOOTING SECTION - fields
    // =============================================================================================
    private Turret turret;
    private TurretVision vision;
    private TurretAimController aimController;
    private Launcher launcher;
    private Intake intake;
    private LaunchVelocityTable velocityTable;

    private Alliance alliance = Alliance.RED;
    private boolean autoAim = true;
    private double manualTurretAngleDeg = 0.0;

    /** gamepad2 edge-detect state. Kept separate from the gamepad1 prev* fields above on purpose. */
    private boolean g2PrevX, g2PrevA, g2PrevDpadLeft, g2PrevDpadRight, g2PrevDpadDown;

    /** Manual jog rate, DEGREES/SECOND. */
    public static double MANUAL_JOG_DEG_PER_SEC = 60.0;

    /** Right-trigger pull that counts as "spin up". */
    public static double SPINUP_TRIGGER_THRESHOLD = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- DRIVE SETUP - verbatim from driveTesting.java --------------------------------------
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
        // ---- end verbatim drive setup -------------------------------------------------------------

        // ---- TURRET SETUP -------------------------------------------------------------------------
        turret = new Turret(hardwareMap, telemetry);
        vision = new TurretVision(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        velocityTable = new LaunchVelocityTable();
        aimController = new TurretAimController(turret, alliance, telemetry);

        launcher.init();
        turret.center();

        // Mirror the live stream (with the crosshair overlay) to the dashboard.
        // Null-guarded: a camera that failed to open must not take the opmode down.
        if (vision.getCameraStreamSource() != null) {
            FtcDashboard.getInstance().startCameraStream(vision.getCameraStreamSource(), 0);
        }

        // ---- INIT LOOP: pick the alliance -------------------------------------------------------
        //
        // On gamepad1 deliberately. The drive loop does not run until start(), so gamepad1.x and
        // gamepad1.b are free here even though they are heading-hold and unused during play. Doing
        // it on gamepad2 would collide with intake-reverse on B.
        while (opModeInInit()) {
            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;

            vision.update();

            telemetry.addLine("TURRET DRIVE + SHOOT");
            telemetry.addLine("gamepad1: B = RED    X = BLUE");
            telemetry.addData(">>> ALLIANCE", alliance);
            telemetry.addLine("");
            telemetry.addData("Camera", vision.getCameraStateString());
            telemetry.addData("Camera setup", vision.getCameraSetupReport());
            telemetry.addData("Feeder", launcher.hasFeeder() ? "ok" : "NONE - fire disabled");
            telemetry.addData("Turret hardware", turret.isHardwareOk() ? "ok" : turret.getHardwareFault());
            telemetry.addData("Launcher hardware", launcher.isHardwareOk() ? "ok" : launcher.getHardwareFault());
            telemetry.addData("Intake hardware", intake.isHardwareOk() ? "ok" : "MISSING");
            telemetry.addData("Velocity table", velocityTable.describe());
            telemetry.addLine("");
            telemetry.addLine("gamepad1 = DRIVE ONLY   gamepad2 = SHOOT/INTAKE ONLY");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // Lock the alliance in.
        aimController.setAlliance(alliance);

        telemetryTimer.reset();
        double lastLoopTime = System.nanoTime();
        double loopMsMax = 0;
        double lastJogSeconds = System.nanoTime() / 1e9;

        while (!isStopRequested() && opModeIsActive()) {
            bulkRead.clearCache();

            boolean sendTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;

            // =========================================================================================
            // DRIVE - gamepad1 ONLY. Verbatim from driveTesting.java.
            // =========================================================================================

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

            // =========================================================================================
            // TURRET / LAUNCHER / INTAKE - gamepad2 ONLY
            // =========================================================================================

            // The turret needs a field pose EVERY loop - it aims by geometry from the odometry pose
            // to the estimated target coordinate, whether or not a tag is visible. driveTesting only
            // updates the pose when a drive feature wants it, so top it up here when the drive
            // block above skipped it. Written as a complement of the drive block's own condition so
            // the pose is still read exactly once per loop and the copied drive code is untouched.
            if (!mecaTank.smoothDriveNeedsPose()) {
                mecaTank.updatePoseEstimate();
            }

            Pose2d robotPose = mecaTank.getPoseEstimate();
            double robotAngVel = mecaTank.getMeasuredAngVel();

            vision.update();
            // ALL valid alliance clusters, not just the best one. An alliance has two hives and the
            // controller keeps a separate estimate for each, so handing it only the best-covered
            // cluster would starve whichever hive was further away or partly occluded this frame.
            List<TurretTarget> targets = vision.getAllianceTargets(alliance);

            double nowSeconds = System.nanoTime() / 1e9;
            double jogDt = nowSeconds - lastJogSeconds;
            lastJogSeconds = nowSeconds;

            // ---- Auto-aim / manual jog toggle -----------------------------------------------------
            if (gamepad2.x && !g2PrevX) {
                autoAim = !autoAim;
                if (!autoAim) manualTurretAngleDeg = turret.getCommandedAngleDeg();
            }
            g2PrevX = gamepad2.x;

            // ---- Hive override (gamepad2 dpad) ----------------------------------------------------
            // The two hives are only ~24 in apart, so automatic selection can reasonably pick the
            // one the driver did not want - a partner robot parked in front of one, say, which the
            // camera cannot know about. Dpad left/right commit to a hive outright and bypass the
            // hysteresis entirely, because a driver pressing a button wants the turret to move now.
            if (gamepad2.dpad_left && !g2PrevDpadLeft) aimController.setHiveOverride(HiveSide.LEFT);
            if (gamepad2.dpad_right && !g2PrevDpadRight) aimController.setHiveOverride(HiveSide.RIGHT);
            if (gamepad2.dpad_down && !g2PrevDpadDown) aimController.clearHiveOverride();
            g2PrevDpadLeft = gamepad2.dpad_left;
            g2PrevDpadRight = gamepad2.dpad_right;
            g2PrevDpadDown = gamepad2.dpad_down;

            if (autoAim) {
                aimController.update(robotPose, robotAngVel, targets);
            } else {
                // Manual jog for bench testing. The aim controller is not run at all, so it does
                // not fight the stick, and its target estimate simply goes stale until auto-aim is
                // switched back on.
                double jog = -gamepad2.left_stick_x;
                if (Math.abs(jog) > 0.05) {
                    manualTurretAngleDeg += jog * MANUAL_JOG_DEG_PER_SEC * jogDt;
                }
                turret.setOutputAngleDeg(manualTurretAngleDeg);
            }

            // Push aim state to the crosshair overlay every loop, so the stream shows where the
            // turret is actually pointed against where the cell opening actually is.
            aimController.updateOverlay(vision.getOverlay());

            // ---- Launcher -------------------------------------------------------------------------
            double distanceIn = aimController.getTargetDistanceInches();
            double commandedVelocity = velocityTable.getVelocity(distanceIn);

            boolean wantSpinUp = gamepad2.right_trigger > SPINUP_TRIGGER_THRESHOLD;
            if (wantSpinUp) {
                launcher.setTargetVelocity(commandedVelocity);
            } else {
                launcher.stop();
            }

            // FIRING GATE. All three conditions are required, and none is optional:
            //   readyToFire()      - a LIVE, scorable, alliance-matched cluster is visible and the
            //                        turret is actually pointed at it. Firing on the odometry
            //                        estimate alone would shoot at where the cell used to be, with
            //                        no evidence its opening is still facing up.
            //   isAtVelocity()     - the flywheel is up to speed. Below it the ball throws short.
            //   hasFeeder()        - there is something to actually push a ball in with.
            //
            // The feed mechanism on this robot is UNDEFINED, so hasFeeder() is false today and the
            // fire button cannot do anything. That is intentional. Aiming, spin-up and every
            // telemetry readout still work, so the whole system can be tested on the field right
            // now; only the last few inches of the ball's journey are missing. Telemetry below says
            // so in as many words rather than leaving the button looking broken.
            boolean feederAvailable = launcher.hasFeeder();
            boolean canFire = aimController.readyToFire()
                    && launcher.isAtVelocity()
                    && feederAvailable;
            if (gamepad2.a && !g2PrevA && canFire) {
                launcher.feedOne();
            }
            g2PrevA = gamepad2.a;

            launcher.update();

            // ---- Intake ---------------------------------------------------------------------------
            if (gamepad2.right_bumper) intake.start();
            if (gamepad2.left_bumper) intake.stop();
            if (gamepad2.b) intake.reverse();

            // =========================================================================================
            // TELEMETRY
            // =========================================================================================
            double now = System.nanoTime();
            double loopMs = (now - lastLoopTime) / 1e6;
            lastLoopTime = now;
            if (loopMs > loopMsMax) loopMsMax = loopMs;

            if (sendTelemetry) {
                telemetry.addData("ALLIANCE", alliance);

                telemetry.addLine("--- DRIVE ---");
                telemetry.addData("Precision", gamepad1.left_bumper);
                telemetry.addData("Override", gamepad1.right_bumper);
                telemetry.addData("Field centric", MecaTank.FIELD_CENTRIC);
                telemetry.addData("Heading hold", MecaTank.HEADING_HOLD);
                telemetry.addData("Translation hold", MecaTank.TRANSLATION_HOLD);
                telemetry.addData("Traction control", MecaTank.TRACTION_CONTROL);
                mecaTank.smoothDriveTelemetry();
                telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg",
                        robotPose.position.x, robotPose.position.y,
                        Math.toDegrees(robotPose.heading.toDouble()));

                telemetry.addLine("--- TURRET ---");
                telemetry.addData("Mode", autoAim ? "AUTO-AIM" : "MANUAL JOG");
                turret.telemetry();
                aimController.telemetry();

                telemetry.addLine("--- VISION ---");
                telemetry.addData("Camera", vision.getCameraStateString());
                telemetry.addData("Camera setup", vision.getCameraSetupReport());
                telemetry.addData("Exposure/gain/WB applied", "%d ms / %d / %d K",
                        vision.getAppliedExposureMs(), vision.getAppliedGain(),
                        vision.getAppliedWhiteBalanceK());
                if (targets.isEmpty()) {
                    telemetry.addData("Clusters", "none: " + vision.getLastRejectReason());
                } else {
                    // Both hives listed when both are in frame, so the driver can see what the
                    // automatic selector is choosing between before overriding it.
                    for (int i = 0; i < targets.size(); i++) {
                        telemetry.addData("Cluster " + i, targets.get(i).toString());
                    }
                }
                telemetry.addData("Hive override", aimController.getHiveOverride() == null
                        ? "auto" : aimController.getHiveOverride() + " (dpad down = auto)");

                telemetry.addLine("--- SHOOTER ---");
                telemetry.addData("Distance (in)", Double.isNaN(distanceIn) ? "--"
                        : String.format("%.1f", distanceIn));
                telemetry.addData("Commanded velocity (tps)", "%.0f", commandedVelocity);
                launcher.telemetry();
                intake.telemetry();

                telemetry.addLine("--- FIRE GATE ---");
                if (canFire) {
                    telemetry.addLine("READY TO FIRE - press gamepad2 A");
                } else if (!feederAvailable) {
                    // Called out first and separately, because this one is not a transient
                    // condition the driver can fix by aiming better - the hardware is undefined.
                    telemetry.addLine("NO FEEDER CONFIGURED - fire disabled");
                    telemetry.addLine("  flywheel and auto-aim still work; describe the feed");
                    telemetry.addLine("  mechanism to enable the fire action");
                    telemetry.addData("  aim would be", aimController.readyToFire()
                            ? "READY" : aimController.getNotReadyReason());
                } else {
                    String why = aimController.getNotReadyReason();
                    if (why.isEmpty()) why = launcher.getFireBlockReason();
                    if (why.isEmpty()) why = "unknown";
                    telemetry.addData("BLOCKED", why);
                }

                telemetry.addData("Loop Time (ms)", loopMs);
                telemetry.addData("Loop Time Max (ms)", loopMsMax);
                telemetry.update();
                loopMsMax = 0;
                telemetryTimer.reset();
            }
        }

        launcher.stop();
        intake.stop();
        vision.close();
    }
}
