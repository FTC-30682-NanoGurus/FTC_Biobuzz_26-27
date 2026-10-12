package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Intake;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;

@Config
@TeleOp(name = "Nectar Flywheel + Intake Testing (no drive)", group = "tuning")
public class NectarFlywheelIntakeTestingOpMode extends LinearOpMode {

    public static double START_TARGET_VELOCITY = 1600;

    public static double START_KI = 0;
    public static double START_KD = 0;
    public static double START_KF = 0.000426; //0.000445
    public static double START_KP_RECOVERY = 0.0075;
    public static double START_KP_STABLE = 0.0007;
    public static double START_ALPHA = 0.7;
    public static double START_RECOVERY_THRESHOLD = 90.0;

    public static boolean REVERSED = BiobuzzRobotConstants.nectarFlywheelReversed;

    public static double TARGET_STEP = 25.0;

    public static double AT_SPEED_TOLERANCE = 30.0;

    public static double DIP_DETECT_TICKS = 80.0;

    public static double TELEMETRY_INTERVAL_MS = 100.0;

    BulkRead bulkRead;

    private final ElapsedTime telemetryTimer = new ElapsedTime();

    private NGMotor flywheel;
    private Intake intake;
    private DcMotorEx transfer;
    private double lastTransferPower = Double.NaN;

    private double targetVelocity;
    private boolean flywheelRunning = false;

    private boolean gainsDirty = true;

    private double kI, kD, kF;
    private double kpRecovery, kpStable, alpha, recoveryThreshold, kLoad, kRamp;

    private int selectedParam = 0;

    private static final String[] PARAM_NAMES = {
            "kF", "kP_Stable", "kP_Recovery", "kI", "kD",
            "RECOVERY_THRESHOLD", "ALPHA", "kLoad", "kRamp"
    };

    private static final double[] PARAM_STEPS = {
            0.0000025,
            0.000025,
            0.0005,
            0.00005,
            0.00005,
            10.0,
            0.05,
            0.05,
            0.1
    };

    private boolean g2PrevY, g2PrevX, g2PrevA, g2PrevUp, g2PrevDown, g2PrevLeft, g2PrevRight, g2PrevBack;

    private boolean wasAtSpeed = false;
    private boolean inDip = false;
    private double dipMinVelocity = 0.0;
    private final ElapsedTime dipTimer = new ElapsedTime();
    private double lastDipDepth = Double.NaN;
    private double lastRecoveryMs = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bulkRead = new BulkRead(hardwareMap);
        bulkRead.setManual();

        flywheel = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.nectarFlywheel);
        flywheel.init();
        flywheel.setDirection(REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        intake = new Intake(hardwareMap, telemetry);
        try {
            transfer = hardwareMap.get(DcMotorEx.class, BiobuzzRobotConstants.transferRollers);
            transfer.setDirection(BiobuzzRobotConstants.transferReversed
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            transfer = null;
        }

        targetVelocity = START_TARGET_VELOCITY;
        kI = START_KI;
        kD = START_KD;
        kF = START_KF;
        kpRecovery = START_KP_RECOVERY;
        kpStable = START_KP_STABLE;
        alpha = START_ALPHA;
        recoveryThreshold = START_RECOVERY_THRESHOLD;
        kLoad = NGMotor.kLoad;
        kRamp = NGMotor.kRamp;
        telemetry.addLine("NECTAR FLYWHEEL + INTAKE TESTING (no drive)");
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

            if (gamepad2.right_bumper) intake.start();
            if (gamepad2.left_bumper) intake.stop();
            if (gamepad2.b) intake.reverse();

            if (gamepad2.y && !g2PrevY) {
                flywheelRunning = !flywheelRunning;
                if (!flywheelRunning) {
                    flywheel.targetVelocity = 0.0;
                    flywheel.setAbsPower(0.0);
                    NGMotorFlywheelTuner.resetVelocityFilter(flywheel);
                    resetDipTracking();
                } else {
                    gainsDirty = true;
                }
            }
            g2PrevY = gamepad2.y;

            double targetStep = TARGET_STEP * stepMultiplier();
            if (gamepad2.dpad_up && !g2PrevUp) targetVelocity += targetStep;
            if (gamepad2.dpad_down && !g2PrevDown) targetVelocity -= targetStep;
            if (targetVelocity < 0) targetVelocity = 0;
            g2PrevUp = gamepad2.dpad_up;
            g2PrevDown = gamepad2.dpad_down;

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

            NGMotorFlywheelTuner.setRecoveryKp(flywheel, kpRecovery);
            NGMotorFlywheelTuner.setStableKp(flywheel, kpStable);
            NGMotorFlywheelTuner.setAlpha(flywheel, alpha);
            NGMotorFlywheelTuner.setRecoveryThreshold(flywheel, recoveryThreshold);
            NGMotor.kLoad = kLoad;
            NGMotor.kRamp = kRamp;

            boolean simulatedFeeder = gamepad2.start;
            double transferPower = simulatedFeeder && flywheelRunning ? 1.0 : 0.0;
            if (transfer != null && transferPower != lastTransferPower) {
                transfer.setPower(transferPower);
                lastTransferPower = transferPower;
            }

            if (flywheelRunning) {
                flywheel.targetVelocity = targetVelocity;
                flywheel.updateFlywheels(simulatedFeeder);
            } else {
                flywheel.setAbsPower(0.0);
            }

            double rawVelocity = flywheel.getVelocity();
            trackDip(rawVelocity);

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
                telemetry.addData("transfer motor", transfer == null ? "MISSING ("
                        + BiobuzzRobotConstants.transferRollers + ")" : String.format("%.1f", transferPower));

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

            if (gainsDirty) {
                flywheel.setCustomVelocityPID(targetVelocity, kpStable, kI, kD, kF);
                gainsDirty = false;
            }
        }

        flywheel.setAbsPower(0.0);
        intake.stop();
        if (transfer != null) transfer.setPower(0.0);
    }

    private double stepMultiplier() {
        if (gamepad2.right_trigger > 0.5) return 10.0;
        if (gamepad2.left_trigger > 0.5) return 0.1;
        return 1.0;
    }

    private void adjustSelected(int direction) {
        double step = PARAM_STEPS[selectedParam] * stepMultiplier() * direction;
        switch (selectedParam) {
            case 0: kF = Math.max(0.0, kF + step); gainsDirty = true; break;
            case 1: kpStable = Math.max(0.0, kpStable + step); break;
            case 2: kpRecovery = Math.max(0.0, kpRecovery + step); break;
            case 3: kI = Math.max(0.0, kI + step); gainsDirty = true; break;
            case 4: kD = Math.max(0.0, kD + step); gainsDirty = true; break;
            case 5: recoveryThreshold = Math.max(1.0, recoveryThreshold + step); break;
            case 6: alpha = Math.min(1.0, Math.max(0.01, alpha + step)); break;
            case 7: kLoad = kLoad + step; break;
            case 8: kRamp = kRamp + step; break;
            default: break;
        }
    }

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

    private void trackDip(double velocity) {
        if (!flywheelRunning || targetVelocity <= 0) {
            resetDipTracking();
            return;
        }

        boolean atSpeed = Math.abs(targetVelocity - velocity) <= AT_SPEED_TOLERANCE;

        if (!inDip) {
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
