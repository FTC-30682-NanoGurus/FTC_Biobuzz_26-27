package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.NGMotor; // Adjust package as needed

@Config
@TeleOp(name = "Arm & Slide PID Tester", group = "Testing")
public class ArmTesting extends LinearOpMode {

    // Pivot Motor
    public static double PIVOT_P = 0.005;
    public static double PIVOT_I = 0.0;
    public static double PIVOT_D = 0.0001;
    public static double PIVOT_F = 0.0;

    // Slide Motor
    public static double SLIDE_P = 0.005;
    public static double SLIDE_I = 0.0;
    public static double SLIDE_D = 0.0001;
    public static double SLIDE_F = 0.0;

    // Target Positions
    public static int POS_HOME_PIVOT = 0;
    public static int POS_HOME_SLIDE = 0;

    public static int POS_INTAKE_PIVOT = 300;
    public static int POS_INTAKE_SLIDE = 800;

    public static int POS_SCORE_LOW_PIVOT = 1200;
    public static int POS_SCORE_LOW_SLIDE = 1000;

    public static int POS_SCORE_HIGH_PIVOT = 1800;
    public static int POS_SCORE_HIGH_SLIDE = 2500;

    // Hardware Objects
    NGMotor armPivot;
    NGMotor armSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Motors (Ensure these names match your Control Hub configuration)
        armPivot = new NGMotor(hardwareMap, telemetry, "arm_pivot");
        armSlide = new NGMotor(hardwareMap, telemetry, "arm_slide");

        // 2. Call the built-in init() to reset encoders and set run modes
        armPivot.init();
        armSlide.init();

        // 3. Set Safety Hardstops to prevent destroying your physical limits
        armPivot.setMin(-50);
        armPivot.setMax(3000);

        armSlide.setMin(-50);
        armSlide.setMax(4000);

        telemetry.addLine("Arm & Slide initialized.");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // 4. Live-update the PIDF coefficients from FTC Dashboard
            armPivot.setPIDF(PIVOT_P, PIVOT_I, PIVOT_D, PIVOT_F);
            armSlide.setPIDF(SLIDE_P, SLIDE_I, SLIDE_D, SLIDE_F);

            // 5. Button Logic to trigger asynchronous movement
            if (gamepad1.a) {
                // Home Position
                armPivot.move_async(POS_HOME_PIVOT);
                armSlide.move_async(POS_HOME_SLIDE);
            }
            else if (gamepad1.b) {
                // Intake Position
                armPivot.move_async(POS_INTAKE_PIVOT);
                armSlide.move_async(POS_INTAKE_SLIDE);
            }
            else if (gamepad1.x) {
                // Low Scoring Position
                armPivot.move_async(POS_SCORE_LOW_PIVOT);
                armSlide.move_async(POS_SCORE_LOW_SLIDE);
            }
            else if (gamepad1.y) {
                // High Scoring Position
                armPivot.move_async(POS_SCORE_HIGH_PIVOT);
                armSlide.move_async(POS_SCORE_HIGH_SLIDE);
            }

            // Emergency Encoder Reset (If belts slip and you need to zero out manually)
            if (gamepad1.right_bumper) {
                armPivot.resetEncoder();
                armSlide.resetEncoder();
                armPivot.move_async(0);
                armSlide.move_async(0);
            }

            // 6. CRITICAL: Call update() every loop to calculate math and apply power
            armPivot.update();
            armSlide.update();

            // 7. Telemetry output
            telemetry.addLine("--- Arm Pivot ---");
            armPivot.telemetry();

            telemetry.addLine("--- Arm Slide ---");
            armSlide.telemetry();

            telemetry.update();
        }
    }
}
