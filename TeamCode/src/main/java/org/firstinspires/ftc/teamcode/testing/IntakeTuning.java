package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;


public class IntakeTuning extends TestingOpMode {

    Intake intake;
    public static double TARGET_HEIGHT = 0;
    public static double CLAW_HEIGHT = 8;
    public static double STARTING_HEIGHT = 9;

    public static double SLIDE_WIDTH = 3.25;

    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        intake.calculateOffset();
        intake.setWristPWM(false);
        intake.slides.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.arm.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
//            intake.telemetry();
            telemetry.addData("Offset", intake.getOffset());
            telemetry.addData("Arm Output Position", intake.calculateArmPosition(intake.slides.getCurrentPosition()));
            telemetry.addData("Arm Position", intake.arm.getCurrentPosition());
            telemetry.addData("Arm Angle", intake.getArmAngle());
            telemetry.addData("Expected Height Off The Ground", TARGET_HEIGHT);
            double height = CLAW_HEIGHT + TARGET_HEIGHT - STARTING_HEIGHT;
            double sin_arm = Math.sin(Math.toRadians(intake.getArmAngle()));
            double first_term = (height/sin_arm) * (height/sin_arm);
            double needed_slide_length = Math.sqrt(first_term - SLIDE_WIDTH*SLIDE_WIDTH);
            telemetry.addData("Slide Position", intake.slides.getCurrentPosition());
            telemetry.addData("Needed Raw Slide Length", needed_slide_length);
            telemetry.addData("Needed Slide Hypot", Math.sqrt(needed_slide_length*needed_slide_length + SLIDE_WIDTH*SLIDE_WIDTH));
            telemetry.addData("Calculated Slide Hypot", intake.calculateSlideLength(intake.slides.getCurrentPosition()));
            telemetry.update();
        }

    }
}
