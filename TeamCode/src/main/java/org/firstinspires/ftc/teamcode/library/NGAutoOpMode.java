package org.firstinspires.ftc.teamcode.library;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.Camera;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Distance;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake2_0;
import org.firstinspires.ftc.teamcode.DECODE_subsystems.VihasCameraArm;

public abstract class NGAutoOpMode extends LinearOpMode {
    public static ElapsedTime timer;
    public static Intake intake;
    public static Intake2_0 intake2_0;
    public static MecanumDrive drive;
    //public static TrafficLight trafficLight;
    public static BulkRead bulkRead;
    //public static Rigging rigging;
    public static VihasCameraArm vihasCameraArm;
    public static Gamepad currentGamepad1, previousGamepad1;
    private int arm_height_for_specimen = 560;
//    public static Rigging rigging;
    public void initAuto(Pose2d beginPose){
        bulkRead = new BulkRead(hardwareMap);
        timer = new ElapsedTime();
        //trafficLight = new TrafficLight("front", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led, timer);
        drive = new MecanumDrive(hardwareMap, beginPose);
        //drive.mountTrafficLight(trafficLight);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake2_0 = new Intake2_0(hardwareMap, telemetry, timer);
        Intake2_0.initHood();
        Intake2_0.initFlywheels();
        //RobotConstants.auto_transfer = true;
        //vihasCameraArm = new VihasCameraArm(hardwareMap, telemetry);
        //intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
        //currentGamepad1 = new Gamepad();
        //previousGamepad1 = new Gamepad();
        //rigging = new Rigging(hardwareMap, telemetry, timer);
        //rigging.init();
//        rigging.reset();

        //intake.init();
        //vihasCameraArm.init();

        //intake.slides.setReachedRange(30);
//        intake.calculateOffset();
        //intake.moveClaw(RobotConstants.claw_closed);
    }
    public Action raiseArmForSpecimen(){
        return new SequentialAction(
                new InstantAction(() -> intake.arm.setExitWithTime(true)),
                new InstantAction(() -> intake.slides.setExitWithTime(true)),
                intake.armAction(ARM_LIMIT , 100),
                new ParallelAction(
                        intake.armAction(ARM_LIMIT ),
                        intake.slideAction(100)
                )
        );
    }
    public Action eternalAction(){
        return new SleepAction(30);

    }
    public class obtainSampleWithCameraAction implements Action{
        private boolean first = true;

        private boolean park = false;
        private Camera camera;
        private MecanumDrive drive;

        private TrajectoryActionBuilder backup, forward;
        FailoverAction sleep, armDown2;
        Action toSample;
        Action action;

        public obtainSampleWithCameraAction(Camera camera, MecanumDrive drive){
            this.camera = camera;
            this.drive = drive;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                Pose2d beginPose = drive.pose;
                if(!camera.isCameraOn()){
                    backup = drive.actionBuilder(beginPose).strafeTo(new Vector2d(beginPose.position.x - 5, beginPose.position.y));
                    forward = backup.endTrajectory().fresh().strafeTo(beginPose.position, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 30));
                    park = true;
                    action  = new SequentialAction(
                            backup.build(),
                            new InstantAction(() -> vihasCameraArm.level1()),
                            new SleepAction(0.4),
                            forward.build(),
                            new SleepAction(30)
                    );

                }else {
                    double[] data = camera.getYellow();
                    double diffy_angle = data[0];
                    int slide_position = (int) data[1];
                    telemetry.addData("Diffy Angle From Camera", diffy_angle);
                    telemetry.addData("Slide Position From Camera", slide_position);
                    telemetry.addData("Robot Displacement From Camera", data[2]);
                    telemetry.update();
                    sleep = new FailoverAction(new SleepAction(0.4), new NullAction());
                    armDown2 = new FailoverAction(intake.moveArmFast(180, -0.2), new InstantAction(() -> intake.arm.setManualPower(0)));

                    if (Math.abs(data[2]) > 0) {
                        Vector2d newTarget = new Vector2d(beginPose.position.x, beginPose.position.y - Math.copySign(Math.abs(data[2]) + 0, data[2]));
                        toSample = drive.actionBuilder(beginPose).strafeTo(newTarget).build();
                    } else {
                        toSample = new NullAction();
                    }
                    if(slide_position > 620){
                        slide_position += 40;
                    }
                    action = (new SequentialAction(
                            new ParallelAction(
                                    new SequentialAction(
                                            intake.armAction(475),
                                            new InstantAction(() -> {intake.turnAndRotateClaw(180, diffy_angle);})

                                    ),
                                    intake.slideAction(slide_position , slide_position/2),
                                    toSample
                            ),
                            new ParallelAction(
                            intake.slideAction(slide_position ),
                            new SleepAction(0.3)),
                            intake.armAction(350),
                            new ParallelAction(
                                    new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                                    new SequentialAction(sleep, new InstantAction(armDown2::failover))
                            ),
                            intake.grab(RobotConstants.claw_closed),
                            new SleepAction(0.1),
                            intake.armAction(500, 400),
                            new InstantAction(() -> intake.moveWrist(90))
                    ));

                }
            }
            first = false;
            TelemetryPacket packet = new TelemetryPacket();

            return action.run(packet);
        }
    }
    public Action obtainSampleWithCamera(Camera camera, MecanumDrive drive){

        return new obtainSampleWithCameraAction(camera, drive);

    }

    public class WaitUntilAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return (!gamepad1.a);
        }
    }
    public Action betterEternalAction(){

        return new WaitUntilAction();

    }
    public Action scoreSpecimen(Distance rear_distance){
        return new SequentialAction(
                //drive.moveUsingDistance(rear_distance, 4.5, 4, 4.8, false),
                intake.slideAction(0)
        );
    }
    public Action newScoreSpecimen(Action action, Distance rear_distance){
        return new SequentialAction(
                intake.slideAction(0),
                new ParallelAction(
                        openClawAfterDistance(7.5, rear_distance),
                        action

                )
        );
    }
    public Action collectSampleAndScore(Action sampleScore, double ending_claw_pos){
        return new SequentialAction(
                //drive.moveUsingDistance(intake.distance, RobotConstants.TARGET, RobotConstants.TOO_CLOSE, RobotConstants.TOO_FAR, RobotConstants.GIVE_UP),
                intake.grab(RobotConstants.claw_closed),
                new ParallelAction(
                        intake.raiseArm(),
                        sampleScore
                ),
                intake.score(),
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }
    public Action collectSampleAndScore(Action sampleScore, double ending_claw_pos, boolean use_distance_move){
        return new SequentialAction(
                intake.grab(RobotConstants.claw_closed),
                new ParallelAction(
                        intake.raiseArm(),
                        sampleScore
                ),
                new SleepAction(0.2),
                intake.score(),
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }


    public Action slideCollectSampleAndScore(Action sampleScore, double ending_claw_pos){
        return slideCollectSampleAndScore(sampleScore,  new ParallelAction(
                intake.slideAction(200, 400)), ending_claw_pos);
    }

    public Action slideCollectSampleAndScore(Action sampleScore, Action delay, double ending_claw_pos){
        return slideCollectSampleAndScore(sampleScore, delay, ending_claw_pos, false);
    }
    public Action slideCollectSampleAndScore(Action sampleScore, Action delay, double ending_claw_pos, boolean slow){
        Action score;
        if(slow){
            score = intake.scoreSlidePickupSlow();
        }else{
            score = intake.scoreSlidePickup();
        }
        return new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                delay,
                                new ParallelAction(
                                        intake.raiseArm()
                                )

                        ),
                        new SequentialAction(
                                sampleScore

                        )
                ),
                new InstantAction(() -> intake.closeClaw(-0.05)),
                score,
                new InstantAction(() -> intake.moveClaw(ending_claw_pos))
        );
    }


    public Action slideCollectSampleAndScore(Action sampleScore, double ending_claw_pos, boolean slow){
       return slideCollectSampleAndScore(sampleScore, new NullAction(), ending_claw_pos, slow);
    }

    public Action goToSampleWithSlides(Action sample){
        return goToSampleWithSlides(sample, 0);

    }

    public Action goToSampleWithSlides(Action sample, double claw_angle){
     return goToSampleWithSlides(sample, claw_angle, 750);
    }
    public Action goToSampleWithSlides(Action sample, double claw_angle, int collapse_value){
        Intake.moveArmAction armDown = intake.armAction(375, 1000);
        FailoverAction sleep = new FailoverAction(new SleepAction(0.4), new NullAction());
        FailoverAction armDown2 = new FailoverAction(intake.moveArmFast(250, -0.2), new InstantAction(() -> intake.arm.setManualPower(0)));
        return new SequentialAction(
                new InstantAction(() ->
                {
                    intake.arm.setManualPower(0);
                    intake.moveClaw(RobotConstants.claw_flat);
                    intake.turnAndRotateClaw(180, claw_angle);
                }),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.1),
                                intake.slideAction(collapse_value, collapse_value + 400),
                                armDown,
                                intake.slideAction(1000)
                        ),
                        sample
                ),
                new ParallelAction(
                        new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                        new SequentialAction(sleep, new InstantAction(armDown2::failover))
                ),
                intake.grab(RobotConstants.claw_closed),
                new SleepAction(0.1),
                new InstantAction(() -> intake.moveArm(500))
        );
    }
    public Action goToSampleWithSlides(Action sample, FailoverAction delay_action){
        Intake.moveArmAction armDown = intake.armAction(300, 700);
        FailoverAction sleep = new FailoverAction(new SleepAction(0.1), new NullAction());
        FailoverAction armDown2 = new FailoverAction(intake.moveArmFast(250, -0.3), new InstantAction(() -> intake.arm.setManualPower(0)));
        return new SequentialAction(
                new ParallelAction(
                        new SequentialAction(

                                armDown,
                                intake.slideAction(0),
                                new InstantAction(() ->
                                {
                                    intake.moveClaw(RobotConstants.claw_flat);
                                    intake.moveWrist(180);
                                }),
                                delay_action,
                                intake.slideAction(1000)

                        ),
                        sample

                ),
                new ParallelAction(
                        new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                        new SequentialAction(sleep, new InstantAction(armDown2::failover))
                ),
                intake.grab(RobotConstants.claw_closed),
                new SleepAction(0.1),
                new InstantAction(() -> intake.moveArm(500))
        );
    }
    public Action goToSample(FailoverAction sample, FailoverAction distance){
        return new SequentialAction(
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup)),
                new ParallelAction(
                        new SequentialAction(
                                intake.armAction(0, 1000),
                                new ParallelAction(
                                        new SequentialAction(
                                                distance,
                                                new InstantAction(sample::failover)
                                        ),
                                        sample
                                )
                        ),
                        intake.slideAction(0)

                )

        );
    }


    public Action openClawAfterDistance(double distance, Distance sensor){
        return new SequentialAction(
          sensor.waitAction(distance),
          new InstantAction(() -> intake.moveClaw(RobotConstants.claw_floor_pickup))
        );
    }
    public Action pickupAfterDistance(double distance, Distance sensor){
        return new SequentialAction(
                sensor.waitAction(distance),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_closed))
        );
    }
    public Action afterDistance(double distance, Distance sensor, Action action){
        return new SequentialAction(
                sensor.waitAction(distance),
                action
        );
    }
    public Action telemetryLine(String string){
        return new SequentialAction(
                new InstantAction(() -> telemetry.addLine(string)),
                new InstantAction(() ->
                telemetry.update())
        );
    }
    public class FailoverAction implements Action{
        private final Action mainAction;
        private final Action failoverAction;
        private boolean failedOver = false;
        private boolean on = true;

        public FailoverAction(Action mainAction, Action failoverAction) {
            this.mainAction = mainAction;
            this.failoverAction = failoverAction;
        }
        public FailoverAction(Action mainAction, Action failoverAction, boolean on) {
            this.mainAction = mainAction;
            this.on = on;
            this.failoverAction = failoverAction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!on){
                return true;
            }
            if (failedOver) {
                return failoverAction.run(telemetryPacket);
            }

            return mainAction.run(telemetryPacket);
        }
        public void enable(){
            on = true;
        }
        public void failover() {
            failedOver = true;
        }
    }
    public Action scoreSpecimen(Action toSpecimen, Action after){
        FailoverAction extendSlides = new FailoverAction(intake.slideAction(300), new NullAction());
        FailoverAction timer = new FailoverAction(new SleepAction(0.4), new NullAction());
        FailoverAction rotateArm = new FailoverAction(intake.armAction(1600), new NullAction());
        FailoverAction timer2 = new FailoverAction(new SleepAction(0.3), new NullAction());


        return new SequentialAction(
                new ParallelAction(
                    new SequentialAction(timer, new InstantAction(extendSlides::failover)),
                    new SequentialAction(extendSlides, new InstantAction(timer::failover))
                ),

                new InstantAction(() -> {
                    intake.enableLevel(false);
                    intake.setFourBar(false);
                    intake.closeClaw(-0.04);
                }),
                new SleepAction(0.25),
                new ParallelAction(
                        toSpecimen,
                        intake.armAction(1550),
                        intake.slideAction(310),
                        new SequentialAction(
                            new InstantAction(() -> intake.moveWrist(0)),
                                new SleepAction(0.5),
                                new InstantAction(() -> intake.closeClaw(-0.03))
                        )
                ),
                new ParallelAction(
                        new SequentialAction(timer2, new InstantAction(rotateArm::failover)),
                        new SequentialAction(rotateArm, new InstantAction(timer2::failover))
                ),
                intake.slideAction(750),
                new InstantAction(() -> intake.openClaw()),
                new ParallelAction(
                        after,
                        new InstantAction(() -> {
                            intake.turnAndRotateClaw(110,0);
                            intake.moveClaw(RobotConstants.claw_flat);
                        }),
                        intake.slideAction(100),
                        intake.armAction(arm_height_for_specimen)


                )
//                ,new InstantAction(() -> {
//                    intake.setTargetHeight(intake.getSpecimenHeight());
//                    intake.enableLevel(true);
//
//                }),
//                new SleepAction(0.75)
        );
    }
    public Action scoreSpecimenSampleSide(Action toSpecimen){
        FailoverAction extendSlides = new FailoverAction(intake.slideAction(300), new NullAction());
        FailoverAction timer = new FailoverAction(new SleepAction(0.4), new NullAction());
        FailoverAction rotateArm = new FailoverAction(intake.armAction(1600), new NullAction());
        FailoverAction timer2 = new FailoverAction(new SleepAction(0.3), new NullAction());


        return new SequentialAction(
                new ParallelAction(
                        toSpecimen,
                        intake.armAction(1600),
                        intake.slideAction(325),

                                new InstantAction(() -> intake.moveWrist(0))


                ),
//                new ParallelAction(
//                        new SequentialAction(timer2, new InstantAction(rotateArm::failover)),
//                        new SequentialAction(rotateArm, new InstantAction(timer2::failover))
//                ),
                intake.slideAction(650),
                new InstantAction(() -> intake.openClaw()),
                intake.slideAction(500)
//                ,new InstantAction(() -> {
//                    intake.setTargetHeight(intake.getSpecimenHeight());
//                    intake.enableLevel(true);
//
//                }),
//                new SleepAction(0.75)
        );
    }

    public Action transferSample(Action toSample, Action after, double claw_angle, int slide_length){
        FailoverAction sleep = new FailoverAction(new SleepAction(0.2), new NullAction());
        FailoverAction armDown2 = new FailoverAction(intake.moveArmFast(250, -0.3), new InstantAction(() -> intake.arm.setManualPower(0)));

        return new SequentialAction(

                new ParallelAction(
                        toSample,
                        new SequentialAction(
                                intake.armAction(300, 600),
                                new ParallelAction(
                                    intake.slideAction(slide_length),
                                    new InstantAction(() -> {
                                        intake.turnAndRotateClaw(180, claw_angle);
                                        intake.moveClaw(RobotConstants.claw_flat);
                                    })
                                )
                        )
                ),
                new ParallelAction(
                        new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                        new SequentialAction(sleep, new InstantAction(armDown2::failover))
                ),
                intake.grab(RobotConstants.claw_closed),
                new SleepAction(0.1),
                new InstantAction(() -> intake.moveArm(300)),
                new ParallelAction(
                        after,
                        intake.slideAction(slide_length)

                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat))



        );
    }
    public Action transferSample(Action toSample, Action after, double claw_angle, int slide_length, boolean last){
        FailoverAction sleep = new FailoverAction(new SleepAction(0.2), new NullAction());
        FailoverAction armDown2 = new FailoverAction(intake.moveArmFast(250, -0.3), new InstantAction(() -> intake.arm.setManualPower(0)));

        return new SequentialAction(

                new ParallelAction(
                        toSample,
                        new SequentialAction(

                                intake.armAction(300, 600),
                                new ParallelAction(
                                        intake.slideAction(slide_length),
                                        new InstantAction(() -> {
                                            intake.turnAndRotateClaw(180, claw_angle);
                                            intake.moveClaw(RobotConstants.claw_flat);
                                        })
                                )
                        )
                ),
                new ParallelAction(
                        new SequentialAction(armDown2, new InstantAction(sleep::failover)),
                        new SequentialAction(sleep, new InstantAction(armDown2::failover))
                ),
                intake.grab(RobotConstants.claw_closed),
                new SleepAction(0.1),
                new ParallelAction(
                        intake.armAction(arm_height_for_specimen),
                        after,
                        new InstantAction(() -> intake.turnAndRotateClaw(110,0)),
                        intake.slideAction(100)

                ),
                new InstantAction(() -> intake.moveClaw(RobotConstants.claw_flat))

//                ,new InstantAction(() -> {
//                    intake.setTargetHeight(intake.getSpecimenHeight());
//                    intake.enableLevel(true);
//                    intake.setTargetAngle(-10);
//                    intake.setFourBar(true);
//                })



        );
    }
}
