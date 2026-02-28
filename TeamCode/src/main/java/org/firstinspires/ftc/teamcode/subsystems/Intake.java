package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.Potentiometer;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Intake extends Subsystem {
    public static PIDFCoefficients armsPID = new PIDFCoefficients(0.003,0.00006,0.00002,0.0008);
    public static PIDFCoefficients armsLevelPID = new PIDFCoefficients(0.008,0,0.0000005,0.0008);

    public NGMotor arm;
    private double claw_offset = 0;

    public NGMotor slides;
    public static PIDFCoefficients slidesPID = new PIDFCoefficients(0.0015, 0.00007, 0.00008, 0.0005);
    public static PIDFCoefficients gunToPointSlidesPID = new PIDFCoefficients(0.004, 0.00007, 0.00003, 0.0005);
    DigitalChannel magnet_sensor;
    DigitalChannel touch_sensor;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;

    public TrafficLight trafficLight;

    public Distance distance;

    public Potentiometer potentiometer;


    private boolean claw_open = false;
    private boolean wrist_open =false;

    public boolean disable_up_hardstop = false;
    private boolean slide_current_stop = false;
    private double slide_current_stop_delay = 0;
    private int level_offset=  0;

    private boolean inside_pick_up = false;
    public boolean power_four_bar_enabled = false;
    
    public boolean to_hardstop = false;

    private boolean use_fast_pid = false;
    private boolean distance_update = true;
    private boolean distance_scope = false;


    private boolean use_velo = false;
    private double use_velo_power = 0;
    public static int SLIDE_CURRENT_LIMIT = 8500;
    public static int ARM_CURRENT_LIMIT = 50000;


    private boolean previous_magnet_on = false;
    public static double VELO_THRESHOLD = 0;
    private boolean level_on = false;

    private boolean dpad_up = false;
    private boolean dpad_down = false;
    private double joystick_power = 0;

    public void setDpadUp(boolean on){
        dpad_up= on;
    }
    public void setDpadDown(boolean on){
        dpad_down = on;
    }
    public void setJoyStickPower(double power){
        joystick_power = power;
    }

    private boolean robot_go_kaboom = false;
    public void setRobotGoKaboom(boolean on){
        robot_go_kaboom = on;
    }
    private boolean maintain_level = false;
    public void setMaintainLevel(boolean on){
        maintain_level = on;
    }
    private boolean arm_stabilizer = false;
    private double target_angle = 0;//To the y-axis
    public double target_height = 0;//To the floor



    private final double[] pickup_position_1 = {100,250};//slide, motor
    private final double[] pickup_position_2 = {800.0,220};

    private final double[] slide_distance_position_1 = {200,4.5};
    private final double[] slide_distance_position_2 = {800,10.6};
    // Adjust current threshold based on battery voltage
    public DiffyClaw diffyClaw;
//    public MecanumDrive mecanumDrive;
    public static int distance_to_camera = 225;
    public static double feedforward_turning_point = 0;
    public static int arm_at_0_ticks = 180;

    public static double arm_stabilization_factor = 0;
    public static int arm_at_90_ticks = 1380;
    public static double wrist_90 = 0.27;
    public static double wrist_180 = 0.62;
    public static double slide_ticks_to_inches = 0.0171875;
    public static double slide_starting_length = 13;
    public static double slide_starting_height = 9.125;
    public static double slide_width = 3.125;
    public static double claw_height = 0;
    public static int offset = 0;
    public static int offset_distance_to_0 = 0;
    public static double DISTANCE_FILTER = 0.6;

    public static boolean beta_mode = false;//for beta

    public boolean specimen_level_on = false;
    private double new_target_height = 17.1851;

    private boolean lower_arm_speed = false;

    private boolean hold_at_hardstop = false;
    private int lower_arm_position = 0;

    public static int position_at_90 = 1445;
    public static int position_at_0 = 190;
    
    public static int hard_stop_ticks = 1550;

    public static double real_zero_degrees = 10;

    private boolean pot_on = false;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        arm = new NGMotor(hardwareMap, telemetry, RobotConstants.intakeMotor);
        arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        arm.setDirection(DcMotor.Direction.REVERSE);
        timer = new ElapsedTime();
        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);
        magnet_sensor = hardwareMap.get(DigitalChannel.class, RobotConstants.magnet_sensor);
        touch_sensor = hardwareMap.get(DigitalChannel.class, RobotConstants.touch_sensor);
        touch_sensor.setMode(DigitalChannel.Mode.INPUT);
        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);
        slides = new NGMotor(hardwareMap, telemetry, RobotConstants.slidesMotor);
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        diffyClaw = new DiffyClaw(hardwareMap, telemetry);
//        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)));

        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        diffyClaw.mountTrafficLight(trafficLight);
        arm.setMin(-500);
        arm.setReachedRange(50);
        potentiometer = new Potentiometer(hardwareMap, telemetry, "pot");
        potentiometer.set0Position(position_at_0);
        potentiometer.set90Position(position_at_90);
        potentiometer.setRealZeroDegrees(real_zero_degrees);
        arm.setAlternateEncoder(potentiometer);
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, TrafficLight trafficLight){
        this(hardwareMap, telemetry);
        this.timer = timer;
//        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)));

        diffyClaw.left.mountTimer(timer);
        diffyClaw.right.mountTimer(timer);
        diffyClaw.mountTrafficLight(trafficLight);
        this.trafficLight = trafficLight;

    }
    public void mountDistance(Distance distance){
        distance_update = false;
        this.distance = distance;
    }
    public boolean hardStopActivated(){
        return !touch_sensor.getState();
    }
    public void setClawOffset(double offset){
        claw_offset = offset;
    }
    public void setInsidePick(boolean on){
        inside_pick_up = on;
        if(claw_open){
            openClaw();
        }else{
            closeClaw();
        }
    }
    public void resetSpecimenHeight(){
        new_target_height = 17.1851;
    }
    public void lowerArmSpeed(double power, int position){
        lower_arm_speed = true;
        lower_arm_position = arm.targetPos + position;
        arm.setManualPower(power);
    }
    public boolean loweringArm(){
        return lower_arm_speed;
    }
    public void usePot(boolean on){
        pot_on = on;
    }
    public void setLowerArm(boolean on){
        lower_arm_speed = on;
    }
    public void toggleInsidePick(){
        setInsidePick(!inside_pick_up);
    }
    public void toggleClaw(){
        claw_open = !claw_open;
        if(claw_open){
            openClaw();
        }else{
            closeClaw();
        }
    }
    private boolean magnet_activated(){
        return !magnet_sensor.getState();
    }
    public boolean magnetOnRisingEdge(){
        return magnet_activated() && !previous_magnet_on;
    }
    public void useFastPID(boolean on){
        use_fast_pid = on;
    }
    public void calculateOffset(){
        arm.setAbsPower(-0.3);

        double time_offset = timer.time();
        while(!magnet_activated()){
            telemetry.addData("Position",arm.getCurrentPosition());
            telemetry.update();
            if(timer.time() - time_offset > 1.2){
                return;
            }
        }
        arm.resetEncoder();
//        offset = arm.getCurrentPosition();
        telemetry.addData("Offset", offset);

        arm.setAbsPower(0);

    }
    public int getOffset(){
        return offset;
    }
    public void setFourBar(boolean state){
        power_four_bar_enabled = state;
    }
    public void setTargetAngle(double target_angle){
        this.target_angle = target_angle;
    }
    public void moveArmToHardstop(){
        moveArm(hard_stop_ticks);
        to_hardstop = true;
    }
    public void moveArm(int targetPosition){
        if(arm.getCurrentPosition() > ARM_LIMIT - 400 && targetPosition < 200 && slides.getCurrentPosition() > 300){
            trafficLight.flashRed(1,1);
            return;
        }
        arm.move_async(targetPosition + offset);
    }
    public void moveArm(int targetPosition, boolean safety_is_cringe){
        arm.move_async(targetPosition + offset);
    }
    public void moveArmUntilZeroSpeed(double power){
        use_velo = true;
        use_velo_power = power;

    }

    public void useVelo(boolean on){
        use_velo = on;
    }
    public boolean getVeloOn(){
        return use_velo;
    }
    public void moveWrist(double wristAngle){
        diffyClaw.setWristAngle(wristAngle);
    }
    public void moveClaw(double clawPosition){
        diffyClaw.moveClaw(clawPosition + claw_offset);
    }
    public void turnClaw(double clawAngle){
        diffyClaw.setClawAngle(-clawAngle);
    }
    public void turnAndRotateClaw(double wristAngle, double clawAngle){
        moveWrist(wristAngle);
        turnClaw(clawAngle);
    }
    public void moveSlides(int targetPosition){
        slides.move_async(targetPosition);
    }
    public void setRotationPower(double power){
        if(!lower_arm_speed) {
            arm.setManualPower(power);
        }
    }
    public boolean levelOn(){
        return level_on;
    }
    public void setNewTargetHeight(){
        new_target_height = calculateTargetHeight(arm.getCurrentPosition(), slides.getCurrentPosition());
    }
    public double getSpecimenHeight(){
        return new_target_height;
    }
    public void setTargetHeight(double target_height){ this.target_height = target_height; }

    public void setHardstopTicks(int ticks){
        hard_stop_ticks = ticks;
    }
    public void setAbsRotationPower(double power){ arm.setAbsPower(power);}
    public void setSlidePower(double power){slides.setManualPower(power);}
    public void setAbsSlidePower(double power){
        slides.setAbsPower(power);
    }
    public void openClaw() {
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_open + claw_offset);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_open + claw_offset - 0.05);
        }
        claw_open=true;
    }

    public void setDistanceScoping(boolean on){
        if(!on){
            trafficLight.green(false);
        }
        distance_scope = on;
    }


    public Action yeetSample(){
        //TODO make sure that it "yeets" the sample
        return new SequentialAction(
                grab(0.95),
                armAction(ARM_LIMIT),
                grab(0.74),
                armAction(0)
        );
    }

    public Action grab(){
        return new SequentialAction(
                new InstantAction(this::closeClaw),
                new SleepAction(0.3)
        );
    }
    public Action grab(double position){
        return new SequentialAction(
                new InstantAction(() -> moveClaw(position)),
                new SleepAction(0.3)
        );
    }
    public Action raiseArm(){
        return raiseArm(true);
    }
    public Action raiseArmButNotSnagOnBasket(){
        return new SequentialAction(
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> arm.setExitWithTime(true)),
                new InstantAction(() -> arm.setMaxVelocity(3000)),
                armAction(ARM_LIMIT,ARM_LIMIT - 300),
                new InstantAction(() -> moveWrist(180)),
                new ParallelAction(
                        slideAction(1400),
                        armAction(ARM_LIMIT - 25)
                ),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
                new InstantAction(() -> moveWrist(30)),
                new SleepAction(0.2)


        );
    }
    public Action raiseArm(boolean arm_exit_with_time){
        return new SequentialAction(
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> arm.setExitWithTime(arm_exit_with_time)),
                new InstantAction(() -> {arm.setMaxVelocity(6000); arm.setMax(ARM_LIMIT + 200);}),

                armAction(ARM_LIMIT + 50,ARM_LIMIT - 1000),
                new InstantAction(() -> {
                    turnAndRotateClaw(145,0);
                    closeClaw(-0.035);
                }),
                new InstantAction(() -> arm.setExitWithTime(false)),

                new SequentialAction(
                        slideAction(1500, 1350),
                        new InstantAction(() -> {moveWrist(25);})
                ),
                slideAction(1500),
                new InstantAction(() -> {arm.setMaxVelocity(8000); }),
                new SleepAction(0.15)

        //arm.setManualPower(0.4);
//                ,new InstantAction(() -> {moveWrist(35);}),
//                new SleepAction(0.15)


        );
    }

    public class moveArmToHardstopAction implements Action{
        boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                first = false;
                to_hardstop = true;
            }
            return to_hardstop;
        }
    }
    public Action moveArmToHardstopAction(){
        return new moveArmToHardstopAction();
    }
    public Action score(){
        return new SequentialAction(
                new InstantAction(() -> moveClaw(RobotConstants.claw_floor_pickup)),
                new SleepAction(0.1),
                new InstantAction(() -> moveWrist(100)),
                new SleepAction(0.2),
                moveArmFast(ARM_LIMIT - 200, -0.2),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
//                armAction(ARM_LIMIT - 300),
                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> turnClaw(0)),
                slideAction(0)


        );
    }
    public Action scoreSlidePickup(){

        return new SequentialAction(
                new InstantAction(() -> moveClaw(RobotConstants.claw_floor_pickup)),
                new SleepAction(0.3),
                new InstantAction(() -> moveWrist(90)),
//                new InstantAction(() -> arm.setManualPower(0)),

//                new SleepAction(0.2),
                new InstantAction(() -> arm.setUseMotionProfile(false)),

//                armAction(ARM_LIMIT - 200, ARM_LIMIT -150),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
//                armAction(ARM_LIMIT - 300),
                new InstantAction(() -> arm.setUseMotionProfile(true)),

                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> turnClaw(0))
        );
    }
    public Action scoreSlidePickupSlow(){

        return new SequentialAction(
                new InstantAction(() -> arm.setManualPower(0.3)),
                new SleepAction(0.2),
                new InstantAction(() -> arm.setManualPower(0)),
                new InstantAction(() -> moveClaw(RobotConstants.claw_floor_pickup)),
                new SleepAction(0.3),
                new InstantAction(() -> moveWrist(90)),

//                new SleepAction(0.2),
                new InstantAction(() -> arm.setUseMotionProfile(false)),

//                armAction(ARM_LIMIT - 200, ARM_LIMIT - 150),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
//                armAction(ARM_LIMIT - 300),
                new InstantAction(() -> arm.setUseMotionProfile(true)),

                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> turnClaw(0))


        );
    }
    public Action scoreSlidePickup(boolean last){

        return new SequentialAction(
                new InstantAction(() -> moveClaw(RobotConstants.claw_floor_pickup)),
                new SleepAction(0.1),
                new InstantAction(() -> moveWrist(90)),
//                new SleepAction(0.2),
                new InstantAction(() -> arm.setUseMotionProfile(false)),

//                armAction(ARM_LIMIT - 200, ARM_LIMIT -150),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
//                armAction(ARM_LIMIT - 300),
                new InstantAction(() -> arm.setUseMotionProfile(true)),

                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> turnClaw(0)),
                slideAction(200, 600)

        );
    }
    public Action score(boolean extra_claw_clearance){
        return new SequentialAction(
                new InstantAction(this::openClaw),
                new SleepAction(0.1),
                new InstantAction(() -> turnClaw(90)),
                new InstantAction(() -> moveWrist(extra_claw_clearance ? 100 : 90)),
                new SleepAction(extra_claw_clearance ? 0.4 : 0),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                slideAction(0)

        );
    }
    public Action scoreAndFold(){

        return new SequentialAction(
                new InstantAction(() -> moveClaw(RobotConstants.claw_open)),
                new SleepAction(0.1),
                new InstantAction(() -> turnClaw(0)),
                new InstantAction(() -> moveWrist(100)),
                new SleepAction(0.4),
                new InstantAction(() -> arm.setMaxVelocity(5000)),
                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                slideAction(0),
                new InstantAction(() -> moveWrist(0))


        );
    }
    public void setClawPWM(boolean on){
        if(on){
            diffyClaw.claw.enableServo();
            return;
        }
        diffyClaw.claw.disableServo();
    }
    public void setWristPWM(boolean on){
        if(on){
            diffyClaw.left.enableServo();
            diffyClaw.right.enableServo();
            return;
        }
        diffyClaw.left.disableServo();
        diffyClaw.right.disableServo();    }
    public boolean WristPWMOn(){
        return diffyClaw.left.isPWMEnabled();
    }
    public boolean ClawPWMOn(){
        return diffyClaw.claw.isPWMEnabled();
    }
    public void closeClaw() {
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_closed + claw_offset);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_closed + claw_offset);
        }
        claw_open=false;
    }
    public void closeClaw(double offset) {
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_closed + claw_offset + offset);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_closed + claw_offset + offset);
        }
        claw_open=false;
    }
    public void plowClaw(){
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_open + claw_offset);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_open + claw_offset);
        }
        claw_open=true;
    }
    public boolean isInsidePick(){
        return inside_pick_up;
    }
    public void enableLevel(boolean on){
        level_on = on;
        setDamp(on);
    }
    public void setDamp(boolean on){
        arm.setPowerDamp(level_on);
    }


    public void foldWrist(){
        diffyClaw.setWristAngle(RobotConstants.wrist_folded); wrist_open=false;
    }
    public int getHardstopTicks(){
        return hard_stop_ticks;
    }

    public double getArmAngle(){
        if(pot_on){
            return potentiometer.getAngle();
        }

        return (double) (arm.getCurrentPosition()  - arm_at_0_ticks -  offset ) / (arm_at_90_ticks - arm_at_0_ticks) * 90.0;
    }
    public double calculateWristPosition(){
        double arm_angle = getArmAngle();
        double target_position = 90 + arm_angle + target_angle;

        return target_position;
    }
    public double calculateSlideLength(int slide_position){
        telemetry.addData("Calculated Slide Length", slide_starting_length + slide_position * slide_ticks_to_inches);
        telemetry.addData("Slide Length Slide Positions", slide_position);


        return Math.sqrt(Math.pow(slide_starting_length + slide_position * slide_ticks_to_inches,2) + Math.pow(slide_width,2));
    }
    public int calculateArmPosition(int slide_position){

        double arm_angle = Math.asin((target_height + claw_height - slide_starting_height) / (calculateSlideLength(slide_position)));
        int arm_angle_in_ticks = (int) (arm_angle / Math.toRadians(90.0) * (arm_at_90_ticks - arm_at_0_ticks)) + arm_at_0_ticks + offset;

        if(beta_mode){
            arm_angle_in_ticks = (int) ((pickup_position_1[1] - pickup_position_2[1])/(pickup_position_1[0] - pickup_position_2[0]) * (slide_position - pickup_position_1[0]) + pickup_position_1[1]) + (inside_pick_up ? 70 : 15);
        }

        return arm_angle_in_ticks;
    }
    public double calculateTargetHeight(int arm_position, int slide_position) {

        // Constants
        double slide_length = calculateSlideLength(slide_position);
        // Calculate the arm angle from ticks
        double arm_angle_in_radians = ((arm_position - arm_at_0_ticks - offset) / (double) (arm_at_90_ticks - arm_at_0_ticks)) * Math.toRadians(90.0);
        // Calculate the target height based on the arm angle
        double target_height = (Math.sin(arm_angle_in_radians) * slide_length) + slide_starting_height - claw_height;

        return target_height;
    }
    public void incrementLevelOffset(int a){
        if(calculateArmPosition(slides.getCurrentPosition()) + a + level_offset >= arm.maxHardstop){
            return;
        }
        level_offset += a;
//        if(level_offset < 0){
//            if(a < 0) {
//                level_offset -= a;
//            }
//        }
    }
    public boolean isSlideStoppedWithCurrent(){
        return slide_current_stop;
    }
    public class currentWaitAction implements Action{
        private boolean canceled = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (canceled){
                return false;
            }
            return !isSlideStoppedWithCurrent();
        }
        public void cancel(){
            canceled = true;
        }
    }
    public Action currentWaitAction(){
        return new currentWaitAction();
    }

    public void absIncrementLevelOffset(int a){
        level_offset += a;
    }
    public int getLevelOffset(){
        return level_offset;
    }
//    public void mountMecanumDrive(MecanumDrive mec){
//        this.mecanumDrive = mec;
//    }
    public void holdAtHardstop(){
        hold_at_hardstop = true;
    }
    public void stopHoldingAtHardstop(){
        hold_at_hardstop = false;
    }
    public void update(boolean auto){
        potentiometer.set0Position(position_at_0);
        potentiometer.set90Position(position_at_90);
        potentiometer.setRealZeroDegrees(real_zero_degrees);

        diffyClaw.update();
        trafficLight.update();
        if(!auto) {
            if (to_hardstop) {
                if (dpad_down || dpad_up) {
                    to_hardstop = false;
                }
                to_hardstop = !hardStopActivated();
                if (!to_hardstop) {
                    moveArm(arm.getCurrentPosition());
                    arm.setManualPower(0);
                    hard_stop_ticks = arm.getCurrentPosition();
                } else if (!arm.isBusy() && to_hardstop) {
                    arm.setManualPower(0.35);
                }
            } else if (hold_at_hardstop) {
                if (dpad_down || dpad_up) {
                    hold_at_hardstop = false;
                }
                arm.setManualPower(0.2);
            } else if (dpad_down) {
                if (maintain_level) {
                    incrementLevelOffset(-10);
                    setRotationPower(-0.25);
                } else {
                    setRotationPower(-0.4);
                }
            } else if (dpad_up) {
                if (maintain_level) {
                    incrementLevelOffset(10);
                    setRotationPower(0.25);
                } else {
                    setRotationPower(0.4);
                }
            } else if (joystick_power != 0) {
                if (!robot_go_kaboom) {
                    setRotationPower(joystick_power);
                } else {
                    arm.setAbsPower(joystick_power);
                }
            } else if (!robot_go_kaboom) {
                setRotationPower(0);
            } else {
                arm.setAbsPower(0);
            }
        }


        previous_magnet_on = magnet_activated();
        if (!disable_up_hardstop) {
            arm.setExternalUpHardstop(hardStopActivated());
        }
        if (distance_scope) {
            trafficLight.green(distance.getFilteredDist() < 10);
        }
        if (slides.getCurrent() > SLIDE_CURRENT_LIMIT) {
            slide_current_stop = true;
            slide_current_stop_delay = timer.seconds();
            moveSlides(slides.getCurrentPosition());
            trafficLight.flashRed(0.5, 2);
        }
//        if(arm.getCurrent() > ARM_CURRENT_LIMIT){
//            moveArm(arm.getCurrentPosition());
//            trafficLight.flashRed(0.5, 2);
//        }
        if (lower_arm_speed) {
            if (arm.getCurrentPosition() < lower_arm_position) {
                arm.setManualPower(0);
                lower_arm_speed = false;
                return;
            }
        }
        if (timer.seconds() - slide_current_stop_delay > 3) {
            slide_current_stop = false;
        }
        if (hardStopActivated()) {
            hard_stop_ticks = arm.getCurrentPosition();
        }
        //disable this to remove automatic alignment shenanigans
        /**if(to_hardstop){
         *  to_hardstop = false;
         * } */


        if(level_on) {

            moveArm(calculateArmPosition(slides.getCurrentPosition()) + level_offset);
//           slidesStuck();
        }


        distance.setFilter(DISTANCE_FILTER);
        //psuedo gun to point
        if (Math.abs(slides.getCurrentPosition() -  slides.targetPos) < 200 && slides.targetPos < 400){
            slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        }else{
            slides.setPIDF(gunToPointSlidesPID.p , gunToPointSlidesPID.i, gunToPointSlidesPID.d, gunToPointSlidesPID.f);
        }

        if(use_velo) {
            use_velo = arm.getVelocity() >= VELO_THRESHOLD;
            arm.setManualPower(use_velo_power);
            if (!use_velo) {
                arm.setManualPower(0);
            }
        }
        if(!level_on){
            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, Math.cos(Math.toRadians(getArmAngle())) * calculateSlideLength(slides.getCurrentPosition()) * armsPID.f);

        }else{
            arm.setPIDF(armsLevelPID.p, armsLevelPID.i, armsLevelPID.d, Math.cos(Math.toRadians(getArmAngle())) * calculateSlideLength(slides.getCurrentPosition()) * armsLevelPID.f);
        }

        if(power_four_bar_enabled){
            moveWrist(calculateWristPosition());
        }
        if(distance_update) {
            distance.update();
        }
//        if(arm_stabilizer){
//            arm.addPower(arm_stabilization_factor/100000000.0 * calculateSlideLength(slides.getCurrentPosition()) * accel * arm.getCurrentPosition());
//        }else{
//            arm.addPower(0);
//        }
//        telemetry.addData("armTargetPositions", armTargetPositions.toString());
        arm.update();
        slides.update();
    }
    @Override
    public void update() {
        update(false);
//        past_velo = poseVelocity2d;
//        poseVelocity2d = mecanumDrive.updatePoseEstimate();
//        double accel = 0;
//        if (past_velo != null){
//            double dt = timer.milliseconds() - accel_time;
//            accel = (poseVelocity2d.linearVel.sqrNorm() - poseVelocity2d.linearVel.sqrNorm())/dt;
//            telemetry.addData("Drive Acceleration", accel);
//        }
//        accel_time = timer.milliseconds();


    }
    public void clearStoppedWithCurrent(){
        slide_current_stop = false;
    }
    public int calculateSlidePositionForFloorPickup(double distance){
        double slope = (slide_distance_position_2[0] - slide_distance_position_1[0])/(slide_distance_position_2[1] - slide_distance_position_1[1]);

        return (int) (slide_distance_position_1[0] + slope * (distance - slide_distance_position_1[1]));
    }
    @Override
    public void telemetry() {
        telemetry.addData("Arm Angle", getArmAngle());
        telemetry.addData("Pot: Arm Angle", potentiometer.getAngle());
        telemetry.addData("Pot: Arm Position", potentiometer.getCurrentPosition());
        telemetry.addData("Pot: Voltage", potentiometer.getVoltage());

        telemetry.addData("Slide Stopped With Current", slide_current_stop);
        telemetry.addData("Touch Sensor Activated", hardStopActivated());

        diffyClaw.telemetry();
        slides.telemetry();
        arm.telemetry();
    }
    public void init_without_encoder_reset(){

        arm.setMaxAcceleration(5000);
        arm.setMaxDeceleration(1500);
        arm.setMaxVelocity(8000);
        arm.setUseMotionProfile(true);

        slides.setUseMotionProfile(true);
        slides.setMax(1500);
        arm.setMax(ARM_LIMIT );
        arm.setMin(-1000);
        slides.setMaxAcceleration(15000);
        slides.setMaxVelocity(12000);
        slides.setMaxDeceleration(5000);
    }

    @Override
    public void init() {
        init_without_encoder_reset();
        arm.init();
        diffyClaw.init();
        slides.init();

        moveWrist(0);
        closeClaw();
    }
    public class moveSlidesAction implements Action {
        private boolean first = true;
        private int target_pos = 0;
        private int endpos = 0;
        private boolean partial_motion = false;

        private boolean direction_up = false;
        public moveSlidesAction(int position){
            target_pos = position;
        }
        public moveSlidesAction(int position, int endpos){
            this.endpos = endpos;
            partial_motion = true;
            target_pos = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                moveSlides(target_pos);
                first = false;
                direction_up = slides.getCurrentPosition() < endpos;
            }

            if(partial_motion){
                if(!slides.isBusy()){
                    return false;
                }
                return direction_up ? slides.getCurrentPosition() < endpos : slides.getCurrentPosition() > endpos;
            }
            return slides.isBusy();
        }
    }
    public Action slideAction(int position) {
        return new moveSlidesAction(position);
    }
    public Action slideAction(int position, int end_position) {
        return new moveSlidesAction(position, end_position);
    }
    public class moveArmFast implements  Action{
        boolean direction = false;
        int target = 0;
        double power = 0;

        boolean first = true;

        public moveArmFast(int target, double power){
            this.target = target;
            this.power = power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                first = false;
                direction = arm.getCurrentPosition() < target;
                arm.setManualPower(power);
                telemetryPacket.put("Arm Moving Fast to", target);

            }
            telemetryPacket.put("Not first hawk", target);

            boolean ongoing = direction ? arm.getCurrentPosition() < target : arm.getCurrentPosition() > target;
            if(!ongoing){
                arm.setManualPower(0);
            }
            return ongoing;
        }
    }
    public Action moveArmFast(int target, double power){
        return new moveArmFast(target, power);
    }
    public class moveArmAction implements Action {
        private int endpos = 0;
        private boolean partial_motion = false;
        private boolean first = true;
        private int target_pos = 0;

        private boolean direction_up = true;

        private boolean canceled = false;

        private boolean fast = false;
        private boolean ongoing = true;

        private double power = 0;
        public moveArmAction(int position){
            target_pos = position;
        }
        public moveArmAction(int position, int endpos){
            this.endpos = endpos;
            partial_motion = true;
            target_pos = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (canceled){
                telemetryPacket.put("Canceled", true);

                ongoing = false;
                return false;
            }

            if(first){
                moveArm(target_pos, true);


                first = false;
                direction_up = arm.getCurrentPosition() < endpos;
            }


            if(partial_motion){
                if(!arm.isBusy()){
                    ongoing = false;
                    return false;
                }
                ongoing = direction_up ? arm.getCurrentPosition() < endpos : arm.getCurrentPosition() > endpos;
                return ongoing;
            }
            ongoing = arm.isBusy();
            return ongoing;
        }
        public void cancel(){
            canceled = true;
        }
    }
    public class updateAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update(true);
            return true;
        }
    }
    public Action updateAction(){
        return new updateAction();
    }
    public moveArmAction armAction(int position ) {
        return new moveArmAction(position);
    }
    public moveArmAction armAction(int position, int end_position) {
        return new moveArmAction(position, end_position);
    }


}
