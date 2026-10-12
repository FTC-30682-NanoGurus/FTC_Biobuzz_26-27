package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGCRServo;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGMotorFlywheelTuner;
import org.firstinspires.ftc.teamcode.library.NGServo;

@Config
public class Intake2_0 {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;
    public NGMotor rollers;
    public static NGMotor flywheels;
    /** Null when "nectarFlywheel" is not in the configuration, so a missing motor cannot crash an auto. */
    public static NGMotor nectarFlywheel;
    /** Null when "flowerRamp" is not in the configuration; collectFromFlower() then just runs the rollers. */
    public NGServo flowerRamp;

    // Flywheel PIDF, kLoad/kRamp: all read from BiobuzzRobotConstants, shared with both teleops.

    /**
     * True for exactly as long as artifacts are being pushed into the POLLEN flywheel. Set and
     * cleared by transferUsingRollersForTime(), read every loop by updateFlywheelPID().
     *
     * Kept as an explicit flag rather than read back from the motor, so the boost lines up exactly
     * with the transfer window in transferUsingRollersForTime().
     */
    private volatile boolean feedingPollen = false;
    /** Same for nectar. Nothing in auto feeds nectar yet, so nothing sets it. */
    private volatile boolean feedingNectar = false;
    public static Servo hoodAdjuster;
    public NGMotor transferRollers;
    public NGServo gate;
    public NGCRServo flicker;
    public NGMotor interTransfer;
    public boolean autoFinished = false;

    /**
     * The gate in front of the POLLEN flywheel. Looked up as "pollenGate" first, then the legacy
     * single "gate", and left null if neither is configured, so a missing gate never stops an auto.
     */
    public double gateOpenPos = BiobuzzRobotConstants.pollenGateOpenPos;
    public double gateClosedPos = BiobuzzRobotConstants.pollenGateClosedPos;

    public Intake2_0(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        timer = new ElapsedTime();
        rollers = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.rollers);
        flywheels = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.pollenFlywheel);
        gate = findGate(hardwareMap, telemetry);
        //flicker = new NGCRServo(hardwareMap, telemetry, BiobuzzRobotConstants.flicker);
        transferRollers = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.transferRollers);
        //interTransfer = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.interTransfer);
        //interTransfer.setDirection(DcMotor.Direction.FORWARD);
        transferRollers.setDirection(BiobuzzRobotConstants.transferReversed
                ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        NGMotor.kLoad = BiobuzzRobotConstants.flywheelKLoad;
        NGMotor.kRamp = BiobuzzRobotConstants.flywheelKRamp;
        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();
        flywheels.setDirection(BiobuzzRobotConstants.pollenFlywheelReversed
                ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        pinFlywheelGains(flywheels, true);

        try {
            nectarFlywheel = new NGMotor(hardwareMap, telemetry, BiobuzzRobotConstants.nectarFlywheel);
            nectarFlywheel.init();
            nectarFlywheel.setZeroPowerBehavior_Brake();
            nectarFlywheel.setDirection(BiobuzzRobotConstants.nectarFlywheelReversed
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            pinFlywheelGains(nectarFlywheel, false);
        } catch (Exception e) {
            nectarFlywheel = null;
            telemetry.addLine("NECTAR FLYWHEEL NOT CONFIGURED - nectar shooting disabled");
        }

        try {
            flowerRamp = new NGServo(hardwareMap, telemetry, BiobuzzRobotConstants.flowerRamp);
            flowerRamp.setPosition(BiobuzzRobotConstants.flowerRampStowedPos);
        } catch (Exception e) {
            flowerRamp = null;
            telemetry.addLine("FLOWER RAMP NOT CONFIGURED - flower intake runs rollers only");
        }
        //hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);
    }
    public Intake2_0(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer) {
        this(hardwareMap, telemetry);
        this.timer = timer;
    }

    public void setAutoFinished(){
        autoFinished = true;
    }
    public boolean isAutoFinished(){
        return autoFinished;
    }

    public Action setFinished() { return new InstantAction(() -> setAutoFinished()); }

    public Action isFinished() { return new InstantAction(() -> isAutoFinished());}

    public void runRollers(double power){
        rollers.setPower(power);
        //interTransfer.setPower(power);
    }
    public void stopRollers(){
        rollers.setPower(0);
        //interTransfer.setPower(0);
    }
    private NGServo findGate(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            return new NGServo(hardwareMap, telemetry, BiobuzzRobotConstants.pollenGate);
        } catch (Exception ignored) {
        }
        try {
            NGServo legacy = new NGServo(hardwareMap, telemetry, BiobuzzRobotConstants.gate);
            gateOpenPos = BiobuzzRobotConstants.gateOpenPos;
            gateClosedPos = BiobuzzRobotConstants.gateClosedPos;
            return legacy;
        } catch (Exception ignored) {
        }
        telemetry.addLine("NO GATE CONFIGURED - transfers run without one");
        return null;
    }
    public void openGate(){
        if (gate != null) gate.setPosition(gateOpenPos);
    }
    public void closeGate(){
        if (gate != null) gate.setPosition(gateClosedPos);
    }
    /** The gains setCustomVelocityPID cannot carry - see the PIDF block above. */
    private static void pinFlywheelGains(NGMotor m, boolean pollen) {
        NGMotorFlywheelTuner.setStableKp(m, pollen ? BiobuzzRobotConstants.pollenKpStable : BiobuzzRobotConstants.nectarKpStable);
        NGMotorFlywheelTuner.setRecoveryKp(m, pollen ? BiobuzzRobotConstants.pollenKpRecovery : BiobuzzRobotConstants.nectarKpRecovery);
        NGMotorFlywheelTuner.setAlpha(m, pollen ? BiobuzzRobotConstants.pollenAlpha : BiobuzzRobotConstants.nectarAlpha);
        NGMotorFlywheelTuner.setRecoveryThreshold(m, pollen ? BiobuzzRobotConstants.pollenRecoveryThreshold : BiobuzzRobotConstants.nectarRecoveryThreshold);
    }
    private static void setPollenVelocity(double vel){
        flywheels.setCustomVelocityPID(vel, BiobuzzRobotConstants.pollenKpStable, BiobuzzRobotConstants.pollenKI, BiobuzzRobotConstants.pollenKD, BiobuzzRobotConstants.pollenKF);
    }
    private static void setNectarVelocity(double vel){
        if (nectarFlywheel == null) return;
        nectarFlywheel.setCustomVelocityPID(vel, BiobuzzRobotConstants.nectarKpStable, BiobuzzRobotConstants.nectarKI, BiobuzzRobotConstants.nectarKD, BiobuzzRobotConstants.nectarKF);
    }
    public void runFlywheels(double vel){
        setPollenVelocity(vel);
    }
    public void stopFlywheels(){
        setPollenVelocity(0);
    }
    public void runNectarFlywheel(double vel){
        setNectarVelocity(vel);
    }
    public void stopNectarFlywheel(){
        setNectarVelocity(0);
    }
    public static void initFlywheels(){
        setPollenVelocity(0);
        setNectarVelocity(0);
    }
    public void runTransferRollers(double power){
        transferRollers.setPower(power);
        if (interTransfer != null) interTransfer.setPower(power);
    }
    public void stopTransferRollers(){
        transferRollers.setPower(0);
        if (interTransfer != null) interTransfer.setPower(0);
    }
    public void hoodAdjusterToPos(double position){
        hoodAdjuster.setPosition(position);
    }
    public void updateVelocity(){
        if(DECODERobotConstants.flywheelsActive) {
            flywheels.updateFlywheels(transferRollers.getPower() > 0);
        }
    }
    /*public Action updateFlywheelPID(){
        return new InstantAction(() -> updateVelocity());
    }*/

    public class updateAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // The flag must stay true for the whole transfer and be read every loop: NGMotor starts
            // the ramp timer on the first loop it sees true and resets it when it goes false, so the
            // boost ramps up across the entire transfer and drops back to zero right after.
            flywheels.updateFlywheels(feedingPollen || transferRollers.getPower() > 0.1);
            if (nectarFlywheel != null) nectarFlywheel.updateFlywheels(feedingNectar);

            telemetry.addData("FW Current Vel", "%.1f", flywheels.getVelocity());
            if (nectarFlywheel != null) {
                telemetry.addData("Nectar FW Current Vel", "%.1f", nectarFlywheel.getVelocity());
            }

            return true;
        }
    }
    public Action updateFlywheelPID(){
        return new updateAction();
    }

    public static void initHood(){
        hoodAdjuster.setPosition(0.9);
    }

    /*public void updateFlywheels(){
        flywheels.update();
    }*/
    public void prepShooter(double power){
        //hoodAdjuster.setPosition(0.5);
        flywheels.setPower(power);
        telemetry.addLine("Ready To Shoot");
        telemetry.addData("Flywheel Velocity: ", flywheels.getVelocity());
        telemetry.update();
    }
    public void shoot(){
        transferArtifacts();
        telemetry.update();
    }
    public void transferArtifacts(){
        rollers.setPower(1);
        transferRollers.setPower(1);
        telemetry.update();
    }
    public void resetOuttake(){
        rollers.setPower(0);
        transferRollers.setPower(0);
        flywheels.setPower(0);
        telemetry.clear();
        telemetry.addLine("Reset Complete");
        telemetry.update();
    }
    public Action runShooter(double vel, double time){
        return new SequentialAction(
                new InstantAction(() -> runFlywheels(vel)),
                new SleepAction(time),
                new InstantAction(() -> stopFlywheels())
        );
    }
    /** Spins BOTH flywheels for {@code time} seconds, then stops both. Used by the BIOBUZZ autos. */
    public Action runShooter(double pollenVel, double nectarVel, double time){
        return new SequentialAction(
                new InstantAction(() -> runFlywheels(pollenVel)),
                new InstantAction(() -> runNectarFlywheel(nectarVel)),
                new SleepAction(time),
                new InstantAction(() -> stopFlywheels()),
                new InstantAction(() -> stopNectarFlywheel())
        );
    }
    public Action stopShooter(){
        return new SequentialAction(
                new InstantAction(() -> stopFlywheels())
        );
    }
    public Action shoot(double time, double vel){
        return new SequentialAction(
                new InstantAction(() -> runFlywheels(vel)),
                new SleepAction(time),
                new InstantAction(() -> stopFlywheels())
        );
    }
    public Action collect(double time) {
        return new SequentialAction(
                new InstantAction(() -> runRollers(1)),
                new SleepAction(time),
                new InstantAction(() -> stopRollers())
        );
    }
    /**
     * Intake from a FLOWER: swing the ramp down to the intake position, run the rollers for
     * {@code time} seconds, then stop them and stow the ramp again. With no ramp configured it is
     * just collect(time).
     */
    public Action collectFromFlower(double time) {
        return new SequentialAction(
                new InstantAction(() -> setFlowerRamp(BiobuzzRobotConstants.flowerRampIntakePos)),
                new InstantAction(() -> runRollers(1)),
                new SleepAction(time),
                new InstantAction(() -> stopRollers()),
                new InstantAction(() -> setFlowerRamp(BiobuzzRobotConstants.flowerRampStowedPos))
        );
    }
    /**
     * Swings the flower ramp to its intake position and returns immediately, so it can be placed at
     * the start of the turn toward a FLOWER and deploy while the robot is still turning.
     * collectFromFlower() sets the same position again (harmless) and stows the ramp when done.
     */
    public Action deployFlowerRamp() {
        return new InstantAction(() -> setFlowerRamp(BiobuzzRobotConstants.flowerRampIntakePos));
    }
    public void setFlowerRamp(double position) {
        if (flowerRamp != null) flowerRamp.setPosition(position);
    }

    /**
     * Stops everything an interrupted action could have left running: rollers, transfer, gate, the
     * load-boost flags and the flower ramp. Used when the park watchdog cuts an auto short.
     */
    public void stopAllFeeding() {
        feedingPollen = false;
        feedingNectar = false;
        stopRollers();
        stopTransferRollers();
        closeGate();
        setFlowerRamp(BiobuzzRobotConstants.flowerRampStowedPos);
    }
    public Action preventEscape(double time){
        return new SequentialAction(
                new InstantAction(() -> runRollers(0.3)),
                new SleepAction(time),
                new InstantAction(() -> stopRollers())
        );
    }
    /**
     * One volley into the POLLEN flywheel: gate open, TRANSFER motor on (plus the intake if
     * transferAlsoRunsIntake), for {@code time} seconds, then everything off. Same duration as
     * before the motors were split, so auto timing is unchanged.
     */
    public Action transferUsingRollersForTime(double time, double p){
        return new SequentialAction(
                new InstantAction(() -> openGate()),
                new InstantAction(() -> feedingPollen = true),
                new InstantAction(() -> runTransferRollers(p)),
                new InstantAction(() -> {
                    if (BiobuzzRobotConstants.transferAlsoRunsIntake) runRollers(p);
                }),
                new SleepAction(time),
                new InstantAction(() -> closeGate()),
                new InstantAction(() -> stopTransferRollers()),
                new InstantAction(() -> stopRollers()),
                new InstantAction(() -> feedingPollen = false)
        );
    }
    public Action transferUsingRollers(double p){
        return new ParallelAction(
                new InstantAction(() -> runTransferRollers(p)),
                new InstantAction(() -> runRollers(p))
        );
    }
    public Action disableTransfer(){
        return new ParallelAction(
                new InstantAction(() -> stopTransferRollers()),
                new InstantAction(() -> stopRollers())
        );
    }
    public Action setHoodAdjuster(double pos){
        return new SequentialAction(
                new InstantAction(() -> hoodAdjusterToPos(pos))
        );
    }
}
