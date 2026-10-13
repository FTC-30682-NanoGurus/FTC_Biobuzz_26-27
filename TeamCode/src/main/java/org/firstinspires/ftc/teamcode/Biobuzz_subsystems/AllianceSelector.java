package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BiobuzzRobotConstants;

/**
 * Alliance selection for the BIOBUZZ autos, done with either gamepad during INIT:
 * B (or circle) = RED, X (or square) = BLUE. The choice is stored in
 * {@link BiobuzzRobotConstants#allianceRed} and kept until the Robot Controller app restarts, so the
 * previous choice is the default and is shown in large text every INIT loop.
 *
 * Rules (Competition Manual TU03): selecting and INITing the OpMode happens during MATCH setup
 * (10.3.3, G305), and the only restriction on touching the OPERATOR CONSOLE before the MATCH is
 * G401, which starts "as soon as the countdown to start the MATCH begins". So pick the alliance
 * after pressing INIT and before the countdown. INIT code must not move the robot (G304.H) - this
 * only reads buttons.
 */
public final class AllianceSelector {

    private AllianceSelector() {
    }

    /** Reads both gamepads; returns true if the alliance changed. */
    public static boolean poll(Gamepad g1, Gamepad g2) {
        boolean red = BiobuzzRobotConstants.allianceRed;
        // A button pressed with START held is the Driver Station pairing gamepad 2 (START+B), not a choice.
        if ((g1.b && !g1.start) || (g2.b && !g2.start)) red = true;
        if ((g1.x && !g1.start) || (g2.x && !g2.start)) red = false;
        boolean changed = red != BiobuzzRobotConstants.allianceRed;
        BiobuzzRobotConstants.allianceRed = red;
        return changed;
    }

    public static boolean red() {
        return BiobuzzRobotConstants.allianceRed;
    }

    /** Adds the alliance lines to the INIT telemetry. */
    public static void show(Telemetry telemetry) {
        boolean red = BiobuzzRobotConstants.allianceRed;
        telemetry.addLine(red ? "ALLIANCE: ===== RED =====" : "ALLIANCE: ===== BLUE =====");
        telemetry.addLine(red ? "  press X (square) for BLUE" : "  press B (circle) for RED");
        telemetry.addLine("  start at the SAME spot as seen from your own drivers' station");
    }
}
