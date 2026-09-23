package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * REV Color Sensor V3 game-element classifier and counter for BIOBUZZ (2026-27).
 *
 * Sorts whatever is in front of the sensor into one of three game elements by hue, and keeps a
 * running count of how many separate pieces have passed it. The count is the point: it is what an
 * intake or a sorter would use to know how much it is carrying, and this opmode is the place to
 * prove the thresholds and the debounce before any mechanism depends on them.
 *
 * HARDWARE
 * --------
 * One REV Color Sensor V3, configured in the robot config as "REV Color Sensor V3" with the name
 * {@link #SENSOR_NAME}. Nothing else - no drivetrain, no intake. Safe to run on a bench with the
 * sensor in your hand.
 *
 * CHECK THE CONFIGURATION TYPE FIRST. The V2 and the V3 are DIFFERENT entries on the Driver
 * Station: the V2 is "REV Color/Range Sensor" and the V3 is "REV Color Sensor V3". They load
 * different drivers for different chips and are not interchangeable. If the config still says
 * V2 while a V3 is plugged in, the readings will be wrong or absent long before any threshold here
 * matters. This is the first thing to fix after swapping the hardware.
 *
 * Under the hood the V3 is the SDK's RevColorSensorV3, a Broadcom APDS-9151 - a different chip
 * from the V2's AMS TCS34725, with its own driver stack (BroadcomColorSensorImpl rather than
 * AMSColorSensorImpl). Colour is read through {@link NormalizedColorSensor} and proximity through
 * {@link DistanceSensor}, both generic interfaces, so the sensing code itself is unchanged.
 *
 * WHAT CHANGED COMING BACK FROM THE V2
 * ------------------------------------
 * 1. THE HUE THRESHOLDS ARE NOW SUSPECT AND MUST BE RE-MEASURED. This is the important one. The
 *    bands below were measured on the V2's TCS34725. The V3's APDS-9151 has a different spectral
 *    response, so the same physical piece reports a different hue. Nothing will crash; the counts
 *    will just be wrong in ways that look like a logic bug. Hold each element at the working
 *    distance on the init screen, read the H value off telemetry, and reset the bands from what
 *    you actually see before trusting a single count.
 *
 * 2. THE LED CAN NOW BE SWITCHED FROM CODE, and this opmode does it (press X while running).
 *    Neither sensor implements SwitchableLight, so the generic interface route does not work, but
 *    the concrete RevColorSensorV3 inherits a public enableLed(boolean) from
 *    BroadcomColorSensorImpl. That is why this file now holds a {@link RevColorSensorV3} reference
 *    alongside the generic one. Turning the LED off is a genuine diagnostic: if the classification
 *    barely changes with it off, the sensor is reading room light rather than the element, and the
 *    real fix is a shroud or a shorter working distance.
 *
 * 3. DISTANCE IS STILL REFLECTANCE, NOT TIME OF FLIGHT. The V3's proximity comes from its own IR
 *    channel rather than the colour channels, so it is cleaner than the V2's, but it is still
 *    measuring how much light bounces back. A dark red nectar therefore reads as FURTHER AWAY than
 *    a yellow pollen at the same physical distance, and a proximity gate tuned against yellow will
 *    quietly reject red. Tune MAX_DISTANCE_MM against the DARKEST element you care about, or press
 *    Y to switch the gate off and let the V floor do the work alone. This is still the single most
 *    likely reason for "it counts pollen fine but never sees nectar".
 *
 *    The V3's useful proximity span is roughly 10-100 mm and its driver caps the reported value at
 *    an internal maxDist, so anything past that pins rather than rising smoothly. Do not read a
 *    large steady number as "far away"; read it as "out of range".
 *
 * 4. setGain() IS STILL A SOFTWARE MULTIPLIER on both sensors, not the chip's hardware gain. It
 *    scales R, G and B by the same factor inside getNormalizedColors(). That is good news for the
 *    thresholds - scaling all three channels equally leaves H and S untouched and moves only V -
 *    but it means gain amplifies sensor noise along with signal, so the smallest gain that clears
 *    the V floor is the right one. Expect to re-tune GAIN for the V3 as well: a different chip at
 *    the same gain lands at a different V.
 *
 * CONTROLS
 * --------
 *   INIT    B ......... alliance = RED      (which nectar is ours)
 *           X ......... alliance = BLUE
 *   RUN     A ......... reset the counts to zero
 *           Y ......... toggle the proximity gate
 *           X ......... toggle the sensor's white LED (V3 only)
 *
 * HOW A "DETECTION" IS DEFINED
 * ----------------------------
 * A detection is one PIECE, not one loop iteration. The loop runs a few hundred times a second, so
 * counting every loop that sees yellow would turn a single piece of pollen sitting still in front
 * of the sensor into thousands of detections. Instead the counter is edge-triggered with hysteresis:
 *
 *   1. A colour must classify the same way for CONFIRM_MS before it counts. This throws away the
 *      mixed readings taken while a piece is halfway across the sensor's field, which otherwise
 *      flicker between the real colour and whatever is behind it.
 *   2. After a piece is counted, the sensor must go back to NONE for CLEAR_MS before anything can
 *      be counted again. Without this, a piece that momentarily reads as nothing - a specular
 *      highlight, a seam - and then reads as yellow again would be counted twice.
 *
 * Both are in milliseconds rather than loop counts on purpose: loop rate changes when telemetry is
 * on, when the dashboard connects, and when this logic gets pasted into a real opmode next to a
 * drivetrain, and a debounce measured in loops would silently change length every time.
 *
 * The cost of rule 2 is that two pieces touching each other - no gap between them - count as one,
 * even when they are different colours. That is deliberate: the alternative, re-arming whenever the
 * colour changes, double-counts every single piece whose hue drifts across a band edge on its way
 * past, which is far more common than back-to-back pieces with no gap. If your intake really does
 * present pieces with no space between them, CLEAR_MS is the number to shorten, and the mechanism
 * that meters them is the real fix.
 *
 * TUNING THE GAIN - READ THIS IF NOTHING EVER COUNTS
 * --------------------------------------------------
 * getNormalizedColors() returns values scaled so that 1.0 is the top of the range. At low gain a
 * typical game element reads only a few hundredths, which puts V near 0.03 and means the V >= 0.25
 * floor rejects EVERYTHING. That is not a bug in the thresholds; it is what gain is for. Raise
 * {@link #GAIN} on the init screen until the V shown on telemetry sits comfortably above the floor
 * for a piece held at the working distance - roughly 0.4-0.8 - and stop before any channel pins at
 * 1.00. A clipped channel destroys the hue, because hue is defined by the RATIO between channels,
 * and two different colours that both clip red become the same hue with no way back.
 *
 * H is 0-360 degrees, S and V are 0-1. The bands below are the ones measured on the OLD V2 and
 * are kept only as a starting point - see point 1 above, they need re-measuring on the V3:
 *
 *   Red nectar ...... H <= 20 or H >= 340   (the hue wheel wraps through 0 at red)
 *   Blue nectar ..... 170 <= H <= 250
 *   Yellow pollen ... 45 <= H <= 90
 *   all three ....... S >= 0.5 and V >= 0.25
 *
 * WHAT THE COUNT INCLUDES
 * -----------------------
 * {@link #totalCount} is pollen plus OUR alliance's nectar together, which is the number a sorter
 * cares about - the pieces worth keeping. The opposing alliance's nectar is classified and shown
 * too, but counted separately into {@link #opponentNectarCount} so it can be rejected rather than
 * stored. If you want the total to mean "every game element seen regardless of alliance", set
 * {@link #COUNT_OPPONENT_NECTAR} true.
 */
@Config
@TeleOp(name = "Color Sensor V3 Testing (Pollen/Nectar)", group = "testing")
public class colorSensorTesting extends LinearOpMode {

    /** Robot-config name of the REV Color Sensor V3 (config type "REV Color Sensor V3"). */
    public static String SENSOR_NAME = "colorSensor";

    // ---- hue / saturation / value bands ----------------------------------------------------
    // H in degrees 0-360, S and V in 0-1, as produced by Color.colorToHSV().
    // TODO-REMEASURE: these were measured on the V2's AMS TCS34725. The V3's Broadcom APDS-9151
    // has a different spectral response and will report different hues for the same elements.

    /** Red wraps through 0, so it needs two bounds instead of a low/high pair. */
    public static double RED_H_MAX = 20;    // H <= this counts as red ...
    public static double RED_H_MIN = 340;   // ... and so does H >= this

    public static double BLUE_H_MIN = 170;
    public static double BLUE_H_MAX = 250;

    public static double YELLOW_H_MIN = 44;
    public static double YELLOW_H_MAX = 62;

    /** Shared by all three: below these the hue is not trustworthy enough to classify. */
    public static double S_MIN = 0.3;
    public static double V_MIN = 0.2;

    // ---- sensor setup ----------------------------------------------------------------------
    /**
     * Colour gain. A SOFTWARE multiplier on R, G and B - see the class header. This is a starting
     * point only; tune it on the init screen against a real piece before trusting any count.
     */
    public static float GAIN = 4.0f;

    // ---- proximity gate --------------------------------------------------------------------
    public static boolean USE_DISTANCE_GATE = true;
    /**
     * Anything read further away than this is treated as empty, whatever colour it looks like.
     *
     * Still reflectance-based on the V3, so dark elements read as further away than bright ones.
     * Tune it against red nectar, not yellow.
     *
     * TODO-RETUNE for the V3, whose useful span is roughly 10-100 mm and whose driver pins the
     * reading at an internal maxDist beyond that. 100 mm sits right at the top of that span, so
     * this gate is currently close to no gate at all. Measure what a piece actually reads at the
     * working distance and set this a little above it.
     */
    public static double MAX_DISTANCE_MM = 100.0;

    // ---- detection debounce ----------------------------------------------------------------
    /** How long one colour must hold steady before it is counted as a piece. */
    public static double CONFIRM_MS = 60.0;
    /** How long the sensor must read empty afterwards before the next piece can be counted. */
    public static double CLEAR_MS = 120.0;

    /** True makes totalCount include the opposing alliance's nectar as well. */
    public static boolean COUNT_OPPONENT_NECTAR = false;

    public static double TELEMETRY_INTERVAL_MS = 100.0;

    /** The three game elements, plus the absence of one. */
    private enum Element { NONE, RED_NECTAR, BLUE_NECTAR, YELLOW_POLLEN }

    private enum Alliance { RED, BLUE }

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;   // null if this sensor has no range half

    /**
     * The same device as {@link #colorSensor}, typed concretely, or null if what is plugged in is
     * not a V3. Held ONLY for enableLed(): that method lives on the concrete Broadcom driver and
     * is not reachable through NormalizedColorSensor, and neither REV sensor implements
     * SwitchableLight. Every actual reading still goes through the generic interfaces, so a
     * mis-configured or swapped sensor costs the LED toggle and nothing else.
     */
    private RevColorSensorV3 revV3;

    /** Desired LED state. The sensor powers up with it on, which is what the thresholds assume. */
    private boolean ledOn = true;

    private Alliance alliance = Alliance.RED;

    // ---- the counts, kept as fields so another opmode can read them straight off this class --
    /** Pollen + our nectar. The headline number. */
    private int totalCount = 0;
    private int pollenCount = 0;
    private int allianceNectarCount = 0;
    private int opponentNectarCount = 0;

    // ---- debounce state --------------------------------------------------------------------
    private Element candidate = Element.NONE;     // what the sensor has been reading lately
    private Element lastCounted = Element.NONE;   // most recent piece actually counted
    private boolean armed = true;                 // false until the sensor has cleared again
    private final ElapsedTime stableTimer = new ElapsedTime();   // how long candidate has held
    private final ElapsedTime clearTimer = new ElapsedTime();    // how long NONE has held

    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean prevA, prevY, prevX;
    private boolean prevInitB, prevInitX;

    // Scratch array for Color.colorToHSV, reused every loop - allocating a new float[3] a few
    // hundred times a second is free garbage for no reason.
    private final float[] hsv = new float[3];

    // The most recent reading, cached so telemetry can print exactly the numbers the classifier
    // acted on. Reading the sensor a second time for the display would print a DIFFERENT sample -
    // which is maddening when the count and the hue on screen disagree at a band edge, and is the
    // first thing you would (wrongly) blame the thresholds for.
    private NormalizedRGBA lastColors;
    private double lastDistanceMm = Double.MAX_VALUE;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, SENSOR_NAME);
        colorSensor.setGain(GAIN);

        // The V3 reports proximity too, but a bare colour sensor configured under the same name
        // would not. Asking for the cast rather than assuming it means a wrong config entry
        // degrades to "no proximity gate" instead of crashing on the first loop.
        distanceSensor = (colorSensor instanceof DistanceSensor) ? (DistanceSensor) colorSensor : null;

        // Same check, same reason, for the LED. A V2 left in the config, or a plain colour sensor,
        // leaves this null and simply disables the X toggle rather than failing at runtime.
        revV3 = (colorSensor instanceof RevColorSensorV3) ? (RevColorSensorV3) colorSensor : null;
        setLed(ledOn);

        // ---- INIT LOOP: pick the alliance ---------------------------------------------------
        // This has to be a loop rather than a bare waitForStart(), because the answer comes from
        // the gamepad and nothing reads the gamepad unless something is polling it. Live colour
        // readout comes along for free, which is also where the gain gets tuned: hold a piece at
        // the working distance and watch V before pressing START.
        while (opModeInInit()) {
            boolean b = gamepad1.b && !prevInitB;
            boolean x = gamepad1.x && !prevInitX;
            prevInitB = gamepad1.b;
            prevInitX = gamepad1.x;

            if (b) alliance = Alliance.RED;
            if (x) alliance = Alliance.BLUE;

            colorSensor.setGain(GAIN);   // so dashboard edits take effect while aiming
            Element seen = classify(readHsv(), readDistanceMm());

            telemetry.addLine("ALLIANCE SELECT - press B for RED, X for BLUE");
            telemetry.addLine();
            telemetry.addData(">> ALLIANCE", alliance);
            telemetry.addData("Our nectar", alliance == Alliance.RED ? "RED" : "BLUE");
            telemetry.addLine();
            addSensorTelemetry(seen);
            telemetry.addLine();
            telemetry.addLine("Raise GAIN until V reads about 0.4-0.8 on a real piece,");
            telemetry.addLine("but stop before R, G or B pins at 1.00.");
            telemetry.addLine();
            telemetry.addLine("Press START when the classification is stable.");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Seed the edge detector from the CURRENT state. X selects BLUE on the init screen, so
        // without this a driver still holding X at START would immediately toggle the LED off.
        prevX = gamepad1.x;

        stableTimer.reset();
        clearTimer.reset();
        telemetryTimer.reset();

        while (!isStopRequested() && opModeIsActive()) {

            boolean aPressed = gamepad1.a && !prevA;
            boolean yPressed = gamepad1.y && !prevY;
            boolean xPressed = gamepad1.x && !prevX;
            prevA = gamepad1.a;
            prevY = gamepad1.y;
            prevX = gamepad1.x;

            if (aPressed) resetCounts();
            if (yPressed) USE_DISTANCE_GATE = !USE_DISTANCE_GATE;
            if (xPressed) setLed(!ledOn);

            colorSensor.setGain(GAIN);

            double distanceMm = readDistanceMm();
            float[] readHsv = readHsv();
            Element seen = classify(readHsv, distanceMm);

            updateCount(seen);

            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetry.addData("ALLIANCE", alliance);
                telemetry.addLine();
                telemetry.addData(">>> TOTAL DETECTIONS", totalCount);
                telemetry.addData("    pollen (yellow)", pollenCount);
                telemetry.addData("    our nectar (" + alliance + ")", allianceNectarCount);
                telemetry.addData("    opposing nectar", "%d%s", opponentNectarCount,
                        COUNT_OPPONENT_NECTAR ? " (included in total)" : " (NOT in total)");
                telemetry.addData("    last counted", lastCounted);
                telemetry.addLine();
                addSensorTelemetry(seen);
                telemetry.addData("Armed", armed ? "yes - ready to count"
                        : "no - waiting for the sensor to clear");
                telemetry.addLine();
                telemetry.addLine("A = reset counts   Y = proximity gate   X = LED");
                telemetry.update();
                telemetryTimer.reset();
            }
        }
    }

    // ===========================================================================================
    // Sensing
    // ===========================================================================================

    /**
     * One colour reading, converted to H (0-360), S (0-1), V (0-1).
     *
     * toColor() packs the four normalized floats back into a 32-bit ARGB int, clipping anything
     * above 1.0 on the way. That clipping is exactly why GAIN must not be pushed until a channel
     * saturates: two different colours that both clip red land on the same int and become the same
     * hue, and no threshold downstream can tell them apart again.
     */
    private float[] readHsv() {
        lastColors = colorSensor.getNormalizedColors();
        Color.colorToHSV(lastColors.toColor(), hsv);
        return hsv;
    }

    /**
     * Distance in mm, or {@link Double#MAX_VALUE} when there is no usable reading.
     *
     * On the V3 this is a curve fit (inFromOptical) over the raw IR proximity reading, and the fit
     * can go imaginary when almost nothing is reflected back - which surfaces as NaN. Both NaN and
     * "no sensor" have to collapse to a large number rather than being passed through, because
     * every comparison against NaN is false, so a raw NaN would slip through the gate's `>` test
     * and be treated as close enough to classify.
     */
    private double readDistanceMm() {
        if (distanceSensor == null) return Double.MAX_VALUE;
        double d = distanceSensor.getDistance(DistanceUnit.MM);
        lastDistanceMm = (Double.isNaN(d) || Double.isInfinite(d)) ? Double.MAX_VALUE : d;
        return lastDistanceMm;
    }

    /**
     * Turns one HSV reading into a game element, or NONE.
     *
     * Order matters only in that the bands must not overlap - they do not, with the season's
     * numbers - so the first match wins and nothing is ambiguous.
     */
    private Element classify(float[] c, double distanceMm) {
        if (USE_DISTANCE_GATE && distanceMm > MAX_DISTANCE_MM) return Element.NONE;

        double h = c[0], s = c[1], v = c[2];
        if (s < S_MIN || v < V_MIN) return Element.NONE;

        // Red is the wrap-around case: its band is split across the 0/360 seam, so it is the one
        // colour that cannot be written as a single low <= h <= high test.
        if (h <= RED_H_MAX || h >= RED_H_MIN) return Element.RED_NECTAR;
        if (h >= BLUE_H_MIN && h <= BLUE_H_MAX) return Element.BLUE_NECTAR;
        if (h >= YELLOW_H_MIN && h <= YELLOW_H_MAX) return Element.YELLOW_POLLEN;

        return Element.NONE;
    }

    // ===========================================================================================
    // Counting
    // ===========================================================================================

    /**
     * Debounced edge counter. Called once per loop with the current classification.
     *
     * Three things are tracked: which element the sensor has settled on ({@link #candidate}), how
     * long it has been settled there ({@link #stableTimer}), and whether a new piece is allowed to
     * count at all yet ({@link #armed}). A piece counts when all three line up - the candidate is
     * a real element, it has held for CONFIRM_MS, and the counter is armed - and the counter then
     * disarms until the sensor has read NONE continuously for CLEAR_MS.
     */
    private void updateCount(Element seen) {
        if (seen != candidate) {
            candidate = seen;
            stableTimer.reset();
        }

        if (seen == Element.NONE) {
            // Only a sustained gap re-arms. A single empty loop between two readings of the same
            // piece - a highlight, a seam, a gap in the intake - must not look like the piece left.
            if (!armed && clearTimer.milliseconds() >= CLEAR_MS) {
                armed = true;
            }
            return;
        }

        // Something is there, so the clear timer restarts from now. Whenever this line stops
        // running for CLEAR_MS straight, the branch above re-arms.
        clearTimer.reset();

        if (armed && stableTimer.milliseconds() >= CONFIRM_MS) {
            count(seen);
            armed = false;
        }
    }

    private void count(Element e) {
        lastCounted = e;

        boolean ours = (e == Element.YELLOW_POLLEN)
                || (e == Element.RED_NECTAR && alliance == Alliance.RED)
                || (e == Element.BLUE_NECTAR && alliance == Alliance.BLUE);

        if (e == Element.YELLOW_POLLEN) {
            pollenCount++;
        } else if (ours) {
            allianceNectarCount++;
        } else {
            opponentNectarCount++;
        }

        if (ours || COUNT_OPPONENT_NECTAR) totalCount++;
    }

    /**
     * Turns the sensor's white illumination LED on or off.
     *
     * Goes through the concrete {@link RevColorSensorV3}, because enableLed() is not on any generic
     * interface and neither REV sensor implements SwitchableLight. Falls back to SwitchableLight
     * for any other device that does implement it, and is a silent no-op when neither applies, so
     * nothing here can fail on a swapped sensor.
     *
     * Remember that the thresholds were measured with the LED ON. Turning it off is a diagnostic,
     * not a mode: every band below will need different numbers under ambient light alone.
     */
    private void setLed(boolean on) {
        ledOn = on;
        if (revV3 != null) {
            revV3.enableLed(on);
        } else if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(on);
        }
    }

    private void resetCounts() {
        totalCount = 0;
        pollenCount = 0;
        allianceNectarCount = 0;
        opponentNectarCount = 0;
        lastCounted = Element.NONE;
        candidate = Element.NONE;
        armed = true;
        stableTimer.reset();
        clearTimer.reset();
    }

    // ===========================================================================================
    // Plumbing
    // ===========================================================================================

    /**
     * The raw readout, shared by the init loop and the run loop so both show the same thing.
     *
     * Prints the CACHED reading rather than taking a fresh one - see the note on lastColors.
     */
    private void addSensorTelemetry(Element seen) {
        telemetry.addData("SEEING", seen);
        telemetry.addData("H / S / V", "%.0f deg   %.3f   %.3f", hsv[0], hsv[1], hsv[2]);
        if (lastColors != null) {
            telemetry.addData("R / G / B", "%.3f  %.3f  %.3f",
                    lastColors.red, lastColors.green, lastColors.blue);
        }
        telemetry.addData("Gain", "%.1f (software)", colorSensor.getGain());

        if (distanceSensor == null) {
            telemetry.addData("Distance", "no range sensor under '" + SENSOR_NAME + "'");
        } else if (lastDistanceMm == Double.MAX_VALUE) {
            telemetry.addData("Distance", "out of range");
        } else {
            telemetry.addData("Distance", "%.1f mm (IR reflectance - darker reads further)",
                    lastDistanceMm);
        }

        if (USE_DISTANCE_GATE) {
            telemetry.addData("Proximity gate", "ON, <= %.0f mm", MAX_DISTANCE_MM);
        } else {
            telemetry.addData("Proximity gate", "OFF");
        }

        // The LED state is read back from the device rather than echoed from our own flag, so a
        // driver that silently refused the write shows up here instead of being believed. A dark
        // LED is a real failure mode and this is the only place it would surface.
        String ledState = (colorSensor instanceof Light)
                ? (((Light) colorSensor).isLightOn() ? "on" : "OFF")
                : "unknown";

        if (revV3 != null) {
            telemetry.addData("LED", "%s  (X toggles)", ledState);
        } else if (colorSensor instanceof SwitchableLight) {
            telemetry.addData("LED", "%s  (X toggles)", ledState);
        } else {
            telemetry.addData("LED", "%s  (no control on this device)", ledState);
        }

        // Says outright whether a V3 is really what is plugged in. Getting this wrong is the most
        // likely single cause of bad readings after swapping sensors, and it is invisible
        // otherwise - a V2 under a V3 config still returns numbers, just wrong ones.
        telemetry.addData("Driver", revV3 != null
                ? "RevColorSensorV3 - correct"
                : "NOT a V3: " + colorSensor.getClass().getSimpleName()
                  + " - check the config type is 'REV Color Sensor V3'");
    }
}
