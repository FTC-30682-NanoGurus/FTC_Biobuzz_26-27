package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import javax.imageio.ImageIO;

public class MeepMeepTestingNikhil {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity sampleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints:
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints:
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0, 0, 100);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0, 0, 0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0, 0, 255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255, 255, 255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0, 100, 100);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0, 155, 200);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0, 0, 100);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0, 0, 185);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0, 50, 255);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0, 50, 255);
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0.7;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0.3;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_PATH_COLOR() {
                        return new Color(0, 80, 255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0, 80, 255);
                    }

                })
                .build();

        RoadRunnerBotEntity sampleBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints:
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0, 100, 0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0, 0, 0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0, 255, 0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255, 255, 255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0, 100, 20);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0, 155, 0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0, 50, 0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0, 100, 0);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0, 200, 50);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0, 200, 50);
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0.7;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0.3;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_PATH_COLOR() {
                        return new Color(0, 255, 0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0, 255, 0);
                    }

                })
                .build();

        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            return new MinMax(-10, 50);
        };
        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < -30.0) {
                return new MinMax(-5, 5);
            } else {
                return new MinMax(-120, 120);
            }
        };
        AccelConstraint intakeAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10, 22);
            } else {
                return new MinMax(-30, 50);
            }
        };
        VelConstraint intakeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > 50.0) {
                return 15;
            } else {
                return 60;
            }
        };

        VelConstraint fastShootVel = (robotPose, _path, _disp) -> {
            return 65;
        };

        VelConstraint parkTurnVel = (robotPose, _path, _disp) -> {
            return 25;
        };


        // Red side (Hive-up side start)
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -12, Math.toRadians(180)))
                .lineToX(55)
                    .waitSeconds(1.5)
                    .setReversed(false)
                .splineToLinearHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), intakeVel)
                    .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), fastShootVel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), fastShootVel)
                .splineToLinearHeading(new Pose2d(-54, -12, Math.toRadians(360)), Math.toRadians(-220), new TranslationalVelConstraint(45))
                    .waitSeconds(1.5)
                    .setReversed(false)
                .lineToXLinearHeading(-58, Math.toRadians(180), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        return 4;
                    }
                })
                    .waitSeconds(5)
                .lineToXLinearHeading(-54, Math.toRadians(360))
                    .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(-50, -17, Math.toRadians(-90)), Math.toRadians(-90), parkTurnVel)
                .splineToLinearHeading(new Pose2d(-50, -57, Math.toRadians(-90)), Math.toRadians(-90), new TranslationalVelConstraint(70))
                //.splineToLinearHeading(new Pose2d(-35, -63, Math.toRadians(-90)), Math.toRadians(-90))
                                //.splineToLinearHeading()
                //.splineToSplineHeading()
                .build());*/
        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -12, Math.toRadians(180)))
                        .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(55, -25, Math.toRadians(180)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-32, -58, Math.toRadians(180)), Math.toRadians(190))
                .build());

        // Red side (Hive-up side start) -- MAX SPEED (green bot)
        // Same waypoints, headings, tangents, and waits as the original. intakeVel is untouched.
        // Every limit is the real robot's (MecanumDrive PARAMS), not MeepMeep's 15" / 50 in/s default bot.
        double TRACK_WIDTH = 0.00303299391 * 3990.098938918289;           // inPerTick * trackWidthTicks
        double LATERAL_MULT = 0.00303299391 / 0.0023898724154397585;      // inPerTick / lateralInPerTick
        double MAX_WHEEL_VEL = 64;      // in/s. kS/kV top out ~68 in/s at 12 V; 64 keeps a sliver for feedback
        double MAX_PATH_ACCEL = 100;    // in/s^2 on straight drives (~0.26 g)
        double MAX_WHEEL_ACCEL = 120;   // in/s^2 per wheel, shared between driving, strafing, and turning
        double MAX_LATERAL_ACCEL = 150; // in/s^2 cornering grip before the mecanum slides off the path
        MecanumKinematics robotKinematics = new MecanumKinematics(TRACK_WIDTH, LATERAL_MULT);

        // Slow down in tight curves: v <= sqrt(a_lat / curvature). Without this the curve into the hive
        // asks for ~0.85 g of sideways grip at full speed.
        VelConstraint cornerVel = (robotPose, posePath, disp) -> {
            double curvature = posePath.get(disp, 3).position.drop(2).value().norm();
            return curvature < 1e-6 ? 1e9 : Math.sqrt(MAX_LATERAL_ACCEL / curvature);
        };
        // Wheel speed covers turning too (omega * track width), so no separate angular cap is needed:
        // the robot spins as fast as the wheels allow (~265 deg/s).
        // Slow down where the turn rate changes quickly (SplineHeading): that term is v^2 * d2(heading)/ds2 and
        // the accel limit can't see it, so without this a spline-heading spin asks for ~4x the wheel accel.
        VelConstraint turnRateVel = (robotPose, posePath, disp) -> {
            Rotation2dDual<Arclength> heading = posePath.get(disp, 3).heading;
            double headingCurvature = Math.abs(heading.real.get(0) * heading.imag.get(2) - heading.imag.get(0) * heading.real.get(2));
            return headingCurvature < 1e-9 ? 1e9 : Math.sqrt(MAX_WHEEL_ACCEL / (headingCurvature * TRACK_WIDTH));
        };
        VelConstraint maxVel = new MinVelConstraint(Arrays.asList(
                robotKinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                cornerVel,
                turnRateVel
        ));
        // Intake slow zone is only the corner now (same y < -42 region as intakeAccel). With the start at
        // x = 61, intakeVel's x > 50 check kept the WHOLE run down the wall at 15 in/s.
        double INTAKE_ZONE_Y = -42; // slow below this y (and x > 50)
        double INTAKE_VEL = 15;     // in/s while intaking
        VelConstraint cornerIntakeVel = (robotPose, _path, _disp) -> {
            boolean inIntakeZone = robotPose.position.x.value() > 50.0 && robotPose.position.y.value() < INTAKE_ZONE_Y;
            return inIntakeZone ? INTAKE_VEL : 60;
        };
        VelConstraint maxIntakeVel = new MinVelConstraint(Arrays.asList(cornerIntakeVel, maxVel));

        // Accel limit that knows how hard the wheels are working. Straight driving gets the full
        // MAX_PATH_ACCEL; strafing (lateral multiplier) and turning (heading change per inch * track width)
        // make each inch of path cost more wheel travel, so accel drops to keep the wheels at MAX_WHEEL_ACCEL.
        // A flat ProfileAccelConstraint lets the 180 deg spins demand ~10x the wheel accel the motors have.
        AccelConstraint maxAccel = (robotPose, posePath, disp) -> {
            Vector2d travel = robotPose.heading.value().inverse().times(robotPose.position.drop(1).value());
            double turnPerInch = Math.abs(robotPose.heading.velocity().value());
            double wheelPerInch = Math.abs(travel.x) + LATERAL_MULT * Math.abs(travel.y) + turnPerInch * TRACK_WIDTH;
            double accel = Math.min(MAX_PATH_ACCEL, MAX_WHEEL_ACCEL / wheelPerInch);
            return new MinMax(-accel, accel);
        };

        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -14, Math.toRadians(180)))
                    .waitSeconds(2.5)
                    .setTangent(Math.toRadians(-155)) // leave down-and-in instead of straight at the center: shorter loop
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-52, -16, Math.toRadians(360)), Math.toRadians(-220), maxVel, maxAccel)
                    .waitSeconds(1.5)
                // Flower: turn first, then move, as ONE connected trajectory (no stop between turn and move).
                // SplineHeading -> ConstantHeading is the only pairing Road Runner joins without stopping; a
                // LinearHeading turn has a turn-rate jump at the join, which forces a stop. The turn finishes in
                // the first ~30% of the move (shorter splits spike the wheels). Turns through -90 as before.
                    .setTangent(Math.toRadians(-172))
                .splineToSplineHeading(new Pose2d(-52.6, -17.5, Math.toRadians(180)), Math.toRadians(-112), maxVel, maxAccel)
                .splineToConstantHeading(new Vector2d(-55, -22), Math.toRadians(-82), maxVel, maxAccel)
                    .waitSeconds(5)
                // Back to shoot: move first, then turn over the last ~30%, again one connected trajectory.
                    .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(360)), Math.toRadians(90), maxVel, maxAccel)
                    .waitSeconds(2)
                // Park facing forward (90 deg, red perspective). One straight strafe with the 90 deg turn spread
                // over the whole drive: ~0.4 s faster than turning first/last or in a separate stage.
                .strafeToLinearHeading(new Vector2d(-50, -57), Math.toRadians(90), maxVel, maxAccel)
                .build());*/

        // Red side (Hive-down side start)
        // Field (BIOBUZZ manual TU02/TU03 + Event Field Setup Guide): red AUTO side is y < 0 (G402), so the robot
        // body stays below y = 0. Red HIVE CELLS sit either side of the 49.46 x 38.95 in frame at the center; the
        // audience-side (x > 0) CELL starts up and each TIP flips which CELL is up, so shots alternate sides.
        // FLOWER next to the start is on the left wall at about (-70, -24).
        // 3-TIP, alliance-preload compatible (mirrors Biobuzz3Tip_AlliancePreLoad_Compatible). Waits are the real
        // action times, so this timeline matches the robot: 29.5 s of the 30 s AUTO. Partner parks at (-32, -58).
        // Two 180-degree turns aim 0.5-1 degree off on purpose: an exact 180 is a tie Road Runner breaks on
        // floating-point rounding, which can flip the turn direction (and its time) when an earlier path changes.
        sampleBot2.runAction(sampleBot2.getDrive().actionBuilder(new Pose2d(-65, -11, Math.toRadians(0)))
                // Reposition to the left shooting pose DURING the 5 s wait (pull off the wall first so the bottom
                // edge stays clear of FLOWER 1), so the preload goes at 5.0 s and FLOWER 1 is a short hop away
                    .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel)
                    .waitSeconds(4.2) // preload shot starts at 5.02 s, after the partner's preload TIP
                    .waitSeconds(1)   // transfer: shoot preloads, left CELL
                // FLOWER 1: one connected spline, turning through -90 (1.20 s; the old U-turn off the wall was 1.98 s)
                    .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(-55, -22, Math.toRadians(-179.5)), Math.toRadians(210), maxVel, maxAccel)
                    .waitSeconds(3)   // collectFromFlower(3), starts on arrival
                // Back to the left shooting pose turning through +90, swinging away from the FLOWER (1.29 s, was 1.61 s)
                    .setTangent(Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(165), maxVel, maxAccel)
                    .waitSeconds(1)   // transfer: shoot FLOWER 1 pollen, left CELL (TIP -> right CELL up)
                // FLOWER 2 (red alliance wall at x = 24): under the HIVE frame (2.8 in) above the parked partner
                // (2.8 in), arriving head-on
                    .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(-10)), Math.toRadians(0), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(24, -55, Math.toRadians(-90)), Math.toRadians(-90), maxVel, maxAccel)
                    .waitSeconds(3)   // collectFromFlower(3), starts on arrival
                // Back off the wall to the right shooting pose, turning -90 -> -179.5
                    .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-179.5)), Math.toRadians(15), maxVel, maxAccel)
                    .waitSeconds(1)   // transfer: shoot FLOWER 2 pollen, right CELL (TIP -> left CELL up)
                // Corner GARDEN pollen, same path as before. collect(2.5) starts 2.1 s into this spline: 0.4 s before
                // the first pollen contact (2.52 s), through the last contact (2.93 s), and 1.7 s past it.
                    .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .setReversed(true)
                // Run to the left shooting pose along the bottom of the frame (3.1 in), clearing the partner (3.0 in)
                .splineToLinearHeading(new Pose2d(20, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-24, -37, Math.toRadians(360)), Math.toRadians(180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-52, -16, Math.toRadians(360)), Math.toRadians(120), maxVel, maxAccel)
                    .waitSeconds(1)   // transfer: shoot corner pollen, left CELL
                // Park facing forward, partly in the LOADING ZONE, 3 in from the partner: square past the FLOWER, then turn
                    .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50, -26, Math.toRadians(20)), Math.toRadians(-90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-53.5, -57, Math.toRadians(90)), Math.toRadians(-90), maxVel, maxAccel)
                .build());


        // Blue side
        /*sampleBot2.runAction(sampleBot2.getDrive().actionBuilder(new Pose2d(5, 12, Math.toRadians(180)))
                .lineToX(55)
                    .waitSeconds(2)
                    .setReversed(false)
                .splineToLinearHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), intakeVel)
                    .setReversed(true)
                .splineToLinearHeading(new Pose2d(-26, -40, Math.toRadians(360)), Math.toRadians(170), new TranslationalVelConstraint(55))
                .splineToLinearHeading(new Pose2d(-54, -12, Math.toRadians(360)), Math.toRadians(-220), new TranslationalVelConstraint(45))
                    .waitSeconds(2)
                .lineToXLinearHeading(-58, Math.toRadians(180))
                    .waitSeconds(5)
                .lineToXLinearHeading(-54, Math.toRadians(360))
                    .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(-38, -55), new TranslationalVelConstraint(60))
                //.splineToLinearHeading(new Pose2d(-35, -63, Math.toRadians(-90)), Math.toRadians(-90))
                                //.splineToLinearHeading()
                //.splineToSplineHeading()
                .build());*/

        // Load the 2026-27 BioBuzz field image from disk.
        // >>> Change this path to wherever you saved the BioBuzz field PNG. <<<
        //   Windows: "C:\\Users\\YourName\\Documents\\biobuzz_field.png"
        //   Mac:     "/Users/YourName/Documents/biobuzz_field.png"
        Image biobuzzField = null;
        try {
            biobuzzField = ImageIO.read(new File("C:\\Users\\nikmu\\Downloads\\biobuzz_field.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setBackground(biobuzzField)
                .addEntity(sampleBot2)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(sampleBot)
                .start();
    }
}
