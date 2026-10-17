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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
    // Head-on FLOWER approach, as in the autos: BiobuzzRobotConstants.flowerApproachIn and
    // HiveAutoKit.flowerStaging / approachFlower. Keep in sync with those.
    static final double FLOWER_APPROACH_IN = 6.0;
    // Leaving a FLOWER: straight back this far with the ramp still down, then the ramp goes up
    // (BiobuzzRobotConstants.flowerRampClearIn, HiveAutoKit.departFlower / leaveFlower).
    static final double FLOWER_RAMP_CLEAR_IN = 8.0;

    // true: the old paths (before the head-on FLOWER approach) run too, so the two can be compared.
    //   2 TIP: old = white bot, new = orange bot.   3 TIP: old = green bot, new = red bot.
    static final boolean SHOW_OLD_PATHS = true;

    /** The pose FLOWER_APPROACH_IN straight back from a FLOWER intake pose, facing the FLOWER. */
    static Pose2d flowerStaging(Pose2d flower) {
        double h = flower.heading.toDouble();
        return new Pose2d(flower.position.x - FLOWER_APPROACH_IN * Math.cos(h),
                flower.position.y - FLOWER_APPROACH_IN * Math.sin(h), h);
    }

    /** To the staging pose already facing and moving at the FLOWER, then straight in, without stopping. */
    static TrajectoryActionBuilder approachFlower(TrajectoryActionBuilder b, Pose2d flower, VelConstraint vel, AccelConstraint accel) {
        double h = flower.heading.toDouble();
        return b.splineToSplineHeading(flowerStaging(flower), h, vel, accel)
                .splineToConstantHeading(flower.position, h, vel, accel);
    }

    /** Straight back out of a FLOWER, still moving, so the next spline carries on without stopping. */
    static TrajectoryActionBuilder departFlower(TrajectoryActionBuilder b, Pose2d flower, VelConstraint vel, AccelConstraint accel) {
        double h = flower.heading.toDouble(), back = h + Math.PI;
        Vector2d out = new Vector2d(flower.position.x - FLOWER_RAMP_CLEAR_IN * Math.cos(h), flower.position.y - FLOWER_RAMP_CLEAR_IN * Math.sin(h));
        return b.setTangent(back).splineToConstantHeading(out, back, vel, accel);
    }

    /** Heading of the straight line from a pose to a FLOWER's staging pose: RouteOptimizer's start tangent. */
    static double tangentToStaging(Pose2d from, Pose2d flower) {
        Vector2d s = flowerStaging(flower).position;
        return Math.atan2(s.y - from.position.y, s.x - from.position.x);
    }

    static ColorScheme scheme(Color bg, Color body, Color wheel, Color path) {
        return new ColorScheme() {
            @NotNull @Override public Color getUI_MAIN_BG() { return bg; }
            @NotNull @Override public Color getTRAJECTORY_TEXT_COLOR() { return new Color(0, 0, 0); }
            @NotNull @Override public Color getTRAJECTORY_SLIDER_FG() { return path; }
            @NotNull @Override public Color getTRAJECTORY_SLIDER_BG() { return new Color(255, 255, 255); }
            @NotNull @Override public Color getTRAJECTORY_MARKER_COLOR() { return wheel; }
            @Override public boolean isDark() { return false; }
            @NotNull @Override public Color getBOT_BODY_COLOR() { return body; }
            @NotNull @Override public Color getBOT_WHEEL_COLOR() { return wheel; }
            @NotNull @Override public Color getBOT_DIRECTION_COLOR() { return wheel; }
            @NotNull @Override public Color getAXIS_X_COLOR() { return path; }
            @NotNull @Override public Color getAXIS_Y_COLOR() { return path; }
            @Override public double getAXIS_NORMAL_OPACITY() { return 0.7; }
            @Override public double getAXIS_HOVER_OPACITY() { return 0.3; }
            @NotNull @Override public Color getTRAJECTORY_PATH_COLOR() { return path; }
            @NotNull @Override public Color getTRAJECTORY_TURN_COLOR() { return path; }
        };
    }

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

        RoadRunnerBotEntity twoTipNewBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(scheme(new Color(100, 50, 0), new Color(255, 140, 0), new Color(120, 60, 0), new Color(255, 150, 0)))
                .build();

        RoadRunnerBotEntity threeTipNewBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(scheme(new Color(100, 0, 0), new Color(220, 30, 30), new Color(100, 0, 0), new Color(255, 40, 40)))
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
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -12, Math.toRadians(180)))
                        .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(55, -25, Math.toRadians(180)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-32, -58, Math.toRadians(180)), Math.toRadians(190))
                .build());*/

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

//        // Red side (Hive-up side start) -- 2 TIP, current speed-optimised version (Biobuzz_2_Tip_HiveAuto,
//        // nominal run: the preload tips the RIGHT CELL, so the wall set and flower one go LEFT).
//        // Wall set runs reversed (heading 360 while travelling -x = driving straight BACKWARD to the left side).
//        // Park is the route RouteOptimizer picks: one smooth spline with the 90 deg turn spread over the drive
//        // (1.71 s vs 1.78 s for the old strafe).
//        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -14, Math.toRadians(180)))
//                    .waitSeconds(2)   // preload volley into the RIGHT CELL (+ up to 1 s more if its tip has not started)
//                    .setTangent(Math.toRadians(-155))
//                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
//                    .setReversed(true)
//                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
//                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
//                .splineToLinearHeading(new Pose2d(-52, -16, Math.toRadians(360)), Math.toRadians(-220), maxVel, maxAccel)
//                    .waitSeconds(1)   // shot point LEFT: wall-set volley
//                    .setTangent(Math.toRadians(-172))
//                .splineToSplineHeading(new Pose2d(-52.6, -17.5, Math.toRadians(180)), Math.toRadians(-112), maxVel, maxAccel)
//                .splineToConstantHeading(new Vector2d(-55, -22), Math.toRadians(-82), maxVel, maxAccel)
//                    .waitSeconds(3.4) // collectFromFlower(3), starting 0.4 s after arriving (ramp deployed on the way)
//                    .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
//                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(360)), Math.toRadians(90), maxVel, maxAccel)
//                    .waitSeconds(1)   // shot point LEFT: flower-one volley (second TIP)
//                    .setTangent(Math.atan2(-57 - -14, -50 - -52))
//                .splineToSplineHeading(new Pose2d(-50, -57, Math.toRadians(90)), Math.atan2(-57 - -14, -50 - -52), maxVel, maxAccel)
//                .build());

        // Red side (Hive-up side start) -- 2 TIP (Biobuzz_2_Tip_HiveAuto), PRELOAD TIPS (normal run).
        // OLD PATH (white bot), before the head-on FLOWER approach. The NEW path (orange bot) follows it.
        // Wall set is shot at an ANGLE at the end of the wall-set run instead of driving up to (-52, -16):
        // (-53.48, -43.82) heading 30 = 49.05 in from the LEFT CELL opening (same distance as the proven shot,
        // so 1600 tps holds), 30 deg off its facing, aimed at its centre. Flower one, back to (-52, -14) for the
        // 8th pollen (TIP 2), then FLOWER 2 as a backup: if any pollen missed, its 4 make the LEFT CELL tip; if
        // TIP 2 already happened the robot holds them. Park 6 in deep in the LOADING ZONE. Parked ~26.5 s.
        Pose2d leftNear2 = new Pose2d(-53.48, -43.82, Math.toRadians(30));
        Pose2d flowerOne2 = new Pose2d(-55, -22, Math.toRadians(180));
        Pose2d flowerTwo2 = new Pose2d(24, -55, Math.toRadians(-90));
        Vector2d park2 = new Vector2d(-49.5, -58);
        double laToF1 = Math.atan2(flowerOne2.position.y - leftNear2.position.y, flowerOne2.position.x - leftNear2.position.x);
        double f2ToLa = Math.atan2(leftNear2.position.y - flowerTwo2.position.y, leftNear2.position.x - flowerTwo2.position.x);
        double laToPark = Math.atan2(park2.y - leftNear2.position.y, park2.x - leftNear2.position.x);
        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -14, Math.toRadians(180)))
                    .waitSeconds(2)    // preload volley into the RIGHT CELL (TIP 1)
                    .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .waitSeconds(0.25) // let the last GARDEN pollen in; rollers run 2.1 s in -> arrival (3.7 s)
                    .setReversed(true) // drive straight BACKWARD along the bottom of the field
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToSplineHeading(leftNear2, Math.toRadians(180), maxVel, maxAccel)
                    .waitSeconds(1)    // angled wall-set volley, LEFT CELL
                    .setTangent(laToF1)
                .splineToSplineHeading(flowerOne2, laToF1, maxVel, maxAccel)
                    .waitSeconds(3.4)  // collectFromFlower(3), 0.4 s after arriving
                    .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-54, -16.1), Math.toRadians(90), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(360)), Math.toRadians(90), maxVel, maxAccel)
                    .waitSeconds(1)    // FLOWER 1 volley: 8th pollen -> TIP 2
                    .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(-10)), Math.toRadians(0), maxVel, maxAccel)
                .splineToSplineHeading(flowerTwo2, Math.toRadians(-90), maxVel, maxAccel)
                    .waitSeconds(3)    // collectFromFlower(3)
                    .setTangent(f2ToLa) // back to the angled pose driving BACKWARD (heading ~-8 on the long run)
                .splineToSplineHeading(new Pose2d(new Vector2d((flowerTwo2.position.x + leftNear2.position.x) / 2,
                        (flowerTwo2.position.y + leftNear2.position.y) / 2), f2ToLa - Math.PI), f2ToLa, maxVel, maxAccel)
                .splineToSplineHeading(leftNear2, f2ToLa, maxVel, maxAccel)
                    .waitSeconds(1)    // backup volley (only if the LEFT CELL is still up)
                    .setTangent(laToPark)
                .splineToSplineHeading(new Pose2d(park2, Math.toRadians(90)), laToPark, maxVel, maxAccel)
                .build());

        // 2 TIP, NEW PATH (orange bot) - exactly what Biobuzz_2_Tip_HiveAuto now drives, same run as above.
        // Both FLOWER legs end on the staging pose FLOWER_APPROACH_IN in out from the FLOWER, already facing it,
        // then drive straight in without stopping; the ramp is lowered when each leg starts.
        //   FLOWER 1: kit.flowerRoute(leftNear, flowerOne) - RouteOptimizer picks route M: (1.94 s, was 1.47 s).
        //   FLOWER 2: waypoint moved from (-4, -38, -10) to (0, -41, -15) so the turn to -90 finishes before the
        //             straight run (2.90 s, was 2.61 s), with the same 4.3 in HIVE gap.
        // Every leg away from a FLOWER backs straight out FLOWER_RAMP_CLEAR_IN first with the ramp still down, and
        // the ramp goes up once clear, so it never swings up into the FLOWER. Parked ~1.3 s later than the old path.
        TrajectoryActionBuilder twoTip = twoTipNewBot.getDrive().actionBuilder(new Pose2d(61, -14, Math.toRadians(180)))
                    .waitSeconds(2)    // preload volley into the RIGHT CELL (TIP 1)
                    .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .waitSeconds(0.25)
                    .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, -48, Math.toRadians(360)), Math.toRadians(-180), maxVel, maxAccel)
                .splineToLinearHeading(new Pose2d(-33, -44, Math.toRadians(360)), Math.toRadians(170), maxVel, maxAccel)
                .splineToSplineHeading(leftNear2, Math.toRadians(180), maxVel, maxAccel)
                    .waitSeconds(1)    // angled wall-set volley, LEFT CELL
                    .setTangent(tangentToStaging(leftNear2, flowerOne2));
        twoTip = approachFlower(twoTip, flowerOne2, maxVel, maxAccel)
                    .waitSeconds(3.4); // SleepAction(0.4) + collectFromFlower(3)
        // Leave FLOWER 1 straight back 8 in (ramp still down), ramp up, then on to the shot (1.71 s, was 1.66 s).
        twoTip = departFlower(twoTip, flowerOne2, maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(155), maxVel, maxAccel)
                    .waitSeconds(1)    // FLOWER 1 volley: 8th pollen -> TIP 2
                    .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -41, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel);
        twoTip = approachFlower(twoTip, flowerTwo2, maxVel, maxAccel)
                    .waitSeconds(3)    // collectFromFlower(3)
                // kit.routeFromFlower(flowerTwo, leftNear): straight back 8 in to (24, -47) and stop there (ramp up),
                // then RouteOptimizer route M: (3.09 s in all, was 2.57 s)
                .strafeToConstantHeading(new Vector2d(24, -47), maxVel, maxAccel)
                    .setTangent(Math.toRadians(177.65))
                .splineToSplineHeading(new Pose2d(-14.74, -45.41, Math.toRadians(-2.35)), Math.toRadians(177.65), maxVel, maxAccel)
                .splineToSplineHeading(leftNear2, Math.toRadians(177.65), maxVel, maxAccel)
                    .waitSeconds(1)    // backup volley (only if the LEFT CELL is still up)
                    .setTangent(laToPark)
                .splineToSplineHeading(new Pose2d(park2, Math.toRadians(90)), laToPark, maxVel, maxAccel);
        twoTipNewBot.runAction(twoTip.build());

        // Same auto, PRELOAD DOES NOT TIP: wall set is shot at the RIGHT CELL from the angled pose near the corner
        // (56.69, -40.01, 155) - that is TIP 1 - then FLOWER 2 and FLOWER 1 both go LEFT (4 + 4 = 8 -> TIP 2).
        // Parked ~26.2 s before the head-on FLOWER approach (both FLOWER legs are now RouteOptimizer routes M:
        // through the staging pose: FLOWER 2 1.98 s, was 1.71 s; FLOWER 1 1.94 s, was 1.47 s).
        // Swap it in for the NEW run above to see it.
        /*Pose2d rightNear2 = new Pose2d(56.69, -40.01, Math.toRadians(155));
        double lToPark = Math.atan2(park2.y - -14, park2.x - -52);
        TrajectoryActionBuilder noTip = twoTipNewBot.getDrive().actionBuilder(new Pose2d(61, -14, Math.toRadians(180)))
                    .waitSeconds(3)    // preload volley + 1 s waiting for a tip that does not start
                    .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .waitSeconds(0.25)
                    .setTangent(Math.atan2(10, -4))
                .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                .splineToSplineHeading(rightNear2, Math.atan2(rightNear2.position.y + 48, rightNear2.position.x - 58), maxVel, maxAccel)
                    .waitSeconds(1)    // angled wall-set volley, RIGHT CELL -> TIP 1
                    .setTangent(tangentToStaging(rightNear2, flowerTwo2));
        noTip = approachFlower(noTip, flowerTwo2, maxVel, maxAccel)
                    .waitSeconds(3)    // collectFromFlower(3)
                .strafeToConstantHeading(new Vector2d(24, -47), maxVel, maxAccel)   // routeFromFlower: back out 8 in, ramp up
                    .setTangent(Math.toRadians(177.65))
                .splineToSplineHeading(new Pose2d(-14.74, -45.41, Math.toRadians(-2.35)), Math.toRadians(177.65), maxVel, maxAccel)
                .splineToSplineHeading(leftNear2, Math.toRadians(177.65), maxVel, maxAccel)
                    .waitSeconds(1)    // FLOWER 2 volley, LEFT CELL
                    .setTangent(tangentToStaging(leftNear2, flowerOne2));
        noTip = approachFlower(noTip, flowerOne2, maxVel, maxAccel)
                    .waitSeconds(3.4);
        noTip = departFlower(noTip, flowerOne2, maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(155), maxVel, maxAccel)
                    .waitSeconds(1)    // FLOWER 1 volley: 8th pollen -> TIP 2
                    .setTangent(lToPark)
                .splineToSplineHeading(new Pose2d(park2, Math.toRadians(450)), lToPark, maxVel, maxAccel);
        twoTipNewBot.runAction(noTip.build());*/


        // Red side (Hive-down side start)
        // Field (BIOBUZZ manual TU02/TU03 + Event Field Setup Guide): red AUTO side is y < 0 (G402), so the robot
        // body stays below y = 0. Red HIVE CELLS sit either side of the 49.46 x 38.95 in frame at the center; the
        // audience-side (x > 0) CELL starts up and each TIP flips which CELL is up, so shots alternate sides.
        // FLOWER next to the start is on the left wall at about (-70, -24).
        // 3-TIP, alliance-preload compatible (mirrors Biobuzz3Tip_AlliancePreLoad_Compatible). Waits are the real
        // action times, so this timeline matches the robot: 29.5 s of the 30 s AUTO. Partner parks at (-32, -58).
        // Two 180-degree turns aim 0.5-1 degree off on purpose: an exact 180 is a tie Road Runner breaks on
        // floating-point rounding, which can flip the turn direction (and its time) when an earlier path changes.
        /*sampleBot2.runAction(sampleBot2.getDrive().actionBuilder(new Pose2d(-65, -11, Math.toRadians(0)))
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
                .build());*/

        // OLD PATH (green bot), before the head-on FLOWER approach. The NEW path (red bot) follows it.
        // 3-TIP, HIVE-detection version (Biobuzz3Tip_HiveAuto), nominal run with the alliance wait at its
        // 4.2 s maximum. Same as the old path up to the right shooting pose; then the wall set is shot from an
        // ANGLED pose instead of driving back up to (61, -14):
        //   - the angled pose is 49.05 in from the RIGHT CELL opening centre (12.232, -19.277) - exactly the
        //     distance of the proven (61, -14) shot, so the fixed 1600 tps flywheel speed still fits - but 25 deg
        //     round from the opening's facing toward the wall set: (56.69, -40.01), heading 155 = aimed dead at
        //     the opening centre (the opening still presents 18.1 of its 20 in);
        //   - off the wall it swings out through (58, -48) first, so the 165 deg turn never brings a corner of the
        //     15 x 18 robot within 1 in of the wall;
        //   - park: partly in the LOADING ZONE on its nearer (right) side, 6 in deep in x and y, heading 180,
        //     clear of a partner parked left of x = -31. RouteOptimizer route M: (2.24 s).
        // Timeline: right-pose volley ends 20.96, wall set 25.65, volley 26.65, parked 28.89 s.
        Pose2d nearShot3 = new Pose2d(56.69, -40.01, Math.toRadians(155));
        Vector2d parkClose = new Vector2d(-21, -59.5);
        double toPark = Math.atan2(parkClose.y - nearShot3.position.y, parkClose.x - nearShot3.position.x);
        sampleBot2.runAction(sampleBot2.getDrive().actionBuilder(new Pose2d(-65, -11, Math.toRadians(0)))
                    .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel)
                    .waitSeconds(4.2) // alliance wait (ends early once the LEFT CELL is up and at rest)
                    .waitSeconds(1)   // preload volley, LEFT CELL
                    .setTangent(Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(-55, -22, Math.toRadians(-179.5)), Math.toRadians(210), maxVel, maxAccel)
                    .waitSeconds(3)   // collectFromFlower(3) (ramp deployed during the turn)
                    .setTangent(Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(165), maxVel, maxAccel)
                    .waitSeconds(1)   // shot point LEFT: FLOWER 1 volley (8th pollen -> TIP, RIGHT CELL up)
                    .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(-10)), Math.toRadians(0), maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(24, -55, Math.toRadians(-90)), Math.toRadians(-90), maxVel, maxAccel)
                    .waitSeconds(3)   // collectFromFlower(3)
                    .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-179.5)), Math.toRadians(15), maxVel, maxAccel)
                    .waitSeconds(1)   // shot point RIGHT: FLOWER 2 volley
                    .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .waitSeconds(0.25) // let the last GARDEN pollen in (collect(2.5) runs 2.1 -> 4.6 s into this leg)
                    .setTangent(Math.atan2(10, -4))
                .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                .splineToSplineHeading(nearShot3, Math.atan2(nearShot3.position.y + 48, nearShot3.position.x - 58), maxVel, maxAccel)
                    .waitSeconds(1)   // angled volley into the RIGHT CELL (8th pollen -> third TIP)
                    .setTangent(toPark)
                .splineToSplineHeading(new Pose2d(new Vector2d((nearShot3.position.x + parkClose.x) / 2, (nearShot3.position.y + parkClose.y) / 2),
                        toPark + Math.toRadians(360)), toPark, maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(parkClose, Math.toRadians(180)), toPark, maxVel, maxAccel)
                .build());

        // 3 TIP, NEW PATH (red bot) - exactly what Biobuzz3Tip_HiveAuto now drives, same nominal run as above.
        //   FLOWER 1: start tangent -30 -> -32, then the staging pose 6 in out and straight in (1.60 s, was 1.20 s);
        //             the ramp is lowered at the start of the leg, 1.22 s before the straight run.
        //   FLOWER 2: waypoint (-4, -38, -10) -> (0, -41, -15), ramp lowered there, then staging + straight in
        //             (2.90 s, was 2.61 s). Same 4.3 in HIVE gap.
        //   Leaving FLOWER 1: straight back 8 in with the ramp down, ramp up, then to the shot (1.71 s, was 1.28 s).
        //   Leaving FLOWER 2 already backs straight out 8 in, so that leg is unchanged.
        // Nominal timeline ~1.1 s later: parked ~30.0 s at the longest alliance wait (was 28.89). On the robot the
        // budget checks will drop or shorten the wall set when the alliance wait runs long; the park watchdog still parks.
        Pose2d flowerOne3 = new Pose2d(-55, -22, Math.toRadians(-179.5));
        Pose2d flowerTwo3 = new Pose2d(24, -55, Math.toRadians(-90));
        TrajectoryActionBuilder threeTip = threeTipNewBot.getDrive().actionBuilder(new Pose2d(-65, -11, Math.toRadians(0)))
                    .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(-75), maxVel, maxAccel)
                    .waitSeconds(4.2) // alliance wait (ends early once the LEFT CELL is up and at rest)
                    .waitSeconds(1)   // preload volley, LEFT CELL
                    .setTangent(Math.toRadians(-32));
        threeTip = approachFlower(threeTip, flowerOne3, maxVel, maxAccel)
                    .waitSeconds(3);  // collectFromFlower(3)
        // Leave FLOWER 1 straight back 8 in (ramp still down), ramp up, then on to the shot (1.71 s, was 1.28 s).
        threeTip = departFlower(threeTip, flowerOne3, maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(1)), Math.toRadians(155), maxVel, maxAccel)
                    .waitSeconds(1)   // shot point LEFT: FLOWER 1 volley (8th pollen -> TIP, RIGHT CELL up)
                    .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -41, Math.toRadians(-15)), Math.toRadians(-15), maxVel, maxAccel);
        threeTip = approachFlower(threeTip, flowerTwo3, maxVel, maxAccel)
                    .waitSeconds(3)   // collectFromFlower(3)
                    .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-179.5)), Math.toRadians(15), maxVel, maxAccel)
                    .waitSeconds(1)   // shot point RIGHT: FLOWER 2 volley
                    .setTangent(Math.toRadians(-155))
                .splineToSplineHeading(new Pose2d(62, -58, Math.toRadians(360)), Math.toRadians(25), maxIntakeVel, maxAccel)
                    .waitSeconds(0.25)
                    .setTangent(Math.atan2(10, -4))
                .splineToSplineHeading(new Pose2d(58, -48, Math.toRadians(30)), Math.toRadians(105.57), maxVel, maxAccel)
                .splineToSplineHeading(nearShot3, Math.atan2(nearShot3.position.y + 48, nearShot3.position.x - 58), maxVel, maxAccel)
                    .waitSeconds(1)   // angled volley into the RIGHT CELL (8th pollen -> third TIP)
                    .setTangent(toPark)
                .splineToSplineHeading(new Pose2d(new Vector2d((nearShot3.position.x + parkClose.x) / 2, (nearShot3.position.y + parkClose.y) / 2),
                        toPark + Math.toRadians(360)), toPark, maxVel, maxAccel)
                .splineToSplineHeading(new Pose2d(parkClose, Math.toRadians(180)), toPark, maxVel, maxAccel);
        threeTipNewBot.runAction(threeTip.build());


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
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f);
        if (SHOW_OLD_PATHS) meepMeep.addEntity(sampleBot2).addEntity(sampleBot);
        meepMeep.addEntity(threeTipNewBot)
                .addEntity(twoTipNewBot)
                .start();
    }
}
