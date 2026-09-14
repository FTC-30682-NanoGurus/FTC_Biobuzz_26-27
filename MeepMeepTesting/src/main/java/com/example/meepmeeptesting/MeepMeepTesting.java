package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity sampleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0,0,100);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0,0,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0,0,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255,255,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0,100,100);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0,155,200);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0,0,100);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0,0,185);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0,50,255);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0,50,255);
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
                        return  new Color(0,80,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0,80,255);
                    }

                })
                .build();
        RoadRunnerBotEntity sampleBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0,100,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0,0,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0,255,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255,255,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0,100,20);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0,155,0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0,50,0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0, 100, 0);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0,200,50);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0,200,50);
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
                        return  new Color(0,255,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0,255,0);
                    }

                })
                .build();
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10,50);
            } else {
                return new MinMax(-30,50);
            }
        };
        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-20,80);
            } else {
                return new MinMax(-30,80);
            }
        };
        AccelConstraint pickSampleAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10,22);
            } else {
                return new MinMax(-30,50);
            }
        };
        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 20;
            } else {
                return 50;
            }
        };



        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-32.5, -64 , Math.toRadians(0)))
                .setReversed(true)
//                .afterTime(0.5, new InstantAction(() -> intake.moveWrist(RobotConstants.floor_pickup_position)))
                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-39, -60), Math.toRadians(135))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(24), new ProfileAccelConstraint(-10,40))
                .setReversed(false)
                .setTangent(Math.toRadians(45))

                .splineToLinearHeading(new Pose2d(-48, -52.5, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(40), new ProfileAccelConstraint(-12,32))
                .setReversed(true)

                .setTangent(Math.PI + Math.atan2( -52.5 - basket.position.y,-48 - basket.position.x))

                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40))
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-59, -52.5, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32))
                .setReversed(true)

                .setTangent(Math.PI + Math.atan2( -52.5 - basket.position.y,-59 - basket.position.x))

                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40))
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-58, -47, Math.toRadians(118.5)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32))
                .setReversed(true)
                .setTangent(Math.toRadians(298.5))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40))
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-42, -45, Math.toRadians(0)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(80))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(basket, Math.toRadians(225))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-15,50))

                .build());
        sampleBot2.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-10, -66, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-2,-45, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-2,-33), Math.toRadians(90))
                .setReversed(false)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-48,-52, Math.toRadians(90)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10,50))

                .splineToLinearHeading(basket, Math.toRadians(225   ), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40))
                .setReversed(false)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-59, -52.5, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32))
                .setReversed(true)

                .setTangent(Math.PI + Math.atan2( -52.5 - basket.position.y,-59 - basket.position.x))

                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40))
                .setReversed(false)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-58, -47, Math.toRadians(118.5)), Math.toRadians(90), new TranslationalVelConstraint(40),  new ProfileAccelConstraint(-12,32))
                .setReversed(true)
                .setTangent(Math.toRadians(298.5))
                .splineToLinearHeading(basket, Math.toRadians(225), new TranslationalVelConstraint(36), new ProfileAccelConstraint(-10,40))
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(80))

                .build());
        Pose2d firstSample = new Pose2d(31, -34, Math.toRadians(30));
        Pose2d secondSample = new Pose2d(41, -34, Math.toRadians(30));
        Pose2d thirdSample = new Pose2d(48, -34, Math.toRadians(30));
        Pose2d pickupPosition = new Pose2d(40, -63,  Math.toRadians(270));

        specimenBot.runAction(specimenBot.getDrive().actionBuilder(new Pose2d(10, -64 , Math.toRadians(0)))
                        .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(4,-45, Math.toRadians(270)), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(4,-34), Math.toRadians(90), null, smartScore)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(firstSample, Math.toRadians(30), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15, 35))
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(firstSample.position.x, -37, Math.toRadians(-45)), Math.toRadians(0), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 25))

                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(secondSample, Math.toRadians(30), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15, 35))
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(pickupPosition,Math.toRadians(270),new TranslationalVelConstraint(20), new ProfileAccelConstraint(-8, 25))
                        .setTangent(Math.toRadians(170))
//                        .splineToConstantHeading(new Vector2d(20,-56),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6, -34), Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(40, -63), Math.toRadians(0))

                .setTangent(Math.toRadians(180))

                .splineToConstantHeading(new Vector2d(8, -34), Math.toRadians(90))
                .setTangent(Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(40, -63), Math.toRadians(0))
                .setTangent(Math.toRadians(180))

                .splineToConstantHeading(new Vector2d(10, -34), Math.toRadians(90))
                .setTangent(Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(40, -63), Math.toRadians(0))
                .setTangent(Math.toRadians(180))

                .splineToConstantHeading(new Vector2d(12, -34), Math.toRadians(90))
                .setTangent(Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(40, -63), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(sampleBot)
                .addEntity(sampleBot2)
                .addEntity(specimenBot)
                .start();

    }


}