package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;

import javax.swing.ProgressMonitor;

public class  MeepMeepTestingNikhil {
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
                return new MinMax(-10,50);
        };
        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < -30.0) {
                return new MinMax(-5,5);
            } else {
                return new MinMax(-120,120);
            }
        };
        AccelConstraint intakeAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10,22);
            } else {
                return new MinMax(-30,50);
            }
        };
        VelConstraint intakeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 20;
            } else {
                return 50;
            }
        };
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(64, -36 , Math.toRadians(-90)))
                .lineToY(-5)
                .setReversed(true)
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.2)
                //.lineToX(-35)
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-60, -48.5, Math.toRadians(70)), Math.toRadians(240))
                        //.turn(-0.2)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.2)
                //.lineToX(-35)
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-60, -48.5, Math.toRadians(70)), Math.toRadians(240))
                        //.turn(-0.2)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-29, -11, Math.toRadians(0)), Math.toRadians(0))
                .build());
         */
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-56, -44 , Math.toRadians(55)))
                .lineToY(36)
                    .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(-90)), Math.toRadians(10))
                       // .waitSeconds(0.00001)
                .lineToYConstantHeading(52, new TranslationalVelConstraint(15))
                .strafeToSplineHeading(new Vector2d(-50, 36), Math.toRadians(-55))
                        .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(-90)), Math.toRadians(5), new TranslationalVelConstraint(60))
                //.splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(0)), Math.toRadians(0))
                //    .waitSeconds(0.1)
                //.strafeToSplineHeading(new Vector2d(-52, -34), Math.toRadians(65))
                .build());*/
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, 9, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-50, 36, Math.toRadians(-55)), Math.toRadians(50), new TranslationalVelConstraint(80))
                    .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(-90)), Math.toRadians(40), new TranslationalVelConstraint(60))
                    //.waitSeconds(0.2)
                .lineToY(53, new TranslationalVelConstraint(15))
                .strafeToSplineHeading(new Vector2d(-50, 36), Math.toRadians(-55), new TranslationalVelConstraint(80))
                        .waitSeconds(4)

                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(90)), Math.toRadians(-20), new TranslationalVelConstraint(60))
                        .lineToY(-53, new TranslationalVelConstraint(15))
                .strafeToSplineHeading(new Vector2d(-50, -36), Math.toRadians(55), new TranslationalVelConstraint(80))
                .build())
*/
        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-52, 48, Math.toRadians(-235)))

                .lineToY(20, new TranslationalVelConstraint(45), new ProfileAccelConstraint(-30, 40))
                        .waitSeconds(1.0)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-22, 20, Math.toRadians(-270)), Math.toRadians(-5), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-60, 60))
                .splineToLinearHeading(new Pose2d(-22, 50, Math.toRadians(-270)), Math.toRadians(100), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 30))
                .splineToSplineHeading(new Pose2d(-12, 30, Math.toRadians(-270)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-12, 43, Math.toRadians(-270)), Math.toRadians(90), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-10, 30))
                .waitSeconds(1.0)
                .splineToSplineHeading(new Pose2d(-26, 20, Math.toRadians(-233)), Math.toRadians(-160), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(2.5, 17, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-10, 60))
                .splineToLinearHeading(new Pose2d(7, 49, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(35))
                .splineToSplineHeading(new Pose2d(7, 30, Math.toRadians(90)), Math.toRadians(-90), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, 25, Math.toRadians(-232)), Math.toRadians(-180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 40))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(26, 20, Math.toRadians(-270)), Math.toRadians(0), new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-60, 60))
                .splineToLinearHeading(new Pose2d(29, 49, Math.toRadians(-270)), Math.toRadians(90), new TranslationalVelConstraint(35))
                .splineToSplineHeading(new Pose2d(29, 35, Math.toRadians(-270)), Math.toRadians(-90), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(-228)), Math.toRadians(-180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40))
                /*.lineToY(-25, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-150, 150))
                        .waitSeconds(1)

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(11, -17, Math.toRadians(270)), Math.toRadians(0), new TranslationalVelConstraint(30),  new ProfileAccelConstraint(-120, 120))
                .splineToLinearHeading(new Pose2d(11, -49, Math.toRadians(270)), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToSplineHeading(new Pose2d(6, -38, Math.toRadians(270)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, -48, Math.toRadians(270)), Math.toRadians(-90))
                        .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(6, -30, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(229)), Math.toRadians(150), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40))
                .waitSeconds(1)

                .setReversed(true)
                        .splineToSplineHeading(new Pose2d(8, -22, Math.toRadians(245)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-60, 60))
                        .splineToSplineHeading(new Pose2d(8, -51, Math.toRadians(245)), Math.toRadians(-90), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-35, 35))
                .waitSeconds(1.5)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(8, -25, Math.toRadians(245)), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
                        .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(229)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))

                .setReversed(true)
                    .splineToSplineHeading(new Pose2d(8, -22, Math.toRadians(245)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-60, 60))
                    .splineToSplineHeading(new Pose2d(8, -51, Math.toRadians(245)), Math.toRadians(-90), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-35, 35))
                .waitSeconds(1.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(8, -25, Math.toRadians(245)), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
                .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(229)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-15, -21, Math.toRadians(270)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 60))
                        .splineToLinearHeading(new Pose2d(-12, -48.5, Math.toRadians(270)), Math.toRadians(-100), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-20, 30))
                        .splineToSplineHeading(new Pose2d(-32, -30, Math.toRadians(230)), Math.toRadians(100), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-80, 80))
                            .waitSeconds(1)

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(37, -22, Math.toRadians(270)), Math.toRadians(-20), new TranslationalVelConstraint(55),  new ProfileAccelConstraint(-120, 120))
                .splineToLinearHeading(new Pose2d(37, -50, Math.toRadians(270)), Math.toRadians(-90), new TranslationalVelConstraint(38))
                .splineToSplineHeading(new Pose2d(20, -35, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(234)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40))
                            .waitSeconds(1)*/
                .build());

        sampleBot2.runAction(sampleBot2.getDrive().actionBuilder(new Pose2d(60, -12, Math.toRadians(180)))
                        .lineToXLinearHeading(52, Math.toRadians(212))
                        .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(35, -25, Math.toRadians(-90)), Math.toRadians(5), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-40, 50))
                .splineToLinearHeading(new Pose2d(35, -51, Math.toRadians(-90)), Math.toRadians(70), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-10, 20))
                .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70))
                        .waitSeconds(2)
                .strafeToSplineHeading(
                        new Vector2d(52, -35),
                        Math.toRadians(-90),
                        new TranslationalVelConstraint(100),
                        new ProfileAccelConstraint(-90, 90)
                )

                .splineToLinearHeading(
                        new Pose2d(56, -58,
                        Math.toRadians(-90)),
                        Math.toRadians(120),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-20, 20)
                )
                        .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70))
                        .waitSeconds(2)
                .strafeToSplineHeading(
                        new Vector2d(52, -35),
                        Math.toRadians(-90),
                        new TranslationalVelConstraint(100),
                        new ProfileAccelConstraint(-90, 90)
                )

                .splineToLinearHeading(
                        new Pose2d(56, -58,
                                Math.toRadians(-90)),
                        Math.toRadians(120),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-20, 20)
                )
                .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .addEntity(sampleBot2)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(sampleBot)
                .start();
    }
}
