package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
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
            if (robotPose.position.x.value() > 52.0) {
                return 15;
            } else {
                return 40;
            }
        };

        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, -12, Math.toRadians(180)))
                .lineToX(55)
                    .waitSeconds(2)
                    .setReversed(false)
                .splineToLinearHeading(new Pose2d(62, -58, Math.toRadians(180)), Math.toRadians(25), intakeVel)
                    .setReversed(false)
                .splineToLinearHeading(new Pose2d(-26, -40, Math.toRadians(180)), Math.toRadians(170), new TranslationalVelConstraint(55))
                .splineToLinearHeading(new Pose2d(-54, -12, Math.toRadians(180)), Math.toRadians(-220), new TranslationalVelConstraint(45))
                    .waitSeconds(2)
                .lineToXLinearHeading(-58, Math.toRadians(180))
                    .waitSeconds(5)
                .lineToXLinearHeading(-54, Math.toRadians(360))
                    .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(-38, -55), new TranslationalVelConstraint(60))
                //.splineToLinearHeading(new Pose2d(-35, -63, Math.toRadians(-90)), Math.toRadians(-90))
                                //.splineToLinearHeading()
                //.splineToSplineHeading()
                .build());

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
