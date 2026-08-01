package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Color;

/**
 * BIOBUZZ (2026-27) autonomous pathing simulation.
 *
 * ---------------------------------------------------------------------------------------------
 * EXACTNESS
 * ---------------------------------------------------------------------------------------------
 * This is a 1:1 mirror of
 *   TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/Biobuzz_PathingAuto.java
 * Every constant, helper, builder call, constraint object and hook duration below is copied
 * verbatim from that file. If you retune the opmode at Kickoff, retune the FIELD GEOMETRY block
 * here too -- the MeepMeepTesting module does not depend on TeamCode, so these cannot be shared.
 *
 * The simulated trajectories are numerically identical to the ones the robot will build, because:
 *
 *  1. MeepMeep 0.1.7 depends on com.acmerobotics.roadrunner:core:1.0.1 and :actions:1.0.1 --
 *     the same versions TeamCode compiles against. Same solver, same code.
 *
 *  2. MeepMeep's DriveShim.actionBuilder() constructs
 *         TrajectoryBuilderParams(1e-6, ProfileParams(0.25, 0.1, 0.01))
 *     which is byte-identical to MecanumDrive.actionBuilder()'s params, so path discretisation
 *     and profile resolution match exactly.
 *
 *  3. Every translational leg in Biobuzz_PathingAuto passes an EXPLICIT TranslationalVelConstraint
 *     and ProfileAccelConstraint, which override the drive defaults entirely. Those explicit
 *     objects are reproduced here, so the velocity profiles are exact. (This matters: MeepMeep
 *     builds its default MecanumKinematics with lateralMultiplier = 1.0, whereas MecanumDrive uses
 *     inPerTick / lateralInPerTick = 1.2691. That difference only ever reaches the DEFAULT vel
 *     constraint, which no translational leg in this file uses.)
 *
 *  4. The single leg that does use defaults is the turnTo() tag look. It resolves against
 *     TurnConstraints(maxAngVel, -maxAngAccel, maxAngAccel), and the bot constraints below pass
 *     maxAngVel = maxAngAccel = PI, matching MecanumDrive.PARAMS exactly.
 *
 * ---------------------------------------------------------------------------------------------
 * BACKGROUND
 * ---------------------------------------------------------------------------------------------
 * GRID_GRAY on purpose. MeepMeep 0.1.7 has no BIOBUZZ field art (Kickoff is 12 SEP 2026), and
 * drawing these predicted coordinates over the DECODE field would imply a layout that does not
 * exist. The grid shows the true 144" x 144" envelope and nothing misleading.
 *
 * ---------------------------------------------------------------------------------------------
 * BOTS -- one per routine, so overlapping alternatives stay readable
 * ---------------------------------------------------------------------------------------------
 *   BLUE    routine 0  full 30 s competition run
 *   CYAN    routine 1  staged pollen lines only
 *   GREEN   routine 2  border + corner harvest only
 *   ORANGE  routine 3  pile scoop only
 *   PURPLE  routine 4  park only
 *   YELLOW  routine 5  SIM ONLY -- exercises the spare legs toSideWall + toAudWall
 *   RED     routine 6  SIM ONLY -- exercises the spare leg toLowGoal
 *
 * Routines 0-4 exist in the opmode's ROUTINE switch. Routines 5 and 6 do not; they are groupings
 * added here purely so that all 34 trajectories built by the opmode get drawn. The individual
 * legs inside them are the same objects the opmode builds, unmodified.
 *
 * ---------------------------------------------------------------------------------------------
 * MEASURED LEG DURATIONS (solved, not estimated -- seconds)
 * ---------------------------------------------------------------------------------------------
 *   tagLook           1.73    toPileNear        2.78    toCornerAud        2.93
 *   preloadToScore    1.82    plungePileNear    2.19    cornerAudToWall    1.37
 *   startToScore      1.82    backOutPileNear   2.19    intoCornerAud      2.06
 *   toLine1           2.17    pileToScore       2.78    outOfCornerAud     2.06
 *   sweepLine1        2.44    toAudWall         3.21    toCornerBack       1.20
 *   line1ToScore      2.92    sweepAudWall      5.39    intoCornerBack     2.06
 *   toLine2           2.36    audWallToScore    2.86    outOfCornerBack    2.26
 *   sweepLine2        2.44    toSideWall        2.87    toLowGoal          2.63
 *   line2ToScore      2.36    toSideWallFromFar 2.66    parkFromScore      1.23
 *   toLine3           1.80    sweepSideWall     5.01    parkFromFar        2.07
 *   sweepLine3        2.44    sideWallToScore   1.56    parkFromStart      1.35
 *   line3ToScore      2.49
 *
 * !! ROUTINE 0 AS SEQUENCED DOES NOT FIT A 30 s AUTONOMOUS PERIOD !!
 *   driving 34.23 s + mechanism hooks 5.70 s = 39.93 s.
 *   Cumulative after each scoring cycle:
 *     preload             4.70 s
 *     + line 1           13.28 s
 *     + back corner      20.20 s
 *     + line 2           28.41 s   <-- last cycle that fits; + parkFromFar 2.07 = 30.48 s
 *     + side-wall run    38.69 s
 *     + park             39.93 s
 *   So the realistic 30 s cut is: drop leg 6 (the alliance-side border run) and park from
 *   scoreFar. That still lands ~0.5 s long, which the hook placeholders absorb once the real
 *   mechanism timings replace them. Do not treat this as settled -- both the geometry and the
 *   30 s auto length are pre-Kickoff predictions. Re-measure here after retuning.
 */
public class MeepMeepTestingNikhil {

    // ============================================================================================
    // ALLIANCE  (mirrors Biobuzz_PathingAuto.RED)
    // ============================================================================================
    public static boolean RED = false;

    // ============================================================================================
    // DRIVE CONSTRAINTS -- from MecanumDrive.PARAMS
    // ============================================================================================
    static final double IN_PER_TICK = 0.00303299391;
    static final double TRACK_WIDTH_TICKS = 3990.098938918289;
    /** MecanumDrive kinematics track width = inPerTick * trackWidthTicks = 11.9822 in. */
    static final double TRACK_WIDTH = IN_PER_TICK * TRACK_WIDTH_TICKS;
    static final double MAX_WHEEL_VEL = 50;      // PARAMS.maxWheelVel
    static final double MAX_PROFILE_ACCEL = 50;  // PARAMS.maxProfileAccel
    static final double MAX_ANG_VEL = Math.PI;   // PARAMS.maxAngVel
    static final double MAX_ANG_ACCEL = Math.PI; // PARAMS.maxAngAccel

    /** Rendering only -- does not affect any trajectory. */
    static final double BOT_WIDTH = 18, BOT_LENGTH = 18;

    // ============================================================================================
    // PREDICTED FIELD GEOMETRY -- keep in sync with Biobuzz_PathingAuto's @Config block
    // ============================================================================================
    public static double FIELD_HALF = 72;
    public static double WALL_STANDOFF = 10.5;

    public static double START_GOAL_X = -62, START_GOAL_Y = 12, START_GOAL_H = 0;
    public static double START_FAR_X = -62, START_FAR_Y = -36, START_FAR_H = 0;

    public static double HIVE_X = -58, HIVE_Y = 58;
    public static double SCORE_CLOSE_X = -40, SCORE_CLOSE_Y = 34;
    public static double SCORE_FAR_X = -14, SCORE_FAR_Y = 6;

    public static double LOW_GOAL_X = -58, LOW_GOAL_Y = 0, LOW_GOAL_H = 180;

    public static double LINE1_HEAD_X = -36, LINE1_HEAD_Y = -10;
    public static double LINE1_TAIL_X = -36, LINE1_TAIL_Y = -46;
    public static double LINE2_HEAD_X = -12, LINE2_HEAD_Y = -10;
    public static double LINE2_TAIL_X = -12, LINE2_TAIL_Y = -46;
    public static double LINE3_HEAD_X = 12, LINE3_HEAD_Y = -10;
    public static double LINE3_TAIL_X = 12, LINE3_TAIL_Y = -46;
    public static double LINE_SWEEP_H = -90;

    public static double PILE_NEAR_X = -46, PILE_NEAR_Y = -54;
    public static double PILE_FAR_X = 2, PILE_FAR_Y = -58;
    public static double PILE_PLUNGE = 8;

    public static double AUD_WALL_START_X = -50, AUD_WALL_END_X = 22;
    public static double SIDE_WALL_START_Y = -40, SIDE_WALL_END_Y = 26;

    public static double PARK_X = -50, PARK_Y = 24, PARK_H = 90;

    public static double TAG_LOOK_H = 135;

    // ============================================================================================
    // SPEED PROFILES
    // ============================================================================================
    public static double VEL_TRANSIT = 55;
    public static double VEL_INTAKE = 22;
    public static double VEL_WALL = 16;
    public static double VEL_CORNER = 18;
    public static double ACCEL_MIN = -30, ACCEL_MAX = 50;

    // ============================================================================================

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity botFull    = bot(meepMeep, new Color(45, 120, 255), new Color(70, 150, 255));
        RoadRunnerBotEntity botLines   = bot(meepMeep, new Color(0, 190, 200), new Color(0, 225, 235));
        RoadRunnerBotEntity botBorder  = bot(meepMeep, new Color(30, 175, 70), new Color(50, 220, 90));
        RoadRunnerBotEntity botPile    = bot(meepMeep, new Color(230, 130, 20), new Color(255, 160, 40));
        RoadRunnerBotEntity botPark    = bot(meepMeep, new Color(160, 80, 220), new Color(190, 110, 250));
        RoadRunnerBotEntity botAltWall = bot(meepMeep, new Color(215, 200, 40), new Color(245, 230, 60));
        RoadRunnerBotEntity botAltLow  = bot(meepMeep, new Color(215, 55, 55), new Color(250, 80, 80));

        botFull.runAction(buildRoutine(botFull.getDrive(), 0));
        botLines.runAction(buildRoutine(botLines.getDrive(), 1));
        botBorder.runAction(buildRoutine(botBorder.getDrive(), 2));
        botPile.runAction(buildRoutine(botPile.getDrive(), 3));
        botPark.runAction(buildRoutine(botPark.getDrive(), 4));
        botAltWall.runAction(buildRoutine(botAltWall.getDrive(), 5));
        botAltLow.runAction(buildRoutine(botAltLow.getDrive(), 6));

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botFull)
                .addEntity(botLines)
                .addEntity(botBorder)
                .addEntity(botPile)
                .addEntity(botPark)
                .addEntity(botAltWall)
                .addEntity(botAltLow)
                .start();
    }

    private static RoadRunnerBotEntity bot(MeepMeep meepMeep, Color body, Color path) {
        return new DefaultBotBuilder(meepMeep)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .setConstraints(MAX_WHEEL_VEL, MAX_PROFILE_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setColorScheme(new Scheme(body, path))
                .build();
    }

    // ============================================================================================
    // PATHS -- verbatim from Biobuzz_PathingAuto.runOpMode()
    // ============================================================================================
    private static Action buildRoutine(DriveShim drive, int routine) {
        Pose2d beginPose = pose(START_GOAL_X, START_GOAL_Y, START_GOAL_H);

        // --- POSES (mirrored automatically when RED == true) ------------------------------------
        Pose2d scoreClose = facing(SCORE_CLOSE_X, SCORE_CLOSE_Y, HIVE_X, HIVE_Y);
        Pose2d scoreFar = facing(SCORE_FAR_X, SCORE_FAR_Y, HIVE_X, HIVE_Y);
        Pose2d lowGoal = pose(LOW_GOAL_X, LOW_GOAL_Y, LOW_GOAL_H);

        Pose2d line1Head = pose(LINE1_HEAD_X, LINE1_HEAD_Y, LINE_SWEEP_H);
        Pose2d line1Tail = pose(LINE1_TAIL_X, LINE1_TAIL_Y, LINE_SWEEP_H);
        Pose2d line2Head = pose(LINE2_HEAD_X, LINE2_HEAD_Y, LINE_SWEEP_H);
        Pose2d line2Tail = pose(LINE2_TAIL_X, LINE2_TAIL_Y, LINE_SWEEP_H);
        Pose2d line3Head = pose(LINE3_HEAD_X, LINE3_HEAD_Y, LINE_SWEEP_H);
        Pose2d line3Tail = pose(LINE3_TAIL_X, LINE3_TAIL_Y, LINE_SWEEP_H);

        double audWallY = -FIELD_HALF + WALL_STANDOFF;
        double sideWallX = -FIELD_HALF + WALL_STANDOFF;

        Pose2d audWallStart = pose(AUD_WALL_START_X, audWallY, 0);
        Pose2d audWallEnd = pose(AUD_WALL_END_X, audWallY, 0);
        Pose2d sideWallStart = pose(sideWallX, SIDE_WALL_START_Y, 90);
        Pose2d sideWallEnd = pose(sideWallX, SIDE_WALL_END_Y, 90);

        Pose2d cornerAudApproach = pose(-FIELD_HALF + 26, -FIELD_HALF + 26, -135);
        Pose2d cornerAudPlunge = pose(-FIELD_HALF + WALL_STANDOFF + 2, -FIELD_HALF + WALL_STANDOFF + 2, -135);
        Pose2d cornerBackApproach = pose(-FIELD_HALF + 26, FIELD_HALF - 26, 135);
        Pose2d cornerBackPlunge = pose(-FIELD_HALF + WALL_STANDOFF + 2, FIELD_HALF - WALL_STANDOFF - 2, 135);

        Pose2d park = pose(PARK_X, PARK_Y, PARK_H);

        // --- PATH 0 -- randomization tag look ---------------------------------------------------
        TrajectoryActionBuilder tagLook = drive.actionBuilder(beginPose)
                .turnTo(heading(TAG_LOOK_H));

        // --- PATH 1 -- preload delivery ---------------------------------------------------------
        TrajectoryActionBuilder preloadToScore = drive.actionBuilder(pose(START_GOAL_X, START_GOAL_Y, TAG_LOOK_H))
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder startToScore = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 2/3 -- LINE 1 -----------------------------------------------------------------
        TrajectoryActionBuilder toLine1 = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(line1Head.position, line1Head.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder sweepLine1 = drive.actionBuilder(line1Head)
                .strafeToConstantHeading(line1Tail.position,
                        new TranslationalVelConstraint(VEL_INTAKE),
                        new ProfileAccelConstraint(ACCEL_MIN, 25));

        TrajectoryActionBuilder line1ToScore = drive.actionBuilder(line1Tail)
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 4/5 -- LINE 2 -----------------------------------------------------------------
        TrajectoryActionBuilder toLine2 = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(line2Head.position, line2Head.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder sweepLine2 = drive.actionBuilder(line2Head)
                .strafeToConstantHeading(line2Tail.position,
                        new TranslationalVelConstraint(VEL_INTAKE),
                        new ProfileAccelConstraint(ACCEL_MIN, 25));

        TrajectoryActionBuilder line2ToScore = drive.actionBuilder(line2Tail)
                .strafeToSplineHeading(scoreFar.position, scoreFar.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 6/7 -- LINE 3 -----------------------------------------------------------------
        TrajectoryActionBuilder toLine3 = drive.actionBuilder(scoreFar)
                .strafeToSplineHeading(line3Head.position, line3Head.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder sweepLine3 = drive.actionBuilder(line3Head)
                .strafeToConstantHeading(line3Tail.position,
                        new TranslationalVelConstraint(VEL_INTAKE),
                        new ProfileAccelConstraint(ACCEL_MIN, 25));

        TrajectoryActionBuilder line3ToScore = drive.actionBuilder(line3Tail)
                .strafeToSplineHeading(scoreFar.position, scoreFar.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 8 -- PILE SCOOP ---------------------------------------------------------------
        Pose2d pileNearFace = facing(PILE_NEAR_X + 16, PILE_NEAR_Y + 16, PILE_NEAR_X, PILE_NEAR_Y);
        Pose2d pileNearDeep = alongHeading(pileNearFace, 16 + PILE_PLUNGE);

        TrajectoryActionBuilder toPileNear = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(pileNearFace.position, pileNearFace.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder plungePileNear = drive.actionBuilder(pileNearFace)
                .strafeToConstantHeading(pileNearDeep.position,
                        new TranslationalVelConstraint(VEL_INTAKE),
                        new ProfileAccelConstraint(-20, 20));

        TrajectoryActionBuilder backOutPileNear = drive.actionBuilder(pileNearDeep)
                .strafeToConstantHeading(pileNearFace.position,
                        new TranslationalVelConstraint(VEL_INTAKE),
                        new ProfileAccelConstraint(-20, 20));

        TrajectoryActionBuilder pileToScore = drive.actionBuilder(pileNearFace)
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 9 -- AUDIENCE BORDER RUN ------------------------------------------------------
        TrajectoryActionBuilder toAudWall = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(audWallStart.position, audWallStart.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder sweepAudWall = drive.actionBuilder(audWallStart)
                .strafeToConstantHeading(audWallEnd.position,
                        new TranslationalVelConstraint(VEL_WALL),
                        new ProfileAccelConstraint(-18, 18));

        TrajectoryActionBuilder audWallToScore = drive.actionBuilder(audWallEnd)
                .strafeToSplineHeading(scoreFar.position, scoreFar.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 10 -- ALLIANCE-SIDE BORDER RUN ------------------------------------------------
        TrajectoryActionBuilder toSideWall = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(sideWallStart.position, sideWallStart.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder toSideWallFromFar = drive.actionBuilder(scoreFar)
                .strafeToSplineHeading(sideWallStart.position, sideWallStart.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder sweepSideWall = drive.actionBuilder(sideWallStart)
                .strafeToConstantHeading(sideWallEnd.position,
                        new TranslationalVelConstraint(VEL_WALL),
                        new ProfileAccelConstraint(-18, 18));

        TrajectoryActionBuilder sideWallToScore = drive.actionBuilder(sideWallEnd)
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 11 -- CORNER HARVEST (audience-side own corner) -------------------------------
        TrajectoryActionBuilder toCornerAud = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(cornerAudApproach.position, cornerAudApproach.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder cornerAudToWall = drive.actionBuilder(cornerAudApproach)
                .strafeToSplineHeading(audWallStart.position, audWallStart.heading,
                        new TranslationalVelConstraint(VEL_CORNER),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder intoCornerAud = drive.actionBuilder(cornerAudApproach)
                .strafeToConstantHeading(cornerAudPlunge.position,
                        new TranslationalVelConstraint(VEL_CORNER),
                        new ProfileAccelConstraint(-18, 18));

        TrajectoryActionBuilder outOfCornerAud = drive.actionBuilder(cornerAudPlunge)
                .strafeToConstantHeading(cornerAudApproach.position,
                        new TranslationalVelConstraint(VEL_CORNER),
                        new ProfileAccelConstraint(-18, 18));

        // --- PATH 12 -- CORNER HARVEST (back own corner) ----------------------------------------
        TrajectoryActionBuilder toCornerBack = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(cornerBackApproach.position, cornerBackApproach.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder intoCornerBack = drive.actionBuilder(cornerBackApproach)
                .strafeToConstantHeading(cornerBackPlunge.position,
                        new TranslationalVelConstraint(VEL_CORNER),
                        new ProfileAccelConstraint(-18, 18));

        TrajectoryActionBuilder outOfCornerBack = drive.actionBuilder(cornerBackPlunge)
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_CORNER),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 13 -- LOW GOAL fallback -------------------------------------------------------
        TrajectoryActionBuilder toLowGoal = drive.actionBuilder(line2Tail)
                .strafeToSplineHeading(lowGoal.position, lowGoal.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- PATH 14 -- PARK --------------------------------------------------------------------
        TrajectoryActionBuilder parkFromScore = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(park.position, park.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder parkFromFar = drive.actionBuilder(scoreFar)
                .strafeToSplineHeading(park.position, park.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        TrajectoryActionBuilder parkFromStart = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(park.position, park.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // --- BUILD ------------------------------------------------------------------------------
        Action aTagLook = tagLook.build();
        Action aPreload = preloadToScore.build();
        Action aStartToScore = startToScore.build();

        Action aToLine1 = toLine1.build();
        Action aSweepLine1 = sweepLine1.build();
        Action aLine1ToScore = line1ToScore.build();

        Action aToLine2 = toLine2.build();
        Action aSweepLine2 = sweepLine2.build();
        Action aLine2ToScore = line2ToScore.build();

        Action aToLine3 = toLine3.build();
        Action aSweepLine3 = sweepLine3.build();
        Action aLine3ToScore = line3ToScore.build();

        Action aToPileNear = toPileNear.build();
        Action aPlungePile = plungePileNear.build();
        Action aBackOutPile = backOutPileNear.build();
        Action aPileToScore = pileToScore.build();

        Action aToAudWall = toAudWall.build();
        Action aSweepAudWall = sweepAudWall.build();
        Action aAudWallToScore = audWallToScore.build();

        Action aToSideWall = toSideWall.build();
        Action aToSideWallFromFar = toSideWallFromFar.build();
        Action aSweepSideWall = sweepSideWall.build();
        Action aSideWallToScore = sideWallToScore.build();

        Action aToCornerAud = toCornerAud.build();
        Action aIntoCornerAud = intoCornerAud.build();
        Action aOutOfCornerAud = outOfCornerAud.build();
        Action aCornerAudToWall = cornerAudToWall.build();

        Action aToCornerBack = toCornerBack.build();
        Action aIntoCornerBack = intoCornerBack.build();
        Action aOutOfCornerBack = outOfCornerBack.build();

        Action aToLowGoal = toLowGoal.build();
        Action aParkFromScore = parkFromScore.build();
        Action aParkFromFar = parkFromFar.build();
        Action aParkFromStart = parkFromStart.build();

        switch (routine) {
            case 1: // staged lines only
                return new SequentialAction(
                        aStartToScore, score(),
                        aToLine1, sweep(), aSweepLine1, aLine1ToScore, score(),
                        aToLine2, sweep(), aSweepLine2, aLine2ToScore, score(),
                        aToLine3, sweep(), aSweepLine3, aLine3ToScore, score(),
                        aParkFromFar
                );

            case 2: // border + corner harvest only
                return new SequentialAction(
                        aStartToScore,
                        aToCornerAud, aIntoCornerAud, dwell(), aOutOfCornerAud,
                        aCornerAudToWall, sweep(), aSweepAudWall,
                        aAudWallToScore, score(),
                        aToSideWallFromFar, sweep(), aSweepSideWall, aSideWallToScore, score()
                );

            case 3: // pile only
                return new SequentialAction(
                        aStartToScore,
                        aToPileNear, sweep(), aPlungePile, dwell(), aBackOutPile,
                        aPileToScore, score()
                );

            case 4: // park only
                return new SequentialAction(aParkFromStart);

            case 5: // SIM ONLY -- spare legs toSideWall + toAudWall (never used by the opmode)
                return new SequentialAction(
                        aStartToScore,
                        aToSideWall, sweep(), aSweepSideWall, aSideWallToScore, score(),
                        aToAudWall, sweep(), aSweepAudWall, aAudWallToScore, score()
                );

            case 6: // SIM ONLY -- spare leg toLowGoal (never used by the opmode)
                return new SequentialAction(
                        aStartToScore,
                        aToLine2, sweep(), aSweepLine2,
                        aToLowGoal, score()
                );

            case 0:
            default: // full 30 s routine
                return new SequentialAction(
                        // 1. read randomization tag from the start tile
                        aTagLook,
                        readTag(),

                        // 2. preload
                        aPreload,
                        score(),

                        // 3. nearest staged line -> score
                        aToLine1,
                        sweep(),
                        aSweepLine1,
                        aLine1ToScore,
                        score(),

                        // 4. own back corner: closest loose pollen to the hive
                        aToCornerBack,
                        aIntoCornerBack,
                        dwell(),
                        aOutOfCornerBack,
                        score(),

                        // 5. second staged line -> score from the longer pose
                        aToLine2,
                        sweep(),
                        aSweepLine2,
                        aLine2ToScore,
                        score(),

                        // 6. own-side border run, ends already in scoring range.
                        // Entered from scoreFar because leg 5 ends there.
                        aToSideWallFromFar,
                        sweep(),
                        aSweepSideWall,
                        aSideWallToScore,
                        score(),

                        // 7. park clear of the partner
                        aParkFromScore
                );
        }
    }

    // ============================================================================================
    // MECHANISM HOOK PLACEHOLDERS -- same durations as Biobuzz_PathingAuto, so the sim timeline
    // matches the real auto's timeline.
    // ============================================================================================
    private static Action score()   { return new SleepAction(0.9); }
    private static Action sweep()   { return new SleepAction(0.15); }
    private static Action dwell()   { return new SleepAction(0.5); }
    private static Action readTag() { return new SleepAction(0.25); }

    // ============================================================================================
    // ALLIANCE MIRROR + POSE HELPERS -- verbatim from Biobuzz_PathingAuto
    // ============================================================================================
    private static Pose2d pose(double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        return RED ? new Pose2d(-x, y, Math.PI - h) : new Pose2d(x, y, h);
    }

    private static double heading(double headingDeg) {
        double h = Math.toRadians(headingDeg);
        return RED ? Math.PI - h : h;
    }

    private static Pose2d facing(double x, double y, double targetX, double targetY) {
        double h = Math.toDegrees(Math.atan2(targetY - y, targetX - x));
        return pose(x, y, h);
    }

    private static Pose2d alongHeading(Pose2d p, double distance) {
        double h = p.heading.toDouble();
        return new Pose2d(
                new Vector2d(p.position.x + distance * Math.cos(h), p.position.y + distance * Math.sin(h)),
                h);
    }

    // ============================================================================================
    // COLOR SCHEME
    // ============================================================================================
    private static final class Scheme extends ColorScheme {
        private final Color body, path;

        Scheme(Color body, Color path) {
            this.body = body;
            this.path = path;
        }

        @Override public boolean isDark() { return true; }
        @Override public Color getBOT_BODY_COLOR() { return body; }
        @Override public Color getBOT_WHEEL_COLOR() { return body.darker().darker(); }
        @Override public Color getBOT_DIRECTION_COLOR() { return body.brighter(); }
        @Override public Color getAXIS_X_COLOR() { return path; }
        @Override public Color getAXIS_Y_COLOR() { return path; }
        @Override public double getAXIS_NORMAL_OPACITY() { return 0.35; }
        @Override public double getAXIS_HOVER_OPACITY() { return 0.85; }
        @Override public Color getTRAJECTORY_PATH_COLOR() { return path; }
        @Override public Color getTRAJECTORY_TURN_COLOR() { return path.brighter(); }
        @Override public Color getTRAJECTORY_MARKER_COLOR() { return Color.WHITE; }
        @Override public Color getTRAJECTORY_SLIDER_BG() { return new Color(45, 45, 45); }
        @Override public Color getTRAJECTORY_SLIDER_FG() { return path; }
        @Override public Color getTRAJECTORY_TEXT_COLOR() { return Color.WHITE; }
        @Override public Color getUI_MAIN_BG() { return new Color(24, 24, 24); }
    }
}

// ================================================================================================
// OLD DECODE / INTO THE DEEP PATHS -- commented out, kept for reference.
// (Java has no nestable block comments and the original already used /* */, so these are
//  line-commented rather than wrapped.)
// ================================================================================================
// package com.example.meepmeeptesting;
//
// import com.acmerobotics.roadrunner.AccelConstraint;
// import com.acmerobotics.roadrunner.Arclength;
// import com.acmerobotics.roadrunner.CompositeVelConstraint;
// import com.acmerobotics.roadrunner.InstantAction;
// import com.acmerobotics.roadrunner.MinMax;
// import com.acmerobotics.roadrunner.MinVelConstraint;
// import com.acmerobotics.roadrunner.NullAction;
// import com.acmerobotics.roadrunner.Pose2d;
// import com.acmerobotics.roadrunner.Pose2dDual;
// import com.acmerobotics.roadrunner.PosePath;
// import com.acmerobotics.roadrunner.ProfileAccelConstraint;
// import com.acmerobotics.roadrunner.SleepAction;
// import com.acmerobotics.roadrunner.Trajectory;
// import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
// import com.acmerobotics.roadrunner.TrajectoryBuilder;
// import com.acmerobotics.roadrunner.TranslationalVelConstraint;
// import com.acmerobotics.roadrunner.Vector2d;
// import com.acmerobotics.roadrunner.VelConstraint;
// import com.noahbres.meepmeep.MeepMeep;
// import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
// import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
// import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
// import org.jetbrains.annotations.NotNull;
//
// import java.awt.Color;
//
// import javax.swing.ProgressMonitor;
//
// public class  MeepMeepTestingNikhil {
//     public static void main(String[] args) {
//         MeepMeep meepMeep = new MeepMeep(800);
//
//         RoadRunnerBotEntity sampleBot = new DefaultBotBuilder(meepMeep)
//                 // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                 .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
//                 .build();
//         RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
//                 // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                 .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
//                 .setColorScheme(new ColorScheme() {
//                     @NotNull
//                     @Override
//                     public Color getUI_MAIN_BG() {
//                         return new Color(0,0,100);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_TEXT_COLOR() {
//                         return new Color(0,0,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_SLIDER_FG() {
//                         return new Color(0,0,255);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_SLIDER_BG() {
//                         return new Color(255,255,255);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_MARKER_COLOR() {
//                         return new Color(0,100,100);
//                     }
//
//                     @Override
//                     public boolean isDark() {
//                         return false;
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getBOT_BODY_COLOR() {
//                         return new Color(0,155,200);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getBOT_WHEEL_COLOR() {
//                         return new Color(0,0,100);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getBOT_DIRECTION_COLOR() {
//                         return new Color(0,0,185);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getAXIS_X_COLOR() {
//                         return new Color(0,50,255);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getAXIS_Y_COLOR() {
//                         return new Color(0,50,255);
//                     }
//
//                     @Override
//                     public double getAXIS_NORMAL_OPACITY() {
//                         return 0.7;
//                     }
//
//                     @Override
//                     public double getAXIS_HOVER_OPACITY() {
//                         return 0.3;
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_PATH_COLOR() {
//                         return  new Color(0,80,255);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_TURN_COLOR() {
//                         return new Color(0,80,255);
//                     }
//
//                 })
//                 .build();
//         RoadRunnerBotEntity sampleBot2 = new DefaultBotBuilder(meepMeep)
//                 // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                 .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
//                 .setColorScheme(new ColorScheme() {
//                     @NotNull
//                     @Override
//                     public Color getUI_MAIN_BG() {
//                         return new Color(0,100,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_TEXT_COLOR() {
//                         return new Color(0,0,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_SLIDER_FG() {
//                         return new Color(0,255,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_SLIDER_BG() {
//                         return new Color(255,255,255);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_MARKER_COLOR() {
//                         return new Color(0,100,20);
//                     }
//
//                     @Override
//                     public boolean isDark() {
//                         return false;
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getBOT_BODY_COLOR() {
//                         return new Color(0,155,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getBOT_WHEEL_COLOR() {
//                         return new Color(0,50,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getBOT_DIRECTION_COLOR() {
//                         return new Color(0, 100, 0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getAXIS_X_COLOR() {
//                         return new Color(0,200,50);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getAXIS_Y_COLOR() {
//                         return new Color(0,200,50);
//                     }
//
//                     @Override
//                     public double getAXIS_NORMAL_OPACITY() {
//                         return 0.7;
//                     }
//
//                     @Override
//                     public double getAXIS_HOVER_OPACITY() {
//                         return 0.3;
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_PATH_COLOR() {
//                         return  new Color(0,255,0);
//                     }
//
//                     @NotNull
//                     @Override
//                     public Color getTRAJECTORY_TURN_COLOR() {
//                         return new Color(0,255,0);
//                     }
//
//                 })
//                 .build();
//         AccelConstraint highMode = (robotPose, _path, _disp) -> {
//                 return new MinMax(-10,50);
//         };
//         Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));
//
//         AccelConstraint smartScore = (robotPose, _path, _disp) -> {
//             if (robotPose.position.x.value() < -30.0) {
//                 return new MinMax(-5,5);
//             } else {
//                 return new MinMax(-120,120);
//             }
//         };
//         AccelConstraint intakeAccel = (robotPose, _path, _disp) -> {
//             if (robotPose.position.y.value() < -42.0) {
//                 return new MinMax(-10,22);
//             } else {
//                 return new MinMax(-30,50);
//             }
//         };
//         VelConstraint intakeVel = (robotPose, _path, _disp) -> {
//             if (robotPose.position.y.value() < -25.0) {
//                 return 20;
//             } else {
//                 return 50;
//             }
//         };
//         /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(64, -36 , Math.toRadians(-90)))
//                 .lineToY(-5)
//                 .setReversed(true)
//                 .setTangent(90)
//                 .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(0)), Math.toRadians(90))
//                 .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
//                 .waitSeconds(1.2)
//                 //.lineToX(-35)
//                 .setTangent(-90)
//                 .splineToSplineHeading(new Pose2d(-60, -48.5, Math.toRadians(70)), Math.toRadians(240))
//                         //.turn(-0.2)
//                 .waitSeconds(0.5)
//                 .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
//                 .waitSeconds(1.2)
//                 //.lineToX(-35)
//                 .setTangent(-90)
//                 .splineToSplineHeading(new Pose2d(-60, -48.5, Math.toRadians(70)), Math.toRadians(240))
//                         //.turn(-0.2)
//                 .waitSeconds(0.5)
//                 .splineToSplineHeading(new Pose2d(-29, -11, Math.toRadians(0)), Math.toRadians(0))
//                 .build());
//          */
//         /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-56, -44 , Math.toRadians(55)))
//                 .lineToY(36)
//                     .waitSeconds(4)
//                 .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(-90)), Math.toRadians(10))
//                        // .waitSeconds(0.00001)
//                 .lineToYConstantHeading(52, new TranslationalVelConstraint(15))
//                 .strafeToSplineHeading(new Vector2d(-50, 36), Math.toRadians(-55))
//                         .waitSeconds(4)
//                 .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(-90)), Math.toRadians(5), new TranslationalVelConstraint(60))
//                 //.splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(0)), Math.toRadians(0))
//                 //    .waitSeconds(0.1)
//                 //.strafeToSplineHeading(new Vector2d(-52, -34), Math.toRadians(65))
//                 .build());*/
//         /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, 9, Math.toRadians(0)))
//                 .splineToSplineHeading(new Pose2d(-50, 36, Math.toRadians(-55)), Math.toRadians(50), new TranslationalVelConstraint(80))
//                     .waitSeconds(4)
//                 .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(-90)), Math.toRadians(40), new TranslationalVelConstraint(60))
//                     //.waitSeconds(0.2)
//                 .lineToY(53, new TranslationalVelConstraint(15))
//                 .strafeToSplineHeading(new Vector2d(-50, 36), Math.toRadians(-55), new TranslationalVelConstraint(80))
//                         .waitSeconds(4)
//
//                 .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(90)), Math.toRadians(-20), new TranslationalVelConstraint(60))
//                         .lineToY(-53, new TranslationalVelConstraint(15))
//                 .strafeToSplineHeading(new Vector2d(-50, -36), Math.toRadians(55), new TranslationalVelConstraint(80))
//                 .build())
// */
//         sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-52, 48, Math.toRadians(-235)))
//
//                 .lineToY(20, new TranslationalVelConstraint(45), new ProfileAccelConstraint(-30, 40))
//                         .waitSeconds(1.0)
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(-22, 20, Math.toRadians(-270)), Math.toRadians(-5), new TranslationalVelConstraint(45), new ProfileAccelConstraint(-60, 60))
//                 .splineToLinearHeading(new Pose2d(-22, 50, Math.toRadians(-270)), Math.toRadians(100), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 30))
//                 .splineToSplineHeading(new Pose2d(-12, 30, Math.toRadians(-270)), Math.toRadians(-90))
//                 .splineToSplineHeading(new Pose2d(-12, 43, Math.toRadians(-270)), Math.toRadians(90), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-10, 30))
//                 .waitSeconds(1.0)
//                 .splineToSplineHeading(new Pose2d(-26, 20, Math.toRadians(-233)), Math.toRadians(-160), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30, 50))
//
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(2.5, 17, Math.toRadians(90)), Math.toRadians(0), new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-10, 60))
//                 .splineToLinearHeading(new Pose2d(7, 49, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(35))
//                 .splineToSplineHeading(new Pose2d(7, 30, Math.toRadians(90)), Math.toRadians(-90), new TranslationalVelConstraint(50))
//                 .splineToSplineHeading(new Pose2d(-26, 25, Math.toRadians(-232)), Math.toRadians(-180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 40))
//
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(26, 20, Math.toRadians(-270)), Math.toRadians(0), new TranslationalVelConstraint(50),  new ProfileAccelConstraint(-60, 60))
//                 .splineToLinearHeading(new Pose2d(29, 49, Math.toRadians(-270)), Math.toRadians(90), new TranslationalVelConstraint(35))
//                 .splineToSplineHeading(new Pose2d(29, 35, Math.toRadians(-270)), Math.toRadians(-90), new TranslationalVelConstraint(50))
//                 .splineToSplineHeading(new Pose2d(-26, 22, Math.toRadians(-228)), Math.toRadians(-180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40))
//                 /*.lineToY(-25, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-150, 150))
//                         .waitSeconds(1)
//
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(11, -17, Math.toRadians(270)), Math.toRadians(0), new TranslationalVelConstraint(30),  new ProfileAccelConstraint(-120, 120))
//                 .splineToLinearHeading(new Pose2d(11, -49, Math.toRadians(270)), Math.toRadians(-90), new TranslationalVelConstraint(30))
//                 .splineToSplineHeading(new Pose2d(6, -38, Math.toRadians(270)), Math.toRadians(90))
//                 .splineToSplineHeading(new Pose2d(6, -48, Math.toRadians(270)), Math.toRadians(-90))
//                         .waitSeconds(0.5)
//                 .splineToSplineHeading(new Pose2d(6, -30, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(50))
//                 .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(229)), Math.toRadians(150), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40))
//                 .waitSeconds(1)
//
//                 .setReversed(true)
//                         .splineToSplineHeading(new Pose2d(8, -22, Math.toRadians(245)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-60, 60))
//                         .splineToSplineHeading(new Pose2d(8, -51, Math.toRadians(245)), Math.toRadians(-90), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-35, 35))
//                 .waitSeconds(1.5)
//                         .setReversed(true)
//                         .splineToSplineHeading(new Pose2d(8, -25, Math.toRadians(245)), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
//                         .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(229)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
//
//                 .setReversed(true)
//                     .splineToSplineHeading(new Pose2d(8, -22, Math.toRadians(245)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-60, 60))
//                     .splineToSplineHeading(new Pose2d(8, -51, Math.toRadians(245)), Math.toRadians(-90), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-35, 35))
//                 .waitSeconds(1.5)
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(8, -25, Math.toRadians(245)), Math.toRadians(90), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
//                 .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(229)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
//
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(-15, -21, Math.toRadians(270)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 60))
//                         .splineToLinearHeading(new Pose2d(-12, -48.5, Math.toRadians(270)), Math.toRadians(-100), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-20, 30))
//                         .splineToSplineHeading(new Pose2d(-32, -30, Math.toRadians(230)), Math.toRadians(100), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-80, 80))
//                             .waitSeconds(1)
//
//                 .setReversed(true)
//                 .splineToSplineHeading(new Pose2d(37, -22, Math.toRadians(270)), Math.toRadians(-20), new TranslationalVelConstraint(55),  new ProfileAccelConstraint(-120, 120))
//                 .splineToLinearHeading(new Pose2d(37, -50, Math.toRadians(270)), Math.toRadians(-90), new TranslationalVelConstraint(38))
//                 .splineToSplineHeading(new Pose2d(20, -35, Math.toRadians(270)), Math.toRadians(90), new TranslationalVelConstraint(50))
//                 .splineToSplineHeading(new Pose2d(-26, -22, Math.toRadians(234)), Math.toRadians(180), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-40, 40))
//                             .waitSeconds(1)*/
//                 .build());
//
//         sampleBot2.runAction(sampleBot2.getDrive().actionBuilder(new Pose2d(60, -12, Math.toRadians(180)))
//                         .lineToXLinearHeading(52, Math.toRadians(212))
//                         .waitSeconds(2)
//                 .splineToSplineHeading(new Pose2d(35, -25, Math.toRadians(-90)), Math.toRadians(5), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-40, 50))
//                 .splineToLinearHeading(new Pose2d(35, -51, Math.toRadians(-90)), Math.toRadians(70), new TranslationalVelConstraint(42), new ProfileAccelConstraint(-10, 20))
//                 .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(90), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70))
//                         .waitSeconds(2)
//                 .strafeToSplineHeading(
//                         new Vector2d(52, -35),
//                         Math.toRadians(-90),
//                         new TranslationalVelConstraint(100),
//                         new ProfileAccelConstraint(-90, 90)
//                 )
//
//                 .splineToLinearHeading(
//                         new Pose2d(56, -58,
//                         Math.toRadians(-90)),
//                         Math.toRadians(120),
//                         new TranslationalVelConstraint(50),
//                         new ProfileAccelConstraint(-20, 20)
//                 )
//                         .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70))
//                         .waitSeconds(2)
//                 .strafeToSplineHeading(
//                         new Vector2d(52, -35),
//                         Math.toRadians(-90),
//                         new TranslationalVelConstraint(100),
//                         new ProfileAccelConstraint(-90, 90)
//                 )
//
//                 .splineToLinearHeading(
//                         new Pose2d(56, -58,
//                                 Math.toRadians(-90)),
//                         Math.toRadians(120),
//                         new TranslationalVelConstraint(50),
//                         new ProfileAccelConstraint(-20, 20)
//                 )
//                 .splineToSplineHeading(new Pose2d(52, -12, Math.toRadians(212)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 70))
//
//                 .build());
//
//         meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
//                 .addEntity(sampleBot2)
//                 .setDarkMode(true)
//                 .setBackgroundAlpha(0.95f)
//                 .addEntity(sampleBot)
//                 .start();
//     }
// }
