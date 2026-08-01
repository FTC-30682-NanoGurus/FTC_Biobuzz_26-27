package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

/**
 * BIOBUZZ (2026-27) autonomous PATHING skeleton.
 *
 * ---------------------------------------------------------------------------------------------
 * WHAT THIS IS
 * ---------------------------------------------------------------------------------------------
 * Pure RoadRunner pathing. No subsystem calls at all -- every place a mechanism would fire is a
 * SleepAction placeholder with a "// HOOK:" comment. Drop your BIOBUZZ intake/outtake Actions into
 * those slots (same ParallelAction/SequentialAction pattern as BlueGoalSide_12ArtifactAuto) once
 * the robot exists.
 *
 * Uses the current MecanumDrive tuning constants (inPerTick 0.00303299391, lateralInPerTick
 * 0.0023898724154397585, trackWidthTicks 3990.098938918289, kS/kV/kA, gains 4/4/6) unchanged.
 *
 * ---------------------------------------------------------------------------------------------
 * FIELD FRAME (retune at Kickoff, 12 SEP 2026)
 * ---------------------------------------------------------------------------------------------
 *   origin        = field center, 144" x 144" soft-tile field
 *   +x            = toward the RED alliance wall        (x = +72)
 *   -x            = toward the BLUE alliance wall       (x = -72)
 *   +y            = toward the far / back wall          (y = +72)
 *   -y            = toward the audience wall            (y = -72)
 *   heading 0 rad = robot nose pointing +x
 *
 * Alliance mirror is across x = 0: (x, y, h) -> (-x, y, PI - h). Set RED = true for the red
 * mirror of every path in this file; no second file needed.
 *
 * ---------------------------------------------------------------------------------------------
 * WHAT IS KNOWN vs PREDICTED
 * ---------------------------------------------------------------------------------------------
 * KNOWN (FIRST Game Preview, May 2026):
 *   - Scoring element is POLLEN: yellow plastic sphere, 2.8" +/- 0.1" dia, 0.055 lb.
 *   - FIRST explicitly told teams to prepare for: intaking pollen off the foam surface; intaking
 *     MULTIPLE pollen at once "arranged in lines and piles"; pollen that "will naturally roll
 *     against the field border, and into the field corners"; and autonomously navigating
 *     "between known locations" to intake pollen.
 *   - StarterBot Bases shipped as drivetrain + intake only (goBILDA: drop-center 6WD + gecko
 *     wheel intake). Scoring mechanism withheld until Kickoff.
 * PREDICTED (everything below this line, i.e. all coordinates):
 *   - 144" field, unchanged perimeter, per-alliance goal ("HIVE") in the back corner, a
 *     secondary low goal on the alliance wall, staged pollen in lines + piles, AprilTag
 *     randomization, 30 s auto.
 *
 * Every predicted number lives in the @Config block below, so at Kickoff you retune ~20 doubles
 * from FTC Dashboard and all 14 paths re-solve. Nothing is hardcoded inside a trajectory.
 */
@Config
@Autonomous(name = "BIOBUZZ Pathing Auto", group = "biobuzz")
public class Biobuzz_PathingAuto extends LinearOpMode {

    // ============================================================================================
    // ALLIANCE + ROUTINE SELECT
    // ============================================================================================

    /** false = blue alliance, true = red alliance (mirrors every path across x = 0). */
    public static boolean RED = false;

    /**
     * 0 = full 30 s routine
     * 1 = staged pollen lines only        (line-sweep tuning)
     * 2 = border + corner harvest only    (wall-follow tuning)
     * 3 = pile scoop only                 (pile approach tuning)
     * 4 = drive to park only              (endgame staging tuning)
     */
    public static int ROUTINE = 0;

    // ============================================================================================
    // PREDICTED FIELD GEOMETRY -- retune all of this at Kickoff
    // ============================================================================================

    /** Half the field, inches. The one number that is not a guess. */
    public static double FIELD_HALF = 72;

    /** Distance from a wall the robot center sits when hugging it (robot half-width + clearance). */
    public static double WALL_STANDOFF = 10.5;

    // --- Starting positions (robot back against the blue alliance wall) -------------------------
    public static double START_GOAL_X = -62, START_GOAL_Y = 12, START_GOAL_H = 0;      // hive side
    public static double START_FAR_X = -62, START_FAR_Y = -36, START_FAR_H = 0;        // audience side

    // --- Predicted HIVE (primary goal), blue back corner ---------------------------------------
    // Predicted because BIOBUZZ = bees, and CANOPY/biodiversity theming plus a light 2.8" ball
    // points at a raised corner receptacle, same family as DECODE's corner goal.
    public static double HIVE_X = -58, HIVE_Y = 58;
    /** Where the robot sits to score into the hive (standoff pose, not the hive itself). */
    public static double SCORE_CLOSE_X = -40, SCORE_CLOSE_Y = 34;
    /** Longer-range scoring pose, for if BIOBUZZ turns out to be a launcher game. */
    public static double SCORE_FAR_X = -14, SCORE_FAR_Y = 6;

    // --- Predicted secondary / low goal on the alliance wall -----------------------------------
    public static double LOW_GOAL_X = -58, LOW_GOAL_Y = 0, LOW_GOAL_H = 180;

    // --- Predicted staged pollen LINES ---------------------------------------------------------
    // Each line is defined by a HEAD (entry) and TAIL (exit). The sweep is a straight,
    // constant-heading crawl from head to tail so a full-width intake eats the whole row.
    public static double LINE1_HEAD_X = -36, LINE1_HEAD_Y = -10;
    public static double LINE1_TAIL_X = -36, LINE1_TAIL_Y = -46;
    public static double LINE2_HEAD_X = -12, LINE2_HEAD_Y = -10;
    public static double LINE2_TAIL_X = -12, LINE2_TAIL_Y = -46;
    public static double LINE3_HEAD_X = 12, LINE3_HEAD_Y = -10;
    public static double LINE3_TAIL_X = 12, LINE3_TAIL_Y = -46;
    /** Heading held through a line sweep, degrees. Nose points down the line. */
    public static double LINE_SWEEP_H = -90;

    // --- Predicted pollen PILES ----------------------------------------------------------------
    public static double PILE_NEAR_X = -46, PILE_NEAR_Y = -54;
    public static double PILE_FAR_X = 2, PILE_FAR_Y = -58;
    /** How far past the pile face the robot plunges before backing out. */
    public static double PILE_PLUNGE = 8;

    // --- Predicted loose pollen against the border ---------------------------------------------
    // FIRST called this out by name, so a wall run is near-certain to be worth points.
    public static double AUD_WALL_START_X = -50, AUD_WALL_END_X = 22;
    public static double SIDE_WALL_START_Y = -40, SIDE_WALL_END_Y = 26;

    // --- Park / endgame staging ----------------------------------------------------------------
    // CANOPY theming makes a teleop ascent likely, so auto ends staged near the alliance wall.
    public static double PARK_X = -50, PARK_Y = 24, PARK_H = 90;

    // --- Randomization AprilTag standoff -------------------------------------------------------
    /** Heading, degrees, the robot turns to at t=0 to put the randomization tag in frame. */
    public static double TAG_LOOK_H = 135;

    // ============================================================================================
    // SPEED PROFILES
    // ============================================================================================

    /** Transit between known locations -- no pollen contact, go fast. */
    public static double VEL_TRANSIT = 55;
    /** Crawl used while an intake is eating a line or a pile. Slow enough not to scatter 25 g balls. */
    public static double VEL_INTAKE = 22;
    /** Wall-hug crawl. Slower still: pollen against a wall has nowhere to escape but sideways. */
    public static double VEL_WALL = 16;
    /** Careful approach into a corner. */
    public static double VEL_CORNER = 18;

    public static double ACCEL_MIN = -30, ACCEL_MAX = 50;

    // ============================================================================================

    private MecanumDrive drive;
    private BulkRead bulkRead;
    private ElapsedTime autoTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.resetPose();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = pose(START_GOAL_X, START_GOAL_Y, START_GOAL_H);

        bulkRead = new BulkRead(hardwareMap);
        drive = new MecanumDrive(hardwareMap, beginPose);
        autoTimer = new ElapsedTime();

        telemetry.addLine("Building paths...");
        telemetry.update();

        // ----------------------------------------------------------------------------------------
        // POSES (mirrored automatically when RED == true)
        // ----------------------------------------------------------------------------------------
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

        // Corner harvest poses. 45 deg nose-in so a front intake meets the corner square.
        Pose2d cornerAudApproach = pose(-FIELD_HALF + 26, -FIELD_HALF + 26, -135);
        Pose2d cornerAudPlunge = pose(-FIELD_HALF + WALL_STANDOFF + 2, -FIELD_HALF + WALL_STANDOFF + 2, -135);
        Pose2d cornerBackApproach = pose(-FIELD_HALF + 26, FIELD_HALF - 26, 135);
        Pose2d cornerBackPlunge = pose(-FIELD_HALF + WALL_STANDOFF + 2, FIELD_HALF - WALL_STANDOFF - 2, 135);

        Pose2d park = pose(PARK_X, PARK_Y, PARK_H);

        // ----------------------------------------------------------------------------------------
        // PATH 0 -- randomization tag look. Turn in place only; costs ~0.4 s, no translation.
        // ----------------------------------------------------------------------------------------
        TrajectoryActionBuilder tagLook = drive.actionBuilder(beginPose)
                .turnTo(heading(TAG_LOOK_H));

        // ----------------------------------------------------------------------------------------
        // PATH 1 -- preload delivery. Wall start -> close scoring pose, arriving already aimed.
        // ----------------------------------------------------------------------------------------
        TrajectoryActionBuilder preloadToScore = drive.actionBuilder(pose(START_GOAL_X, START_GOAL_Y, TAG_LOOK_H))
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // Same leg but straight off the start heading, i.e. skipping the tag look. Used as the
        // lead-in for the single-segment tuning routines so they can start from the wall.
        TrajectoryActionBuilder startToScore = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(scoreClose.position, scoreClose.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // ----------------------------------------------------------------------------------------
        // PATH 2/3 -- LINE 1: enter at the head, crawl the full row, then return to score.
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // PATH 4/5 -- LINE 2
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // PATH 6/7 -- LINE 3 (far side, only reachable if the earlier cycles ran clean)
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // PATH 8 -- PILE SCOOP. Arrive head-on (spline so the nose is square to the heap), plunge
        // slowly through it, then reverse straight back out so nothing is dragged sideways.
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // PATH 9 -- AUDIENCE BORDER RUN. Constant heading, wall-parallel, WALL_STANDOFF off the
        // border, at VEL_WALL. This is the path that harvests pollen that has rolled to the edge.
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // PATH 10 -- ALLIANCE-SIDE BORDER RUN. Same idea, own-side wall, drives toward the hive so
        // the run ends where scoring happens.
        // ----------------------------------------------------------------------------------------
        TrajectoryActionBuilder toSideWall = drive.actionBuilder(scoreClose)
                .strafeToSplineHeading(sideWallStart.position, sideWallStart.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // Same destination from the long scoring pose. Needed because the routine reaches the side
        // wall from scoreFar, and a trajectory must be built from the pose it actually starts at.
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

        // ----------------------------------------------------------------------------------------
        // PATH 11 -- CORNER HARVEST (audience-side own corner). 45 deg diagonal in, short dwell,
        // 45 deg diagonal out. Straight-in/straight-out keeps the corner from wedging the robot.
        // ----------------------------------------------------------------------------------------
        // Corner first, then the wall run outbound from it: the corner is the deepest accumulation
        // point, and clearing it first means the wall sweep is not pushing pollen into a full corner.
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

        // ----------------------------------------------------------------------------------------
        // PATH 12 -- CORNER HARVEST (back own corner, next to the hive). Cheapest corner to run:
        // it is already inside scoring range.
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // PATH 13 -- LOW GOAL fallback. If the hive turns out to be unreachable/contested, dump on
        // the alliance wall instead. Kept as a separate leg so it can be swapped in at an event.
        // ----------------------------------------------------------------------------------------
        TrajectoryActionBuilder toLowGoal = drive.actionBuilder(line2Tail)
                .strafeToSplineHeading(lowGoal.position, lowGoal.heading,
                        new TranslationalVelConstraint(VEL_TRANSIT),
                        new ProfileAccelConstraint(ACCEL_MIN, ACCEL_MAX));

        // ----------------------------------------------------------------------------------------
        // PATH 14 -- PARK. Ends off the alliance wall, nose up-field, clear of the partner's lane.
        // ----------------------------------------------------------------------------------------
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

        // ----------------------------------------------------------------------------------------
        // BUILD
        // ----------------------------------------------------------------------------------------
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

        // Spare legs: built and validated but not wired into any routine below. They are the
        // event-day swap-ins -- aToLowGoal if the hive is contested/unreachable, aToAudWall to
        // enter the border run directly instead of via the corner, aToSideWall if a routine
        // reaches the side wall from scoreClose instead of scoreFar.
        Action aToLowGoal = toLowGoal.build();
        Action aParkFromScore = parkFromScore.build();
        Action aParkFromFar = parkFromFar.build();
        Action aParkFromStart = parkFromStart.build();

        telemetry.addLine("Paths Built");
        telemetry.addData("Alliance", RED ? "RED (mirrored)" : "BLUE");
        telemetry.addData("Routine", ROUTINE);
        telemetry.addData("Start", "(%.1f, %.1f) @ %.0f deg",
                beginPose.position.x, beginPose.position.y, Math.toDegrees(beginPose.heading.toDouble()));
        telemetry.addLine("NOTE: all field coords are PRE-KICKOFF PREDICTIONS. Retune @Config 12 SEP 2026.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        autoTimer.reset();

        Action routine;
        switch (ROUTINE) {
            case 1: // staged lines only
                routine = new SequentialAction(
                        aStartToScore, score(),
                        aToLine1, sweep(), aSweepLine1, aLine1ToScore, score(),
                        aToLine2, sweep(), aSweepLine2, aLine2ToScore, score(),
                        aToLine3, sweep(), aSweepLine3, aLine3ToScore, score(),
                        aParkFromFar
                );
                break;

            case 2: // border + corner harvest only
                routine = new SequentialAction(
                        aStartToScore,
                        aToCornerAud, aIntoCornerAud, dwell(), aOutOfCornerAud,
                        aCornerAudToWall, sweep(), aSweepAudWall,
                        aAudWallToScore, score(),
                        aToSideWallFromFar, sweep(), aSweepSideWall, aSideWallToScore, score()
                );
                break;

            case 3: // pile only
                routine = new SequentialAction(
                        aStartToScore,
                        aToPileNear, sweep(), aPlungePile, dwell(), aBackOutPile,
                        aPileToScore, score()
                );
                break;

            case 4: // park only
                routine = new SequentialAction(aParkFromStart);
                break;

            case 0:
            default: // full 30 s routine
                routine = new SequentialAction(
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
                break;
        }

        // Pathing-only, so the hubs are left in AUTO bulk caching (MecanumDrive's constructor sets
        // that) and there is nothing to hand-clear. Once real subsystems go in here, call
        // bulkRead.setManual() after constructing the drive and wrap this in a RaceAction with
        // bulkRead.update() -- NOT a ParallelAction. update() never returns false, so a
        // ParallelAction would never terminate and the pose store below would never run.
        Actions.runBlocking(routine);

        drive.updatePoseEstimate();
        telemetry.addData("Auto time", "%.2f s", autoTimer.seconds());
        telemetry.addData("End pose", "(%.1f, %.1f) @ %.0f deg",
                drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        if (RED) {
            PoseStorage.storeRedPose(drive.pose);
        } else {
            PoseStorage.storeBluePose(drive.pose);
        }
    }

    // ============================================================================================
    // MECHANISM HOOK PLACEHOLDERS
    // Replace each of these with the real BIOBUZZ subsystem Action once hardware exists. The
    // timings below are budget reservations, not real behaviour.
    // ============================================================================================

    /** HOOK: run outtake / launcher until empty. */
    private Action score() {
        return new SleepAction(0.9);
    }

    /** HOOK: spin up intake before entering a line/pile/wall run. */
    private Action sweep() {
        return new SleepAction(0.15);
    }

    /** HOOK: hold in a pile/corner while the intake clears the heap. */
    private Action dwell() {
        return new SleepAction(0.5);
    }

    /** HOOK: AprilTag randomization read. Return the branch you need once the tag IDs are known. */
    private Action readTag() {
        return new SleepAction(0.25);
    }

    // ============================================================================================
    // ALLIANCE MIRROR + POSE HELPERS
    // ============================================================================================

    /** Blue-frame pose, mirrored across x = 0 when RED. Heading in degrees. */
    private static Pose2d pose(double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        return RED ? new Pose2d(-x, y, Math.PI - h) : new Pose2d(x, y, h);
    }

    /** Blue-frame heading in degrees -> mirrored radians. */
    private static double heading(double headingDeg) {
        double h = Math.toRadians(headingDeg);
        return RED ? Math.PI - h : h;
    }

    /** A pose at (x, y) whose nose points at (targetX, targetY). Both given in the blue frame. */
    private static Pose2d facing(double x, double y, double targetX, double targetY) {
        double h = Math.toDegrees(Math.atan2(targetY - y, targetX - x));
        return pose(x, y, h);
    }

    /** Departure tangent from `from` toward `to`, both already in field frame. */
    private static double headingToward(Pose2d from, Pose2d to) {
        return Math.atan2(to.position.y - from.position.y, to.position.x - from.position.x);
    }

    /** `distance` inches forward along `p`'s own heading. Used for pile plunge/back-out. */
    private static Pose2d alongHeading(Pose2d p, double distance) {
        double h = p.heading.toDouble();
        return new Pose2d(
                new Vector2d(p.position.x + distance * Math.cos(h), p.position.y + distance * Math.sin(h)),
                h);
    }
}
