package org.firstinspires.ftc.teamcode.Biobuzz_subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Biobuzz_subsystems.TurretConstants.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
// ===== BEGIN SDK 12 CLUSTER CODE - DISABLED =====
// import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
// ===== END SDK 12 CLUSTER CODE - DISABLED =====
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Owns the C920: the {@link VisionPortal}, the {@link AprilTagProcessor}, the camera's manual
 * exposure settings, and the job of turning a frame full of detections into AT MOST ONE target
 * worth aiming at.
 *
 * <h2>What this class is NOT for</h2>
 * It does not localize the robot, and nothing here ever touches the drivetrain's pose. In BioBuzz
 * the AprilTags ride on the HIVE CELLs, which MOVE, and FIRST states plainly that they are not
 * usable as field references. Odometry owns the global pose; the camera owns aiming and target
 * selection. Treating a cell tag as a survey marker would inject the hive's own motion straight
 * into the robot's pose estimate and corrupt every autonomous path on the robot.
 *
 * <h2>Target selection, in order</h2>
 * <ol>
 *   <li><b>Cluster, not single tag.</b> BioBuzz goals are always four-tag clusters, so anything
 *       that is not an {@code AprilTagClusterDetection} is skipped outright.</li>
 *   <li><b>Alliance, from the metadata NAME.</b> Cluster names begin with 'R' or 'B'. This is the
 *       authoritative selector - not which half of the field the robot happens to be standing in,
 *       which is only ever a last-resort guess elsewhere in this subsystem.</li>
 *   <li><b>Scorable right now.</b> A CELL only accepts a ball while its opening faces up, which is
 *       true iff {@code abs(ftcPose.roll) < 90}. A flipped cell is a wall.</li>
 *   <li><b>Coverage gate, then best coverage wins.</b> Clusters below
 *       {@link TurretConstants#MIN_PERCENT_CLUSTER_FOUND} are rejected, and among the survivors the
 *       one with the most tags visible is chosen, because coverage is also a direct proxy for how
 *       well-conditioned the pose solution is.</li>
 * </ol>
 *
 * <h2>CLUSTER DETECTION IS CURRENTLY DISABLED</h2>
 * This project builds against FTC SDK 10.1.0, which does not contain
 * {@code AprilTagClusterDetection} - that class arrives with the BioBuzz season SDK 12.0.0.
 * Every line that touches it is parked inside blocks marked
 * {@code ===== BEGIN/END SDK 12 CLUSTER CODE - DISABLED =====}, and the complete working versions
 * are saved at {@code TeamCode/sdk12-cluster-backup/}.
 *
 * What still works: the VisionPortal, the C920 manual exposure/gain/white-balance/focus setup, the
 * crosshair overlay, and every accessor. What does not: {@link #getAllianceTargets} always returns
 * an empty list, so the turret flies on odometry and the surveyed hive coordinates alone and
 * {@code readyToFire()} never goes true. That is the safe failure - it holds fire rather than
 * shooting blind.
 *
 * <h2>Null-safety</h2>
 * Every accessor tolerates a portal that is still initializing, a camera that failed to open, and
 * a frame with nothing in it. {@link #getBestTarget(Alliance)} returns null in all of those cases
 * and never throws, because a vision fault during a match must degrade the turret to odometry
 * aiming rather than end the opmode.
 */
public class TurretVision {

    private final Telemetry telemetry;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    /** Held so the opmode/controller can push aim state to the crosshair every loop. */
    private final TurretAimOverlay overlay = new TurretAimOverlay();

    /** Camera settings can only be applied once the stream is actually running. */
    private boolean cameraSettingsApplied = false;

    /**
     * How many loops to keep retrying camera setup before giving up. A C920's UVC control interface
     * can lag the first streaming frame, so the first attempt genuinely does fail sometimes.
     */
    private static final int MAX_SETUP_ATTEMPTS = 20;
    private int setupAttempts = 0;

    /** Human-readable account of what the last setup attempt did or could not do. Telemetry only. */
    private String cameraSetupReport = "not attempted";

    /** What was actually accepted by the camera, as opposed to what was requested. Telemetry only. */
    private int appliedGain = -1;
    private int appliedExposureMs = -1;
    private int appliedWhiteBalanceK = -1;
    private String appliedFocusMode = "not set";

    /** Last coverage seen per cluster name, PERCENT, for the coverage-collapse tip detector. */
    private final Map<String, Double> lastCoverageByName = new HashMap<>();

    /** Diagnostics for telemetry - never used for control decisions. */
    private int lastDetectionCount = 0;
    private int lastClusterCount = 0;
    private String lastRejectReason = "no frames yet";

    /**
     * Builds the AprilTag processor and the vision portal and starts streaming.
     *
     * The overlay processor is added to the SAME portal as the tag processor, so the crosshair is
     * drawn onto the live stream the decoder is already reading. Two portals on one camera is not
     * possible, and two cameras would need two USB devices.
     *
     * @param hardwareMap the opmode's hardware map
     * @param telemetry   used only for warnings; no per-loop spam from this class
     */
    public TurretVision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            aprilTag = new AprilTagProcessor.Builder()
                    // The BioBuzz tag library. Never hard-code tag IDs: the library is the single
                    // source of truth for which IDs exist this season, how big they are and what
                    // each cluster is called.
                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    // Report inches and degrees so nothing downstream has to convert, and so the
                    // constants file can state plain units.
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    // These are the SDK's own built-in C920 640x480 values. Setting them explicitly
                    // costs nothing and protects against the camera enumerating under a name the
                    // SDK does not recognise, which would silently fall back to a generic guess and
                    // bias every range and bearing.
                    .setLensIntrinsics(TurretConstants.CAMERA_FX,
                                       TurretConstants.CAMERA_FY,
                                       TurretConstants.CAMERA_CX,
                                       TurretConstants.CAMERA_CY)
                    .setDrawTagOutline(true)
                    .setDrawTagID(true)
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .build();

            // Decimation trades detection range against frame rate. 2 decodes on a half-scale
            // image, which is the usual sweet spot for goal-sized clusters.
            aprilTag.setDecimation(TurretConstants.APRILTAG_DECIMATION);

            overlay.setCameraModel(TurretConstants.CAMERA_FX,
                                   TurretConstants.CAMERA_FY,
                                   TurretConstants.CAMERA_CX,
                                   TurretConstants.CAMERA_CY,
                                   TurretConstants.CAMERA_MOUNT_YAW_DEG);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, TurretConstants.WEBCAM_NAME))
                    .setCameraResolution(new Size(TurretConstants.STREAM_WIDTH,
                                                  TurretConstants.STREAM_HEIGHT))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(false)
                    .addProcessors(aprilTag, overlay)
                    .build();

        } catch (Exception e) {
            // A missing or unopenable webcam must not take the opmode down with it. The turret
            // still aims from odometry and the seed coordinate; it just never gets a correction.
            aprilTag = null;
            visionPortal = null;
            if (telemetry != null) {
                telemetry.addLine("TURRET VISION FAILED TO START: " + e.getMessage());
            }
        }
    }

    // =============================================================================================
    // Per-loop housekeeping
    // =============================================================================================

    /**
     * Applies the manual exposure / gain / focus / white-balance settings once the stream reaches
     * STREAMING, then does nothing on every later call.
     *
     * Call this every loop. It is deliberately NON-BLOCKING: the usual sample code busy-waits for
     * the portal to come up, which stalls the opmode for a second or more. Here the settings are
     * simply applied on whichever loop finds the camera ready.
     *
     * If a control is not ready yet - which happens on a C920, where the UVC control interface can
     * lag the first streaming frame by a few hundred milliseconds - the attempt is RETRIED on
     * later loops up to {@link #MAX_SETUP_ATTEMPTS} times rather than being given up on after one
     * try. A single failed attempt at the exact moment the stream starts is the most common reason
     * a camera silently ends up on auto-exposure for a whole match.
     */
    public void update() {
        if (visionPortal == null || cameraSettingsApplied) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        if (setupAttempts >= MAX_SETUP_ATTEMPTS) return;

        setupAttempts++;
        if (applyCameraSettings()) {
            cameraSettingsApplied = true;
        } else if (setupAttempts >= MAX_SETUP_ATTEMPTS && telemetry != null) {
            telemetry.addLine("TURRET CAMERA: gave up configuring after "
                    + MAX_SETUP_ATTEMPTS + " attempts - " + cameraSetupReport);
        }
    }

    /**
     * Interrogates the camera and pushes the exposure/gain/white-balance/focus settings onto it.
     *
     * <h3>Everything here is fetched from the camera at RUNTIME</h3>
     * Nothing assumes what a particular C920 supports. Maximum gain, and the supported white
     * balance temperature range, are read off the device and the requested values are coerced into
     * them. That matters because these limits differ between C920 hardware revisions and between
     * firmware versions, so a value hard-coded from one camera is quietly rejected by the next one
     * - and a rejected setting leaves the camera on AUTO, which is the failure this whole method
     * exists to prevent.
     *
     * <h3>Why minimum exposure and maximum gain</h3>
     * A tag is decoded from the position of its corners. Motion blur smears those corners, and
     * blur is set by EXPOSURE TIME, not by brightness: at 3 ms the shutter is open for 3 ms no
     * matter how fast the robot turns. The light thrown away by the short exposure is bought back
     * with sensor gain, which adds noise but does not move the corners. This is the standard FTC
     * technique and is why the turret can still decode while the chassis is driving.
     *
     * @return true when every requested control was applied; false if any was missing or refused,
     *         which schedules a retry on a later loop
     */
    private boolean applyCameraSettings() {
        if (visionPortal == null) return false;

        StringBuilder report = new StringBuilder();
        boolean allOk = true;

        // ---- Exposure: manual, in milliseconds ---------------------------------------------------
        if (TurretConstants.MANUAL_EXPOSURE) {
            try {
                ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
                if (exposure == null) {
                    report.append("no ExposureControl; ");
                    allOk = false;
                } else if (!exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    report.append("manual exposure unsupported; ");
                    allOk = false;
                } else {
                    if (exposure.getMode() != ExposureControl.Mode.Manual
                            && !exposure.setMode(ExposureControl.Mode.Manual)) {
                        report.append("exposure setMode refused; ");
                        allOk = false;
                    }
                    // Coerce into the range this camera actually accepts. Sending a value outside
                    // it is simply refused, which would leave the exposure wherever it was.
                    long lo = exposure.getMinExposure(TimeUnit.MILLISECONDS);
                    long hi = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                    long want = TurretConstants.EXPOSURE_MS;
                    long use = want;
                    if (hi > 0) { // a camera that cannot report its range returns 0
                        use = Math.max(lo, Math.min(hi, want));
                    }
                    if (use != want) {
                        report.append("exposure ").append(want).append("ms coerced to ")
                              .append(use).append("ms (camera range ").append(lo).append("-")
                              .append(hi).append("); ");
                    }
                    if (exposure.setExposure(use, TimeUnit.MILLISECONDS)) {
                        appliedExposureMs = (int) use;
                    } else {
                        report.append("setExposure refused; ");
                        allOk = false;
                    }
                }
            } catch (Exception e) {
                report.append("exposure threw: ").append(e.getMessage()).append("; ");
                allOk = false;
            }

            // ---- Gain: maximum, read from the camera ---------------------------------------------
            try {
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                if (gainControl == null) {
                    report.append("no GainControl; ");
                    allOk = false;
                } else {
                    // GAIN = -1 is the "use whatever this camera's maximum is" sentinel. Any
                    // non-negative value in constants overrides it, and is still clamped to the
                    // camera's own reported range.
                    int max = gainControl.getMaxGain();
                    int min = gainControl.getMinGain();
                    int want = (TurretConstants.GAIN < 0) ? max : TurretConstants.GAIN;
                    int use = Math.max(min, Math.min(max, want));
                    if (use != want) {
                        report.append("gain ").append(want).append(" coerced to ").append(use)
                              .append(" (camera range ").append(min).append("-").append(max)
                              .append("); ");
                    }
                    if (gainControl.setGain(use)) {
                        appliedGain = use;
                    } else {
                        report.append("setGain refused; ");
                        allOk = false;
                    }
                }
            } catch (Exception e) {
                report.append("gain threw: ").append(e.getMessage()).append("; ");
                allOk = false;
            }
        }

        // ---- White balance: manual, temperature clamped to the camera's range --------------------
        //
        // White balance does not affect a greyscale tag decode directly, but an auto-WB event
        // changes exposure metering on this sensor, so it is pinned for the same reason exposure is.
        //
        // WhiteBalanceControl has no isModeSupported(), unlike the exposure and focus controls, so
        // the only way to know whether manual is available is to try it and read the return value.
        if (TurretConstants.LOCK_WHITE_BALANCE) {
            try {
                WhiteBalanceControl wb = visionPortal.getCameraControl(WhiteBalanceControl.class);
                if (wb == null) {
                    report.append("no WhiteBalanceControl; ");
                    allOk = false;
                } else if (wb.getMode() != WhiteBalanceControl.Mode.MANUAL
                        && !wb.setMode(WhiteBalanceControl.Mode.MANUAL)) {
                    report.append("manual white balance refused; ");
                    allOk = false;
                } else {
                    int lo = wb.getMinWhiteBalanceTemperature();
                    int hi = wb.getMaxWhiteBalanceTemperature();
                    int want = TurretConstants.WHITE_BALANCE_TEMPERATURE_K;
                    int use = want;
                    if (hi > lo) { // guard a camera that cannot report its range
                        use = Math.max(lo, Math.min(hi, want));
                    }
                    if (use != want) {
                        report.append("white balance ").append(want).append("K coerced to ")
                              .append(use).append("K (camera range ").append(lo).append("-")
                              .append(hi).append("K); ");
                    }
                    if (wb.setWhiteBalanceTemperature(use)) {
                        appliedWhiteBalanceK = use;
                    } else {
                        report.append("setWhiteBalanceTemperature refused; ");
                        allOk = false;
                    }
                }
            } catch (Exception e) {
                report.append("white balance threw: ").append(e.getMessage()).append("; ");
                allOk = false;
            }
        }

        // ---- Focus: fixed ------------------------------------------------------------------------
        //
        // Focus hunting drops several frames each time it fires, and a C920 will re-hunt whenever a
        // robot crosses the frame. Fixed focus keeps a goal-distance target sharp permanently.
        // Infinity is accepted as a fallback because some C920 firmware exposes that but not Fixed,
        // and at goal distance the two are equivalent.
        if (TurretConstants.LOCK_FOCUS) {
            try {
                FocusControl focus = visionPortal.getCameraControl(FocusControl.class);
                if (focus == null) {
                    report.append("no FocusControl; ");
                    allOk = false;
                } else if (focus.isModeSupported(FocusControl.Mode.Fixed)) {
                    if (focus.setMode(FocusControl.Mode.Fixed)) {
                        appliedFocusMode = "Fixed";
                    } else {
                        report.append("focus Fixed refused; ");
                        allOk = false;
                    }
                } else if (focus.isModeSupported(FocusControl.Mode.Infinity)) {
                    if (focus.setMode(FocusControl.Mode.Infinity)) {
                        appliedFocusMode = "Infinity";
                    } else {
                        report.append("focus Infinity refused; ");
                        allOk = false;
                    }
                } else {
                    report.append("no fixed focus mode supported; ");
                    allOk = false;
                }
            } catch (Exception e) {
                report.append("focus threw: ").append(e.getMessage()).append("; ");
                allOk = false;
            }
        }

        cameraSetupReport = (report.length() == 0) ? "all controls applied" : report.toString();
        if (!allOk && telemetry != null) {
            telemetry.addLine("TURRET CAMERA (attempt " + setupAttempts + "): " + cameraSetupReport);
        }
        return allOk;
    }

    // =============================================================================================
    // Target selection
    // =============================================================================================

    /**
     * Returns EVERY currently-valid cluster for this alliance, newest frame only.
     *
     * An alliance has TWO hives, so this list can legitimately hold two entries, and the aim
     * controller needs both: it maintains a separate filtered field estimate per hive, and handing
     * it only the "best" one would starve whichever hive happened to be further away or partly
     * occluded that frame. That hive's estimate would then go stale precisely while the robot was
     * driving toward it.
     *
     * A cluster is included when it is a cluster detection (not a lone tag), has a pose solution,
     * matches the alliance by metadata name prefix, is scorable (opening up), and clears the
     * coverage gate. Each carries its OWN transitioning flag, because one hive can be mid-tip while
     * the other is perfectly shootable.
     *
     * Never throws. Returns an empty list when vision is down or nothing qualifies.
     *
     * @param alliance the alliance locked in during init
     * @return zero, one or two immutable snapshots, ordered by coverage, best first
     */
    public List<TurretTarget> getAllianceTargets(Alliance alliance) {
        List<TurretTarget> out = new ArrayList<>(2);

        if (aprilTag == null || alliance == null) {
            lastRejectReason = "vision not running";
            return out;
        }

        List<AprilTagDetection> detections;
        try {
            detections = aprilTag.getDetections();
        } catch (Exception e) {
            lastRejectReason = "getDetections threw: " + e.getMessage();
            return out;
        }
        if (detections == null || detections.isEmpty()) {
            lastDetectionCount = 0;
            lastClusterCount = 0;
            lastRejectReason = "no detections";
            return out;
        }

        lastDetectionCount = detections.size();
        int clusters = 0;
        int rejectedAlliance = 0, rejectedFlipped = 0, rejectedCoverage = 0;

        // ===== BEGIN SDK 12 CLUSTER CODE - DISABLED =====
        // Restore this loop together with the import, buildTarget() and the tuning opmode's
        // listing. Full copy at TeamCode/sdk12-cluster-backup/TurretVision.java.bak
        //
        // for (AprilTagDetection detection : detections) {
        //     // A BioBuzz goal is always a four-tag CLUSTER. A lone AprilTagSingleDetection here is
        //     // either a stray tag or one corner of a cluster the SDK could not group, and its pose
        //     // is far too weak to shoot on, so it is skipped rather than used.
        //     if (!(detection instanceof AprilTagClusterDetection)) continue;
        //     AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
        //     clusters++;
        //
        //     // A cluster detection is guaranteed to carry metadata, so there is nothing to
        //     // null-check there - but ftcPose can still be absent if the solver failed this frame.
        //     if (cluster.ftcPose == null) continue;
        //
        //     if (allianceOf(cluster.metadata.name) != alliance) {
        //         rejectedAlliance++;
        //         continue;
        //     }
        //
        //     if (Math.abs(cluster.ftcPose.roll) >= TurretConstants.SCORABLE_MAX_ROLL_DEG) {
        //         // Opening is facing away or down. Shooting at it scores nothing.
        //         rejectedFlipped++;
        //         continue;
        //     }
        //
        //     double coverage = normalizeCoverage(cluster.percentClusterFound);
        //     if (coverage < TurretConstants.MIN_PERCENT_CLUSTER_FOUND) {
        //         rejectedCoverage++;
        //         continue;
        //     }
        //
        //     out.add(buildTarget(cluster, coverage, alliance));
        // }
        // ===== END SDK 12 CLUSTER CODE - DISABLED =====

        // Referenced below so the counters do not become dead locals while the loop is parked.
        clusters = 0;
        rejectedAlliance = rejectedFlipped = rejectedCoverage = 0;

        lastClusterCount = clusters;

        if (out.isEmpty()) {
            lastRejectReason = String.format(
                    "%d clusters: %d wrong alliance, %d flipped, %d below %.0f%% coverage",
                    clusters, rejectedAlliance, rejectedFlipped, rejectedCoverage,
                    TurretConstants.MIN_PERCENT_CLUSTER_FOUND);
        } else {
            lastRejectReason = "";
            // Best coverage first, so getBestTarget() is just the head of this list.
            Collections.sort(out, new Comparator<TurretTarget>() {
                @Override
                public int compare(TurretTarget a, TurretTarget b) {
                    return Double.compare(b.percentFound, a.percentFound);
                }
            });
        }
        return out;
    }

    /**
     * The single best cluster to aim at for this alliance, or null when there is none.
     *
     * "Best" is highest coverage, which is a direct proxy for how well-conditioned the pose
     * solution is. Kept for callers that only want one target; the aim controller uses
     * {@link #getAllianceTargets(Alliance)} instead so it can track both hives.
     *
     * Never throws. A null return means "aim on odometry this loop".
     */
    public TurretTarget getBestTarget(Alliance alliance) {
        List<TurretTarget> all = getAllianceTargets(alliance);
        return all.isEmpty() ? null : all.get(0);
    }

    // ===== BEGIN SDK 12 CLUSTER CODE - DISABLED =====
    // Full copy at TeamCode/sdk12-cluster-backup/TurretVision.java.bak
//     /** Copies the chosen cluster into an immutable {@link TurretTarget}, computing the tip signal. */
//     private TurretTarget buildTarget(AprilTagClusterDetection cluster,
//                                      double coverage,
//                                      Alliance alliance) {
//         String name = cluster.metadata.name;
//         double roll = cluster.ftcPose.roll;
//
//         // ---- TRANSITIONING: is THIS hive mid-tip right now? ------------------------------------
//         //
//         // Tracked per cluster name, not globally, because an alliance has two hives and one can be
//         // tipping while the other is perfectly shootable. A single shared flag would hold fire on
//         // both.
//         //
//         // Two independent signs, either of which is enough:
//         //
//         //  1. Roll is approaching the +-90 flip point. A cell passing through 90 degrees has an
//         //     opening that is edge-on and closing, and a pose solution that is about to become
//         //     ill-conditioned. Firing into it wastes the ball.
//         //  2. Coverage collapsed since the previous frame. A hive rotating away sheds visible tags
//         //     quickly, faster than a robot driving past occludes them, so a sharp drop is a decent
//         //     tip detector even before the roll has moved much.
//         //
//         // While this flag is set the controller FREEZES that hive's estimate rather than tracking a
//         // rotating cell across the field, and blocks firing at it.
//         boolean nearFlip = Math.abs(roll)
//                 >= (TurretConstants.SCORABLE_MAX_ROLL_DEG - TurretConstants.TRANSITION_ROLL_MARGIN_DEG);
//
//         Double previousCoverage = lastCoverageByName.get(name);
//         boolean coverageCollapsed = previousCoverage != null
//                 && (previousCoverage - coverage) >= TurretConstants.TRANSITION_COVERAGE_DROP_PCT;
//         lastCoverageByName.put(name, coverage);
//
//         boolean transitioning = nearFlip || coverageCollapsed;
//
//         return new TurretTarget(
//                 name,
//                 alliance,
//                 cluster.ftcPose.bearing,
//                 cluster.ftcPose.range,
//                 cluster.ftcPose.x,
//                 cluster.ftcPose.y,
//                 cluster.ftcPose.z,
//                 roll,
//                 coverage,
//                 true,               // scorable - already gated by the caller
//                 transitioning,
//                 cluster.frameAcquisitionNanoTime);
//     }
    // ===== END SDK 12 CLUSTER CODE - DISABLED =====

    /**
     * Decodes alliance from a BioBuzz cluster name. Names begin with 'R' for red and 'B' for blue.
     *
     * @return the alliance, or null when the name is empty or starts with neither letter, in which
     *         case the cluster is simply not a target for either side.
     */
    public static Alliance allianceOf(String metadataName) {
        if (metadataName == null || metadataName.isEmpty()) return null;
        char c = Character.toUpperCase(metadataName.charAt(0));
        if (c == 'R') return Alliance.RED;
        if (c == 'B') return Alliance.BLUE;
        return null;
    }

    /**
     * Widens {@code percentClusterFound} to a double.
     *
     * Verified against the SDK 12.0.0 Vision library: the field is an {@code int} already
     * expressed in PERCENT (0-100), not a 0..1 fraction. No scaling is applied, and
     * {@link TurretConstants#MIN_PERCENT_CLUSTER_FOUND} is therefore also in percent. This wrapper
     * exists so that if the SDK ever changes the field's units, there is exactly one place to fix
     * rather than three call sites.
     */
    private static double normalizeCoverage(int rawPercent) {
        return rawPercent;
    }

    // =============================================================================================
    // Accessors
    // =============================================================================================

    /** The crosshair processor, so the opmode/controller can push aim state to it each loop. */
    public TurretAimOverlay getOverlay() {
        return overlay;
    }

    /**
     * The portal as a dashboard-streamable source, or null when vision failed to start.
     *
     * Handing this to {@code FtcDashboard.startCameraStream} mirrors the annotated stream - tag
     * outlines AND the turret crosshair - to a laptop, which is the only practical way to check
     * the crosshair while someone else is driving.
     */
    public CameraStreamSource getCameraStreamSource() {
        return visionPortal;
    }

    /** True once the camera is actually streaming frames. */
    public boolean isStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Current portal state as a string, or "NO PORTAL". Telemetry only. */
    public String getCameraStateString() {
        if (visionPortal == null) return "NO PORTAL";
        return String.valueOf(visionPortal.getCameraState());
    }

    /** True once manual exposure/gain have actually been pushed to the camera. */
    public boolean areCameraSettingsApplied() {
        return cameraSettingsApplied;
    }

    /**
     * What the last camera-setup attempt did, or why it could not finish. Telemetry only.
     * Read this first when tags decode badly - a camera stuck on auto-exposure says so here.
     */
    public String getCameraSetupReport() {
        return cameraSetupReport;
    }

    /** Gain the camera actually accepted, raw units, or -1 if not set yet. */
    public int getAppliedGain() {
        return appliedGain;
    }

    /** Exposure the camera actually accepted, MILLISECONDS, or -1 if not set yet. */
    public int getAppliedExposureMs() {
        return appliedExposureMs;
    }

    /** White balance temperature the camera actually accepted, KELVIN, or -1 if not set yet. */
    public int getAppliedWhiteBalanceK() {
        return appliedWhiteBalanceK;
    }

    /** Focus mode the camera actually accepted, or "not set". */
    public String getAppliedFocusMode() {
        return appliedFocusMode;
    }

    /** Total detections in the last processed frame. Telemetry only. */
    public int getLastDetectionCount() {
        return lastDetectionCount;
    }

    /** Cluster detections in the last processed frame. Telemetry only. */
    public int getLastClusterCount() {
        return lastClusterCount;
    }

    /** Human-readable reason the last {@link #getBestTarget} returned null. Telemetry only. */
    public String getLastRejectReason() {
        return lastRejectReason;
    }

    /**
     * Every cluster in the current frame, unfiltered, for the tuning opmode's listing.
     * Returns an empty list rather than null when vision is down.
     */
    public List<AprilTagDetection> getRawDetections() {
        if (aprilTag == null) return java.util.Collections.emptyList();
        try {
            List<AprilTagDetection> d = aprilTag.getDetections();
            return (d == null) ? java.util.Collections.emptyList() : d;
        } catch (Exception e) {
            return java.util.Collections.emptyList();
        }
    }

    /**
     * Changes exposure and gain at runtime. Used by the tuning opmode to sweep for the shortest
     * reliable exposure without restarting the stream.
     *
     * @param exposureMs exposure, MILLISECONDS
     * @param gain       sensor gain, raw units; negative means "use the camera's maximum"
     */
    public void setExposureAndGain(int exposureMs, int gain) {
        if (visionPortal == null) return;
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure != null) {
                if (exposure.getMode() != ExposureControl.Mode.Manual) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                }
                if (exposure.setExposure((long) exposureMs, TimeUnit.MILLISECONDS)) {
                    appliedExposureMs = exposureMs;
                }
            }
            GainControl gc = visionPortal.getCameraControl(GainControl.class);
            if (gc != null) {
                // Same -1 = "camera maximum" sentinel as the main setup path, and the same clamp
                // into the range the camera reports, so a swept value cannot be silently refused.
                int want = (gain < 0) ? gc.getMaxGain() : gain;
                int use = Math.max(gc.getMinGain(), Math.min(gc.getMaxGain(), want));
                if (gc.setGain(use)) appliedGain = use;
            }
        } catch (Exception ignored) {
            // Sweeping is a tuning convenience; a camera that refuses is not worth an exception.
        }
    }

    /** The camera's maximum supported gain, or -1 when it cannot be read. Tuning opmode only. */
    public int getMaxGain() {
        if (visionPortal == null) return -1;
        try {
            GainControl gc = visionPortal.getCameraControl(GainControl.class);
            return (gc == null) ? -1 : gc.getMaxGain();
        } catch (Exception e) {
            return -1;
        }
    }

    /** Releases the camera. Safe to call twice, and safe to call when startup failed. */
    public void close() {
        if (visionPortal != null) {
            try {
                visionPortal.close();
            } catch (Exception ignored) {
            }
            visionPortal = null;
        }
    }
}
