# SDK 12 AprilTag cluster code - parked

These `.bak` files are the FULL, WORKING versions of two files, written against the
FTC SDK 12.0.0 AprilTag cluster API (`AprilTagClusterDetection`, `percentClusterFound`).

They are parked here because `build.dependencies.gradle` is pinned to SDK **10.1.0**,
which does not contain `AprilTagClusterDetection` at all. Verified by listing the classes
inside `Vision-10.1.0.aar`: the class does not exist in that release.

The live copies in `src/` have had ONLY the cluster-dependent code disabled, inside blocks
marked:

    // ===== BEGIN SDK 12 CLUSTER CODE - DISABLED =====
    // ===== END SDK 12 CLUSTER CODE - DISABLED =====

Everything else in those files still compiles and runs: the VisionPortal, the C920 exposure
and gain configuration, the crosshair overlay, and every getter.

## To restore

1. Bump every `org.firstinspires.ftc:*` line in `build.dependencies.gradle` from
   `10.1.0` to `12.0.0` (the BioBuzz season SDK), and Gradle-sync.
2. Copy these two `.bak` files back over their `src/` counterparts, dropping `.bak`.
3. Fix the two PRE-EXISTING files that SDK 12 also breaks, which are unrelated to the
   turret and were already broken before this change:
       DECODE_subsystems/DecodeCAM.java          lines 95,97,98,100,178,194,196,198,212,213
       opmodes/testing_opmodes/AprilTagLocalization.java   line 51
   Both use `detection.metadata` / `detection.id`, which SDK 12 moved off the now-abstract
   `AprilTagDetection` onto its `AprilTagSingleDetection` subclass.

Do not edit the `.bak` files directly. If the live files change in ways worth keeping,
re-park them here.
