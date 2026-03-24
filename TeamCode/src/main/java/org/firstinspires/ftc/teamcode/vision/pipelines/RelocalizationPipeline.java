package org.firstinspires.ftc.teamcode.vision.pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionResult;

/**
 * MegaTag2-based relocalization pipeline. On every call to {@link #process},
 * it feeds the robot's current odometry heading into the Limelight (required
 * for MT2 to resolve tag ambiguity), reads the resulting bot-pose, runs
 * outlier rejection, and – if the fix is valid – directly corrects the
 * robot's global pose via {@link Robot#setPose}.
 *
 * <p>Tunable via FTC Dashboard:
 * <ul>
 *   <li>{@link #MIN_TAGS_REQUIRED} – minimum number of visible AprilTags</li>
 *   <li>{@link #MAX_POSE_JUMP_INCHES} – outlier rejection threshold (0 = disabled)</li>
 * </ul>
 */
@Config
public class RelocalizationPipeline implements VisionPipeline {

    private static final int PIPELINE_INDEX = 2;
    private static final double METERS_TO_INCHES = 39.3701;

    public static int MIN_TAGS_REQUIRED = 1;
    /** Reject fixes whose XY distance from odometry pose exceeds this (inches). 0 = disabled. */
    public static double MAX_POSE_JUMP_INCHES = 24.0;

    private final Telemetry telemetry;
    private Limelight3A limelight;
    private VisionResult result = new VisionResult();

    // State exposed to callers (e.g. test OpMode)
    private Pose lastCorrectedPose = null;
    private int correctionCount = 0;
    private boolean correctedThisLoop = false;
    private int lastTagCount = 0;

    public RelocalizationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(PIPELINE_INDEX);
    }

    @Override
    public void process(Robot robot) {
        correctedThisLoop = false;
        lastTagCount = 0;

        Pose odometryPose = robot.getPose();

        // MT2 requires heading every loop to resolve tag ambiguity
        limelight.updateRobotOrientation(Math.toDegrees(odometryPose.getHeading()));

        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            result = new VisionResult();
            return;
        }

        lastTagCount = llResult.getFiducialResults().size();
        if (lastTagCount < MIN_TAGS_REQUIRED) {
            result = new VisionResult();
            return;
        }

        Pose3D pose3D = llResult.getBotpose_MT2();
        if (pose3D == null) {
            result = new VisionResult();
            return;
        }

        result = new VisionResult(pose3D, lastTagCount);

        double x = pose3D.getPosition().x * METERS_TO_INCHES;
        double y = pose3D.getPosition().y * METERS_TO_INCHES;
        double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);
        Pose visionPose = new Pose(x, y, heading);

        // Outlier rejection: skip large single-loop jumps
        if (MAX_POSE_JUMP_INCHES > 0) {
            double dx = visionPose.getX() - odometryPose.getX();
            double dy = visionPose.getY() - odometryPose.getY();
            if (Math.hypot(dx, dy) > MAX_POSE_JUMP_INCHES) {
                return;
            }
        }

        // Apply correction globally
        robot.setPose(visionPose);
        lastCorrectedPose = visionPose;
        correctionCount++;
        correctedThisLoop = true;
    }

    @Override
    public VisionResult getResult() {
        return result;
    }

    @Override
    public int getPipelineIndex() {
        return PIPELINE_INDEX;
    }

    @Override
    public String name() {
        return "Relocalization";
    }

    /** The last pose accepted and applied as a correction, or {@code null} if none yet. */
    public Pose getLastCorrectedPose() {
        return lastCorrectedPose;
    }

    /** Total number of corrections applied since initialization. */
    public int getCorrectionCount() {
        return correctionCount;
    }

    /** True if a correction was applied during the most recent {@link #process} call. */
    public boolean correctedThisLoop() {
        return correctedThisLoop;
    }

    /** Number of AprilTags seen in the most recent result (0 if no valid result). */
    public int getLastTagCount() {
        return lastTagCount;
    }
}
