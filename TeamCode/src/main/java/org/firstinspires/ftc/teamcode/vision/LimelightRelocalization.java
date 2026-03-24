package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class LimelightRelocalization {

    private static final int LOCALIZATION_PIPELINE = 2;
    private static final double METERS_TO_INCHES = 39.3701;

    public static int MIN_TAGS_REQUIRED = 1;
    public static double MAX_POSE_JUMP_INCHES = 24.0;

    private final Limelight3A limelight;
    private Pose lastAcceptedPose = null;

    public LimelightRelocalization(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LOCALIZATION_PIPELINE);
        limelight.start();
    }


    public Pose update(double currentHeadingRad, Pose currentOdometryPose) {
        limelight.updateRobotOrientation(Math.toDegrees(currentHeadingRad));

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        int tagCount = result.getFiducialResults().size();
        if (tagCount < MIN_TAGS_REQUIRED) {
            return null;
        }

        Pose3D pose3D = result.getBotpose_MT2();
        if (pose3D == null) {
            return null;
        }

        double x = pose3D.getPosition().x * METERS_TO_INCHES;
        double y = pose3D.getPosition().y * METERS_TO_INCHES;
        double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);
        Pose visionPose = new Pose(x, y, heading);

        if (currentOdometryPose != null && MAX_POSE_JUMP_INCHES > 0) {
            double dx = visionPose.getX() - currentOdometryPose.getX();
            double dy = visionPose.getY() - currentOdometryPose.getY();
            if (Math.hypot(dx, dy) > MAX_POSE_JUMP_INCHES) {
                return null;
            }
        }

        lastAcceptedPose = visionPose;
        return visionPose;
    }

    public Pose getLastAcceptedPose() {
        return lastAcceptedPose;
    }

    public void stop() {
        limelight.stop();
    }
}
