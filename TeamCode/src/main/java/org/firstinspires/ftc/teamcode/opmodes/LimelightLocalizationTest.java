package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@TeleOp(name = "Limelight Localization Test", group = "Test")
public class LimelightLocalizationTest extends LinearOpMode {

    private static final int LOCALIZATION_PIPELINE = 2;
    private static final int MIN_TAGS = 2;
    private static final double METERS_TO_INCHES = 39.3701;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LOCALIZATION_PIPELINE);
        limelight.start();

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid result from Limelight.");
                telemetry.update();
                continue;
            }

            int tagCount = result.getFiducialResults().size();
            telemetry.addData("Tags Visible", tagCount);

            if (tagCount < MIN_TAGS) {
                telemetry.addLine("Need >= 2 tags for a fix.");
                telemetry.update();
                continue;
            }

            Pose3D prevPose = result.getBotpose_MT2();
            double seedHeadingDeg = prevPose != null
                    ? prevPose.getOrientation().getYaw(AngleUnit.DEGREES)
                    : 0.0;
            limelight.updateRobotOrientation(seedHeadingDeg);

            Pose3D pose = result.getBotpose_MT2();

            if (pose == null) {
                telemetry.addLine("MT2 pose unavailable.");
                telemetry.update();
                continue;
            }

            double xIn  = pose.getPosition().x * METERS_TO_INCHES;
            double yIn  = pose.getPosition().y * METERS_TO_INCHES;
            double zIn  = pose.getPosition().z * METERS_TO_INCHES;
            double yawDeg   = pose.getOrientation().getYaw(AngleUnit.DEGREES);
            double pitchDeg = pose.getOrientation().getPitch(AngleUnit.DEGREES);
            double rollDeg  = pose.getOrientation().getRoll(AngleUnit.DEGREES);

            telemetry.addLine("=== MT2 Field Pose ===");
            telemetry.addData("X (in)",    "%.2f", xIn);
            telemetry.addData("Y (in)",    "%.2f", yIn);
            telemetry.addData("Z (in)",    "%.2f", zIn);
            telemetry.addData("Yaw (deg)", "%.2f", yawDeg);
            telemetry.addData("Pitch (deg)", "%.2f", pitchDeg);
            telemetry.addData("Roll (deg)", "%.2f", rollDeg);
            telemetry.addLine("=== Raw (meters) ===");
            telemetry.addData("X (m)", "%.4f", pose.getPosition().x);
            telemetry.addData("Y (m)", "%.4f", pose.getPosition().y);
            telemetry.addData("Z (m)", "%.4f", pose.getPosition().z);

            telemetry.update();
        }

        limelight.stop();
    }
}
