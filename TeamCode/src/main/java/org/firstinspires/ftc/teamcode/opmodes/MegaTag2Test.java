package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@Disabled
@TeleOp(name = "MegaTag2 Test", group = "Test")
public class MegaTag2Test extends LinearOpMode {

    private static final int LOCALIZATION_PIPELINE = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LOCALIZATION_PIPELINE);
        limelight.start();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D pose = result.getBotpose_MT2();
                int tagCount = result.getFiducialResults().size();

                if (pose != null) {
                    telemetry.addData("MT2 X (m)", "%.3f", pose.getPosition().x);
                    telemetry.addData("MT2 Y (m)", "%.3f", pose.getPosition().y);
                    telemetry.addData("MT2 Z (m)", "%.3f", pose.getPosition().z);
                    telemetry.addData("MT2 Yaw (deg)", "%.2f", pose.getOrientation().getYaw(AngleUnit.DEGREES));
                    telemetry.addData("MT2 Pitch (deg)", "%.2f", pose.getOrientation().getPitch(AngleUnit.DEGREES));
                    telemetry.addData("MT2 Roll (deg)", "%.2f", pose.getOrientation().getRoll(AngleUnit.DEGREES));

                    telemetry.addData("X", "%.2f", pose.getPosition().x * 39.3701);
                    telemetry.addData("Y", "%.2f", pose.getPosition().y * 39.3701);
                    telemetry.addData("Heading (rad)", "%.3f", pose.getOrientation().getYaw(AngleUnit.RADIANS));

                    telemetry.addData("Tags Detected", tagCount);
                } else {
                    telemetry.addLine("Pose is null");
                }
            } else {
                telemetry.addLine("No valid result");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
