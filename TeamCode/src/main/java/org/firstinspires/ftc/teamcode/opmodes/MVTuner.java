package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.WelfordVariance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Measurement STDEV Tuner")
@Config("Measurement STDEV Tuner")
public class MVTuner extends LinearOpMode {
    public static Follower follower;

    private static final Position cameraPosition = new Position(
            DistanceUnit.MM,
            75.17500,
            86.72638,
            194.45158+2+105.85503,
            0
    );
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -60,
            0,
            0
    );
    public static double actualX = 0;
    public static double actualY = 0;
    public static double actualHeading = 0;
    private final WelfordVariance varianceX = new WelfordVariance();
    private final WelfordVariance varianceY = new WelfordVariance();
    private final WelfordVariance varianceHeading = new WelfordVariance();
    private AprilTagProcessor processor;
    Pose startPose = new Pose(17.735, 110.63, Math.toRadians(180));


    public void initialize() {

        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(544.2876017217492, 543.8059217350639, 332.20336755183894, 248.65289514406953)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();

        follower = PedroConstants.createPinpointFollower(hardwareMap);
        follower.setPose(startPose);
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void runOpMode() {
        initialize();


        waitForStart();

        while (opModeIsActive()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            List<AprilTagDetection> detections = processor.getDetections();
            telemetry.addData("AprilTag/Detections", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.metadata == null || detection.metadata.name.contains("Obelisk"))
                    continue;

                telemetry.addData(detection.metadata.name, detection.robotPose);

                Pose pose = new Pose(
                        detection.robotPose.getPosition().y + 72,
                        -detection.robotPose.getPosition().x + 72,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                );
                actualX = follower.getPose().getX();
                actualY = follower.getPose().getY();

                actualHeading = follower.getPose().getHeading();

                //context.addPose(detection.metadata.name, pose);

                varianceX.update(actualX - pose.getX());
                varianceY.update(actualY - pose.getY());
                varianceHeading.update(actualHeading - pose.getHeading());
            }


            telemetry.addData("variance x", varianceX.variance());
            telemetry.addData("variance y", varianceY.variance());
            telemetry.addData("variance heading", varianceHeading.variance());

            telemetry.addData("stdev x", varianceX.stdDev());
            telemetry.addData("stdev y", varianceY.stdDev());
            telemetry.addData("stdev heading", varianceHeading.stdDev());

            telemetry.addData("mean x", varianceX.mean());
            telemetry.addData("mean y", varianceY.mean());
            telemetry.addData("mean heading", varianceHeading.mean());
            telemetry.update();

        }
    }
}
