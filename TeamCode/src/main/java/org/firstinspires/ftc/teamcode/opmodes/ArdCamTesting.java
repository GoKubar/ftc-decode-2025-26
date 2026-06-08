package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Drivetrains;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.shooter.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp
public class ArdCamTesting extends LinearOpMode {

    private static final double SHOOTER_OFFSET_X = VelocityCompensationCalculator.SHOOTER_OFFSET_X; // inches, forward in robot frame

    public static final Style STYLE_PINPOINT = new Style("", "#3F51B5", 0.75); // blue
    public static final Style STYLE_LL_TURRET = new Style("", "#FF9800", 0.75); // orange
    public static final Style STYLE_LL_ROBOT = new Style("", "#FF0000", 0.75); // green
    public static final Style STYLE_FUSION = new Style("", "#10e044", 0.75);


    private static final Position cameraPosition = new Position(
            DistanceUnit.MM,
            130.17,
            85.73,
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
    public static int latencyMs = 10;
    private AprilTagProcessor processor;
    private FusionLocalizer fusion;
    public static double decisionMarginThreshold = 50;

    Follower follower;
    Turret turret;
    Drivetrain drivetrain;
    VisionPortal visionPortal;

    Localizer pinpoint;

    double turretTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        Constants.color = Constants.Color.RED;

        follower = PedroConstants.createPinpointFollower(hardwareMap);
        Pose startPose = new Pose(17.735, 110.63, Math.toRadians(180)).mirror();
        //PedroConstants.getPinpointLocalizer().setPose(startPose);
        follower.setPose(startPose);

        drivetrain = Drivetrains.SWERVE_HEADING_LOCK.build(null, follower, telemetry);
        turret = new Turret(hardwareMap);

        Drawing.init();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        pinpoint = PedroConstants.getPinpointLocalizer();

        fusion = new FusionLocalizer(
                pinpoint,
                new Pose(0.5, 0.5, Math.toRadians(2)),
                new Pose(.5, .5, Math.toRadians(0.2) / 60),
                new Pose(1, 1, .02),
                100
        );

        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(544.2876017217492, 543.8059217350639, 332.20336755183894, 248.65289514406953)
                .build();

        visionPortal  = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();

        Pose arduTurretPose = null;
        Pose arduRobotPose = null;
        waitForStart();

        while (opModeIsActive()) {
            fusion.update();
            pinpoint.update();
            Pose pinpointPose = pinpoint.getPose();
            double pinpointHeading = pinpointPose.getHeading();


            List<AprilTagDetection> detections = processor.getDetections();
            telemetry.addData("AprilTags/detections", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.metadata == null || detection.metadata.name.contains("Obelisk"))
                    continue;
                telemetry.addData("AprilTags/" + detection.metadata.name + " margin", detection.decisionMargin);
                if (detection.decisionMargin <= decisionMarginThreshold) continue;

                Pose pose = new Pose( // pedro coordinates
                        detection.robotPose.getPosition().y + 72,
                        -detection.robotPose.getPosition().x + 72,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                );


                Pose2D botpose2D = new Pose2D( // standard ftc coordinates
                        DistanceUnit.INCH,
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        AngleUnit.DEGREES,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                );


                Pose ftcPose = PoseConverter.pose2DToPose(botpose2D, InvertedFTCCoordinates.INSTANCE);
                Pose turretPedro = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                arduTurretPose = turretPedro;


                double robotX = turretPedro.getX() - SHOOTER_OFFSET_X * Math.cos(pinpointHeading) + 72;
                double robotY = turretPedro.getY() - SHOOTER_OFFSET_X * Math.sin(pinpointHeading) + 72;
                arduRobotPose = new Pose(robotX, robotY, pinpointHeading);

                telemetry.addData("AprilTags/Robot x", pose.getX());
                telemetry.addData("AprilTags/Robot y", pose.getY());
                telemetry.addData("AprilTags/Robot heading (deg)", Math.toDegrees(pose.getHeading()));

                fusion.addMeasurement(arduRobotPose, System.nanoTime() - latencyMs * 1_000_000L);
                Drawing.drawRobot(pose, STYLE_LL_TURRET);

            }


            // --- Drive ---
            drivetrain.update(gamepad1);


            // --- Telemetry ---
            telemetry.addLine("=== Pinpoint Pose ===");
            telemetry.addData("  x (in)", pinpointPose.getX());
            telemetry.addData("  y (in)", pinpointPose.getY());
            telemetry.addData("  heading (deg)", Math.toDegrees(pinpointPose.getHeading()));

            telemetry.addData("turret angle (deg)", Math.toDegrees(turret.getTargetAngle()));
            telemetry.update();

            // --- Drawing ---
            Drawing.drawRobot(pinpointPose, STYLE_PINPOINT);
            if (arduTurretPose != null) Drawing.drawRobot(arduTurretPose, STYLE_LL_TURRET);
            if (arduRobotPose != null) Drawing.drawRobot(arduRobotPose, STYLE_LL_ROBOT);
            Drawing.drawRobot(fusion.getPose(), STYLE_FUSION);
            Drawing.addTelemetry("Bright green > fusion");
            Drawing.addTelemetry("Orange > arduTurretPose");
            Drawing.addTelemetry("Blue > pinpoint");
            if (arduRobotPose!= null) Drawing.addTelemetry("ardRobotPose", arduRobotPose.getPose());
            Drawing.sendPacket();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}