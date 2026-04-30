package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Drivetrains;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.shooter.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.MathHelpers;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

import java.util.List;

@Disabled
@TeleOp
public class LimelightTest extends LinearOpMode {

    private static final double SHOOTER_OFFSET_X = VelocityCompensationCalculator.SHOOTER_OFFSET_X; // inches, forward in robot frame

    private static final Style STYLE_PINPOINT  = new Style("", "#3F51B5", 0.75); // blue
    private static final Style STYLE_LL_TURRET = new Style("", "#FF9800", 0.75); // orange
    private static final Style STYLE_LL_ROBOT  = new Style("", "#4CAF50", 0.75); // green

    Follower follower;
    Turret turret;
    Drivetrain drivetrain;
    Limelight3A limelight;

    PinpointLocalizer pinpoint;

    double turretTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        Constants.color = Constants.Color.RED;

        follower = PedroConstants.createFollower(hardwareMap);
        Pose startPose = new Pose(17.735, 108.74, Math.toRadians(180)).mirror();
        PedroConstants.getPinpointLocalizer().setPose(startPose);
        follower.setPose(startPose);

        drivetrain = Drivetrains.SWERVE_HEADING_LOCK.build(null, follower, telemetry);
        turret = new Turret(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        Drawing.init();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        pinpoint = PedroConstants.getPinpointLocalizer();

        waitForStart();

        while (opModeIsActive()) {
            // --- Pose 1: Pinpoint ---
            pinpoint.update();
            Pose pinpointPose = pinpoint.getPose();
            double pinpointHeading = pinpointPose.getHeading(); // radians, Pedro frame

            // --- Turret aim ---
            Pose goal = Constants.BLUE_GOAL_POSE.mirror();
            turretTarget = Math.atan2(goal.getY() - pinpointPose.getY(),
                    goal.getX() - pinpointPose.getX());
            turretTarget -= pinpointHeading;
            turret.setTurretAngle(turretTarget);

            // --- Feed heading to Limelight for MegaTag2 ---
            // Camera is on the turret: total yaw = robot heading + turret angle (both Pedro radians)
            // Pedro→FTC: Pedro 0° (+x right) = FTC 90° (+y), so ftcDeg = pedroDeg + 90
            double cameraYawPedroRad = pinpointHeading + turret.getTargetAngle();
            double ftcYawDegrees = MathHelpers.wrapAngleDegrees(Math.toDegrees(cameraYawPedroRad) + 90.0);
            limelight.updateRobotOrientation(ftcYawDegrees);

            // --- Get MegaTag2 pose ---
            LLResult result = limelight.getLatestResult();
            boolean hasValidResult = result != null && result.isValid();

            Pose limelightTurretPose = null; // Pose 2: limelight in Pedro coords, at turret position
            Pose limelightRobotPose  = null; // Pose 3: limelight in Pedro coords, at robot center

            if (hasValidResult) {
                Pose3D botpose3D = result.getBotpose_MT2();
                if (botpose3D != null) {
                    // Pack into Pose2D so PoseConverter can handle unit conversion (meters → inches)
                    Pose2D botpose2D = new Pose2D(
                            DistanceUnit.METER,
                            botpose3D.getPosition().x,
                            botpose3D.getPosition().y,
                            AngleUnit.DEGREES,
                            botpose3D.getOrientation().getYaw()
                    );

                    // Convert InvertedFTC (meters) → Pedro coords (inches)
                    Pose ftcPose = PoseConverter.pose2DToPose(botpose2D, InvertedFTCCoordinates.INSTANCE);
                    Pose turretPedro = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                    // Pose 2: turret position in Pedro, heading from pinpoint (not limelight,
                    // since we fed limelight the camera yaw which includes turret angle)
                    limelightTurretPose = turretPedro;

                    // Pose 3: subtract turret offset (SHOOTER_OFFSET_X inches forward in robot frame,
                    // rotated by pinpoint heading into field frame) to get robot center
                    double robotX = turretPedro.getX() - SHOOTER_OFFSET_X * Math.cos(pinpointHeading);
                    double robotY = turretPedro.getY() - SHOOTER_OFFSET_X * Math.sin(pinpointHeading);
                    limelightRobotPose = new Pose(robotX, robotY, pinpointHeading);
                }
            }

            // --- Drive ---
            drivetrain.update(gamepad1);

            // --- Telemetry ---
            telemetry.addLine("=== Pinpoint Pose ===");
            telemetry.addData("  x (in)", pinpointPose.getX());
            telemetry.addData("  y (in)", pinpointPose.getY());
            telemetry.addData("  heading (deg)", Math.toDegrees(pinpointPose.getHeading()));

            telemetry.addLine("=== Limelight Turret Pose (Pedro, pre-transform) ===");
            if (limelightTurretPose != null) {
                telemetry.addData("  x (in)", limelightTurretPose.getX());
                telemetry.addData("  y (in)", limelightTurretPose.getY());
                telemetry.addData("  heading (deg)", Math.toDegrees(limelightTurretPose.getHeading()));
            } else {
                telemetry.addLine("  no valid MT2 result");
            }

            telemetry.addLine("=== Limelight Robot Pose (Pedro, post-transform) ===");
            if (limelightRobotPose != null) {
                telemetry.addData("  x (in)", limelightRobotPose.getX());
                telemetry.addData("  y (in)", limelightRobotPose.getY());
                telemetry.addData("  heading (deg)", Math.toDegrees(limelightRobotPose.getHeading()));
            } else {
                telemetry.addLine("  no valid MT2 result");
            }

            telemetry.addData("limelight valid", hasValidResult);
            telemetry.addData("ftcYaw sent (deg)", ftcYawDegrees);
            telemetry.addData("turret angle (deg)", Math.toDegrees(turret.getTargetAngle()));
            telemetry.update();

            // --- Drawing ---
            Drawing.drawRobot(pinpointPose, STYLE_PINPOINT);
            if (limelightTurretPose != null) Drawing.drawRobot(limelightTurretPose, STYLE_LL_TURRET);
            if (limelightRobotPose  != null) Drawing.drawRobot(limelightRobotPose,  STYLE_LL_ROBOT);
            Drawing.sendPacket();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }

        limelight.stop();
    }
}
