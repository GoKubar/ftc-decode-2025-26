package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.shooter.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.WelfordVariance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class LLMVTuner extends LinearOpMode {
    public static Follower follower;

    private static final Style STYLE_PINPOINT = new Style("", "#3F51B5", 0.75); // blue
    private static final Style STYLE_LL_TURRET = new Style("", "#FF9800", 0.75); // orange
    private static final Style STYLE_LL_ROBOT = new Style("", "#FF0000", 0.75); // green
    private static final Style STYLE_FUSION = new Style("", "#10e044", 0.75);

    public static final double llXOffset = 4.4144; // INCHES
    public static final double llYOffset = 4.7717;
    public static final double turretOffset = VelocityCompensationCalculator.SHOOTER_OFFSET_X;
    private final WelfordVariance varianceX = new WelfordVariance();
    private final WelfordVariance varianceY = new WelfordVariance();
    private final WelfordVariance varianceHeading = new WelfordVariance();
    public static double actualX = 0;
    public static double actualY = 0;
    public static double actualHeading = 0;
    Limelight3A limelight;
    LLResult result;
    Turret turret;
    Pose startPose = new Pose(17.735, 110.63, Math.toRadians(180));


    public void initialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        turret = new Turret(hardwareMap);
        turret.setTurretAngle(0);

        Drawing.init();
        follower = PedroConstants.createPinpointFollower(hardwareMap);
        follower.setPose(startPose);
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void runOpMode() {
        initialize();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            result = limelight.getLatestResult();

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            actualX = follower.getPose().getX();
            actualY = follower.getPose().getY();

            actualHeading = follower.getPose().getHeading();
            //
            Pose2D botPose = null;
            if (result != null && result.isValid()){
                Pose3D pose = result.getBotpose();
                botPose = new Pose2D(DistanceUnit.METER, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES, pose.getOrientation().getYaw());
            }
            Pose pedroPose = null;
            if (botPose != null) {
                pedroPose = PoseConverter.pose2DToPose(botPose, FTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                // 1. Rotate camera offset by turret angle (camera is on turret)
                double turretAngle = turret.getCurrentAngle();
                double camOffsetX_turret = llXOffset * Math.cos(turretAngle) - llYOffset * Math.sin(turretAngle);
                double camOffsetY_turret = llXOffset * Math.sin(turretAngle) - llYOffset * Math.cos(turretAngle);

// 2. Add turret offset to get camera position relative to robot center
                double camOffsetX_robot = camOffsetX_turret + turretOffset;
                double camOffsetY_robot = camOffsetY_turret;

// 3. Rotate by robot heading to get offset in world frame
                double worldCamOffsetX = camOffsetX_robot * Math.cos(pedroPose.getHeading()) - camOffsetY_robot * Math.sin(pedroPose.getHeading());
                double worldCamOffsetY = camOffsetX_robot * Math.sin(pedroPose.getHeading()) + camOffsetY_robot * Math.cos(pedroPose.getHeading());

// 4. Subtract from LL pose to get robot center
                pedroPose = new Pose(pedroPose.getX() - worldCamOffsetX,
                        pedroPose.getY() - worldCamOffsetY,
                        pedroPose.getHeading());


                varianceX.update(actualX - pedroPose.getX());
                varianceY.update(actualY - pedroPose.getY());
                varianceHeading.update(actualHeading - pedroPose.getHeading());
                Drawing.drawRobot(pedroPose, STYLE_LL_ROBOT);

                Drawing.addTelemetry("LL Pose", pedroPose);
                telemetry.addData("LL Pose", pedroPose);

                telemetry.addData("variance x", varianceX.variance());
                telemetry.addData("variance y", varianceY.variance());
                telemetry.addData("variance heading", varianceHeading.variance());

                telemetry.addData("mean x", varianceX.mean());
                telemetry.addData("mean y", varianceY.mean());
                telemetry.addData("mean heading", varianceHeading.mean());

                Drawing.addTelemetry("mean x", varianceX.mean());
                Drawing.addTelemetry("mean y", varianceY.mean());
                Drawing.addTelemetry("mean heading", varianceHeading.mean());
                /*double robotCenterOffsetX = turretOffset * Math.cos(pedroPose.getHeading());
                double robotCenterOffsetY = turretOffset * Math.sin(pedroPose.getHeading());

                pedroPose = new Pose(pedroPose.getX() - robotCenterOffsetX,
                        pedroPose.getY() - robotCenterOffsetY,
                        pedroPose.getHeading());*/
                Drawing.drawRobot(pedroPose, STYLE_LL_ROBOT);
            }

            /*if (result != null && result.isValid()) {
                Pose3D pose = result.getBotpose();

                if (pose != null) {
                    Pose pedroPose = new Pose(
                            pose.getPosition().y + 72,
                            pose.getPosition().x + 72,
                            pose.getOrientation().getYaw(AngleUnit.RADIANS)
                    );
                    Drawing.addTelemetry("LL raw pose", pose);

                    varianceX.update(actualX - pedroPose.getX());
                    varianceY.update(actualY - pedroPose.getY());
                    varianceHeading.update(actualHeading - pedroPose.getHeading());
                    Drawing.drawRobot(pedroPose, STYLE_LL_ROBOT);

                    Drawing.addTelemetry("LL Pose", pedroPose);
                    telemetry.addData("LL Pose", pedroPose);

                    telemetry.addData("variance x", varianceX.variance());
                    telemetry.addData("variance y", varianceY.variance());
                    telemetry.addData("variance heading", varianceHeading.variance());

                    telemetry.addData("stdev x", varianceX.stdDev());
                    telemetry.addData("stdev y", varianceY.stdDev());
                    telemetry.addData("stdev heading", varianceHeading.stdDev());

                    telemetry.addData("mean x", varianceX.mean());
                    telemetry.addData("mean y", varianceY.mean());
                    telemetry.addData("mean heading", varianceHeading.mean());
                }
            }*/

            //context.addPose(detection.metadata.name, pose);

            Drawing.addTelemetry("Follower", follower.getPose());
            Drawing.drawRobot(follower.getPose(), STYLE_PINPOINT);
            Drawing.sendPacket();
            telemetry.update();

        }
    }
}
