package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.opmodes.ArdCamTesting;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.shooter.VelocityCompensationCalculator;

@Configurable
public class MT1Localizer {

    public static final double llXOffset = 4.4144; // INCHES
    public static final double llYOffset = 4.7717;
    public static final double turretOffset = VelocityCompensationCalculator.SHOOTER_OFFSET_X;
    public static double varianceMult = 3;
    private final FusionLocalizer fusion;
    Limelight3A limelight;
    LLResult result;
    public static Pose pedroPose;

    public MT1Localizer(HardwareMap hardwareMap, Localizer localizer) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        fusion = new FusionLocalizer(
                localizer,
                new Pose(.5, .5, Math.toRadians(2)),
                new Pose(1, 1, .1),
                new Pose(-3, -1, -.03),
                100
        );
    }

    /*public void updateResult(double robotHeading){
        limelight.updateRobotOrientation(Math.toDegrees(robotHeading) + 90);
        result = limelight.getLatestResult();
    }*/

    public Localizer getLocalizer() {
        return fusion;
    }

    public Pose updateLLPose(Follower follower, double currentTurretAngle){
        result = limelight.getLatestResult();
        Pose2D botPose = null;
        if (result != null && result.isValid()){
            Pose3D pose = result.getBotpose();
            botPose = new Pose2D(DistanceUnit.METER, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES, pose.getOrientation().getYaw());
        }
        pedroPose = null;
        if (botPose != null) {
            pedroPose = PoseConverter.pose2DToPose(botPose, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            // 1. Rotate camera offset by turret angle (camera is on turret)
            double turretAngle = currentTurretAngle;
            double camOffsetX_turret = llXOffset * Math.cos(turretAngle) - llYOffset * Math.sin(turretAngle);
            double camOffsetY_turret = llXOffset * Math.sin(turretAngle) - llYOffset * Math.cos(turretAngle);

// 2. Add turret offset to get camera position relative to robot center
            double camOffsetX_robot = camOffsetX_turret - turretOffset;
            double camOffsetY_robot = camOffsetY_turret;

// 3. Rotate by robot heading to get offset in world frame
            double worldCamOffsetX = camOffsetX_robot * Math.cos(pedroPose.getHeading()) - camOffsetY_robot * Math.sin(pedroPose.getHeading());
            double worldCamOffsetY = camOffsetX_robot * Math.sin(pedroPose.getHeading()) + camOffsetY_robot * Math.cos(pedroPose.getHeading());

// 4. Subtract from LL pose to get robot center
            pedroPose = new Pose(pedroPose.getX() - worldCamOffsetX,
                    pedroPose.getY() - worldCamOffsetY,
                    pedroPose.getHeading());

            long timestampNanos = System.nanoTime() - result.getStaleness() * 1_000_000L;

            double[] measurementStdDevs = result.getStddevMt1();

            double stdX_in = measurementStdDevs[0] * 39.3701;
            double stdY_in = measurementStdDevs[1] * 39.3701;
            double stdYaw_rad = Math.toRadians(measurementStdDevs[5]);

            Pose measurementVariance = new Pose(
                    stdX_in * stdX_in * varianceMult,
                    stdY_in * stdY_in * varianceMult,
                    stdYaw_rad * stdYaw_rad * varianceMult
            );

            if(pedroPose != null) {
                fusion.addMeasurement(
                        pedroPose,
                        timestampNanos,
                        measurementVariance
                );
                Drawing.drawRobot(pedroPose, ArdCamTesting.STYLE_LL_ROBOT);
            }
            return pedroPose;
        }
        return new Pose(0,0,0);
    }
}