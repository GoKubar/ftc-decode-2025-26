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

    public static final double llOffset = Math.sqrt(Math.pow(5.12, 2) + Math.pow(3.38, 2)); // INCHES
    public static final double turretOffset = VelocityCompensationCalculator.SHOOTER_OFFSET_X;
    public static double varianceMult = 16;
    private final FusionLocalizer fusion;
    Limelight3A limelight;
    LLResult result;

    public MT1Localizer(HardwareMap hardwareMap, Localizer localizer) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        fusion = new FusionLocalizer(
                localizer,
                new Pose(.5, .5, Math.toRadians(2)),
                new Pose(.25, .25, Math.toRadians(0.05) / 60),
                new Pose(0, 0, 0),
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

    public Pose updateLLPose(Follower follower){
        if (limelight.getStatus().getPipelineIndex() != 0){
            limelight.pipelineSwitch(0);
        }
        LLResult result = limelight.getLatestResult();

        Pose2D botPose = null;
        if (result != null && result.isValid()){
            Pose3D pose = result.getBotpose();
            botPose = new Pose2D(DistanceUnit.METER, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES, pose.getOrientation().getYaw());
        }
        Pose pedroPose = null;
        if (botPose != null) {
            pedroPose = PoseConverter.pose2DToPose(botPose, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            double turretOffsetX = llOffset * Math.cos(pedroPose.getHeading());
            double turretOffsetY = llOffset * Math.sin(pedroPose.getHeading());

            pedroPose = new Pose(pedroPose.getX() - turretOffsetX,
                    pedroPose.getY()- turretOffsetY,
                    follower.getHeading());

            double robotCenterOffsetX = turretOffset * Math.cos(pedroPose.getHeading());
            double robotCenterOffsetY = turretOffset * Math.sin(pedroPose.getHeading());

            pedroPose = new Pose(pedroPose.getX() - robotCenterOffsetX,
                    pedroPose.getY()- robotCenterOffsetY,
                    pedroPose.getHeading());
            Drawing.drawRobot(pedroPose);

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


            fusion.addMeasurement(
                    pedroPose,
                    timestampNanos,
                    measurementVariance
            );
            Drawing.drawRobot(pedroPose, ArdCamTesting.STYLE_LL_ROBOT);
            return pedroPose;
        }
        return new Pose(0,0,0);
    }
}