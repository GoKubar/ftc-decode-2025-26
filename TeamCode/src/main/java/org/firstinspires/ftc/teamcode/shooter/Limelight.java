package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;


@Config
public class Limelight {
    Limelight3A limelight;

    public static final double llXOffset = 4.4144; // INCHES
    public static final double llYOffset = 4.7717;
    public static final double turretOffset = VelocityCompensationCalculator.SHOOTER_OFFSET_X;

    private static LLResult result;
    
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose getMT2Pose(double robotAngle, double currentTurretAngle) {
        limelight.updateRobotOrientation(Math.toDegrees(robotAngle - currentTurretAngle));
        result = limelight.getLatestResult();

        Pose2D botPose = null;
        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose_MT2();
            botPose = new Pose2D(DistanceUnit.METER, pose.getPosition().x, pose.getPosition().y, AngleUnit.DEGREES, Math.toDegrees(robotAngle - currentTurretAngle));
        }
        Pose pedroPose = null;
        if (botPose != null) {
            pedroPose = PoseConverter.pose2DToPose(botPose, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            // 1. Rotate camera offset by turret angle (camera is on turret)
            double turretAngle = currentTurretAngle;
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

            Drawing.drawRobot(pedroPose);

            return pedroPose;
        }
        return null;
    }

}
