package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class BlueAutoCompatibility extends CompatabilityAuto {
    @Override
    protected void setPoses() {
        double xOffset = -0.4;
        double yOffset = 0;
        gatePickupPoses = new Pose[] {
                new Pose(13.3 + xOffset, 56 + yOffset, Math.toRadians(150)),
                new Pose(13.3 + xOffset, 55.75 + yOffset, Math.toRadians(150)),
                new Pose(13.3 + xOffset, 55.5 + yOffset, Math.toRadians(150)),
                new Pose(13.3 + xOffset, 55.25 + yOffset, Math.toRadians(150)),
                new Pose(13.3 + xOffset, 55 + yOffset, Math.toRadians(150)),
        };
    }

    @Override
    protected void adjustGoalPose() {
        robot.moveGoalPose(2, 0);
    }

    protected void setColor() {
        Constants.color = Constants.Color.BLUE;
    }
}
