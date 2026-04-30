package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class RedAutoFar extends FarAuto{
    @Override
    protected void setPoses() {
        startPose = startPose.mirror();
        farPickupPose = farPickupPose.mirror();
        farPickupControlPoint = farPickupControlPoint.mirror();
        shootingPose = shootingPose.mirror();
        cornerPickup = cornerPickup.mirror();
        hpEdgePose = hpEdgePose.mirror();
        sweepPose = sweepPose.mirror();
        sweepControlPoint = sweepControlPoint.mirror();
        parkPose = parkPose.mirror();
        goalPose = Constants.RED_GOAL_POSE;
    }

    protected void setColor() {
        Constants.color = Constants.Color.RED;
    }

}
