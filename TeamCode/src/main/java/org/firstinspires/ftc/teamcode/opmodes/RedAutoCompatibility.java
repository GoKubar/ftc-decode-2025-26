package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class RedAutoCompatibility extends CompatabilityAuto {
    @Override
    protected void setPoses() {
        startPose = startPose.mirror();
//        preloadShootingPose = preloadShootingPose.mirror();
        shootingPose = shootingPose.mirror();
        middlePickupPose = middlePickupPose.mirror();
//        middlePickupControlPoint1 = middlePickupControlPoint1.mirror();
        middlePickupControlPoint2 = middlePickupControlPoint2.mirror();
        gateClearPose = gateClearPose.mirror();
        gateClearControlPoint = gateClearControlPoint.mirror();
        gatePickupControlPoint = gatePickupControlPoint.mirror();
        for (int i = 0; i < gatePickupPoses.length; i++) {
            gatePickupPoses[i] = gatePickupPoses[i].mirror();
        }
        closePickupPose = closePickupPose.mirror();
        farPickupPose = farPickupPose.mirror();
        farPickupControlPoint = farPickupControlPoint.mirror();
        cornerPose = cornerPose.mirror();
        farShootingPose = farShootingPose.mirror();
//        cornerControlPoint = cornerControlPoint.mirror();
        cornerBackupPose = cornerBackupPose.mirror();
        parkPose = parkPose.mirror();
        closeParkPose = closeParkPose.mirror();
        goalPose = Constants.RED_GOAL_POSE;
    }

    @Override
    protected void adjustGoalPose() {
        robot.moveGoalPose(-4, 0);
    }

    protected void setColor() {
        Constants.color = Constants.Color.RED;
    }
}
