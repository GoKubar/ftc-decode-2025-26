package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class RedAuto21 extends Auto{
    @Override
    protected void setPoses() {
        startPose = startPose.mirror();
        shootingPose = shootingPose.mirror();
        middlePickupPose = middlePickupPose.mirror();
        middlePickupControlPoint = middlePickupControlPoint.mirror();
        gateClearPose = gateClearPose.mirror();
        gateClearControlPoint = gateClearControlPoint.mirror();
        gatePickupControlPoint = gatePickupControlPoint.mirror();
        gatePickupPose = gatePickupPose.mirror();
        closePickupPose = closePickupPose.mirror();
        farPickupPose = farPickupPose.mirror();
        farPickupControlPoint = farPickupControlPoint.mirror();
        cornerPose = cornerPose.mirror();
//        cornerControlPoint = cornerControlPoint.mirror();
        cornerBackupPose = cornerBackupPose.mirror();
        parkPose = parkPose.mirror();
        goalPose = goalPose.mirror();
    }


    protected void setColor() {
        Constants.color = Constants.Color.RED;
    }
}
