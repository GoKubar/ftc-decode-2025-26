package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class BlueAutoFar extends FarAuto{
    @Override
    protected void setPoses() {
        //do nothing
    }

    protected void setColor() {
        Constants.color = Constants.Color.BLUE;
    }
}
