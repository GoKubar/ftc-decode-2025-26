package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Constants;

@TeleOp
public class BlueTele extends Tele{
    @Override
    protected void setPoses() {
        //do nothing
    }

    protected void setColor() { Constants.color = Constants.Color.BLUE; }
}
