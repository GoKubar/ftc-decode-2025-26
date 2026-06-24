package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Constants;

@TeleOp
public class BlueFarTele extends BlueTele {

    @Override
    public void initialize() {
        super.initialize();
        robot.setInvertedDrive(true);
    }
}
