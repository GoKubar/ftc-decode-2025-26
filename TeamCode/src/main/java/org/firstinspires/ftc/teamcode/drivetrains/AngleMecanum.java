package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AngleMecanum extends Mecanum{
    //AnglePID

    public AngleMecanum(Robot robot, Follower follower, Telemetry telemetry) {
        super(robot, follower, telemetry);
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        //TODO:    implement
        super.update(gamepad1, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotateX, double rotateY, double speed, double rotSpeed) {
        //TODO:    implement
        super.arcade(forward, strafe, rotateX, rotateY, speed, rotSpeed);
    }

    @Override
    public String name() {
        return "Angle Mecanum";
    }
}
