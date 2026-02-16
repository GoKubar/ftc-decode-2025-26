package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

@FunctionalInterface
public interface DrivetrainSupplier {
    Drivetrain get(Robot robot, Follower follower, Telemetry telemetry);
}
