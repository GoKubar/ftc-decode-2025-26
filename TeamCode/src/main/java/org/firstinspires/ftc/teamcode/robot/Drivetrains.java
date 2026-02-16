package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.AngleMecanum;
import org.firstinspires.ftc.teamcode.drivetrains.AngleSwerve;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrains.DrivetrainSupplier;
import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.drivetrains.Swerve;

public enum Drivetrains {
    SWERVE(Swerve::new),
    SWERVE_ANGLE(AngleSwerve::new),
    MECANUM(Mecanum::new),
    MECANUM_ANGLE(AngleMecanum::new);


    DrivetrainSupplier supplier;

    Drivetrains(DrivetrainSupplier supplier) {
        this.supplier = supplier;
    }

    public Drivetrain build(Robot robot, Follower follower, Telemetry telemetry) {
        return supplier.get(robot, follower, telemetry);
    }
}
