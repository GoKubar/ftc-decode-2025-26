package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

public class Swerve implements Drivetrain {
    Telemetry telemetry;
    protected Follower follower;
    protected Robot robot;

    private static boolean fieldCentric = true;

    public Swerve(Robot robot, Follower follower, Telemetry telemetry) {

        this.robot = robot;
        this.follower = follower;
        follower.getDrivetrain().startTeleopDrive();
        this.telemetry = telemetry;
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        arcade(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, -gamepad1.right_stick_y, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotateX, double rotateY, double speed, double rotSpeed) {
        //[strafe, forward] [
        //
        if (fieldCentric) {
            //TODO: Reverse to blue when needed
            double theta = 0;

            double heading = robot != null ? robot.getPose().getHeading() : follower.getHeading();
            if (Constants.color == Constants.Color.RED) {
                theta = -heading;
            } else if (Constants.color == Constants.Color.BLUE) {
                theta = MathHelpers.wrapAngleRadians(-heading + Math.toRadians(180));
            } else if (Constants.color == Constants.Color.AUDIENCE) {
                theta = MathHelpers.wrapAngleRadians(-heading + Math.toRadians(90));
            }

            double cos = Math.cos(theta);
            double sin = Math.sin(theta);
            double strafeRot = strafe * cos - forward * sin;
            double forwardRot = strafe * sin + forward * cos;
            strafe = strafeRot;
            forward = forwardRot;
        }


        //pedro arcade has forward, left, cc as positive
        ((com.pedropathing.ftc.drivetrains.Swerve) follower.getDrivetrain()).arcadeDrive(forward*speed, -strafe*speed, -rotateX*rotSpeed);
    }

    @Override
    public String name() {
        return "Swerve";
    }
}
