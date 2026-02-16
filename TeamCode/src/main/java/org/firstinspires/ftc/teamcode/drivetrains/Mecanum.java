package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Mecanum implements Drivetrain {
    private Telemetry telemetry;
    private com.pedropathing.ftc.drivetrains.Mecanum dt;

    public Mecanum(Robot robot, Follower follower, Telemetry telemetry) {
       dt = (com.pedropathing.ftc.drivetrains.Mecanum) follower.getDrivetrain();
       this.telemetry = telemetry;
    }

    @Override
    public void update(Gamepad gamepad, double speed, double rotSpeed) {
        arcade(-gamepad.left_stick_y, gamepad.left_stick_x*1.1, gamepad.right_stick_x, -gamepad.right_stick_y, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotateX,double rotateY, double speed, double rotSpeed) {
        rotateX *= rotSpeed;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotateX), 1);
        double frontLeftPower = (forward + strafe + rotateX) / denominator;
        double backLeftPower = (forward - strafe + rotateX) / denominator;
        double frontRightPower = (forward - strafe - rotateX) / denominator;
        double backRightPower = (forward + strafe - rotateX) / denominator;

        //pedro impl for run drive sets in this order:
        //motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        dt.runDrive(new double[] {frontLeftPower * speed, backLeftPower * speed, frontRightPower * speed, backRightPower * speed});
    }

    @Override
    public String name() {
        return "Mecanum";
    }
}
