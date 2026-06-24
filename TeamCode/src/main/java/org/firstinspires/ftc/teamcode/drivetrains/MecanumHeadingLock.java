package org.firstinspires.ftc.teamcode.drivetrains;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.MathHelpers;


public class MecanumHeadingLock extends Mecanum {

    private static final double HEADING_POWER_MIN = -1.0;
    private static final double HEADING_POWER_MAX = 1.0;

    public static double ROTATE_STICK_THRESHOLD = 0.025;
    public static double ANGULAR_VELOCITY_THRESHOLD = 0.2;

    PIDFCoefficients anglePID = new PIDFCoefficients(.75, 0, 0.03, 0);


    private final PIDFController headingPIDF =
            new PIDFController(anglePID);
    private final PIDFController secondaryHeadingPIDF =
            new PIDFController(anglePID);

    private boolean headingLockActive = false;
    private double targetHeading = 0.0;

    public MecanumHeadingLock(Robot robot, Follower follower, Telemetry telemetry) {
        super(robot, follower, telemetry);
    }




    @Override
    public void arcade(double forward, double strafe, double rotateX,double rotateY, double speed, double rotSpeed) {
        //Turn off heading lock while shooting for smoother slowdown.  Otherwise the robot oscillates a little and throws off shooting while  moving.
        //Can also look to fix the heading lock PID, but this is a simpler fix for now.

        double heading = robot != null ? robot.getPose().getHeading() : follower.getHeading();

        if(robot.getInvertedDrive()) {

            double theta = 0;

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




        if(robot != null && robot.getCurrentState() == States.SHOOTING) {
            if(robot.getDisableDrive()) {
                super.arcade(0, 0, 0, 0, speed, rotSpeed);
            } else {



                super.arcade(forward, strafe, rotateX, rotateY, speed, rotSpeed);
            }
        } else {

            double angularVelocityRadPerSec = robot != null ? robot.getAngularVelocity() : 0.0;

            boolean rotateStickActive = Math.abs(rotateX) >= ROTATE_STICK_THRESHOLD;

            if (rotateStickActive) {
                headingLockActive = false;
            } else if (!headingLockActive
                    && Math.abs(angularVelocityRadPerSec) < ANGULAR_VELOCITY_THRESHOLD) {
                headingLockActive = true;
                targetHeading = heading;
            }

            double headingPower = rotateX;
            if (headingLockActive) {
                headingPower = -calculateHeadingPower(targetHeading, heading);
            }


            super.arcade(forward, strafe, headingPower, rotateY, speed, rotSpeed);
        }

    }

    private double calculateHeadingPower(double targetHeading, double currentHeading) {
        PIDFCoefficients primaryCoeffs = PedroConstants.headingCoeffs;
        PIDFCoefficients secondaryCoeffs = PedroConstants.secondaryHeadingCoeffs;
        headingPIDF.setCoefficients(primaryCoeffs);
        secondaryHeadingPIDF.setCoefficients(secondaryCoeffs);

        double headingError = MathFunctions.getTurnDirection(currentHeading, targetHeading)
                * MathFunctions.getSmallestAngleDifference(currentHeading, targetHeading);
        double turnDirection = MathFunctions.getTurnDirection(currentHeading, targetHeading);

        if (PedroConstants.followerConstants.useSecondaryHeadingPIDF
                && Math.abs(headingError) < PedroConstants.followerConstants.headingPIDFSwitch) {
            secondaryHeadingPIDF.updateFeedForwardInput(turnDirection);
            secondaryHeadingPIDF.updateError(headingError);
            return MathFunctions.clamp(
                    secondaryHeadingPIDF.run(),
                    HEADING_POWER_MIN,
                    HEADING_POWER_MAX);
        }

        headingPIDF.updateFeedForwardInput(turnDirection);
        headingPIDF.updateError(headingError);
        return MathFunctions.clamp(
                headingPIDF.run(),
                HEADING_POWER_MIN,
                HEADING_POWER_MAX);
    }

    public void setTargetHeading(double radians) {
        targetHeading = radians;
        headingLockActive = true;
    }
}
