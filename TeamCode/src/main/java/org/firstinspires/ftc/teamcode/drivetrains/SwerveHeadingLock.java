package org.firstinspires.ftc.teamcode.drivetrains;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class SwerveHeadingLock extends Swerve {
    private static final double HEADING_POWER_MIN = -1.0;
    private static final double HEADING_POWER_MAX = 1.0;

    public static double ROTATE_STICK_THRESHOLD = 0.025;
    public static double ANGULAR_VELOCITY_THRESHOLD = 0.2;

    private final PIDFController headingPIDF =
            new PIDFController(PedroConstants.secondaryHeadingCoeffs);
    private final PIDFController secondaryHeadingPIDF =
            new PIDFController(PedroConstants.secondaryHeadingCoeffs);

    private boolean headingLockActive = false;
    private double targetHeading = 0.0;

    public SwerveHeadingLock(Robot robot, Follower follower, Telemetry telemetry) {
        super(robot, follower, telemetry);
    }

    @Override
    public void arcade(
            double forward,
            double strafe,
            double rotateX,
            double rotateY,
            double speed,
            double rotSpeed) {
        double heading = robot != null ? robot.getPose().getHeading() : follower.getHeading();
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

    @Override
    public String name() {
        return "Swerve Heading Lock";
    }
}
