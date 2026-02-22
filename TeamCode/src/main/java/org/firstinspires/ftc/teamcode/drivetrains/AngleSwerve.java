package org.firstinspires.ftc.teamcode.drivetrains;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@Config
public class AngleSwerve extends Swerve {
    private static final double HEADING_POWER_MIN = -1.0;
    private static final double HEADING_POWER_MAX = 1.0;

    private final PIDFController headingPIDF =
            new PIDFController(PedroConstants.secondaryHeadingCoeffs);//hack to get better coeffs
    private final PIDFController secondaryHeadingPIDF =
            new PIDFController(PedroConstants.secondaryHeadingCoeffs);

    private double angle = Math.toRadians(90);

    public AngleSwerve(Robot robot, Follower follower, Telemetry telemetry) {
        super(robot, follower, telemetry);
    }

    @Override
    public void update(Gamepad gamepad1, double speed, double rotSpeed) {
        super.update(gamepad1, speed, rotSpeed);
    }

    @Override
    public void arcade(double forward, double strafe, double rotateX, double rotateY, double speed, double rotSpeed) {
        Vector2D driveVector = new Vector2D(rotateX, rotateY);

        if (Constants.color == Constants.Color.RED) {
            driveVector.rotateVector(Math.toRadians(-90));
        } else if (Constants.color == Constants.Color.BLUE){
            driveVector.rotateVector(Math.toRadians(90));
        }


        angle = (driveVector.getMagnitude() > 0.15) ? driveVector.getTheta() : angle;

        double heading = robot != null ? robot.getPose().getHeading() : follower.getHeading();
        double headingPower = -calculateHeadingPower(angle, heading);

        super.arcade(forward, strafe, headingPower, 0, speed, rotSpeed);
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
        return "Angle Swerve";
    }
}
