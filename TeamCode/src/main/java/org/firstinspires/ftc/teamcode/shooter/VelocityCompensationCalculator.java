package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import smile.interpolation.LinearInterpolation;

@Config
public class VelocityCompensationCalculator {
    public static double[] flywheelTicksPerSecond =
            {800.00, 900.00, 1000.00, 1100.00, 1200.00, 1300.00, 1400.00};
    public static double[] calculatedVelocitiesInPerSecond =
            {165.77, 181.93, 203.48, 217.67, 233.57, 248.37, 264.85};
    public static LinearInterpolation inchesToFlywheelTicks =
            new LinearInterpolation(calculatedVelocitiesInPerSecond, flywheelTicksPerSecond);

    public static double MAX_FLYWHEEL_TPS = 1450;

    public static double flywheelBaseHeight = 11.746346063 + (3.7295275591*Math.sin(Hood.MIN_HOOD_ANGLE));
    // maybe account for different heights do to hood angle somehow

    public static double passthroughPointHeight = 41;
    public static double passthroughAngle = -Math.toRadians(30);


    private static final double SHOOTER_OFFSET_X = -1.346; //TODO: double check that these are still accurate
    private static final double SHOOTER_OFFSET_Y = 0.0;

    private static final double g = 386.0886; //g in in/s^2

    public static class ShotParameters {
        public double flywheelTicks;
        public double hoodAngle;
        public double turretAngle;

        public ShotParameters(double flywheelTicks, double hoodAngle, double turretAngle) {
            this.flywheelTicks = flywheelTicks;
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
        }
    }

    public static ShotParameters calculate(Pose robotPose, Vector velocity, Pose goalPose) {
        //Adjust pose to be the shooter pose rather than the robot pose
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());

        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

        robotPose = new Pose(shooterX, shooterY, robotPose.getHeading());

        //Calculate the distances
        double lateralDistance = distance(robotPose, goalPose);
        double verticalDistance = passthroughPointHeight - flywheelBaseHeight;

        //Calculate launch angle (creating a parabola with 2 known points and a known derivative at the second point to solve for derivative at first point)
        double launchAngle = Math.atan((2*verticalDistance) / lateralDistance - Math.tan(passthroughAngle));

        //Calculate desired net velocity given the launch angle
        double desiredVelocity = Math.sqrt(
            (g * lateralDistance * lateralDistance) / (2 * Math.cos(launchAngle) * Math.cos(launchAngle) * (lateralDistance * Math.tan(launchAngle) - verticalDistance))
        );

        double desiredLateralVelocity = desiredVelocity * Math.cos(launchAngle);
        double desiredVerticalVelocity = desiredVelocity * Math.sin(launchAngle);

        //Compensation for robot velocity
        double angleToGoal = Math.atan2(goalPose.getY() - robotPose.getY(), goalPose.getX() - robotPose.getX());
        Vector desiredLateralShootingVector = new Vector(desiredLateralVelocity, angleToGoal);

        Vector compensatedLateralShootingVector = desiredLateralShootingVector.minus(velocity);

        //adjust launch angle given the new compensated velocity
        launchAngle = Math.atan2(desiredVerticalVelocity, compensatedLateralShootingVector.getMagnitude());
        desiredVelocity = Math.hypot(compensatedLateralShootingVector.getMagnitude(), desiredVerticalVelocity);

        return new ShotParameters(
                Math.min(inchesToFlywheelTicks.interpolate(desiredVelocity), MAX_FLYWHEEL_TPS), //flywheel speed
                launchAngleToHoodAngle(launchAngle), //hood angle
                compensatedLateralShootingVector.getTheta() - robotPose.getHeading()//turret angle
        );
    }

    public static double launchAngleToHoodAngle(double launchAngle) {
        return Math.PI/2 - launchAngle;
    }

    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.PI/2 - hoodAngle;
    }
}

