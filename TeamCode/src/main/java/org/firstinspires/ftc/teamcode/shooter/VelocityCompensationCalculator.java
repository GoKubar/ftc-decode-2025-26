package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.LinearInterpolation;

import java.util.Arrays;
import java.util.stream.IntStream;

@Config
public class VelocityCompensationCalculator {
    public static double kAuto = 5;

//    public static double kRadIn = 7;
//    public static double kRadOut = 15;
//    public static double kTan = 10.5;

    public static double kRadIn = 0;
    public static double kRadOut = 0;
    public static double kTan = 0;

    public static int NUM_ITERATIONS = 2;

    private static final double g = 386.0885; // in / s^2

    /** Shooter offset from robot center (inches) */
    private static final double SHOOTER_OFFSET_X = -1.346;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    // Anti-diagonal data points: positions where x + y ≈ 141.5
    public static Pose[] tablePositions = {
            new Pose(104.78, 104.9),
            new Pose(95.20, 92.52),
            new Pose(81.56, 79.82),
            new Pose(71.1, 70.2),
            new Pose(49.36, 93.26),
            new Pose(26.83, 119.07),
            new Pose(61.25, 18.03),
            new Pose(83.14, 4.26),
            new Pose(48.07, 6.16),
    };
    public static double[] distances = IntStream.range(0, tablePositions.length)
            .mapToDouble(i -> distance(new Pose(tablePositions[i].getX() + SHOOTER_OFFSET_X, tablePositions[i].getY()),
                    Constants.BLUE_GOAL_POSE.mirror()))
            .toArray();

    public static double[] flywheelSpeedValues = {918, 925, 1068, 1125, 1161, 1209, 1320, 1317, 1426};

    public static double[] adjustedFlywheelSpeedValues = IntStream.range(0, flywheelSpeedValues.length)
            .mapToDouble(i -> flywheelSpeedValues[i] + 15)
            .toArray();

    public static double[] hoodServoValues      = {0.04, 0.05, 0.06, 0.06, 0.07, 0.08, 0.08, 0.08, 0.09};

    // Interpolators
    public static LinearInterpolation speedInterpolation = new LinearInterpolation(distances, adjustedFlywheelSpeedValues);
    public static LinearInterpolation hoodServoInterpolation = new LinearInterpolation(distances, hoodServoValues);

    private static double distance(Pose a, Pose b) {
        return Math.hypot(-SHOOTER_OFFSET_X + b.getX() - a.getX(), b.getY() - a.getY());
    }

    public static class ShotParameters {
        public double hoodAngle; // radians
        public double turretAngle; // radians (robot frame)
        public double flywheelTicks; // motor ticks/s

        public ShotParameters(double hoodAngle, double turretAngle, double flywheelTicks) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelTicks = flywheelTicks;
        }

        public ShotParameters() {
            this(0, 0, 0);
        }

        public void set(double hoodAngle, double turretAngle, double flywheelTicks) {
            this.hoodAngle = hoodAngle;
            this.turretAngle = turretAngle;
            this.flywheelTicks = flywheelTicks;
        }
    }

    /**
     * Calculate shot parameters with velocity compensation.
     *
     * @param robotPose Robot position (x, y, heading)
     * @param robotVel Robot velocity (vx, vy in in/s, omega in rad/s)
     * @param goalPose Goal Position (x,y)
     */
    public static ShotParameters calculate(Pose robotPose, Vector robotVel, Pose goalPose) {
        return calculate(robotPose, robotVel, goalPose, new ShotParameters());
    }

    public static ShotParameters calculate(Pose robotPose, Vector robotVel, Pose goalPose, ShotParameters output) {

        //shooter offset
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());

        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

        //distance to goal with offset
        double dx = goalPose.getX() - shooterX;
        double dy = goalPose.getY() - shooterY;
        double dist = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        Vector shootingVector = new Vector(1, angleToGoal);
        Vector tangentVector = new Vector(1, angleToGoal + Math.PI/2);

        //velocity decomposition (constant across iterations)
        double vRad = robotVel.dot(shootingVector);
        double vTan = robotVel.dot(tangentVector);
        double radialGain = vRad >= 0 ? kRadIn : kRadOut;
        double tangentialGain = kTan;
        if (Constants.lastOpModeWasAuto) {
            radialGain = kAuto;
            tangentialGain = kAuto;
        }

        //initial TOF estimate from base parameters
        double flywheelTicks = speedInterpolation.interpolate(dist);
        double hoodAngle = Hood.servoToHoodAngle.interpolate(hoodServoInterpolation.interpolate(dist));
        double launchAngle = hoodAngleToLaunchAngle(hoodAngle);
        double tof = dist / (flywheelTicks * Math.cos(launchAngle));

        // iteratively refine: correction → virtual pose → new params → revised TOF
        for (int i = 0; i < NUM_ITERATIONS; i++) {
            double scaledVRad = vRad * tof * radialGain;
            double scaledVTan = vTan * tof * tangentialGain;

            Vector correctionVector = new Vector();
            correctionVector.setOrthogonalComponents(scaledVRad, scaledVTan);
            correctionVector.rotateVector(angleToGoal);

            Pose futurePose = getFuturePose(robotPose, correctionVector);

            double futureShooterX = futurePose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
            double futureShooterY = futurePose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

            dx = goalPose.getX() - futureShooterX;
            dy = goalPose.getY() - futureShooterY;
            dist = Math.sqrt(dx * dx + dy * dy);

            flywheelTicks = speedInterpolation.interpolate(dist);
            flywheelTicks = Math.min(flywheelTicks, Arrays.stream(flywheelSpeedValues).max().getAsDouble());
            hoodAngle = Hood.servoToHoodAngle.interpolate(hoodServoInterpolation.interpolate(dist));
            launchAngle = hoodAngleToLaunchAngle(hoodAngle);
            tof = dist / (flywheelTicks * Math.cos(launchAngle));
        }

        // dx/dy are still signed here for correct angle
        double turretAngle = MathHelpers.wrapAngleRadians(Math.atan2(dy, dx) - robotPose.getHeading());

        output.set(hoodAngle, turretAngle, flywheelTicks);
        return output;
    }


    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.PI / 2 - hoodAngle;
    }

    public static Pose getFuturePose(Pose currentPose, Vector velocityCompensation) {
        return new Pose(
                currentPose.getX() + velocityCompensation.getXComponent(),
                currentPose.getY() + velocityCompensation.getYComponent(),
                currentPose.getHeading()
        );
    }
}
