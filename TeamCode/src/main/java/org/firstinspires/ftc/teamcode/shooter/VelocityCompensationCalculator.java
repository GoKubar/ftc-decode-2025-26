package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.LinearInterpolation;

import java.util.Arrays;
import java.util.stream.IntStream;

@Config
public class VelocityCompensationCalculator {
    public static double minFarZoneSpeed = 1690;
    public static double maxAutoSpeed = 1450;

    public static double kAutoRad = 5;
    public static double kAutoTan = 8;

    public static double angularVelScaling = 0.07;

    public static boolean useAutoLimit = false;

//    public static double kRadIn = 7;
//    public static double kRadOut = 15;
//    public static double kTan = 10.5;

    public static double kRadIn = 4.5;
    public static double kRadOut = 4.8;
    public static double kTan = 5.5;

    public static int NUM_ITERATIONS = 2;

    private static final double g = 386.0885; // in / s^2

    /** Shooter offset from robot center (inches) */
    public static final double SHOOTER_OFFSET_X = 0.566141732283465;
    public static final double SHOOTER_OFFSET_Y = 0.0;

    // Anti-diagonal data points: positions where x + y ≈ 141.5
    public static Pose[] tablePositions = {
            new Pose(37.85, 101.5),
            new Pose(47.69, 93.36),
            new Pose(57.22, 80.95),
            new Pose(59.3, 71.4),
            new Pose(85.85, 83.55),
            new Pose(97.48, 97.69),
            new Pose(119.96, 116.388),
            new Pose(65.135, 25.81),
            new Pose (55.62, 11.185),
            new Pose(73.7, 9.626),
            new Pose(97.16, 8.147)
    };

    public static double[] flywheelSpeedValues = {
            1274,
            1336,
            1438,
            1459,
            1536,
            1576,
            1658,
            1708,
            1761,
            1793,
            1941,
    };

    public static double[] hoodServoValues      = {
            0,
            0,
            0.04,
            0.07,
            0.09,
            0.15,
            0.17,
            0.27,
            0.3,
            0.38,
            0.61
    };

    public static double[] distances = IntStream.range(0, tablePositions.length)
            .mapToDouble(i -> distance(new Pose(tablePositions[i].getX() - SHOOTER_OFFSET_X, tablePositions[i].getY()),
                    Constants.BLUE_GOAL_POSE))
            .toArray();

    public static double[] adjustedFlywheelSpeedValues = IntStream.range(0, flywheelSpeedValues.length)
            .mapToDouble(i -> flywheelSpeedValues[i])
            .toArray();


    // Interpolators
    public static LinearInterpolation speedInterpolation = new LinearInterpolation(distances, adjustedFlywheelSpeedValues);
    public static LinearInterpolation hoodServoInterpolation = new LinearInterpolation(distances, hoodServoValues);

    private static double distance(Pose a, Pose b) {
        return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
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
     * @param robotVel Robot velocity (vx, vy in in/s)
     * @param angularVel Angular velocity (omega in rad/s)
     * @param goalPose Goal Position (x,y)
     */
    public static ShotParameters calculate(Pose robotPose, Vector robotVel, double angularVel, Pose goalPose) {
        return calculate(robotPose, robotVel, angularVel, goalPose, new ShotParameters());
    }

    public static ShotParameters calculate(Pose robotPose, Vector robotVel, double angularVel, Pose goalPose, ShotParameters output) {

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
            radialGain = kAutoRad;
            tangentialGain = kAutoTan;
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
            flywheelTicks = Range.clip(flywheelTicks, Arrays.stream(flywheelSpeedValues).min().getAsDouble(),
                    (Constants.lastOpModeWasAuto && useAutoLimit) ? maxAutoSpeed : Arrays.stream(flywheelSpeedValues).max().getAsDouble());
            hoodAngle = Hood.servoToHoodAngle.interpolate(hoodServoInterpolation.interpolate(dist));
            launchAngle = hoodAngleToLaunchAngle(hoodAngle);
            tof = dist / (flywheelTicks * Math.cos(launchAngle));
        }

        if (robotPose.getY() <= Shooter.transitionYValue) {
            flywheelTicks = Math.max(flywheelTicks, minFarZoneSpeed);
        }

        // dx/dy are still signed here for correct angle
        double turretAngle = MathHelpers.wrapAngleRadians(
                Math.atan2(dy, dx) - robotPose.getHeading() - angularVelScaling * angularVel
        );

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
