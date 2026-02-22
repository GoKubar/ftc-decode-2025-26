package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import kotlin.ranges.IntRange;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.LinearInterpolation;

import java.util.Arrays;
import java.util.stream.IntStream;

@Config
public class VelocityCompensationCalculator {

    public static double kRad = 5;
    public static double kTan = 5;

    private static final double g = 386.0885; // in / s^2

    /** Shooter offset from robot center (inches) */
    private static final double SHOOTER_OFFSET_X = -1.346;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    /** HOOD ANGLE LIMITS */
    public static final double MIN_HOOD_ANGLE = Math.toRadians(35);
    public static final double MAX_HOOD_ANGLE = Math.toRadians(62.7983);

    public static double[] xPositions = {22.75, 46.75, 58.75, 70.75, 82.75, 94.75,	118.75};
    public static double[] xDistances = IntStream.range(0, xPositions.length)
            .mapToDouble(i -> getXDistance(Constants.BLUE_GOAL_POSE, xPositions[i])).toArray();

    public static double[] yPositions = {11.375, 22.75, 46.75, 70.75, 94.75, 118.75};
    public static double[] yDistances = IntStream.range(0, yPositions.length)
            .mapToDouble(i -> getYDistance(Constants.BLUE_GOAL_POSE, yPositions[i])).toArray();

    public static double[][] flywheelSpeeds = {
            {1839, 1753, 1612, 1497, 1434, 1434},  // x=22.75
            {1903, 1862, 1676, 1541, 1420, 1370},  // x=46.75
            {1973, 1892, 1799, 1558, 1410, 1443},  // x=58.75
            {2120, 1960, 1815, 1579, 1496, 1475},  // x=70.75
            {2215, 2039, 1848, 1631, 1544, 1473},  // x=82.75
            {2201, 2016, 1889, 1734, 1612, 1584},  // x=94.75
            {2320, 2266, 2075, 1912, 1810, 1772},  // x=118.75
    };

    public static double[][] hoodServoPositions = {
            {0.27, 0.24, 0.19, 0.13, 0,    0   },  // x=22.75
            {0.43, 0.39, 0.29, 0.17, 0.05, 0   },  // x=46.75
            {0.48, 0.45, 0.41, 0.15, 0.06, 0   },  // x=58.75
            {0.6,  0.46, 0.43, 0.26, 0.14, 0.06},  // x=70.75
            {0.63, 0.5,  0.45, 0.27, 0.13, 0.09},  // x=82.75
            {0.66, 0.53, 0.45, 0.25, 0.14, 0.11},  // x=94.75
            {0.66, 0.61, 0.55, 0.42, 0.31, 0.21},  // x=118.75
    };

    // Interpolators
    public static BilinearInterpolation speedInterpolation = new BilinearInterpolation(xDistances, yDistances, flywheelSpeeds);
    public static BilinearInterpolation hoodServoInterpolation = new BilinearInterpolation(xDistances, yDistances, hoodServoPositions);
//    public static LinearInterpolation vxToDistanceLerp;
//
//    public static final double[] VXS = IntStream.range(0, SPEEDS.length)
//            .mapToDouble(i -> SPEEDS[i] * Math.sin(HOODS[i])).toArray();
//
//    public static final double[] SORTED_VXS;
//    public static final double[] SORTED_VX_DISTANCES;
//
//    static {
//        // Sort VXS and corresponding DISTANCES by VX for the vxToDistance lerp
//        Integer[] vxIndices = IntStream.range(0, VXS.length).boxed().toArray(Integer[]::new);
//        java.util.Arrays.sort(vxIndices, java.util.Comparator.comparingDouble(i -> VXS[i]));
//        SORTED_VXS = java.util.Arrays.stream(vxIndices).mapToDouble(i -> VXS[i]).toArray();
//        SORTED_VX_DISTANCES =
//                java.util.Arrays.stream(vxIndices).mapToDouble(i -> DISTANCES[i]).toArray();
//
//        vxToDistanceLerp = new LinearInterpolation(SORTED_VXS, SORTED_VX_DISTANCES);
//    }

    public static double getXDistance(Pose goalPose, double xPos) {
        return Math.abs(goalPose.getX() - (xPos + 1.062));
    }

    public static double getYDistance(Pose goalPose, double yPos) {
        return Math.abs(goalPose.getY() - (yPos + SHOOTER_OFFSET_X - 1.436)); //offset to account for how it was tuned
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
        double distance = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        dx = Math.abs(dx);
        dy = Math.abs(dy);

        Vector shootingVector = new Vector(1, angleToGoal);
        Vector tangentVector = new Vector(1, angleToGoal + Math.PI/2);

        double baseFlywheelTicks = speedInterpolation.interpolate(dx, dy);
        double baseHoodAngle = Hood.servoToHoodAngle.interpolate(hoodServoInterpolation.interpolate(dx, dy));

        double launchAngle = hoodAngleToLaunchAngle(baseHoodAngle);
        double baseVx = baseFlywheelTicks * Math.cos(launchAngle);
//        double baseVy = baseFlywheelTicks * Math.sin(launchAngle);


        double tof = distance / baseVx;

        // pose + tof * (v_rad * k_rad * v_tan * k_tan)
        //it doesn't matter that tof has the wrong units because of hte scaling factors k_tan and k_rad

        //get components relative to goal for scaling
        double vRad = robotVel.dot(shootingVector);
        double vTan = robotVel.dot(tangentVector);

        double scaledVRad = vRad * tof * kRad;
        double scaledVTan = vTan * tof * kTan;

        Vector correctionVector = new Vector();
        correctionVector.setOrthogonalComponents(scaledVRad, scaledVTan);

        //convert back to field space
        correctionVector.rotateVector(angleToGoal);

        Pose futurePose = getFuturePose(robotPose, correctionVector);

        shooterX = futurePose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        shooterY = futurePose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;

        //distance to goal including velocity and offset
        dx = goalPose.getX() - shooterX;
        dy = goalPose.getY() - shooterY;
        angleToGoal = Math.atan2(dy, dx);

        dx = Math.abs(dx);
        dy = Math.abs(dy);

        double flywheelSpeed = speedInterpolation.interpolate(dx, dy);
        double hoodAngle = Hood.servoToHoodAngle.interpolate(hoodServoInterpolation.interpolate(dx, dy));
        double turretAngle = MathHelpers.wrapAngleRadians(angleToGoal - futurePose.getHeading());

        return new ShotParameters(
                hoodAngle,
                turretAngle,
                flywheelSpeed
        );
    }


    public static double hoodAngleToLaunchAngle(double hoodAngle) {
        return Math.PI / 2 - hoodAngle;
    }

    public static double getMinHoodAngle() {
        return MIN_HOOD_ANGLE;
    }

    public static double getMaxHoodAngle() {
        return MAX_HOOD_ANGLE;
    }

    public static Pose getFuturePose(Pose currentPose, Vector velocityCompensation) {
        return new Pose(
                currentPose.getX() + velocityCompensation.getXComponent(),
                currentPose.getY() + velocityCompensation.getYComponent(),
                currentPose.getHeading()
        );
    }
}
