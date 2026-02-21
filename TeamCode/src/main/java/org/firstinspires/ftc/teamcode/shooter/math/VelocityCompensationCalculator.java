package org.firstinspires.ftc.teamcode.shooter.math;

import static org.firstinspires.ftc.teamcode.shooter.Shooter.distance;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

import smile.interpolation.LinearInterpolation;

import java.util.stream.IntStream;

@Config
public class VelocityCompensationCalculator {

    public static double kRad = 40;
    public static double kTan = 80;

    private static final double g = 386.0885; // in / s^2

    /** Shooter offset from robot center (inches) */
    private static final double SHOOTER_OFFSET_X = -1.346;
    private static final double SHOOTER_OFFSET_Y = 0.0;

    /** HOOD ANGLE LIMITS */
    public static final double MIN_HOOD_ANGLE = Math.toRadians(35);
    public static final double MAX_HOOD_ANGLE = Math.toRadians(62.7983);


    public static final double[] DISTANCES = new double[] {
            distance(new Pose(44.41, 103.331), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
            distance(new Pose(54.74, 94.8), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
            distance(new Pose(64, 83.1), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
            distance(new Pose(77.17, 70.93), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
            distance(new Pose(106, 90.7), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
            distance(new Pose(131.66, 109.4), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
            distance(new Pose(75.84, 23.04), Constants.BLUE_GOAL_POSE) - SHOOTER_OFFSET_X,
    };

    public static final double[] SPEEDS = new double[] {
            1050,
            1095,
            1110,
            1204.6,
            1256,
            1322,
            1492.6275
    };

    public static final double[] HOODS = new double[] {
            Math.toRadians(40),
            Math.toRadians(41.23),
            Math.toRadians(42.05),
            Math.toRadians(42.4578),
            Math.toRadians(43.2771),
            Math.toRadians(44.09),
            Math.toRadians(49.83)
    };

    // Interpolators
    public static LinearInterpolation speedLerp = new LinearInterpolation(DISTANCES, SPEEDS);
    public static LinearInterpolation hoodLerp = new LinearInterpolation(DISTANCES, HOODS);
    public static LinearInterpolation vxToDistanceLerp;

    public static final double[] VXS = IntStream.range(0, SPEEDS.length)
            .mapToDouble(i -> SPEEDS[i] * Math.sin(HOODS[i])).toArray();

    public static final double[] SORTED_VXS;
    public static final double[] SORTED_VX_DISTANCES;

    static {
        // Sort VXS and corresponding DISTANCES by VX for the vxToDistance lerp
        Integer[] vxIndices = IntStream.range(0, VXS.length).boxed().toArray(Integer[]::new);
        java.util.Arrays.sort(vxIndices, java.util.Comparator.comparingDouble(i -> VXS[i]));
        SORTED_VXS = java.util.Arrays.stream(vxIndices).mapToDouble(i -> VXS[i]).toArray();
        SORTED_VX_DISTANCES =
                java.util.Arrays.stream(vxIndices).mapToDouble(i -> DISTANCES[i]).toArray();

        vxToDistanceLerp = new LinearInterpolation(SORTED_VXS, SORTED_VX_DISTANCES);
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

    public static ShotParameters calculate(
            Pose robotPose,
            Vector robotVel,
            Pose goalPose,
            ShotParameters output
    ) {

        // shooter offset
        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());

        double shooterX = robotPose.getX() + SHOOTER_OFFSET_X * cosH - SHOOTER_OFFSET_Y * sinH;
        double shooterY = robotPose.getY() + SHOOTER_OFFSET_X * sinH + SHOOTER_OFFSET_Y * cosH;


        // distance to goal with offset
        double dx = goalPose.getX() - shooterX;
        double dy = goalPose.getY() - shooterY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double initial = speedLerp.interpolate(distance) * Math.sin(hoodLerp.interpolate(distance));
        double angleToGoal = Math.atan2(dy, dx);

        Vector shootingVector = new Vector(initial, angleToGoal);
        Vector tangentVector = new Vector(1, angleToGoal + Math.PI / 2);

        double tof = distance / initial;

        // pose + tof * (v_rad * k_rad * v_tan * k_tan)
        // it doesn't matter that tof has the wrong units because of hte scaling factors k_tan and
        // k_rad

        // get components relative to goal for scaling
        double vRad = robotVel.dot(new Vector(1, angleToGoal));
        double vTan = robotVel.dot(tangentVector);

        double scaledVRad = vRad * tof * kRad;
        double scaledVTan = vTan * tof * kTan;

        Vector correctionVector = new Vector();
        correctionVector.setOrthogonalComponents(scaledVRad, scaledVTan);

        // convert back to field space
        correctionVector.rotateVector(angleToGoal);

        Vector correctedShootingVector = shootingVector.minus(correctionVector);

        double targetDistanceFromVx = vxToDistanceLerp.interpolate(correctedShootingVector.getMagnitude());
        double targetAngle = correctedShootingVector.getTheta();

        double flywheelSpeed = speedLerp.interpolate(targetDistanceFromVx);
        double hoodAngle = hoodLerp.interpolate(targetDistanceFromVx);
        double turretAngle = MathHelpers.wrapAngleRadians(targetAngle - robotPose.getHeading());

        output.set(hoodAngle, turretAngle, flywheelSpeed);
        return output;
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
}
