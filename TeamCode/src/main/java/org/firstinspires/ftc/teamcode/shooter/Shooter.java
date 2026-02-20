package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shooter.math.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.MathHelpers;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

import smile.interpolation.Interpolation;

/**
 * Shooter class using simplified VelocityCompensationCalculator
 */
public class Shooter {

    public static double transitionYValue = 40;

    public static double distance(Pose launchPose, Pose goalPose) {
        return Math.hypot(goalPose.getX() - launchPose.getX(), goalPose.getY() - launchPose.getY());
    }

    // Fallback interpolation tables (used when velocity comp is off)
//    private double[] closeDistances = new double[] {
//            distance(new Pose(48, 96), Constants.BLUE_GOAL_POSE.mirror()),
//            distance(new Pose(72, 72), Constants.BLUE_GOAL_POSE.mirror()),
//            distance(new Pose(88, 85), Constants.BLUE_GOAL_POSE.mirror()),
//            distance(new Pose(90, 90), Constants.BLUE_GOAL_POSE.mirror()),
//            distance(new Pose(96, 96), Constants.BLUE_GOAL_POSE.mirror()),
//            distance(new Pose(102, 102), Constants.BLUE_GOAL_POSE.mirror()),
//    };
//
//    private double[] closeSpeeds = new double[] { 1315, 1220, 1120, 1078, 1034, 1006 };
//    private double[] closeAngles = new double[] {
//            Math.toRadians(54.74), Math.toRadians(42.05), Math.toRadians(40.0),
//            Math.toRadians(40.0), Math.toRadians(40.0), Math.toRadians(40.0)
//    };
//
//    private double[] farDistances = new double[] {
//            distance(new Pose(72, 24), Constants.BLUE_GOAL_POSE),
//            distance(new Pose(84, 12), Constants.BLUE_GOAL_POSE)
//    };
//    private double[] farSpeeds = new double[] { 1712, 1712 };
//    private double[] farAngles = new double[] { Math.toRadians(64.17), Math.toRadians(61.71) };

    Interpolation flywheelSpeeds = VelocityCompensationCalculator.speedLerp;
    Interpolation hoodAngles = VelocityCompensationCalculator.hoodLerp;

    // Tolerances
    public static int flywheelToleranceTicks = 60;
    public static double turretToleranceDegrees = 6.7;
    public static double hoodToleranceDegrees = 2;

    // Gate positions
    public static double openGatePosition = 0.502;
    public static double closedGatePosition = 0.537;

    // Hardware
    Hood hood;
    Flywheel flywheel;
    Turret turret;
    ServoEx gateServo;
    Pose goalPose;

    public double lastTurretAngle;

    private final VelocityCompensationCalculator.ShotParameters cachedShotParameters =
            new VelocityCompensationCalculator.ShotParameters();

    public Shooter(HardwareMap hardwareMap, Pose goalPose) {
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        gateServo = new ServoEx(hardwareMap, "gate");
        this.goalPose = goalPose;
    }

    /**
     * Update shooting subsystems WITH velocity compensation
     */
    public void updateShootingSubsystems(
            Pose pose,
            Vector velocity,
            Telemetry telemetry,
            boolean useVelocityComp
    ) {
        if (!useVelocityComp) {
            updateShootingSubsystems(pose, telemetry);
            return;
        }

        VelocityCompensationCalculator.calculate(
                pose, velocity,
                goalPose,
                cachedShotParameters
        );

        lastTurretAngle = cachedShotParameters.turretAngle;

        flywheel.setTargetAngularVelocity(cachedShotParameters.flywheelTicks);
        hood.setHoodAngle(cachedShotParameters.hoodAngle);
        turret.setTurretAngle(cachedShotParameters.turretAngle);
    }

    /**
     * Update turret only during intaking state
     */
    public void updateTurretOnly(
            Pose pose,
            Vector velocity,
            Telemetry telemetry,
            boolean useVelocityComp
    ) {
        if (!useVelocityComp) {
            updateTurretOnly(pose, telemetry);
            return;
        }

        VelocityCompensationCalculator.calculate(
                pose, velocity,
                goalPose,
                cachedShotParameters
        );

        lastTurretAngle = cachedShotParameters.turretAngle;

        turret.setTurretAngle(cachedShotParameters.turretAngle);
        flywheel.setPower(0);
    }

    public void updateTurretOnly(Pose pose, Telemetry telemetry) {
        double turretAngle = getTargetTurretAngle(pose);

        lastTurretAngle = turretAngle;

        turret.setTurretAngle(turretAngle);
    }

    /**
     * Update shooting subsystems WITHOUT velocity compensation (fallback/simple mode)
     */
    public void updateShootingSubsystems(Pose pose, Telemetry telemetry) {
        Interpolation flywheelInterpolation = flywheelSpeeds;
        Interpolation hoodAngleInterpolation = hoodAngles;

        double dist = distance(pose, goalPose);
        double flywheelSpeed = flywheelInterpolation.interpolate(dist);
        double hoodAngle = hoodAngleInterpolation.interpolate(dist);
        double turretAngle = getTargetTurretAngle(pose);

        lastTurretAngle = turretAngle;

        flywheel.setTargetAngularVelocity(flywheelSpeed);
        hood.setHoodAngle(hoodAngle);
        turret.setTurretAngle(turretAngle);
    }

    public void setTurretAngle(double angle) {
        turret.setTurretAngle(angle);
    }

    public double getTargetTurretAngle(Pose pose) {
        double targetAngle = Math.atan2(
                goalPose.getY() - pose.getY(),
                goalPose.getX() - pose.getX()
        );
        targetAngle -= pose.getHeading();
        targetAngle = MathHelpers.wrapAngleRadians(targetAngle);
        return targetAngle;
    }

    public boolean readyToShoot() {
        return Math.abs(flywheel.getTargetAngularVelocity() - flywheel.getCurrentAngularVel()) < flywheelToleranceTicks;
    }

    public void activate() {
        flywheel.activate();
    }

    public void intakingPos() {
        flywheel.deactivate();
        turret.setTurretAngle(0);
        hood.setHoodAngle(VelocityCompensationCalculator.getMinHoodAngle());
    }

    public void deactivate() {
        flywheel.deactivate();
    }

    public void deactivateFlywheel() {
        flywheel.deactivate();
    }

    public void toggle() {
        flywheel.toggle();
    }

    public void update(Telemetry telemetry) {
        flywheel.update();
    }

    public void setOpenGatePosition() { gateServo.setPosition(openGatePosition); }
    public void setCloseGatePosition() { gateServo.setPosition(closedGatePosition); }

    public double getTurretPos() { return turret.getTargetPosition(); }
    public double getTurretAngle() { return turret.getTargetAngle(); }
    public double getFlywheelAngularVelocity() { return flywheel.getCurrentAngularVel(); }
    public double getFlywheeelTargetAngularVelocity() { return flywheel.getTargetAngularVelocity(); }
    public double getHoodAngle() { return hood.getCurrentHoodAngle(); }
}
