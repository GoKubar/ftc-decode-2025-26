package org.firstinspires.ftc.teamcode.shooter;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.MathHelpers;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

import smile.interpolation.LinearInterpolation;

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

//    LinearInterpolation flywheelSpeeds = VelocityCompensationCalculator.speedInterpolation;
//    LinearInterpolation hoodServoInterpolation = VelocityCompensationCalculator.hoodServoInterpolation;

    // Tolerances
    public static int flywheelToleranceTicks = 60;
    public static double turretToleranceDegrees = 6.7;
    public static double hoodToleranceDegrees = 2;

    // Gate positions
    public static double openGatePosition = 0.4;
    public static double closedGatePosition = 0.48;

    // Hardware
    Hood hood;
    Flywheel flywheel;
    Turret turret;
    ServoEx gateServo;
    Pose goalPose;

    public double lastTurretAngle;

    public Shooter(HardwareMap hardwareMap, Pose goalPose, VoltageSensor voltageSensor) {
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        turret = new Turret(hardwareMap);
        gateServo = new ServoEx(hardwareMap, "gate");
        this.goalPose = goalPose;
    }

    /**
     * Update shooting subsystems WITH velocity compensation
     */
    public void updateShootingSubsystems(Pose pose, Vector velocity, double angularVel, Telemetry telemetry, boolean useVelocityComp) {
        if (!useVelocityComp) {
            updateShootingSubsystems(pose, telemetry);
            return;
        }

        VelocityCompensationCalculator.ShotParameters shotParameters = VelocityCompensationCalculator.calculate(
                pose,
                velocity,
                angularVel,
                goalPose
        );

        lastTurretAngle = shotParameters.turretAngle;

        flywheel.setTargetAngularVelocity(shotParameters.flywheelTicks);
        hood.setHoodAngle(shotParameters.hoodAngle);
        turret.setTurretAngle(shotParameters.turretAngle);
    }

    /**
     * Update turret only during intaking state
     */
    public void updateTurretOnly(
            Pose pose,
            Vector velocity,
            double angularVel,
            Telemetry telemetry,
            boolean useVelocityComp
    ) {
        if (!useVelocityComp) {
            updateTurretOnly(pose, telemetry);
            return;
        }

        VelocityCompensationCalculator.ShotParameters shotParameters = VelocityCompensationCalculator.calculate(
                pose,
                velocity,
                angularVel,
                goalPose
        );

        lastTurretAngle = shotParameters.turretAngle;

        turret.setTurretAngle(shotParameters.turretAngle);
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
        double dist = distance(pose, goalPose);

//        double flywheelSpeed = flywheelSpeeds.interpolate(dist);
//        double hoodAngle = Hood.servoToHoodAngle.interpolate(hoodServoInterpolation.interpolate(dist));
//        double turretAngle = getTargetTurretAngle(pose);

        double flywheelSpeed = 0;
        double hoodAngle = 0;
        double turretAngle = 0;

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
        hood.setHoodAngle(Hood.MIN_HOOD_ANGLE);
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

    public boolean getFlywheelActivated() {
        return flywheel.getActivated();
    }
    public boolean isFlywheelReady() {
        return flywheel.isReady();
    }
}
