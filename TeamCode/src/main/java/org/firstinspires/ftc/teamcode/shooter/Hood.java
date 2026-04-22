package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

import smile.interpolation.LinearInterpolation;

public class Hood {
    private ServoEx hoodServo;

    public static double MIN_HOOD_ANGLE = Math.toRadians(36); //TODO: find
    public static double MAX_HOOD_ANGLE = Math.toRadians(68.742078); //TODO: find

    private double targetHoodAngle;

    public Hood(HardwareMap hardwareMap) {
        hoodServo = new ServoEx(hardwareMap, "hood");
        hoodServo.setCachingTolerance(0.005);
    }

    // Servo position 0 -> 0.83 maps to hood angle min -> max
    //1.4440677966 hood angle change per 0.01 change in servo
    private static double[] servoPositions = new double[] {0.04, 0.250916398006};
    private static double[] hoodAngles = new double[] {
            MIN_HOOD_ANGLE,
            MAX_HOOD_ANGLE
    };


    private static double[] launchAngles = new double[] {
            VelocityCompensationCalculator.hoodAngleToLaunchAngle(MIN_HOOD_ANGLE),
            VelocityCompensationCalculator.hoodAngleToLaunchAngle(MAX_HOOD_ANGLE)
    };

    public static LinearInterpolation hoodAngleToServo = new LinearInterpolation(hoodAngles, servoPositions);
    public static LinearInterpolation servoToHoodAngle = new LinearInterpolation(servoPositions, hoodAngles);

    public static LinearInterpolation launchAngleToServo = new LinearInterpolation(launchAngles, servoPositions);
    public static LinearInterpolation servoToLaunchAngle = new LinearInterpolation(servoPositions, launchAngles);

    public void setTargetPosition(double position) {
        position = Range.clip(position, servoPositions[0], servoPositions[1]);
        targetHoodAngle = servoToHoodAngle.interpolate(position);
        hoodServo.setPosition(position);
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public double getTargetLaunchAngle() {
        return VelocityCompensationCalculator.hoodAngleToLaunchAngle(targetHoodAngle);
    }

    private double getCurrentPosition() {
        return hoodServo.getPosition();
    }

    public void setHoodAngle(double radians) {
        setTargetPosition(hoodAngleToServo.interpolate(radians));
    }

    public void setLaunchAngle(double radians) {
        setTargetPosition(launchAngleToServo.interpolate(radians));
    }

    public double getCurrentLaunchAngle() {
        return servoToLaunchAngle.interpolate(getCurrentPosition());
    }

    public double getCurrentHoodAngle() {
        return servoToHoodAngle.interpolate(getCurrentPosition());
    }
}
