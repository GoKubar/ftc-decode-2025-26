package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.shooter.math.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

import smile.interpolation.LinearInterpolation;

public class Hood {
    private ServoEx hoodServo;

    private double targetHoodAngle;

    public Hood(HardwareMap hardwareMap) {
        hoodServo = new ServoEx(hardwareMap, "hood");
    }

    // Servo position 0 -> 0.83 maps to hood angle min -> max
    private static double[] servoPositions = new double[] {0, 0.77};
    private static double[] hoodAngles = new double[] {
            VelocityCompensationCalculator.getMinHoodAngle(),
            VelocityCompensationCalculator.getMaxHoodAngle()
    };


    private static double[] launchAngles = new double[] {
            VelocityCompensationCalculator.hoodAngleToLaunchAngle(VelocityCompensationCalculator.getMinHoodAngle()),
            VelocityCompensationCalculator.hoodAngleToLaunchAngle(VelocityCompensationCalculator.getMaxHoodAngle())
    };

    public static LinearInterpolation hoodAngleToServo = new LinearInterpolation(hoodAngles, servoPositions);
    public static LinearInterpolation servoToHoodAngle = new LinearInterpolation(servoPositions, hoodAngles);

    public static LinearInterpolation launchAngleToServo = new LinearInterpolation(launchAngles, servoPositions);
    public static LinearInterpolation servoToLaunchAngle = new LinearInterpolation(servoPositions, launchAngles);

    public void setTargetPosition(double position) {
        position = Range.clip(position, 0, 0.83);
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
