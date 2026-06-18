package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

@Config
public class Flywheel {
    private MotorEx shooterMotorL;
    private MotorEx shooterMotorR;

    public static double kS = 0.08, kV = 0.000344, kP = 0.003; //TODO: TUNE VALUES

    private double target = 0;
    private boolean activated = false;

    VoltageSensor voltageSensor;

    public Flywheel(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;

        shooterMotorL = new MotorEx(hardwareMap, "leftFlywheel");
        shooterMotorR = new MotorEx(hardwareMap, "rightFlywheel");

        shooterMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     *
     * @param ticksPerSecond motor speed in ticks per second
     * @return flywheel speed in radians per second
     */
    public static double motorTicksToFlywheelRadians(double ticksPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (ticksPerSecond / 28) * 2 * Math.PI * 1.4;
    }

    /**
     *
     * @param radiansPerSecond flywheel speed in radians per second
     * @return motor speed in ticks per second
     */
    public static double flywheelRadiansToMotorTicks(double radiansPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (radiansPerSecond / (2 * Math.PI * 1.4)) * 28;
    }


    public double getCurrentAngularVel() {
        return shooterMotorL.getVelocity();
    }

    public void setTargetAngularVelocity(double target) {
       this.target = target;
    }


    public double getTargetAngularVelocity() {
        return target;
    }

    public void setPower(double power) {
        shooterMotorL.setPower(power);
        shooterMotorR.setPower(power);
    }

    public void deactivate() {
       activated = false;
       setPower(0);
    }

    public void activate() {
        activated = true;
    }

    public boolean isReady() {
        return Math.abs(getTargetAngularVelocity() - getCurrentAngularVel()) <= 40;
    }

    public boolean getActivated() {
        return activated;
    }

    public double getCurrent() {
        return shooterMotorL.getCurrent() + shooterMotorR.getCurrent();
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(0);
        }
    }

    public void update() {
        if (activated) {
            double power =  (kV * getTargetAngularVelocity()) + (kP * (getTargetAngularVelocity() - getCurrentAngularVel())) + kS;
            power *= 12 / voltageSensor.getVoltage();
            setPower(power);
        }
    }
}
