package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

public class Intake {
    private MotorEx intakeMotorR;
    private MotorEx intakeMotorL;

    public Intake(HardwareMap hardwareMap) {
        intakeMotorR = new MotorEx(hardwareMap, "rightIntake");
        intakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotorL = new MotorEx(hardwareMap, "leftIntake");
        intakeMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        intakeMotorR.setPower(power);
        intakeMotorL.setPower(power);
    }
}