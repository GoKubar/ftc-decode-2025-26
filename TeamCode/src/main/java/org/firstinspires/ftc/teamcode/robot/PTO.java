package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

public class PTO {

    public enum Mode {
        INTAKING,
        LIFTING
    }

    private Mode mode = Mode.INTAKING;

    private MotorEx intakeMotorR;
    private MotorEx intakeMotorL;
    private ServoEx ptoServo;

    public PTO(HardwareMap hardwareMap) {
        intakeMotorR = new MotorEx(hardwareMap, "rightIntake");
        intakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotorL = new MotorEx(hardwareMap, "leftIntake");
        intakeMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ptoServo = new ServoEx(hardwareMap, "pto");
    }

    public void setIntaking() {
        

    }

    public void setPower(double power) {
        intakeMotorR.setPower(power);
        intakeMotorL.setPower(power);
    }
}