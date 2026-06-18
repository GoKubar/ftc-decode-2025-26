package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

@Config
public class PTO {


    public enum Mode {
        INTAKING,
        LIFTING
    }


    private Mode mode = Mode.INTAKING;

    private MotorEx intakeMotorR;
    private MotorEx intakeMotorL;
    private DigitalChannel beam1;
    private DigitalChannel beam2;

    private int liftTarget = -500;


    public PTO(HardwareMap hardwareMap) {
        intakeMotorR = new MotorEx(hardwareMap, "rightIntake");
        intakeMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotorL = new MotorEx(hardwareMap, "leftIntake");
        intakeMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");
        beam2 = hardwareMap.get(DigitalChannel.class, "beam2");
        beam1.setMode(DigitalChannel.Mode.INPUT);
        beam2.setMode(DigitalChannel.Mode.INPUT);

        setIntaking();
    }

    public void setIntaking() {
        mode = Mode.INTAKING;
    }

    public void setLifting() {
        mode = Mode.LIFTING;
        intakeMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runIntake(double power) {
        if (mode == Mode.LIFTING) {
            return;
        }
        intakeMotorR.setPower(power);
        intakeMotorL.setPower(power);
    }

    public boolean isBeamBroken(){
        // Both beams watch the same ball slot; either one can detect a ball if the other sees a hole.
        return !beam1.getState() || !beam2.getState();
    }

    public double getCurrent() {
        return intakeMotorL.getCurrent() + intakeMotorR.getCurrent();
    }

    public void runLift() {
        return;
    }
}
