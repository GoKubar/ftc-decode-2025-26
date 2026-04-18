package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

@Config
public class PTO {


    public enum Mode {
        INTAKING,
        LIFTING
    }


    private Mode mode = Mode.INTAKING;

    private MotorEx intakeMotorR;
    private MotorEx intakeMotorL;

    private int liftTarget = -500;


    public PTO(HardwareMap hardwareMap) {
        intakeMotorR = new MotorEx(hardwareMap, "rightIntake");
        intakeMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotorL = new MotorEx(hardwareMap, "leftIntake");
        intakeMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void runLift() {
        return;
    }
}