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

    public static double kP = 0;
    public static double kD = 0;


    private BasicPID liftPid = new BasicPID(new PIDCoefficients(kP, 0, kD));

    public enum Mode {
        INTAKING,
        LIFTING
    }


    private Mode mode = Mode.INTAKING;

    private double ptoIntakingPos = 0.65;
    private double ptoLiftingPos = 0.6;

    private MotorEx intakeMotorR;
    private MotorEx intakeMotorL;
    private ServoEx ptoServo;

    private double ptoServoTarget = ptoIntakingPos;

    private int liftTarget = -500;


    public PTO(HardwareMap hardwareMap) {
        intakeMotorR = new MotorEx(hardwareMap, "rightIntake");
        intakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotorL = new MotorEx(hardwareMap, "leftIntake");
        intakeMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ptoServo = new ServoEx(hardwareMap, "pto");
        setIntaking();
    }

    public void setIntaking() {
        ptoServoTarget = ptoIntakingPos;
        ptoServo.setPosition(ptoServoTarget);
        mode = Mode.INTAKING;
    }

    public void setLifting() {
        ptoServoTarget = ptoLiftingPos;
        ptoServo.setPosition(ptoServoTarget);
        mode = Mode.LIFTING;
        intakeMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runIntake(double power) {
        if (mode == Mode.LIFTING) {
            return;
        }
        ptoServo.setPosition(ptoServoTarget);
        intakeMotorR.setPower(power);
        intakeMotorL.setPower(power);
    }

    public void runLift() {
        if (mode == Mode.INTAKING) {
            return;
        }
        ptoServo.setPosition(ptoServoTarget);
        double power = Range.clip(liftPid.calculate(liftTarget, intakeMotorR.getCurrentPosition()),
                -1,
                1);

        intakeMotorR.setPower(power);
        intakeMotorL.setPower(power);
    }
}