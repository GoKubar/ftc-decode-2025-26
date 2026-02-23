package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class NewBotTestOpMode extends OpMode {
    private DcMotorEx intakeR;
    private DcMotorEx intakeL;
    private Servo pto;

    private Servo gate;

    @Override
    public void init() {
        intakeR = hardwareMap.get(DcMotorEx.class, "rightIntake");
        intakeR.setPower(0);

        intakeL = hardwareMap.get(DcMotorEx.class, "leftIntake");
        intakeL.setPower(0);

        pto = hardwareMap.get(Servo.class, "pto");
        pto.setPosition(.65); //intaking


        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(.502);//open



    }

    @Override
    public void loop() {

        pto.setPosition(0);

        if (gamepad1.aWasPressed()) {
            intakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(gamepad1.rightBumperWasPressed()) {
            double newPosition =  pto.getPosition()+.05;
            newPosition = newPosition > 1 ? 1 : newPosition;
            pto.setPosition(newPosition);
        }

        if(gamepad1.leftBumperWasPressed()) {
            double newPosition =  pto.getPosition()-.05;
            newPosition = newPosition < 0 ? 0 : newPosition;
            pto.setPosition(newPosition);
        }

        if(gamepad1.dpadRightWasPressed()) {
            double newPosition =  gate.getPosition()+.01;
            newPosition = newPosition > 1 ? 1 : newPosition;
//            gate.setPosition(.537);//closed
            gate.setPosition(newPosition);
        }

        if(gamepad1.dpadLeftWasPressed()) {
            double newPosition =  gate.getPosition()-.01;
            newPosition = newPosition < 0 ? 0 : newPosition;
            gate.setPosition(newPosition);
//            gate.setPosition(.502);
        }

        intakeR.setPower(gamepad1.right_trigger);
        intakeL.setPower(-gamepad1.right_trigger);

        if(pto.getPosition() >= .65) {
            intakeR.setPower(-gamepad1.left_trigger);
            intakeL.setPower(+gamepad1.left_trigger);
        }
        telemetry.addData("intake encoder", intakeR.getCurrentPosition());

        telemetry.addData("\npto pos", pto.getPosition());
        telemetry.addData("gate pos", gate.getPosition());
        telemetry.update();
    }
}
