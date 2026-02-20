package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class NewBotTestOpMode extends OpMode {
    private DcMotor intake;
    private DcMotor intake2;
    private Servo pto;

    private Servo gate;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);

        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setPower(0);

        pto = hardwareMap.get(Servo.class, "pto");
        pto.setPosition(.65);


        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(.502);



    }

    @Override
    public void loop() {

        //pto.setPosition(0);

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
            gate.setPosition(.537);
        }

        if(gamepad1.dpadLeftWasPressed()) {
            double newPosition =  gate.getPosition()-.01;
            newPosition = newPosition < 0 ? 0 : newPosition;
            gate.setPosition(.502);
        }

        intake.setPower(gamepad1.right_trigger);
        intake2.setPower(-gamepad1.right_trigger);

        if(pto.getPosition() >= .65) {
            intake.setPower(-gamepad1.left_trigger);
            intake2.setPower(+gamepad1.left_trigger);
        }
    }
}
