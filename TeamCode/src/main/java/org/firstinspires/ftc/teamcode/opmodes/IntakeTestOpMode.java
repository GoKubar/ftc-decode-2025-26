package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@Disabled
@TeleOp
public class IntakeTestOpMode extends OpMode {
    private DcMotor intake;
    private DcMotor turret;

    private DcMotor leftFlyWheel;
    private DcMotor rightFlyWheel;

    private Servo triggerServo;

    @Override
    public void init() {
        telemetry = new FastTelemetry(telemetry);
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake.setPower(0.0);

        turret = hardwareMap.get(DcMotor.class, "turretMotor");
        turret.setPower(0.0);

        leftFlyWheel = hardwareMap.get(DcMotor.class, "leftFlyWheel");
        leftFlyWheel.setPower(0.0);

        rightFlyWheel = hardwareMap.get(DcMotor.class, "rightFlyWheel");
        rightFlyWheel.setPower(0.0);

        triggerServo = hardwareMap.get(Servo.class, "gate");
    }

    @Override
    public void loop() {

        if(gamepad1.right_trigger > 0) {
            intake.setPower(gamepad1.right_trigger);
        } else if(gamepad1.left_trigger > 0) {
            intake.setPower(-gamepad1.left_trigger);
        } else {
            intake.setPower(0);
        }

        leftFlyWheel.setPower(-Math.abs(gamepad1.left_stick_y));
        rightFlyWheel.setPower(Math.abs(gamepad1.left_stick_y));

        turret.setPower(gamepad1.right_stick_x);

        if(gamepad1.a) {
            triggerServo.setPosition(.3);
        } else if(gamepad1.b) {
            triggerServo.setPosition(.13);
        }
    }
}
