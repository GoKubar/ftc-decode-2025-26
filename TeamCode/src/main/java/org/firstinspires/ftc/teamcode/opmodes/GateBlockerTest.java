package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

//@Disabled
@TeleOp
public class GateBlockerTest extends OpMode {
//    private DcMotor intake;
//    private DcMotor turret;

//    private DcMotor leftFlyWheel;
//    private DcMotor rightFlyWheel;
    double target = Shooter.openGatePosition;

    private ServoEx triggerServo;

    @Override
    public void init() {
        telemetry = new FastTelemetry(telemetry);
//        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
//        intake.setPower(0.0);
//
//        turret = hardwareMap.get(DcMotor.class, "turretMotor");
//        turret.setPower(0.0);
//
//        leftFlyWheel = hardwareMap.get(DcMotor.class, "leftFlyWheel");
//        leftFlyWheel.setPower(0.0);
//
//        rightFlyWheel = hardwareMap.get(DcMotor.class, "rightFlyWheel");
//        rightFlyWheel.setPower(0.0);

        triggerServo = new ServoEx(hardwareMap, "gate");
    }

    @Override
    public void loop() {

//        if(gamepad1.right_trigger > 0) {
//            intake.setPower(gamepad1.right_trigger);
//        } else if(gamepad1.left_trigger > 0) {
//            intake.setPower(-gamepad1.left_trigger);
//        } else {
//            intake.setPower(0);
//        }
//
//        leftFlyWheel.setPower(-Math.abs(gamepad1.left_stick_y));
//        rightFlyWheel.setPower(Math.abs(gamepad1.left_stick_y));
//
//        turret.setPower(gamepad1.right_stick_x);

        if(gamepad1.aWasPressed()) {
            target = Shooter.openGatePosition;
        }

        if(gamepad1.bWasPressed()) {
            target = Shooter.closedGatePosition;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            target -= 0.01;
        }

        if (gamepad1.dpadRightWasPressed()) {
            target += 0.01;
        }

        triggerServo.setPosition(target);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
