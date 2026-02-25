package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@Disabled
@TeleOp
public class MotorOpMode extends OpMode {
    private DcMotor motor;
    private double cachedPower = 0;

    @Override
    public void init() {
        telemetry = new FastTelemetry(telemetry);
        motor = hardwareMap.get(DcMotor.class, "turretMotor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        setPower(0.0);

        telemetry.addData("Ticks:", motor.getCurrentPosition());
        telemetry.update();
    }

    private void setPower(double power) {
        if (Math.abs(power - cachedPower) < Constants.MOTOR_CACHING_TOLERANCE) {
            return;
        }
        cachedPower = power;
        motor.setPower(power);
    }
}
