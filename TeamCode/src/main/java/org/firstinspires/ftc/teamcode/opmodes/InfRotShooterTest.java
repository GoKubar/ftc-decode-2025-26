package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

@TeleOp
public class InfRotShooterTest extends LinearOpMode {

    public static double kS = 0.13, kV = 0.000445, kP = 0.015; //TODO: TUNE VALUES

    MotorEx leftFlywheel;
    MotorEx rightFlywheel;

    double target = 0;
    boolean activated = false;

    public void runOpMode() throws InterruptedException {
        leftFlywheel = new MotorEx(hardwareMap, "leftFlywheel");
        rightFlywheel = new MotorEx(hardwareMap, "rightFlywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.dpadUpWasPressed()) {
                target += 100;
            }

            if (gamepad1.dpadDownWasPressed()) {
                target -= 100;
            }

            if (gamepad1.aWasPressed()) {
                activated = !activated;
            }

            target += gamepad1.right_trigger - gamepad1.left_trigger;
            double angularVel = leftFlywheel.getVelocity();

            double power =  (kV * target) + (kP * (target - angularVel)) + kS;
            power = Range.clip(power, -1, 1);

            telemetry.addData("current speed", angularVel);
            telemetry.addData("target speed", target);
            telemetry.addData("target power", power);

            leftFlywheel.setPower((activated) ? power : 0);
            rightFlywheel.setPower((activated) ? power : 0);

            telemetry.update();
        }
    }
}
