package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

import java.util.List;

@TeleOp
public class TurretTestNew extends LinearOpMode {
    Turret turret;
    double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        turret = new Turret(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        turret.activate();

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (gamepad1.aWasPressed()) {
                target = Math.toRadians(45);
            }

            if (gamepad1.bWasPressed()) {
                target = Math.toRadians(90);
            }


            if (gamepad1.xWasPressed()) {
                target = Math.toRadians(-45);
            }


            if (gamepad1.yWasPressed()) {
                target = Math.toRadians(-90);
            }

            if (gamepad1.dpadDownWasPressed()) {
                target = Math.toRadians(0);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                if (gamepad1.left_bumper) {
                    target += Math.toRadians(1);
//                    target += 0.01;
                } else {
                    target += Math.toRadians(10);
//                    target += 0.05;
                }
            }

            if (gamepad1.dpadRightWasPressed()) {
                if (gamepad1.left_bumper) {
                    target -= Math.toRadians(1);
//                    target -= 0.01;
                } else {
                    target -= Math.toRadians(10);
//                    target -= 0.05;
                }
            }

            turret.setTurretAngle(target);
//            turret.setTargetServoPosition(target);
//            turret.update(telemetry);

            telemetry.addData("Target Angle", Math.toDegrees(turret.getTargetAngle()));
            telemetry.addData("Target Pos", turret.getTargetPosition());
            telemetry.update();
        }
    }
}
