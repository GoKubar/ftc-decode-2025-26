package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.TestTurret;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@TeleOp
public class NoLimelightTurretTest extends LinearOpMode {
    private TestTurret testTurret;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        testTurret = new TestTurret(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            testTurret.turnToAngle(90);
            telemetry.update();
        }
    }
}
