package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Drivetrains;
import org.firstinspires.ftc.teamcode.robot.PTO;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.shooter.Hood;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

import java.util.List;

@TeleOp
public class IntakingTest extends LinearOpMode {

    FtcDashboard dashboard;

    PTO pto;

    ServoEx servo;

    double servoTarget = 0.5;

    Telemetry dashboardTelem;


    @Override
    public void runOpMode() throws InterruptedException {
//        gatePusher.setPosition(Robot.BLUE_SIDE_OUT);

        telemetry = new FastTelemetry(telemetry);

        dashboard = FtcDashboard.getInstance();
        dashboardTelem = dashboard.getTelemetry();

        pto = new PTO(hardwareMap);

        servo = new ServoEx(hardwareMap, "test servo");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(servoTarget);
            if (gamepad1.left_bumper) {
                pto.runIntake(-1);
            } else if (gamepad1.right_bumper) {
                pto.runIntake(1);
            } else {
                pto.runIntake(0);
            }



            telemetry.addData("servo target", servoTarget);
            telemetry.update();
            dashboardTelem.update();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}
