package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
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
public class ShootingTest extends LinearOpMode {

    FtcDashboard dashboard;

    Limelight3A limelight;

    Flywheel flywheel;
    Hood hood;
    PTO pto;
    Turret turret;

    ServoEx gatePusher;

    double flywheelTarget = 1000;
    double hoodTarget = 0;
    double turretTarget = 0;

    Drivetrain drivetrain;
    Follower follower;

    Telemetry dashboardTelem;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        gatePusher = new ServoEx(hardwareMap, "gatePush");
//        gatePusher.setPosition(Robot.BLUE_SIDE_OUT);

        telemetry = new FastTelemetry(telemetry);
        Constants.color = Constants.Color.AUDIENCE;

        dashboard = FtcDashboard.getInstance();
        dashboardTelem = dashboard.getTelemetry();

        follower =  PedroConstants.createFollower(hardwareMap);
        Tuning.Drawing.init();
        Pose startPose = new Pose(16.988, 110.641, Math.toRadians(180));
        startPose = startPose.mirror();
        follower.setPose(startPose);
        drivetrain = Drivetrains.SWERVE_HEADING_LOCK.build(null, follower, telemetry);


        flywheel = new Flywheel(hardwareMap);

        pto = new PTO(hardwareMap);
        turret = new Turret(hardwareMap);

        hood = new Hood(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        flywheel.deactivate();

        waitForStart();

        while (opModeIsActive()) {
            limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading() + Math.PI));
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botPoseMT2 = result.getBotpose_MT2();
                Pose2D pose2D = new Pose2D(
                        DistanceUnit.METER,
                        botPoseMT2.getPosition().x,
                        botPoseMT2.getPosition().y,
                        AngleUnit.DEGREES,
                        botPoseMT2.getOrientation().getYaw()
                );

                Pose pedroPose = PoseConverter.pose2DToPose(pose2D, InvertedFTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                Tuning.Drawing.drawRobot(pedroPose,  new Style("", "#3F51B5", 0.75));
            }
            Tuning.Drawing.drawRobot(follower.getPose(),  new Style("", "#66cc33", 0.75));
            Tuning.Drawing.sendPacket();

            flywheelTarget += gamepad1.right_trigger;
            flywheelTarget -= gamepad1.left_trigger;

            if (gamepad1.left_bumper) {
                pto.runIntake(-1);
            } else if (gamepad1.right_bumper) {
                pto.runIntake(1);
            } else {
                pto.runIntake(0);
            }

            if (gamepad1.aWasPressed()) {
                flywheel.activate();
            }


            if (gamepad1.bWasPressed()) {
                flywheel.deactivate();
            }

            if (gamepad1.dpadUpWasPressed()) {
                flywheelTarget+=100;
            }

            if (gamepad1.dpadRightWasPressed()) {
                hoodTarget += 0.01;
                hoodTarget = Math.min(1, hoodTarget);
            }


            if (gamepad1.dpadDownWasPressed()) {
                flywheelTarget-=100;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                hoodTarget -= 0.01;
                hoodTarget = Math.max(0, hoodTarget);
            }

            turretTarget = Math.atan2(Constants.BLUE_GOAL_POSE.mirror().getY() - follower.getPose().getY(),
                    Constants.BLUE_GOAL_POSE.mirror().getX() - follower.getPose().getX());

            turretTarget -= follower.getHeading();

            flywheel.setTargetAngularVelocity(flywheelTarget);
            flywheel.update();
            hood.setTargetPosition(hoodTarget);
            turret.setTurretAngle(turretTarget);
//            turret.update(telemetry);
            follower.update();
            drivetrain.update(gamepad1);



            dashboardTelem.addData("Current Flyhweel Vel", flywheel.getCurrentAngularVel());
            dashboardTelem.addData("Target Flyhweel Vel", flywheel.getTargetAngularVelocity());

            telemetry.addData("Current Angular Vel", flywheel.getCurrentAngularVel());
            telemetry.addData("Target Angular Vel", flywheel.getTargetAngularVelocity());
            telemetry.addData("\n\nCurrent Hood angle", Math.toDegrees(hood.getCurrentHoodAngle()));
//            telemetry.addData("Target Hood angle", hood.getTargetHoodAngle());
//            telemetry.addData("\nCurrent Launch angle", hood.getCurrentLaunchAngle());
//            telemetry.addData("Target Launch angle", hood.getTargetLaunchAngle());
            telemetry.addData("\nCurrent Hood Servo Position", hoodTarget);
            telemetry.addData("\n pose", follower.getPose());
            telemetry.update();
            dashboardTelem.update();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}
