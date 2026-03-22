package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

public abstract class Tele extends LinearOpMode {
    Robot robot;
    Pose goalPose = Constants.BLUE_GOAL_POSE;
    //default startPose
    //length: 15.28
    //width: 15.12
    Pose startPose = new Pose(18.865, 114.22, Math.toRadians(180));
    double lastTurretTicksAtEndOfAuto = -999999;
    boolean wasLastOpModeAuto = false;


    public void initialize() {
        setPoses();
        Scheduler.reset();

        if (!Constants.lastOpModeWasAuto) {
            wasLastOpModeAuto = false;
            Constants.reset();
            //robot needs to be created after Constants.reset() probably
            robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
            robot.setPose(startPose);
        } else {
            Turret.turretOffsetRad = 0;
            wasLastOpModeAuto = true;
            robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
            robot.setPose(Constants.lastPose);
        }

        robot.setLocalizationMode(Robot.LocalizationMode.PINPOINT);

        setColor();

       Constants.lastOpModeWasAuto = false;

        robot.init();

        robot.updateDriveCommand = infinite(robot::updateDrive);
        Scheduler.schedule(robot.updateDriveCommand);
    }

    abstract void setPoses();

    abstract void setColor();

    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
//        while (opModeInInit()) {
//            telemetry.addData("lastTurretTicksAtEndOfAuto", lastTurretTicksAtEndOfAuto);
//            telemetry.addData("wasLastOpmodeAuto", wasLastOpModeAuto);
//            telemetry.update();
//        }

        initialize();


        waitForStart();
        robot.setState(States.INTAKING);
        while (opModeIsActive()) {
            robot.clearCaches();
            Scheduler.execute();
        }
    }
}
