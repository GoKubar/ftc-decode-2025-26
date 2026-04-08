package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.execute;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

@Autonomous
public class TInitTuner extends LinearOpMode {
    Robot robot;
    Timer timer;

    double targetTicks;

    double t_init;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);
        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, Constants.BLUE_GOAL_POSE);
        timer = new Timer();

        Constants.lastOpModeWasAuto = true;

        robot.init();


        robot.setPose(new Pose(72, 72, Math.toRadians(135)));

        schedule(
                robot.updateShootingSubsystems(),
                sequential(
                        instant(() -> robot.setState(States.SHOOTING)),
                        waitMs(5000),
                        waitUntil(robot::readyToShoot),
                        instant(() -> {
                            timer.resetTimer();
                            targetTicks = robot.getFlywheelAngularVelocity();
                        }),

                        parallel(
                                robot.shoot(),
                                sequential(
                                        waitUntil(() -> targetTicks - robot.getFlywheelAngularVelocity() > 40),
                                        instant(() -> t_init = timer.getElapsedTime())
                                )
                        ),
                        infinite(() -> telemetry.addData("t_init (millis)", t_init))
                )
        );

        waitForStart();

        while (opModeIsActive()) {
            execute();
        }

        Limelight3A device;
    }
}
