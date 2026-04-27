package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.SwerveHeadingLock;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.shooter.Turret;

public class ShootingState implements State {
    Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Command updateShooter;

    private boolean transitioningState = false;

    public ShootingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initialize(Robot robot, State prevState) {
        schedule(robot.setIntakePower(0));
        schedule(robot.activateShooter());
        schedule(robot.closeGate());
        updateShooter = robot.updateShootingSubsystems();

        if (!Constants.lastOpModeWasAuto) {
            schedule(updateShooter);
        }

//        transitioningState = false;
        transitioningState = true;

        schedule(
                sequential(
//                            waitUntil(robot::readyToShoot).raceWith(infinite(() -> {
//                                telemetry.addData("Waiting to shoot...", "");
//                            })).raceWith(waitMs(300)),
                        robot.shootMotif(),
                        instant(() -> cancel(updateShooter)),
                        instant(() -> robot.setState(States.INTAKING))
                )
        );
    }

    public void execute(Robot robot) {
        if (gamepad2.dpadDownWasPressed()) {
            Constants.debugTelemetry = !Constants.debugTelemetry;
        }

        if (gamepad1.aWasPressed() && !Constants.lastOpModeWasAuto && robot.getDrivetrain() instanceof SwerveHeadingLock) {
            double heading = Constants.color == Constants.Color.RED
                    ? Math.toRadians(32)
                    : Math.toRadians(150);
            ((SwerveHeadingLock) robot.getDrivetrain()).setTargetHeading(heading);
        }

        if (gamepad1.bWasPressed() && !Constants.lastOpModeWasAuto && robot.getDrivetrain() instanceof SwerveHeadingLock) {
            double heading = Constants.color == Constants.Color.RED
                    ? Math.toRadians(180)
                    : Math.toRadians(0);
            ((SwerveHeadingLock) robot.getDrivetrain()).setTargetHeading(heading);
        }

        if (gamepad2.dpadLeftWasPressed() && !Constants.lastOpModeWasAuto) {
            Turret.turretOffsetRad += Math.toRadians(3);
        }

        if (gamepad2.dpadRightWasPressed() && !Constants.lastOpModeWasAuto) {
            Turret.turretOffsetRad -= Math.toRadians(3);
        }
    }

    public String name(){
        return "Shooting";
    }
}
