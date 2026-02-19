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
        schedule(robot.openGate());
        updateShooter = robot.updateShootingSubsystems();

        if (!Constants.lastOpModeWasAuto) {
            schedule(updateShooter);
        }

        transitioningState = false;
    }

    public void execute(Robot robot) {
        if (gamepad2.dpadDownWasPressed()) {
            Constants.debugTelemetry = !Constants.debugTelemetry;
        }

        if (gamepad1.aWasPressed() && !transitioningState && !Constants.lastOpModeWasAuto) {
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

        if (gamepad1.bWasPressed() && !transitioningState && !Constants.lastOpModeWasAuto) {
            transitioningState = true;
            schedule(
                    sequential(
                            instant(() -> cancel(updateShooter)),
                            instant(() -> robot.setState(States.INTAKING))
                    )
            );
        }
    }

    public String name(){
        return "Shooting";
    }
}
