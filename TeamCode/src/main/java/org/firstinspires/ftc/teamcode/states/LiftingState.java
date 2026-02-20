package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class LiftingState implements State {
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    public LiftingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    @Override
    public void initialize(Robot robot, State prevState) {
        if (robot.updateDriveCommand != null) {
            cancel(robot.updateDriveCommand);
        }

        schedule(
                sequential(
                        robot.setIntakePower(0),
                        waitMs(50),
                        robot.setPtoToLifting(),
                        waitMs(50),
                        infinite(robot::runLift)
                )
        );
    }

    @Override
    public void execute(Robot robot) {

    }

    @Override
    public String name() {
        return "Lifting";
    }
}
