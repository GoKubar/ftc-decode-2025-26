package org.firstinspires.ftc.teamcode.states;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.bindings.Binding;
import com.pedropathing.ivy.bindings.Bindings;
import com.qualcomm.robotcore.hardware.Gamepad;

import static com.pedropathing.ivy.bindings.Bindings.bind;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.shooter.Turret;

public class IntakingState implements State {

    double length = 15.39;
    double width = 15.12;

    Pose redResetPose = new Pose(length/2, width/2, 0);

    Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Command joystickToIntake;
//    private Command updateTurret;

//    private Command transition;

    private Command updateShooter;

    private boolean transitioning = false;

    public IntakingState(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initialize(Robot robot, State prevState) {
        schedule(robot.closeGate());
        schedule(robot.deactivateFlywheel());
        schedule(robot.setPtoToIntaking());

//        transition = bind(() -> gamepad1.a).and(() -> !Constants.lastOpModeWasAuto).and(() -> !transitioning)
//                .rise(
//                        sequential(
//                                instant(() -> transitioning = true),
//                                instant(() -> cancel(transition)),
//                                robot.setIntakePower(0),
//                                instant(() -> robot.setState(States.SHOOTING))
//                        )
//                );

        updateShooter = robot.updateShootingSubsystems();


        joystickToIntake = robot.joysticksToIntakePower(
                () -> gamepad1.left_trigger,
                () -> gamepad1.right_trigger
        );

//        updateTurret = robot.updateTurret();

        if (!Constants.lastOpModeWasAuto) {
            schedule(joystickToIntake);
//            schedule(updateTurret);
            schedule(updateShooter);
        }

        transitioning = false;
    }

    public void execute(Robot robot) {
        //length = 15.39
        //width = 15.12
        if (!Constants.lastOpModeWasAuto && gamepad1.left_bumper && gamepad1.right_bumper && gamepad2.left_bumper && gamepad2.right_bumper) {
            transitioning = true;
            cancel(joystickToIntake);
            cancel(updateShooter);
            schedule(
                    instant(() -> robot.setState(States.LIFTING))
            );
        }

        if (gamepad2.dpadDownWasPressed()) {
            Constants.debugTelemetry = !Constants.debugTelemetry;
        }

        if (gamepad2.aWasPressed())  {
            robot.setPose((Constants.color == Constants.Color.RED) ? redResetPose : redResetPose.mirror());
        }


        if (gamepad1.aWasPressed() && !Constants.lastOpModeWasAuto && !transitioning) {
            transitioning = true;
            cancel(joystickToIntake);
            cancel(updateShooter);
            schedule(
                    robot.setIntakePower(0),
                    instant(() -> robot.setState(States.SHOOTING))
            );
        }
    }

    public String name(){
        return "Intaking";
    }
}
