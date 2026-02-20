package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.states.IntakingState;
import org.firstinspires.ftc.teamcode.states.LiftingState;
import org.firstinspires.ftc.teamcode.states.None;
import org.firstinspires.ftc.teamcode.states.ShootingState;
import org.firstinspires.ftc.teamcode.states.State;
import org.firstinspires.ftc.teamcode.states.StateSupplier;

import java.util.HashMap;
import java.util.Map;

public enum States {
    INTAKING(IntakingState::new),
    SHOOTING(ShootingState::new),
    LIFTING(LiftingState::new),
    NONE(None::new);

    private final StateSupplier supplier;
    private static final Map<State, States> getStateEnum = new HashMap<>();

    States(StateSupplier supplier) {
        this.supplier = supplier;
    }

    States() {
        this.supplier = (telemetry, gamepad1, gamepad2) -> null;
    }

    public State build(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        State state = supplier.get(telemetry, gamepad1, gamepad2);
        getStateEnum.put(state, this);
        return state;
    }

    public static States get(State state) {
        return getStateEnum.get(state);
    }
}
