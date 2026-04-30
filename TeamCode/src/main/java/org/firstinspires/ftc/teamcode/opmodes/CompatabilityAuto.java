package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

public abstract class CompatabilityAuto extends Auto{

    @Override
    protected void createAutoCommands() {
        updateShooter = robot.updateShootingSubsystems();
        double shootTime = 300;

        schedule(updateShooter,
                sequential(shootPreloads(),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime, 750),
                        gateCycle(shootTime, 1500),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupClose, shootClose, shootTime, 900, 500),
                        gateCycleAndPark(shootTime, 1000),
                        shootAndSetIntaking(),
                        waitMs(500),
                        robot.setIntakePower(0),
                        robot.deactivateShooter()
                )
        );
    }
}