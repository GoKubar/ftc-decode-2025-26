package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.cancel;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitMs;

public abstract class CompatabilityAuto extends Auto{

    @Override
    protected void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();
//        updateTurret = robot.updateTurret();

        double shootTime = 650;

        schedule(
                updateShooter,
                sequential(
                        shootPreloads(),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime),
                        gateCycle(shootTime),
                        gateCycle(shootTime),
                        runCycle(pickupClose, shootCloseAndPark, shootTime, 900, 500),
                        shootAndSetIntaking(),
                        robot.setIntakePower(0),
                        waitMs(500),
                        instant(() -> cancel(updateShooter))
//                        robot.setTurretPos(0)
                ));
    }

}
