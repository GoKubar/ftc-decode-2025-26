package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

public abstract class CompatabilityAuto extends Auto{

    @Override
    protected void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();
//        updateTurret = robot.updateTurret();

        double shootTime = 550;

        schedule(
                updateShooter,
                sequential(
                        shootPreloads(),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime, 750),
                        gateCycle(shootTime, 1500),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupClose, shootCloseAndPark, shootTime, 900, 500),
                        shootAndSetIntaking(),
                        robot.setIntakePower(0)
//                        robot.setTurretPos(0)
                ));
    }

}
