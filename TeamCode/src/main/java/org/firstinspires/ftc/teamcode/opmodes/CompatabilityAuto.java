package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

public abstract class CompatabilityAuto extends Auto{

    @Override
    protected void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();
//        updateTurret = robot.updateTurret();

        double shootTime = 610;

        schedule(
                sequential(
                        shootPreloads(),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime),
                        gateCycle(shootTime),
                        gateCycle(shootTime),
                        runCycle(pickupClose, shootCloseAndPark, shootTime, 750, 500),
                        shootAndSetIntaking(),
                        robot.setIntakePower(0)
//                        robot.setTurretPos(0)
                ));
    }

}
