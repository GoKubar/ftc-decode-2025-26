package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

public abstract class FarAuto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side

    public static double offsetX = -0.66;
    public static double offsetY = -2.325;

    protected Command updateShooter;
    //originally 46.05, 6.81
    protected Pose startPose = new Pose(46.05 + offsetX, 6.81 + offsetY, Math.toRadians(180));
    protected Pose farPickupPose = new Pose(11.59 + offsetX, 33.210 + offsetY, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(49.56 + offsetX + offsetY, 37.3);
    protected Pose shootingPose = new Pose(52.22 + offsetX, 12.93 + offsetY, Math.toRadians(180));
    protected Pose cornerPickup = new Pose(10.42 + offsetX, 6.81 + offsetY, Math.toRadians(180));
    protected Pose hpEdgePose = new Pose(10.42 + offsetX, 22 + offsetY, Math.toRadians(180));
    protected Pose sweepPose = new Pose(9 + offsetX, 40 + offsetY, Math.toRadians(90));
    protected Pose sweepControlPoint = new Pose(6.9 + offsetX + offsetY, 0);
    protected Pose parkPose = new Pose(42 + offsetX, 15 + offsetY, Math.toRadians(180));
    protected Pose goalPose = Constants.BLUE_GOAL_POSE;

    protected PathChain shootPreloads;
    protected PathChain pickupFar;
    protected PathChain shootFar;
    protected PathChain pickupCorner;
    protected PathChain shootCorner;
    protected PathChain pickupHPEdge;
    protected PathChain shootHPEdge;
    protected PathChain sweep;
    protected PathChain shootSweep;
    protected PathChain park;

    abstract void setPoses();
    abstract void setColor();

    protected void createAutoCommands() {
        updateShooter = robot.updateShootingSubsystems();

        double shootTime = 400;

        schedule(
                updateShooter,
                sequential(
                        shootPreloads(),
                        runCycle(pickupFar, shootFar, shootTime, 700, 400),
                        runCycle(pickupCorner, shootCorner, shootTime, 700, 400),
                        runCycle(pickupHPEdge, shootHPEdge, shootTime, 700, 400),
                        runCycle(pickupCorner, shootCorner, shootTime, 700, 400),
                        runCycle(sweep, shootSweep, shootTime, 700, 400),
                        runCycle(pickupCorner, shootCorner, shootTime, 700, 400),
                        runCycle(pickupHPEdge, shootHPEdge, shootTime, 700, 400),
                        park(shootTime)
                ));
    }


    protected Command shootPreloads() {
        return sequential(
                follow(robot.getFollower(), shootPreloads),
                waitUntil(() -> robot.isShooterReady())
        );
    }

    protected Command runCycle(
            PathChain pickupPath,
            PathChain shootPath,
            double shootDelayMs,
            double intakeDelayMs,
            double shootingDelayMs
    ) {
        return sequential(
                parallel(
                        shootAndSetIntaking(),
                        sequential(
                                waitMs(shootDelayMs),
                                parallel(
                                        follow(robot.getFollower(), pickupPath),
                                        sequential(
                                                waitMs(intakeDelayMs),
                                                robot.setIntakePower(1)
                                        )
                                )
                        )
                ),
                parallel(
                        follow(robot.getFollower(), shootPath),
                        sequential(
                                waitMs(shootingDelayMs),
                                robot.setIntakePower(0)
//                                setShooting()
                        )
                )
        );
    }


    protected Command shootAndSetIntaking() {
        return instant(() -> robot.setState(States.SHOOTING));
    }

    protected Command park(double waitMs) {
        return sequential(
                shootAndSetIntaking(),
                waitMs(waitMs),
                follow(robot.getFollower(), park)
        );
    }


    private void generatePaths() {
        shootPreloads = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(startPose, shootingPose))
            .setConstantHeadingInterpolation(startPose.getHeading())
            .build();
        pickupFar = robot.getFollower().pathBuilder()
            .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
            .setConstantHeadingInterpolation(shootingPose.getHeading())
            .build();
        shootFar = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(farPickupPose, shootingPose))
            .setConstantHeadingInterpolation(shootingPose.getHeading())
            .build();
        pickupCorner = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(shootingPose, cornerPickup))
            .setConstantHeadingInterpolation(cornerPickup.getHeading())
            .build();
        shootCorner = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(cornerPickup, shootingPose))
            .setConstantHeadingInterpolation(shootingPose.getHeading())
            .build();
        pickupHPEdge = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(shootingPose, hpEdgePose))
            .setConstantHeadingInterpolation(hpEdgePose.getHeading())
            .build();
        shootHPEdge = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(hpEdgePose, shootingPose))
            .setConstantHeadingInterpolation(shootingPose.getHeading())
            .build();
        sweep = robot.getFollower().pathBuilder()
            .addPath(new BezierCurve(shootingPose, sweepControlPoint, sweepPose))
            .setLinearHeadingInterpolation(shootingPose.getHeading(), sweepPose.getHeading())
            .build();
        shootSweep = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(sweepPose, shootingPose))
            .setConstantHeadingInterpolation(sweepPose.getHeading())
            .build();
        park = robot.getFollower().pathBuilder()
            .addPath(new BezierLine(shootingPose, parkPose))
            .setConstantHeadingInterpolation(parkPose.getHeading())
            .build();
    }

    public void initialize() {
        telemetry = new FastTelemetry(telemetry);
        Constants.reset();
        setColor();
        setPoses();
        Constants.lastOpModeWasAuto = true;
        Scheduler.reset();

        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose);
        robot.setPose(startPose);
        robot.setLocalizationMode(Robot.LocalizationMode.FOLLOWER);
        robot.init();

        generatePaths();
    }

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        robot.setState(States.INTAKING);
        createAutoCommands();
        while (opModeIsActive()) {
            robot.clearCaches();
            Scheduler.execute();
        }
    }
}
