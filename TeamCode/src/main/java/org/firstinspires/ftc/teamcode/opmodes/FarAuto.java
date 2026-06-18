package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.branch;
import static com.pedropathing.ivy.commands.Commands.conditional;
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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

import java.util.LinkedHashMap;

public abstract class FarAuto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side
    public static double offsetX = -1.414;
    public static double offsetY = -0.501;

    protected Command updateShooter;
    //originally 46.05, 6.81
    protected Pose startPose = new Pose(46.05 + offsetX, 6.81 + offsetY, Math.toRadians(180));
    protected Pose farPickupPose = new Pose(13.99 + offsetX, 33.210 + offsetY, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(36 + offsetX, 34 + offsetY);
    protected Pose firstShotPose = new Pose(61 + offsetX, 27 + offsetY, Math.toRadians(180));
    protected Pose shootingPose = new Pose(52.22 + offsetX, 12.93 + offsetY, Math.toRadians(180));
    protected Pose cornerPickup = new Pose(13.25 + offsetX, 6.81 + offsetY, Math.toRadians(180));
    protected Pose hpEdgePose = new Pose(13.25 + offsetX, 22 + offsetY, Math.toRadians(180));
    protected Pose sweepPose = new Pose(13 + offsetX, 40 + offsetY, Math.toRadians(90));
    protected Pose sweepControlPoint = new Pose(10.9 + offsetX + offsetY, 0);
//    protected Pose parkPose = new Pose(42 + offsetX, 15 + offsetY, Math.toRadians(180));
//    protected Pose parkPose = new Pose(39 + offsetX, 22 + offsetY, Math.toRadians(180));
    protected Pose parkPose = new Pose(45 + offsetX, 12.93 + offsetY, Math.toRadians(180));
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

        double shootTime = 350;

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
                        runCycle(sweep, shootSweep, shootTime, 700, 400),
                        runCycle(pickupCorner, shootCorner, shootTime, 700, 400),
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
                                ).raceWith(waitUntil(() -> robot.beamBroken()))
                        )
                ),
                parallel(
                        sequential(
                                waitMs(200),
                                conditional(
                                        () -> robot.beamBroken(),
                                        instant(() -> {}), // do nothing
                                        sequential(
                                                robot.setIntakePower(-1),
                                                waitMs(50),
                                                robot.setIntakePower(0)
                                        )
                                )
                        ),
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
                .addPath(new BezierLine(startPose, firstShotPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        pickupFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(firstShotPose, farPickupControlPoint, farPickupPose))
                .setTangentHeadingInterpolation()
                .build();
        shootFar = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, shootingPose))
                .setTangentHeadingInterpolation()
                .setReversed()
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
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        park = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
//                .setConstantHeadingInterpolation(parkPose.getHeading())
                .setTangentHeadingInterpolation()
                .build();
    }

    public void initialize() {
        telemetry = new FastTelemetry(telemetry);
        Constants.reset();
        setColor();
        setPoses();
        Constants.lastOpModeWasAuto = true;
        Scheduler.reset();

        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose, Robot.LocalizationMode.PINPOINT);
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
