package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.shooter.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

public abstract class Auto extends LinearOpMode {
    Robot robot;
    // default for all poses is blue side


    protected Command updateShooter;
    private int gateCycleNum = -1;

    Pose startPose = new Pose(17.735, 110.63, Math.toRadians(180));

    protected Pose shootingPose = new Pose(58, 71, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(13.990, 55, Math.toRadians(180));
//    protected Pose middlePickupControlPoint1 = new Pose(110.747, 43.599);
    protected Pose middlePickupControlPoint2 = new Pose(42, 57);
    protected Pose closePickupPose = new Pose(20.590, 81.860, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(56.090, 59.210);
    protected Pose gateClearPose = new Pose(19.847, 60.5, Math.toRadians(180));
    protected Pose gatePickupControlPoint = new Pose(20.590, 55.260);
    protected Pose[] gatePickupPoses = {
            new Pose(13.6, 56, Math.toRadians(150)),
            new Pose(13.6, 55.75, Math.toRadians(150)),
            new Pose(13.6, 55.5, Math.toRadians(150)),
            new Pose(14, 55.25, Math.toRadians(150)),
            new Pose(14, 55, Math.toRadians(150)),
    };
    protected Pose farPickupPose = new Pose(11.590, 33.210, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(45, 34);
    // protected Pose cornerPose = new Pose(10.343, 17.111, Math.toRadians(210));
    protected Pose cornerPose = new Pose(13.990, 17.860, Math.toRadians(210));
    protected Pose cornerBackupPose = new Pose(11.690, 8.360, Math.toRadians(180));
    // protected Pose parkPose = new Pose(56.243, 104.761, Math.toRadians(180));
    protected Pose farShootingPose = new Pose(51.247, 10.099);
    protected Pose parkPose = new Pose(45.747, 15.099);
    protected Pose closeParkPose = new Pose(56.990, 102.860, Math.toRadians(180));
    protected Pose goalPose = Constants.BLUE_GOAL_POSE;

    protected PathChain shootPreloads;
    protected PathChain pickupMiddle;
    protected PathChain shootMiddle;
    protected PathChain clearGate;
    // protected PathChain pickupGate1;
    protected PathChain[] pickupGates;
    protected PathChain[] shootGates;
    protected PathChain shootGateAndPark;
    // PathChain clearGate2;
    // PathChain shootGate2;
    protected PathChain pickupClose;
    protected PathChain shootClose;
    protected PathChain shootCloseAndPark;
    protected PathChain pickupFar;
    protected PathChain shootFar;
    protected PathChain shootFarAndPark;
    protected PathChain pickupCorner;
    protected PathChain backupCorner;
    protected PathChain shootCorner;
    protected PathChain park;
    protected PathChain shootCornerClose;

    abstract void setPoses();

    abstract void setColor();

    protected void createAutoCommands() {
        updateShooter = robot.updateShootingSubsystems();

        double shootTime = 300;

        schedule(updateShooter,
                sequential(
                        shootPreloads(),
                        waitMs(250),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupClose, shootClose, shootTime, 900, 500),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupFar, shootFarAndPark, shootTime + 125, 700, 750),
                        shootAndSetIntaking(),
                        waitMs(500),
                        robot.setIntakePower(0),
                        robot.deactivateShooter()
                )
        );
    }


    protected Command shootPreloads() {
        return sequential(startFlywheel(), follow(robot.getFollower(), shootPreloads));
    }

    protected Command runCycle(PathChain pickupPath, PathChain shootPath, double shootDelayMs,
            double intakeDelayMs, double shootingDelayMs) {
        return sequential(
                parallel(shootAndSetIntaking(), sequential(waitMs(shootDelayMs),
                        parallel(follow(robot.getFollower(), pickupPath),
                                sequential(waitMs(intakeDelayMs), robot.setIntakePower(1))))),
                parallel(follow(robot.getFollower(), shootPath),
                        sequential(waitMs(shootingDelayMs), robot.setIntakePower(0))));
    }

    protected Command gateCycle(double shootDelayMs, double gateWaitMs) {
        gateCycleNum++;
        return sequential(
                parallel(shootAndSetIntaking(),
                        sequential(waitMs(shootDelayMs), robot.setIntakePower(1),
                                follow(robot.getFollower(), pickupGates[gateCycleNum]))),
                waitMs(gateWaitMs), parallel(follow(robot.getFollower(), shootGates[gateCycleNum]),
                        sequential(waitMs(1500), robot.setIntakePower(0))));
    }

    protected Command gateCycleAndPark(double shootDelayMs, double gateWaitMs) {
        gateCycleNum++;
        return sequential(
                parallel(shootAndSetIntaking(),
                        sequential(waitMs(shootDelayMs), robot.setIntakePower(1),
                                follow(robot.getFollower(), pickupGates[gateCycleNum]))),
                waitMs(gateWaitMs), parallel(follow(robot.getFollower(), shootGateAndPark),
                        sequential(waitMs(1000), robot.setIntakePower(0))));
    }

    protected Command cornerCycle(double shootDelayMs, double intakeDelayMs,
            double shootingDelayMs) {
        return sequential(parallel(shootAndSetIntaking(),
                // shoot and pickup corner
                sequential(waitMs(shootDelayMs),
                        parallel(follow(robot.getFollower(), pickupCorner),
                                sequential(waitMs(intakeDelayMs), robot.deactivateFlywheel(),
                                        robot.setIntakePower(1))))),
                follow(robot.getFollower(), backupCorner),
                instant(() -> VelocityCompensationCalculator.useAutoLimit = false),
                robot.activateShooter(),
                parallel(sequential(waitMs(shootingDelayMs), robot.setIntakePower(0)),
                        follow(robot.getFollower(), shootCorner)),
                shootAndSetIntaking(), waitMs(shootDelayMs), follow(robot.getFollower(), park));
    }

    protected Command cornerCycleClosePark(double shootDelayMs, double intakeDelayMs,
            double shootingDelayMs) {
        return sequential(
                parallel(shootAndSetIntaking(), sequential(waitMs(shootDelayMs),
                        parallel(follow(robot.getFollower(), pickupCorner),
                                sequential(waitMs(intakeDelayMs), robot.deactivateFlywheel(),
                                        robot.setIntakePower(1))))),
                follow(robot.getFollower(), backupCorner),
                parallel(
                        sequential(waitMs(shootingDelayMs), robot.setIntakePower(0),
                                robot.activateShooter()),
                        follow(robot.getFollower(), shootCornerClose)));
    }

    public static Command turnTo(Follower follower, double radians) {
        return new CommandBuilder().setStart(() -> {
            Pose pose = follower.getPose();
            Path path = new Path(new BezierPoint(pose));
            path.setHeadingInterpolation(HeadingInterpolator.constant(radians));
            follower.followPath(path);
        }).setDone(() -> !follower.isBusy());
    }

    protected Command shootAndSetIntaking() {
        return instant(() -> robot.setState(States.SHOOTING));
    }

    protected Command startFlywheel() {
        return instant(() -> robot.activateShooter());
    }

    private void generatePaths() {
        shootPreloads = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .setTangentHeadingInterpolation()
                .setReversed()
                // .setConstraints(
                // new PathConstraints(0.8,
                // 3,
                // 3,
                // 0.03,
                // 50,
                // 1,
                // 10,
                // 1)
                // )
                .build();

        pickupMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, middlePickupControlPoint2, middlePickupPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        shootMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(middlePickupPose, shootingPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // clearGate = robot.getFollower().pathBuilder()
        // .addPath(new BezierCurve(shootingPose,
        // gateClearControlPoint,
        // gateClearPose
        // )).setConstantHeadingInterpolation(shootingPose.getHeading())
        // .setConstraints(
        // new PathConstraints(0.8,
        // 1,
        // 0.75,
        // 0.03,
        // 50,
        // 1,
        // 10,
        // 1)
        // )
        // .build();

        // pickupGate = robot.getFollower().pathBuilder()
        // .addPath(new BezierCurve(gateClearPose,
        // gatePickupControlPoint,
        // gatePickupPose
        // )).setConstantHeadingInterpolation(gateClearPose.getHeading())
        // .build();

        // pickupGate1 = robot.getFollower().pathBuilder()
        // .addPath(new BezierCurve(shootingPose, gatePickupControlPoint, gatePickupPose))
        // .setLinearHeadingInterpolation(gatePickupPose.getHeading())
        // .build();

        for (int i = 0; i < gatePickupPoses.length; i++) {
            if (i == 0) {
                pickupGates = new PathChain[gatePickupPoses.length];
                shootGates = new PathChain[gatePickupPoses.length];
            }
            pickupGates[i] = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(shootingPose, gateClearControlPoint, gatePickupPose))
                    .addPath(new BezierLine(shootingPose, gatePickupPoses[i]))
                    // .addPath(new BezierCurve(gateClearPose, gatePickupControlPoint, gatePickupPose))
//                .setConstantHeadingInterpolation(gatePickupPose.getHeading())
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0, 0.5, HeadingInterpolator.tangent),
                            new HeadingInterpolator.PiecewiseNode(0.5, 1, HeadingInterpolator.constant(gatePickupPoses[i].getHeading()))
                    ))
                    .build();
//                .setConstraints(new PathConstraints(0.95, 0.5, 0.5, 0.03, 50, 1, 10, 1)).build();

            shootGates[i] = robot.getFollower().pathBuilder()
                    .addPath(new BezierLine(gatePickupPoses[i], shootingPose))
//                .setConstantHeadingInterpolation(gatePickupPose.getHeading()).build();
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }



        shootGateAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gatePickupPoses[gatePickupPoses.length-1], closeParkPose))
//                .setConstantHeadingInterpolation(gatePickupPoses[gatePickupPoses.length-1].getHeading()).build();
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // clearGate2 = robot.getFollower().pathBuilder()
        // .addPath(new BezierCurve(shootingPose,
        // gateClearControlPoint,
        // gateClearPose
        // )).setConstantHeadingInterpolation(gateClearPose.getHeading())
        // .setTValueConstraint(0.75)
        // .build();

        // shootGate2 = robot.getFollower().pathBuilder()
        // .addPath(new BezierLine(gatePickupPose,
        // shootingPose
        // )).setLinearHeadingInterpolation(gatePickupPose.getHeading(), shootingPose.getHeading())
        // .build();


        pickupClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, closePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        // clearGate = robot.getFollower().pathBuilder()
        // .addPath(new BezierCurve(closePickupPose, gateClearControlPoint,gateClearPose))
        // .setLinearHeadingInterpolation(closePickupPose.getHeading(), gateClearPose.getHeading())
        // .build();

        shootClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootCloseAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, closeParkPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        pickupFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        // shootFar = robot.getFollower().pathBuilder()
        // .addPath(new BezierCurve(farPickupPose, farPickupControlPoint, parkPose))
        // .setConstantHeadingInterpolation(parkPose.getHeading())
        // .build();
        shootFar = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootFarAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, closeParkPose))
//                .setConstantHeadingInterpolation(closeParkPose.getHeading()).build();
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupCorner =
                robot.getFollower().pathBuilder().addPath(new BezierLine(shootingPose, cornerPose))
                        .setConstantHeadingInterpolation(cornerPose.getHeading()).build();

        backupCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerPose, cornerBackupPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(),
                        cornerBackupPose.getHeading())
                .setConstraints(new PathConstraints(0.8, 2, 2, 0.03, 50, 1, 10, 1)).build();

        shootCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, farShootingPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();

        park = robot.getFollower().pathBuilder().addPath(new BezierLine(farShootingPose, parkPose))
//                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        shootCornerClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, closeParkPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();
    }

    public void initialize() {
        telemetry = new FastTelemetry(telemetry);
        Constants.reset();
        setColor();
        setPoses();
        Constants.lastOpModeWasAuto = true;
        Constants.currentOpModeIsAuto = true;
        Scheduler.reset();


        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, goalPose, Robot.LocalizationMode.FOLLOWER);
        // Constants.robot = robot;
        robot.setPose(startPose);
        //robot.setLocalizationMode(Robot.LocalizationMode.FOLLOWER);
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
        robot.stop();
    }


}
