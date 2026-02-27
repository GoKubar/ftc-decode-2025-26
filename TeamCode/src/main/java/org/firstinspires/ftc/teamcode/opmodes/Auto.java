package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turn;

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

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

public abstract class Auto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side

    //length = 15.39
    //width = 15.12

    protected Command updateShooter;
//    private Command updateTurret;

//    originally Pose(20.8, 124.1, Math.toRadians(234));
    //moved over -0.9 on x, +0.35 on y
//    protected Pose startPose = new Pose(19.90, 124.45, Math.toRadians(234));
    protected Pose startPose = new Pose(17.745, 110.88, Math.toRadians(180));

    protected Pose shootingPose = new Pose(54, 81, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(14, 55.35, Math.toRadians(180));
    protected Pose middlePickupControlPoint = new Pose(58.1, 48.35);
    protected Pose closePickupPose = new Pose(20.6, 85.35, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(56.1, 61.35);
    protected Pose gateClearPose = new Pose(19.6, 63.4, Math.toRadians(180));
    protected Pose gatePickupControlPoint = new Pose(20.6, 57.4);
    protected Pose gatePickupPose = new Pose(13, 52, Math.toRadians(120));
    protected Pose farPickupPose = new Pose(11.6, 35.35, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(71.1, 20.35);
    protected Pose cornerPose = new Pose(11.1, 17.35, Math.toRadians(210));
    protected Pose cornerBackupPose = new Pose(11.7, 10.5, Math.toRadians(180));
    protected Pose parkPose = new Pose(53, 101, Math.toRadians(180));
    protected Pose goalPose = Constants.BLUE_GOAL_POSE;

    protected PathChain shootPreloads;
    protected PathChain pickupMiddle;
    protected PathChain shootMiddle;
    protected PathChain clearGate;
//    protected PathChain pickupGate1;
    protected PathChain pickupGate;
    protected PathChain shootGate;
//    PathChain clearGate2;
//    PathChain shootGate2;
    protected PathChain pickupClose;
    protected PathChain shootClose;
    protected PathChain pickupFar;
    protected PathChain shootFar;
    protected PathChain pickupCorner;
    protected PathChain backupCorner;
    protected PathChain shootCorner;

    abstract void setPoses();
    abstract void setColor();

    protected void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();
//        updateTurret = robot.updateTurret();

        double shootTime = 500;

        schedule(
                updateShooter,
                sequential(
                        shootPreloads(),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime),
                        runCycle(pickupClose, shootClose, shootTime, 750, 500),
                        gateCycle(shootTime),
                        runCycle(pickupFar, shootFar, shootTime + 125, 700, 1500),
                        cornerCycle(shootTime + 150, 1000, 1500),
                        shootAndSetIntaking(),

                        robot.setIntakePower(0)
//                        robot.setTurretPos(0)
                ));
    }


    protected Command shootPreloads() {
        return
                follow(robot.getFollower(), shootPreloads);
//        return parallel(
//                setShooting()
//                sequential(
//                        robot.setIntakePower(0.4),
//                        waitMs(200),
//                setShooting()
//                )
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

    protected Command gateCycle(double shootDelayMs) {
        return sequential(
                parallel(
                        shootAndSetIntaking(),
                        //gate clear / pickup + shoot
                        sequential(
                                waitMs(shootDelayMs),
                                follow(robot.getFollower(), clearGate)
                        ),
                        sequential(
                                waitMs(500),
                                robot.setIntakePower(1)
                        )
                ),
                follow(robot.getFollower(), pickupGate),
                waitMs(500),
                parallel(
                        follow(robot.getFollower(), shootGate),
                        sequential(
                                waitMs(1000),
                                robot.setIntakePower(0)
//                                setShooting()
                        )
                )
        );
    }

    protected Command cornerCycle(
            double shootDelayMs,
            double intakeDelayMs,
            double shootingDelayMs
    ) {
        return sequential(
                parallel(
                        shootAndSetIntaking(),
                        //shoot and pickup corner
                        sequential(
                                waitMs(shootDelayMs),
                                parallel(
                                        follow(robot.getFollower(), pickupCorner),
                                        sequential(
                                                waitMs(intakeDelayMs),
                                                robot.deactivateFlywheel(),
                                                robot.setIntakePower(1)
                                        )
                                )
                        )
                ),
                follow(robot.getFollower(), backupCorner),
//                waitMs(50),
                parallel(
                        sequential(
                                waitMs(shootingDelayMs),
                                robot.setIntakePower(0),
                                robot.activateShooter()
                        ),
                        follow(robot.getFollower(), shootCorner)
                )
        );
    }

    public static Command turnTo(Follower follower, double radians) {
        return new CommandBuilder()
                .setStart(() -> {
                    Pose pose = follower.getPose();
                    Path path = new Path(new BezierPoint(pose));
                    path.setHeadingInterpolation(HeadingInterpolator.constant(radians));
                    follower.followPath(path);
                })
                .setDone(() -> !follower.isBusy());
    }

//    protected Command setShooting() {
//        return instant(() -> robot.activateShooter());
//        return sequential(
//                robot.setIntakePower(0.4),
//                instant(() -> cancel(updateTurret)),
//                instant(() -> schedule(updateShooter)),
//                instant(() -> robot.setState(States.SHOOTING))
//        );
//    }




    protected Command shootAndSetIntaking() {
        return sequential(
//                        waitUntil(robot::readyToShoot).raceWith(infinite(() -> {
//                            telemetry.addData("Waiting to shoot...", "");
//                        })).raceWith(waitMs(500)),
//                robot.shootMotif(800),
//                instant(() -> cancel(updateShooter)),
//                        instant(() -> schedule(updateTurret)),
//                instant(() -> robot.setState(States.INTAKING))
                instant(() -> robot.setState(States.SHOOTING))
        );
    }


    private void generatePaths() {
        shootPreloads = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .setConstraints(
                        new PathConstraints(0.8,
                                3,
                                3,
                                0.03,
                                50,
                                1,
                                10,
                                1)
                )
                .build();

        pickupMiddle = robot.getFollower().pathBuilder()
                .addPath( new BezierCurve(shootingPose,
                        middlePickupControlPoint,
                        middlePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        shootMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(middlePickupPose,
                        middlePickupControlPoint,
                        shootingPose
                )).setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        clearGate = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose,
                        gateClearControlPoint,
                        gateClearPose
                )).setConstantHeadingInterpolation(shootingPose.getHeading())
                .setConstraints(
                        new PathConstraints(0.8,
                                1,
                                0.75,
                                0.03,
                                50,
                                1,
                                10,
                                1)
                )
                .build();

//        pickupGate = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(gateClearPose,
//                        gatePickupControlPoint,
//                        gatePickupPose
//                )).setConstantHeadingInterpolation(gateClearPose.getHeading())
//                .build();

//        pickupGate1 = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(shootingPose, gatePickupControlPoint, gatePickupPose))
//                .setLinearHeadingInterpolation(gatePickupPose.getHeading())
//                .build();

        pickupGate = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gateClearPose, gatePickupPose))
//                .addPath(new BezierCurve(gateClearPose, gatePickupControlPoint, gatePickupPose))
                .setLinearHeadingInterpolation(gateClearPose.getHeading(), gatePickupPose.getHeading())
                .build();

        shootGate = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(gatePickupPose,
                        shootingPose
                )).setConstantHeadingInterpolation(gatePickupPose.getHeading())
                .build();


//        clearGate2 = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(shootingPose,
//                        gateClearControlPoint,
//                        gateClearPose
//                )).setConstantHeadingInterpolation(gateClearPose.getHeading())
//                .setTValueConstraint(0.75)
//                .build();

//        shootGate2 = robot.getFollower().pathBuilder()
//                .addPath(new BezierLine(gatePickupPose,
//                        shootingPose
//                )).setLinearHeadingInterpolation(gatePickupPose.getHeading(), shootingPose.getHeading())
//                .build();


        pickupClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, closePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

//        clearGate = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(closePickupPose, gateClearControlPoint,gateClearPose))
//                .setLinearHeadingInterpolation(closePickupPose.getHeading(), gateClearPose.getHeading())
//                .build();

        shootClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        pickupFar = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

//        shootFar = robot.getFollower().pathBuilder()
//                .addPath(new BezierCurve(farPickupPose, farPickupControlPoint, parkPose))
//                .setConstantHeadingInterpolation(parkPose.getHeading())
//                .build();
        shootFar = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farPickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        pickupCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(shootingPose, cornerPose))
                .setConstantHeadingInterpolation(cornerPose.getHeading())
                .build();

        backupCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerPose, cornerBackupPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), cornerBackupPose.getHeading())
                .setConstraints(
                        new PathConstraints(0.8,
                                2,
                                2,
                                0.03,
                                50,
                                1,
                                10,
                                1)
                )
                .build();

        shootCorner = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, parkPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading())
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
//        Constants.robot = robot;
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
