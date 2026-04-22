package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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
import org.firstinspires.ftc.teamcode.shooter.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

public abstract class Auto extends LinearOpMode {
    Robot robot;
    //default for all poses is blue side

    //length = 15.39
    //width = 15.12

    protected Command updateShooter;
//    private Command updateTurret;

//    originally startPose = new Pose(18.865, 114.22, Math.toRadians(180));
    //moved over -1.877 on x, -3.579 on y
    Pose startPose = new Pose(16.988, 110.641, Math.toRadians(180));

    protected Pose shootingPose = new Pose(53.243, 80.761, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(13.243, 55.111, Math.toRadians(180));
    protected Pose middlePickupControlPoint1 = new Pose(110, 45.5);
    protected Pose middlePickupControlPoint2 = new Pose(57.343, 48.111);
    protected Pose closePickupPose = new Pose(19.843, 83.761, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(55.343, 61.111);
    protected Pose gateClearPose = new Pose(19.3, 63.161, Math.toRadians(180));
    protected Pose gatePickupControlPoint = new Pose(19.843, 57.161);
    protected Pose gatePickupPose = new Pose(12, 51.761, Math.toRadians(120));
    protected Pose farPickupPose = new Pose(10.843, 35.111, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(70.343, 20.111);
//    protected Pose cornerPose = new Pose(10.343, 17.111, Math.toRadians(210));
    protected Pose cornerPose = new Pose(13.243, 19.761, Math.toRadians(210));
    protected Pose cornerBackupPose = new Pose(10.943, 10.261, Math.toRadians(180));
//    protected Pose parkPose = new Pose(56.243, 104.761, Math.toRadians(180));
    protected Pose farShootingPose = new Pose(50.5, 12);
    protected Pose parkPose = new Pose(45, 17);
    protected Pose closeParkPose = new Pose(54.243, 104.761, Math.toRadians(180));
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
    protected PathChain shootCloseAndPark;
    protected PathChain pickupFar;
    protected PathChain shootFar;
    protected PathChain pickupCorner;
    protected PathChain backupCorner;
    protected PathChain shootCorner;
    protected PathChain park;
    protected PathChain shootCornerClose;

    abstract void setPoses();
    abstract void setColor();

    protected void createAutoCommands() {
//        robot.getFollower().setMaxPower(0.9);
        updateShooter = robot.updateShootingSubsystems();
//        updateTurret = robot.updateTurret();

        double shootTime = 450;

        schedule(
                updateShooter,
                sequential(
                        shootPreloads(),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime),
                        runCycle(pickupClose, shootClose, shootTime, 900, 500),
                        gateCycle(shootTime),
                        runCycle(pickupFar, shootFar, shootTime + 125, 700, 1250),
                        cornerCycle(shootTime + 150, 1250, 1250),
                        robot.setIntakePower(0)
//                        robot.setTurretPos(0)
                ));
    }


    protected Command shootPreloads() {
        return follow(robot.getFollower(), shootPreloads);
//        return sequential(
//                parallel(
//                        follow(robot.getFollower(), shootPreloads),
//                        sequential(
//                                waitUntil(() -> robot.getFollower().getCurrentTValue() > 0.17),
//                                shootAndSetIntaking(),
//                                robot.setIntakePower(1)
//                        )
//                ),
//                parallel(
//                        follow(robot.getFollower(), shootMiddle),
//                        sequential(
//                                waitMs(shootingDelayMS),
//                                robot.setIntakePower(0)
//                        )
//                )
//
//        );
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
                instant(() -> VelocityCompensationCalculator.useAutoLimit = false),
                robot.activateShooter(),
//                waitMs(50),
                parallel(
                        sequential(
                                waitMs(shootingDelayMs),
                                robot.setIntakePower(0)
                        ),
                        follow(robot.getFollower(), shootCorner)
                ),
                shootAndSetIntaking(),
                waitMs(shootDelayMs),
                follow(robot.getFollower(), park)
        );
    }

    protected Command cornerCycleClosePark(
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
                        follow(robot.getFollower(), shootCornerClose)
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
                        middlePickupControlPoint2,
                        middlePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading())
                .build();

        shootMiddle = robot.getFollower().pathBuilder()
                .addPath(new BezierCurve(middlePickupPose,
                        middlePickupControlPoint2,
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

        shootCloseAndPark = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(closePickupPose, closeParkPose))
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
                .addPath(new BezierLine(cornerBackupPose, farShootingPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading())
                .build();

        park = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(farShootingPose, parkPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading())
                .build();

        shootCornerClose = robot.getFollower().pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, closeParkPose))
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
