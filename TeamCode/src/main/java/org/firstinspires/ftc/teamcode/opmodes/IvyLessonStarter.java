package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;

/**
 * Starter OpMode for the Ivy lesson.
 *
 * STUDENT WORKFLOW
 * 1. Change EXERCISE_TO_RUN to the exercise you are working on.
 * 2. Edit only that exercise method.
 * 3. Build, deploy, place the robot at START_POSE, and run this OpMode.
 *
 * Everything else in this file is provided boilerplate:
 * - Robot construction
 * - Pedro Pathing setup
 * - Ivy Scheduler setup and execution
 * - Background robot/shooter updates
 * - Bulk-cache clearing
 * - Safe shutdown
 *
 * REPOSITORY COMMANDS / CONDITIONS YOU MAY USE
 *
 * robot.setIntakePower(power)
 *   Returns an INSTANT command. It sets the intake power and finishes immediately.
 *   The motor keeps that power until another command changes it.
 *
 * robot.activateShooter()
 * robot.deactivateShooter()
 * robot.deactivateFlywheel()
 * robot.shooterIntakingPos()
 * robot.openGate()
 * robot.closeGate()
 *   Each returns an INSTANT command that changes shooter state.
 *
 * robot.shoot()
 *   Returns a reusable SEQUENTIAL command that opens the gate, feeds with the
 *   intake for a timed interval, closes the gate, and updates shooting state.
 *
 * robot.beamBroken()
 *   Returns a debounced boolean indicating that the intake beam is blocked.
 *   This is a condition, not a Command, so pass it with a lambda or method reference:
 *       waitUntil(() -> robot.beamBroken())
 *       waitUntil(robot::beamBroken)
 *
 * robot.isShooterReady()
 *   Returns whether the flywheel is ready:
 *       waitUntil(robot::isShooterReady)
 *
 * robot.updateShootingSubsystems()
 *   Returns an INFINITE background command. This starter schedules it for you.
 *
 * follow(robot.getFollower(), path)
 *   Returns a Pedro/Ivy command that finishes when the follower finishes the path.
 */
@Autonomous(name = "Ivy Lesson Starter", group = "Lessons")
public class IvyLessonStarter extends LinearOpMode {

    /**
     * Change this number to select an exercise.
     *
     * 1 = sequential intake pulse
     * 2 = wait for beam break
     * 3 = beam break or timeout
     * 4 = drive and perform another action in parallel
     * 5 = delayed action nested inside parallel
     * 6 = safe custom command with Command.build()
     * 7 = choose behavior with conditional
     * 8 = final synthesis autonomous
     */
    private static final int EXERCISE_TO_RUN = 1;

    /*
     * Classroom-safe example poses.
     * Adjust these once, before the lesson, to fit the area where the robot will run.
     * Pedro field coordinates are in inches.
     */
    private static final Pose START_POSE =
            new Pose(72, 72, Math.toRadians(0));
    private static final Pose FORWARD_POSE =
            new Pose(96, 72, Math.toRadians(0));

    /*
     * The Robot class needs a goal pose for its shooter calculations.
     * Change this to RED_GOAL_POSE when appropriate.
     */
    private static final Pose GOAL_POSE = Constants.BLUE_GOAL_POSE;

    private Robot robot;
    private PathChain driveForward;
    private PathChain driveBack;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobotAndLesson();

        telemetry.addLine("Ivy lesson ready");
        telemetry.addData("Selected exercise", EXERCISE_TO_RUN);
        telemetry.addLine("Place the robot at START_POSE before pressing Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        robot.setState(States.INTAKING);

        /*
         * robot.init() already scheduled the robot's normal background loop.
         * This additional infinite command continuously updates shooter targeting.
         */
        schedule(
                robot.updateShootingSubsystems(),
                selectedExercise()
        );

        try {
            while (opModeIsActive()) {
                robot.clearCaches();
                Scheduler.execute();
            }
        } finally {
            safelyStopRobot();
        }
    }

    private void initializeRobotAndLesson() {
        telemetry = new FastTelemetry(telemetry);

        Constants.reset();
        Constants.color = Constants.Color.BLUE;
        Constants.lastOpModeWasAuto = true;

        Scheduler.reset();

        robot = new Robot(
                hardwareMap,
                gamepad1,
                gamepad2,
                telemetry,
                GOAL_POSE,
                Robot.LocalizationMode.FOLLOWER
        );

        robot.setPose(START_POSE);
        robot.init();
        buildLessonPaths();
    }

    private void buildLessonPaths() {
        driveForward = robot.getFollower()
                .pathBuilder()
                .addPath(new BezierLine(START_POSE, FORWARD_POSE))
                .setConstantHeadingInterpolation(START_POSE.getHeading())
                .build();

        driveBack = robot.getFollower()
                .pathBuilder()
                .addPath(new BezierLine(FORWARD_POSE, START_POSE))
                .setConstantHeadingInterpolation(START_POSE.getHeading())
                .build();
    }

    private Command selectedExercise() {
        switch (EXERCISE_TO_RUN) {
            case 1:
                return exercise1SequentialIntakePulse();
            case 2:
                return exercise2WaitForBeamBreak();
            case 3:
                return exercise3SensorOrTimeout();
            case 4:
                return exercise4ParallelActions();
            case 5:
                return exercise5NestedCompositions();
            case 6:
                return exercise6SafeCustomCommand();
            case 7:
                return exercise7Conditional();
            case 8:
                return exercise8SynthesisAuto();
            default:
                return notImplemented("Unknown exercise number");
        }
    }

    /**
     * EXERCISE 1 — SEQUENTIAL
     *
     * Goal:
     * 1. Turn the intake on.
     * 2. Wait 1 second.
     * 3. Turn the intake off.
     *
     * New Ivy tools:
     * - sequential(...)
     * - waitMs(...)
     *
     * Important:
     * robot.setIntakePower(...) is already a Command.
     */
    private Command exercise1SequentialIntakePulse() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 1");
    }

    /**
     * EXERCISE 2 — WAIT UNTIL
     *
     * Goal:
     * 1. Turn the intake on.
     * 2. Wait until the beam sensor is blocked.
     * 3. Turn the intake off.
     *
     * New Ivy tool:
     * - waitUntil(...)
     *
     * Try the lambda first:
     *     () -> robot.beamBroken()
     *
     * Then shorten it to the method reference:
     *     robot::beamBroken
     */
    private Command exercise2WaitForBeamBreak() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 2");
    }

    /**
     * EXERCISE 3 — RACE
     *
     * Goal:
     * 1. Turn the intake on.
     * 2. Wait until EITHER:
     *      - the beam is broken, OR
     *      - 1500 ms passes.
     * 3. Turn the intake off.
     *
     * New Ivy tool:
     * - race(...)
     *
     * Test it twice:
     * - Insert an artifact before the timeout.
     * - Leave the intake empty and let the timeout win.
     */
    private Command exercise3SensorOrTimeout() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 3");
    }

    /**
     * EXERCISE 4 — PARALLEL
     *
     * Goal:
     * - Follow driveForward.
     * - At the same time, perform a mechanism action.
     *
     * Suggested mechanism behavior:
     * - Run the intake for 1 second, then stop it.
     *
     * New Ivy tool:
     * - parallel(...)
     *
     * Prediction:
     * When does the parallel group finish?
     */
    private Command exercise4ParallelActions() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 4");
    }

    /**
     * EXERCISE 5 — NESTED COMPOSITIONS
     *
     * Goal:
     * - Begin driveForward immediately.
     * - Wait 500 ms before turning on the intake.
     * - Stop the intake after the parallel section is complete.
     *
     * New idea:
     * - A sequential group is itself a Command.
     * - Therefore it can be placed inside a parallel group.
     */
    private Command exercise5NestedCompositions() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 5");
    }

    /**
     * EXERCISE 6 — COMMAND LIFECYCLE AND CLEANUP
     *
     * Goal:
     * Build a reusable intake command with Command.build():
     * - setStart: turn the intake on.
     * - setDone: finish when the beam is broken.
     * - setEnd: always turn the intake off, including when cancelled.
     *
     * Then race that command against waitMs(1500).
     *
     * New Ivy tools:
     * - Command.build()
     * - setStart(...)
     * - setDone(...)
     * - setEnd(...)
     *
     * CURRENT-REPOSITORY NOTE:
     * Robot exposes intake changes as Command-returning helpers rather than as a
     * public raw setter. A compatible bridge for this lesson is:
     *
     *     .setStart(() -> robot.setIntakePower(1).schedule())
     *     .setEnd(reason -> robot.setIntakePower(0).schedule())
     *
     * Explain that a cleaner long-term Robot API could expose a dedicated
     * lifecycle-safe collect command internally, where it has direct PTO access.
     *
     * This exercise is about command ownership:
     * a reusable command should clean up the hardware it controls.
     */
    private Command exercise6SafeCustomCommand() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 6");
    }

    /**
     * EXERCISE 7 — CONDITIONAL
     *
     * Goal:
     * At the moment this command begins:
     * - If the beam is broken, run one Command.
     * - Otherwise, run a different Command.
     *
     * Safe suggested branches:
     * - TRUE: briefly reverse the intake, then stop.
     * - FALSE: run an instant no-op or telemetry message.
     *
     * New Ivy tool:
     * - conditional(condition, trueCommand, falseCommand)
     *
     * Important:
     * conditional checks its condition once, when it starts.
     */
    private Command exercise7Conditional() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 7");
    }

    /**
     * EXERCISE 8 — SYNTHESIS AUTO
     *
     * Build a short autonomous from the patterns above.
     *
     * Suggested behavior:
     * 1. Drive forward while beginning another robot action.
     * 2. Perform an intake attempt that ends on sensor detection or timeout.
     * 3. Drive back.
     * 4. Choose a final action based on robot.beamBroken().
     *
     * Instructor extension:
     * Introduce robot.activateShooter(), robot.isShooterReady(), and robot.shoot()
     * only after explaining that they are repository helper commands/conditions.
     */
    private Command exercise8SynthesisAuto() {
        // TODO: Replace this placeholder.
        return notImplemented("Exercise 8");
    }

    /**
     * A compiling placeholder so unfinished exercises do not break the project.
     */
    private Command notImplemented(String exerciseName) {
        return instant(() -> {
            telemetry.addLine(exerciseName + " is not implemented yet.");
            telemetry.update();
        });
    }

    /**
     * Stop persistent hardware state even if an exercise was cancelled or the
     * Driver Station stopped the OpMode.
     */
    private void safelyStopRobot() {
        Scheduler.reset();

        schedule(
                robot.setIntakePower(0),
                robot.deactivateShooter()
        );
        Scheduler.execute();
        Scheduler.reset();

        robot.stop();
    }
}
