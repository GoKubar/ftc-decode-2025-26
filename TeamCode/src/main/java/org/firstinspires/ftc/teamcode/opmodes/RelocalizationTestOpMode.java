package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.States;
import org.firstinspires.ftc.teamcode.util.telemetry.FastTelemetry;
import org.firstinspires.ftc.teamcode.vision.Pipelines;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.RelocalizationPipeline;
@TeleOp(name = "Relocalization Test", group = "Test")
public class RelocalizationTestOpMode extends LinearOpMode {

    private static final double ARTIFICIAL_DRIFT_INCHES = 24.0;

    private Robot robot;
    private RelocalizationPipeline relocPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new FastTelemetry(telemetry);

        Constants.reset();
        Scheduler.reset();

        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, Constants.BLUE_GOAL_POSE);
        robot.setPose(new Pose(0, 0, 0));
        robot.setLocalizationMode(Robot.LocalizationMode.PINPOINT);
        Constants.lastOpModeWasAuto = false;

        robot.setPipeline(Pipelines.RELOCALIZATION);

        // Grab a typed reference to the pipeline for rich telemetry
        VisionPipeline rawPipeline = robot.getLimelightManager() != null
                ? robot.getLimelightManager().getCurrentPipeline()
                : null;
        if (rawPipeline instanceof RelocalizationPipeline) {
            relocPipeline = (RelocalizationPipeline) rawPipeline;
        }

        robot.init();
        robot.updateDriveCommand = infinite(robot::updateDrive);
        Scheduler.schedule(robot.updateDriveCommand);
        robot.setState(States.NONE);
        telemetry.update();

        waitForStart();

        boolean prevB = false;

        while (opModeIsActive()) {
            robot.clearCaches();

            boolean currB = gamepad1.b;
            if (currB && !prevB) {
                Pose drifted = new Pose(
                        robot.getPose().getX() + ARTIFICIAL_DRIFT_INCHES,
                        robot.getPose().getY(),
                        robot.getPose().getHeading());
                robot.setPose(drifted);
            }
            prevB = currB;

            addVisionTelemetry();

            Scheduler.execute();
        }
    }

    private void addVisionTelemetry() {
        Pose currentPose = robot.getPose();


        telemetry.addData("Odometry X (in)", "%.2f", currentPose.getX());
        telemetry.addData("Odometry Y (in)", "%.2f", currentPose.getY());
        telemetry.addData("Odometry Heading (deg)", "%.1f", Math.toDegrees(currentPose.getHeading()));

        if (relocPipeline != null) {
            telemetry.addLine("--- MT2 Vision ---");
            telemetry.addData("Tags Visible", relocPipeline.getLastTagCount());
            telemetry.addData("Total Corrections", relocPipeline.getCorrectionCount());
            telemetry.addData("CORRECTED THIS LOOP", relocPipeline.correctedThisLoop() ? "YES <<" : "no");

            Pose lastFix = relocPipeline.getLastCorrectedPose();
            if (lastFix != null) {
                telemetry.addData("Last MT2 X (in)", "%.2f", lastFix.getX());
                telemetry.addData("Last MT2 Y (in)", "%.2f", lastFix.getY());
                telemetry.addData("Last MT2 Heading (deg)", "%.1f", Math.toDegrees(lastFix.getHeading()));

                double dx = currentPose.getX() - lastFix.getX();
                double dy = currentPose.getY() - lastFix.getY();
                telemetry.addData("Delta to Last MT2 (in)", "%.2f", Math.hypot(dx, dy));
            } else {
                telemetry.addLine("No mega 2");
            }
        } else {
            telemetry.addLine("no lim");
        }

    }
}
