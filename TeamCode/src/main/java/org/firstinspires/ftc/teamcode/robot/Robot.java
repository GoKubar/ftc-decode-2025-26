package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.shooter.math.VelocityCompensationCalculator;
import org.firstinspires.ftc.teamcode.states.State;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

import java.util.function.DoubleSupplier;

@Config
public class Robot {

    public static int singleShootTimeMillis = 250;
//    public ServoEx gatePusher;
//    public static double BLUE_SIDE_OUT = 0.33;
//    public static double BLUE_SIDE_IN = 0.93;

    public Command updateDriveCommand;

    private State currentState;

    private boolean useVelocityComp = true;

    Timer timer = new Timer();
    ElapsedTime telemetryTimer = new ElapsedTime();
    int totalMillis = 0;
    int numLoops = 0;
    int maxLoopTime = 0;
    int minLoopTime = 9999;

    boolean currentlyShooting = false;

    PTO pto;
    Shooter shooter;
//    ProximityIndicator proximityIndicator;

    public enum LocalizationMode {
        FOLLOWER,
        PINPOINT
    }

    Follower follower;
    private final GoBildaPinpointDriver pinpoint;
    private LocalizationMode localizationMode = LocalizationMode.FOLLOWER;
    private Pose currentPose;
    private final Vector currentVelocity = new Vector();

    private Drivetrain drivetrain;
//    private LimelightManager limelightManager;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose goalPose) {
        this.telemetry = telemetry;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        pto = new PTO(hardwareMap);
//        proximityIndicator = new ProximityIndicator(hardwareMap);

        follower = PedroConstants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        shooter = new Shooter(hardwareMap, goalPose);
        currentPose = follower.getPose();
        currentVelocity.setOrthogonalComponents(0, 0);

        setDrivetrain(Drivetrains.SWERVE_HEADING_LOCK);
        setState(States.NONE);

//        gatePusher = new ServoEx(hardwareMap, "gatePush");
//        gatePusher.setPosition(BLUE_SIDE_OUT);

//        limelightManager = new LimelightManager(hardwareMap, telemetry);
//        setPipeline(Pipelines.APRIL_TAG);

        // Setup in your Robot class if you have one, or in init at start of opMode
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(10); // Can be adjusted based on user preference - but raising this number further can cause issues
        PhotonCore.enable();
    }


    public void init() {
        timer.resetTimer();
        telemetryTimer.reset();
        schedule(
                infinite(this::loop)
        );
    }


    public Follower getFollower() {
        return follower;
    }

    public void loop() {
        updateLocalization();
        executeCurrentState();
        updateShooter();
//        if (getCurrentState() == States.INTAKING) {
//            updateProximityIndicator();
//        }
        updateTelemetry();
//        updateLastTurretTicks();
    }

//    public void updateLastTurretTicks() {
//        if (shooter.getTurretPos() != 0 || Constants.getLastTurretTicks() == 0)  {
//            Constants.setLastTurretTicks(shooter.getTurretPos());
//        }
//    }

    public void updateLocalization() {
        if (localizationMode == LocalizationMode.PINPOINT) {
            updatePinpoint();
        } else {
            follower.update();
            currentPose = follower.getPose();
        }

        Constants.lastPose = currentPose;
    }

    private void updatePinpoint() {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        currentPose = new Pose(x, y, heading);
        currentVelocity.setOrthogonalComponents(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH));
    }

    public Pose getPose() {
        return currentPose;
    }

    public Vector getVelocity() {
        return localizationMode == LocalizationMode.PINPOINT
                ? currentVelocity
                : follower.getVelocity();
    }

    public double getAngularVelocity() {
        return localizationMode == LocalizationMode.PINPOINT
                ? pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
                : follower.getAngularVelocity();
    }

    public void clearCaches() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }

    public Command deactivateShooter() {
        return instant(() -> shooter.deactivate());
    }

    public Command deactivateFlywheel() {
        return instant(() -> shooter.deactivateFlywheel());
    }

    public Command shooterIntakingPos() {
        return instant(() -> shooter.intakingPos());
    }

    public Command activateShooter() {
        return instant(() -> shooter.activate());
    }

    public Command toggleShooter() {
        return instant(() -> shooter.toggle());
    }

    public Command setIntakePower(double power) {
        return instant(() -> pto.runIntake(power));
    }

    public Command joysticksToIntakePower(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        return infinite(() -> {
            if (rightTrigger.getAsDouble() > 0.02) {
                pto.runIntake(rightTrigger.getAsDouble());
            } else if (leftTrigger.getAsDouble() > 0.02) {
                pto.runIntake(-leftTrigger.getAsDouble());
            } else {
                pto.runIntake(0);
            }
        });
    }

    public void updateShooter() {
        shooter.update(telemetry);
    }

//    public void updateProximityIndicator() {
////        proximityIndicator.update();
//    }

    public Command setTurretPos(double angle) {
        return instant(() -> shooter.setTurretAngle(angle));
    }

    public Command updateShootingSubsystems() {
        return infinite(() -> shooter.updateShootingSubsystems(getPose(), getVelocity(), telemetry, useVelocityComp));
    }

    public Command updateTurret() {
        return infinite(() -> shooter.updateTurretOnly(getPose(), getVelocity(), telemetry, useVelocityComp));
    }

    public boolean readyToShoot() {
        return shooter.readyToShoot();
    }

    public void updateTelemetry() {
        int currentLoopTime = (int) timer.getElapsedTime();
        totalMillis += currentLoopTime;
        numLoops += 1;
        maxLoopTime = Math.max(maxLoopTime, currentLoopTime);
        minLoopTime = Math.min(minLoopTime, currentLoopTime);
        timer.resetTimer();

        if (telemetryTimer.milliseconds() < Constants.TELEMETRY_UPDATE_MS) {
            return;
        }
        telemetryTimer.reset();

        telemetry.addData("turret offset (deg)", Math.toDegrees(Turret.turretOffsetRad));
        telemetry.addData("Updated lastPose", Constants.lastPose);
        telemetry.addData("flywheel velocity", getFlywheelAngularVelocity());
        telemetry.addData("target flywheel velocity", getTargetFlywheelAngularVelocity());
        telemetry.addData("turret angle (deg)", getTurretAngleDegrees());
        telemetry.addData("hood angle (deg)", getHoodAngleDegrees());

        telemetry.addData("avg loop time", totalMillis / numLoops);
        telemetry.addData("max loop time", maxLoopTime);
        telemetry.addData("min loop time", minLoopTime);

        if (Constants.debugTelemetry) {
            telemetry.addData("Drivetrain:", drivetrainName());
            telemetry.addLine("Photon enabled!");
            telemetry.addData("current loop time", currentLoopTime);
            telemetry.addData("Current State: ", currentState.name());
        }

        telemetry.update();
    }

//    public void limelightProcess() {
//        telemetry.addData("State:", getCurrentState().name());
//        telemetry.addData("Pipeline:", limelightManager.getCurrentPipelineName());
//        limelightManager.process(this);
//    }

    public void executeCurrentState() {
        currentState.execute(this);
    }


    public States getCurrentState() {
        if (currentState != null) {
            return States.get(currentState);
        } else {
            return States.NONE;
        }
    }

    public void setState(States newState) {
        State prevState = currentState;
        currentState = newState.build(telemetry, gamepad1, gamepad2);
        currentState.initialize(this, prevState);
    }

    public double getTurretAngleDegrees() {
        return Math.toDegrees(shooter.getTurretAngle());
    }

    public double getHoodAngleDegrees() {
        return Math.toDegrees(shooter.getHoodAngle());
    }

    public double getFlywheelAngularVelocity() {
        return shooter.getFlywheelAngularVelocity();
    }

    public double getTargetFlywheelAngularVelocity() {
        return shooter.getFlywheeelTargetAngularVelocity();
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public void setDrivetrain(Drivetrains drivetrain) {
       this.drivetrain = drivetrain.build(this, follower, telemetry);
    }

    public void setLocalizationMode(LocalizationMode mode) {
        localizationMode = mode;
        if (localizationMode == LocalizationMode.PINPOINT) {
            currentVelocity.setOrthogonalComponents(0, 0);
            setPose(currentPose);
        }
    }

    public void setPose(Pose pose) {
        currentPose = pose;
        follower.setPose(pose);
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    pose.getX(),
                    pose.getY(),
                    AngleUnit.RADIANS,
                    pose.getHeading()));
        }
        Constants.lastPose = pose;
    }

    public void updateDrive() {
        drivetrain.update(gamepad1);
    }

    public void updateDrive(double speed, double rotSpeed) {
        drivetrain.update(gamepad1, speed, rotSpeed);
    }

    public Command openGate() {
        return instant(shooter::setOpenGatePosition);
    }

    public Command shoot() {
        return sequential(
                openGate(),
                instant(() -> currentlyShooting = true),
                setIntakePower(1),
                waitMs(singleShootTimeMillis),
                closeGate(),
                waitMs(150),
                instant(() -> currentlyShooting = false)
        );
    }

    public Command shootMotif() {
        return shootMotif(800);
    }

    public Command shootMotif(int shootingTime) {
        boolean close = getPose().getY() > Shooter.transitionYValue;
        return (close) ? sequential(
                openGate(),
                instant(() -> currentlyShooting = true),
                setIntakePower(1),
                waitMs((close) ? shootingTime : shootingTime+400),
                closeGate(),
                instant(() -> currentlyShooting = false)
        ) : sequential(
                shoot(),
                shoot(),
                shoot()
        );
//        boolean close = follower.getPose().getY() > Shooter.transitionYValue;
//        boolean close = true;
//        return sequential(
//                shoot(),
//                waitUntil(() -> readyToShoot()).raceWith(waitMs(150)),
//                shoot(),
//                waitUntil(() -> readyToShoot()).raceWith(waitMs(150)),
//                shoot()
//        );
    }

    public Command closeGate() {
        return instant(shooter::setCloseGatePosition);
    }

    public String drivetrainName() {
        return drivetrain.name();
    }


    public Command setPtoToIntaking() {
        return instant(() -> pto.setIntaking());
    }

    public Command setPtoToLifting() {
        return instant(() -> pto.setLifting());
    }

    public void runLift() {
        pto.runLift();
    }


//    public void setPipeline(Pipelines pipeline) {
//        limelightManager.setPipeline(pipeline);
//    }
//
//    public void cycleNextPipeline() {
//        limelightManager.cycleNext();
//    }
//
//    public void cyclePreviousPipeline() {
//        limelightManager.cyclePrevious();
//    }
//
//    public Pipelines getCurrentPipeline() {
//        return limelightManager.getCurrentPipelineEnum();
//    }
//
//    public VisionPipeline getVisionPipeline() {
//        return limelightManager.getCurrentPipeline();
//    }
//
//    public LimelightManager getLimelightManager() {
//        return limelightManager;
//    }
//
//    public VisionResult getVisionResult() {
//        return limelightManager.getResult();
//    }
}
