package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import android.media.MediaTimestamp;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.field.Style;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.opmodes.ArdCamTesting;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.shooter.Limelight;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.Turret;
import org.firstinspires.ftc.teamcode.states.State;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.MT1Localizer;

import java.util.function.DoubleSupplier;

@Config
public class Robot {

    public static int singleShootTimeMillis = 250;
//    public ServoEx gatePusher;
//    public static double BLUE_SIDE_OUT = 0.33;
//    public static double BLUE_SIDE_IN = 0.93;

    public Command updateDriveCommand;

    private State currentState;

    Timer timer = new Timer();
    ElapsedTime telemetryTimer = new ElapsedTime();
    int totalMillis = 0;
    int numLoops = 0;
    int maxLoopTime = 0;
    int minLoopTime = 9999;

    boolean currentlyShooting = false;

    PTO pto;
    Shooter shooter;
    Limelight limelight;
//    ProximityIndicator proximityIndicator;

    public enum LocalizationMode {
        FOLLOWER,
        PINPOINT,
        ARDFUSION,
        LLFUSION
    }

    Follower follower;
    AprilTagLocalizer aprilTagLocalizer;
    MT1Localizer mt1Localizer;
    private LocalizationMode localizationMode = LocalizationMode.FOLLOWER;
    private Pose currentPose;

    private Drivetrain drivetrain;

    Gamepad gamepad1;
    Gamepad gamepad2;

    Telemetry telemetry;


    VoltageSensor voltageSensor;

    Pose goalPose;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Pose goalPose, LocalizationMode localizationMode) {
        this.telemetry = telemetry;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        Drawing.init();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pto = new PTO(hardwareMap);
//        proximityIndicator = new ProximityIndicator(hardwareMap);

        this.localizationMode = localizationMode;
        if (localizationMode == LocalizationMode.ARDFUSION){
            follower = PedroConstants.createAprilTagFollower(hardwareMap);
            this.aprilTagLocalizer = PedroConstants.getAprilTagLocalizer();
        } else if (localizationMode == LocalizationMode.LLFUSION) {
            follower = PedroConstants.createMT1Follower(hardwareMap);
            this.mt1Localizer = PedroConstants.getMT1Localizer();
        } else {
            follower = PedroConstants.createPinpointFollower(hardwareMap);
        }

        limelight = new Limelight(hardwareMap);

        currentPose = follower.getPose();

        shooter = new Shooter(hardwareMap, voltageSensor);

        setDrivetrain(Drivetrains.MECANUM);
        setState(States.NONE);

        this.goalPose = goalPose;
//        gatePusher = new ServoEx(hardwareMap, "gatePush");
//        gatePusher.setPosition(BLUE_SIDE_OUT);

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

    public void stop(){
        //aprilTagLocalizer.close();
    }

//    public void updateLastTurretTicks() {
//        if (shooter.getTurretPos() != 0 || Constants.getLastTurretTicks() == 0)  {
//            Constants.setLastTurretTicks(shooter.getTurretPos());
//        }
//    }

    public void updateLocalization() {
        double turretAngle = getTurretAngleDegrees();
        Pose mt1Pose = limelight.getMT1Pose(turretAngle, gamepad1);
        if (mt1Pose != null) {
            follower.setPose(mt1Pose);
        }
        if (localizationMode == LocalizationMode.PINPOINT) {
            updatePinpoint();
        } else if (localizationMode == LocalizationMode.ARDFUSION){
            follower.update();
            aprilTagLocalizer.update(telemetry);
            currentPose = follower.getPose();
            Drawing.drawRobot(follower.getPose(), ArdCamTesting.STYLE_FUSION);
            Drawing.drawRobot(PedroConstants.getPinpointLocalizer().getPose(), ArdCamTesting.STYLE_PINPOINT);
            Drawing.sendPacket();
        } else if (localizationMode == LocalizationMode.LLFUSION) {
            follower.update();
            mt1Localizer.updateLLPose(follower, shooter.getCurrentTurretAngle());
            currentPose = follower.getPose();
            Drawing.drawRobot(follower.getPose(), ArdCamTesting.STYLE_FUSION);
            Drawing.drawRobot(PedroConstants.getPinpointLocalizer().getPose(), ArdCamTesting.STYLE_PINPOINT);
            Drawing.drawRobot(MT1Localizer.pedroPose, ArdCamTesting.STYLE_LL_ROBOT);
            Drawing.sendPacket();
        }
        else {
            follower.update();
            currentPose = follower.getPose();
            Drawing.drawRobot(follower.getPose(), ArdCamTesting.STYLE_LL_TURRET);
            Drawing.sendPacket();
        }

        Constants.lastPose = currentPose;
    }

    private void updatePinpoint() {
        PinpointLocalizer localizer = PedroConstants.getPinpointLocalizer();
        localizer.update();
    }

    public Pose getPose() {
        return currentPose;
    }

    public Vector getVelocity() {
        return localizationMode == LocalizationMode.PINPOINT
                ? PedroConstants.getPinpointLocalizer().getVelocityVector()
                : follower.getVelocity();
    }

    public double getAngularVelocity() {
        return localizationMode == LocalizationMode.PINPOINT
                ? PedroConstants.getPinpointLocalizer().getVelocity().getHeading()
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

    public void moveGoalPose(double dx, double dy) {
        goalPose = new Pose(goalPose.getX() + dx, goalPose.getY() + dy);
    }

    public Command updateShootingSubsystems() {
        return infinite(() -> shooter.updateShootingSubsystems(getPose(), goalPose, getVelocity(), getAngularVelocity(), telemetry));
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
        telemetry.addData("Current goal pose", goalPose);
        telemetry.addLine();
        telemetry.addData("flywheel enabled", getFylwheelActivated());
        telemetry.addData("flywheel velocity", getFlywheelAngularVelocity());
        telemetry.addData("target flywheel velocity", getTargetFlywheelAngularVelocity());
        telemetry.addData("turret angle (deg)", getTurretAngleDegrees());
        telemetry.addData("hood angle (deg)", getHoodAngleDegrees());

        telemetry.addData("avg loop time", totalMillis / numLoops);
        telemetry.addData("max loop time", maxLoopTime);
        telemetry.addData("min loop time", minLoopTime);

        telemetry.addData("currentVoltage", getVoltage());

        if (Constants.debugTelemetry) {
            telemetry.addData("Drivetrain:", drivetrainName());
//            telemetry.addLine("Photon enabled!");
            telemetry.addData("current loop time", currentLoopTime);
            telemetry.addData("Current State: ", currentState.name());
        }

        telemetry.update();
    }

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
        return Math.toDegrees(shooter.getCurrentTurretAngle());
    }

    public double getHoodAngleDegrees() {
        return Math.toDegrees(shooter.getHoodAngle());
    }

    public double getFlywheelAngularVelocity() {
        return shooter.getFlywheelAngularVelocity();
    }

    public boolean getFylwheelActivated() {
        return shooter.getFlywheelActivated();
    }

    public boolean isShooterReady() {
        return shooter.isFlywheelReady();
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
            setPose(currentPose);
        }
    }

    public void setPose(Pose pose) {
        currentPose = pose;

        follower.setPose(pose);
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
//        boolean close = getPose().getY() > Shooter.transitionYValue;

        double intakePower = Math.min(1, getVoltage() / 12.5);

        return sequential(
                openGate(),
                instant(() -> currentlyShooting = true),
                setIntakePower(intakePower),
                waitMs(shootingTime),
                closeGate(),
                instant(() -> currentlyShooting = false)
        );
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

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }
}
