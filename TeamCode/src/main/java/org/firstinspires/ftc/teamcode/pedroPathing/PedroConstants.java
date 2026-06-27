package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.MT1Localizer;

@Config
public class PedroConstants {
    private static AprilTagLocalizer aprilTagLocalizer;
    private static PinpointLocalizer pinpointLocalizer;
    private static MT1Localizer mt1Localizer;

    public static PinpointLocalizer getPinpointLocalizer() {return pinpointLocalizer;}
    public static AprilTagLocalizer getAprilTagLocalizer() { return aprilTagLocalizer; }
    public static MT1Localizer getMT1Localizer() { return mt1Localizer; }

    public static PIDFCoefficients secondaryHeadingCoeffs = new PIDFCoefficients(0.6, 0, 0.005, 0.025);

    public static PIDFCoefficients headingCoeffs = new PIDFCoefficients(1.5, 0, 0.2, 0.055);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-29.286)
            .lateralZeroPowerAcceleration(-68.86)
            .headingPIDFCoefficients(headingCoeffs)
            .secondaryHeadingPIDFCoefficients(secondaryHeadingCoeffs)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.05, 0.06177067553833598, 0.002046302556539832
//                    0.05, 0.03678858798826817, 0.002539134355411748
            ))
//    kQuadraticFriction = 0.002046302556539832
//    kLinearBraking = 0.06177067553833598


            .centripetalScaling(0)
            .mass(11.34);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.16830709)
            .strafePodX(-1.5674507874)
            .distanceUnit(DistanceUnit.INCH).hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants mecanumConstants = new  MecanumConstants()
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .useBrakeModeInTeleOp(true)
            .xVelocity(111.048)
            .yVelocity(79.5);

    // TODO: TUNE THESE, CAN MAKE A HUGE DIFF
    // public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);
    public static PathConstraints pathConstraints =
            new PathConstraints(0.95,
                    0.5,
                    0.5,
                    0.03,
                    50,
                    1,
                    10,
                    1);
//            new PathConstraints(0.9,
//                    1,
//                    1,
//                    0.03,
//                    50,
//                    1.25,
//                    10,
//                    1
//            );

    // default
    // public static PathConstraints defaultConstraints = new PathConstraints(0.995, 0.1, 0.1,
    // 0.007, 100, 1, 10, 1);
    public static Follower createPinpointFollower(HardwareMap hardwareMap) {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, localizerConstants);

        return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
//                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap),
//                        leftBack(hardwareMap), rightBack(hardwareMap))
                .setLocalizer(pinpointLocalizer).build();



//        public FollowerBuilder mecanumDrivetrain(MecanumConstants mecanumConstants) {
//            return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
//        }
    }

    public static Follower createAprilTagFollower(HardwareMap hardwareMap) {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, localizerConstants);
        aprilTagLocalizer = new AprilTagLocalizer(pinpointLocalizer, hardwareMap);
        return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .setLocalizer(aprilTagLocalizer.getLocalizer())
                .build();
    }

    public static Follower createMT1Follower(HardwareMap hardwareMap) {
        pinpointLocalizer = new PinpointLocalizer(hardwareMap, localizerConstants);
        mt1Localizer = new MT1Localizer(hardwareMap, pinpointLocalizer);
        return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .setLocalizer(mt1Localizer.getLocalizer())
                .build();
    }
}
