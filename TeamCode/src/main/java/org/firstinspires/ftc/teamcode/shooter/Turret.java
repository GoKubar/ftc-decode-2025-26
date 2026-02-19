package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import smile.interpolation.LinearInterpolation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

@Config
public class Turret {

    ServoEx turretServoF;
    ServoEx turretServoB;

    public static double MIN_TURRET_ANGLE = Math.toRadians(-90);
    public static double MAX_TURRET_ANGLE = Math.toRadians(90);

    double[] angleValues = new double[] {Math.toRadians(-90), Math.toRadians(90)};
    double[] servoPositions = new double[] {0, 1}; //TODO: tune

    public double getAngleFromServoPos(double servoPos) {
        return (servoPos - servoPositions[0]) *
                (angleValues[1] - angleValues[0])/(servoPositions[1] - servoPositions[0]) //slope
                + angleValues[0]; //intercept
    }

    public double getServoPosFromAngle(double angleRad) {
        return (angleRad - angleValues[0]) *
                (servoPositions[1] - servoPositions[0]) / (angleValues[1] - angleValues[0]) //slope
                + servoPositions[0]; //intercept
    }


    public double MIN_SERVO_POS = getServoPosFromAngle(MIN_TURRET_ANGLE);
    public double MAX_SERVO_POS = getServoPosFromAngle(MAX_TURRET_ANGLE);

    public Turret(HardwareMap hardwareMap) {
        this.turretServoF = new ServoEx(hardwareMap, "turretServoFront");
        this.turretServoB = new ServoEx(hardwareMap, "turretServoBack");
    }

    public double getTargetPosition() {
        return turretServoF.getPosition();
    }

    /**
     * @return The target angle of the turret in radians. Note that counter clockwise is positive with 0 being straight forward.
     */
    public double getTargetAngle() {
        double currentPos = getTargetPosition();
        return getAngleFromServoPos(currentPos);
    }

    private void setTargetServoPosition(double pos) {
        turretServoF.setPosition(pos);
        turretServoB.setPosition(pos);
    }

    public void setTurretAngle(double angleRad) {
        setTargetServoPosition(
                Range.clip(getServoPosFromAngle(angleRad),
                        MIN_SERVO_POS,
                        MAX_SERVO_POS
                )
        );
    }
}
