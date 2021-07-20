package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.ModifiedPIDFController;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Constants.PI;

@Config
public class Turret implements Subsystem {
    public static PIDCoefficients coefficients = new PIDCoefficients(3.0, 1.0, 0.2);
    public static double INTEGRAL_BAND = toRadians(2.0);
    public static double TOLERANCE = toRadians(0.5);

    public static double kV = 1.0 / ticksToRad(2800);
    public static double kStatic = 0.1;

    public static double TICKS_PER_REV = 28 * 26.9;
    public static double RATIO = 72.0 / 24.0;
    public static double OFFSET = 0;

    private Robot robot;
    private DcMotorEx turretMotor;

    ModifiedPIDFController turretController;

    private double targetAngle = 0;
    private double targetAngularVelocity = 0;

    private double lastPosition = 0;
    private double lastVelocity = 0;

    public Telemetry telemetry;

    public Turret(Robot robot) {
        this.robot = robot;
        turretMotor = robot.hwMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new ModifiedPIDFController(coefficients, INTEGRAL_BAND, 1, 0, kStatic * kV);
        turretController.setInputBounds(-PI, PI);
        telemetry = robot.telemetry;
    }

    public static double ticksToRad(double ticks) {
        return 2 * PI * (ticks / (TICKS_PER_REV * RATIO)) + OFFSET;
    }

    @Override
    public void update() {
        double currentPosition = ticksToRad(turretMotor.getCurrentPosition());
        double currentVelocity = ticksToRad(turretMotor.getVelocity());

        turretController.setTargetPosition(targetAngle);
        turretController.setTargetVelocity(targetAngularVelocity);

        double output = turretController.update(currentPosition, currentVelocity) * kV;
        turretMotor.setPower(output);

        lastPosition = currentPosition;
        lastVelocity = currentVelocity;
    }

    public double getPosition() {
        return lastPosition;
    }

    public double getVelocity() {
        return lastVelocity;
    }

    public void setTargetAngle(double a) {
        targetAngle = a;
    }

    public void setTargetAngularVelocity(double w) {
        targetAngularVelocity = w;
    }
}