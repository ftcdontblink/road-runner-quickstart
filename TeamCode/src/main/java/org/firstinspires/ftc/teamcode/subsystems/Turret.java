package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import com.acmerobotics.roadrunner.util.NanoClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.ModifiedPIDFController;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.PoseStorage;

@Config
public class Turret implements Subsystem {
    Robot robot;
    DcMotor turretMotor;
    public static double angle = 2;
    public static double currentPosition = 0;
    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;

    public final double TICKS = 28*13.7;
    public final double RATIO = 72.0/24.0;
    public final double PI = Math.PI;
    public static double OFFSET = 0;

    ModifiedPIDFController turretController;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.01, 0, 0);
    public static double integralBand = 0.7;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public Turret(Robot robot) {
        this.robot = robot;
        turretMotor = robot.hwMap.get(DcMotor.class, "turret");

        turretController = new ModifiedPIDFController(coefficients, 0.7);
    }

    public Turret(HardwareMap map) {
        turretMotor = map.get(DcMotor.class, "turret");

        turretController = new ModifiedPIDFController(coefficients, 0.7);
    }

    public double ticksToRad(double ticks) {
        return 2*PI * (ticks / (TICKS*RATIO));
    }

    public double ticksToDeg(double ticks) {
        return Math.toDegrees(ticksToRad(ticks));
    }

    @Override
    public void update() {
        currentPosition = ticksToRad(turretMotor.getCurrentPosition());

        turretController.setTargetPosition(angle);
        turretController.update(currentPosition);
    }
}
