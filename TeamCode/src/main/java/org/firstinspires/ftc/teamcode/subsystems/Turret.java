package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

public class Turret implements Subsystem {
    Robot robot;
    DcMotor turretMotor;

    public final double TICKS = 537.6;
    public final double RATIO = 72.0/24.0;
    public final double TICKS_TO_RAD = (TICKS*RATIO)/(2*Math.PI);
    public final double TICKS_TO_DEG = (TICKS*RATIO)/(360);
    public static double OFFSET = 0;

    PIDFController turretController;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public Turret(Robot robot) {
        this.robot = robot;
        turretMotor = robot.hwMap.get(DcMotor.class, "turret");

        turretController = new PIDFController(kP, kI, kD, kF);
        turretController.setTolerance(Math.toRadians(0.5));
    }

    public void update() {
        double targetAngle = Angle.normDelta(robot.mecanumDrive.theta);
        double currentAngle = ticksToRad(turretMotor.getCurrentPosition()) + OFFSET;

        double output = turretController.calculate(currentAngle, targetAngle);

        turretMotor.setPower(output);
    }

    public double ticksToRad(double ticks) {
        return TICKS_TO_RAD*ticks;
    }

    public double ticksToDeg(double ticks) {
        return TICKS_TO_DEG*ticks;
    }
}
