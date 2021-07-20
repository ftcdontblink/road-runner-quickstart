package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import com.acmerobotics.roadrunner.util.NanoClock;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.ModifiedPIDFController;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.PoseStorage;
import org.opencv.core.Point;
import org.opencv.core.Rect;

@Config
public class Turret implements Subsystem {
    Robot robot;
    DcMotor turretMotor;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(0.03, 0.06, 0.1, 0.1);
    public static PIDFCoefficients coeffs = new PIDFCoefficients(0.03, 0.06, 0.2, 0);
    public static double kS = 0.75;
    public static double angle = 0;
    public static double currentPosition = 0;
    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;

    public final double TICKS = 28*26.9;
    public final double RATIO = 72.0/24.0;
    public final double PI = Math.PI;
    public static double OFFSET = 0;

    public static PIDFCoefficients customCoeff = new PIDFCoefficients(0.03, 0.0, 0.1, 0.03);


    ElapsedTime timer;
    public static double value;
    double currentTime;


    PIDFController turretController;

    private double lastValue = 0;
    private double lastTime = 0;

    public Turret(Robot robot) {
        timer = new ElapsedTime();

        this.robot = robot;
        turretMotor = robot.hwMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new PIDFController(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
        turretController.setTolerance(0.5);
        value = 0;
    }

    public Turret(HardwareMap map) {
        turretMotor = map.get(DcMotor.class, "turret");
    }

    public double ticksToRad(double ticks) {
        return 2*PI * (ticks / (TICKS*RATIO));
    }

    public double ticksToDeg(double ticks) {
        return Math.toDegrees(ticksToRad(ticks));
    }

    @Override
    public void update() {
//        currentTime = timer.milliseconds();
////
        value = (Math.PI*2) * (turretMotor.getCurrentPosition() / (double)(28*26.9*3));
        value = Math.toDegrees(value);
//
//        if(value < -180) value += 360;
//        if(value > 180) value -= 360;
//
//        value = Range.clip(value, -40, 40);
//
//        double error = angle - value;
//
//        double d = coefficients.d * (((angle - lastValue) / (currentTime - lastTime)));
//
//        double power = (coefficients.p * error) + d + (coefficients.f*error);
//
//        if(Math.abs(error) < 0.4) power = 0;
//        if(robot.gamepad1.a)
//            turretMotor.setPower(power);
//        else
//            turretMotor.setPower(0);
//
////
//        robot.telemetry.addData("Power: ", power);
//        robot.telemetry.addData("Error: ", error);
//        robot.telemetry.addData("Value: ", value);
//        robot.telemetry.update();
//
//        lastValue = angle;
//        lastTime = currentTime;

        turretController.setSetPoint(angle);
        double power = turretController.calculate(value);

        robot.telemetry.addData("power", power);
        robot.telemetry.update();

        if(Math.abs(turretController.getPositionError()) < 0.4) power = 0;
        if(robot.gamepad1.a)
            turretMotor.setPower(power);
        else
            turretMotor.setPower(0);

    }

    public void setAngle(double a) {
        angle = a;
    }
}
