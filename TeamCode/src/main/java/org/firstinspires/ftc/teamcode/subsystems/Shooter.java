package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.TimedAction;

import java.sql.Time;

@Config
public class Shooter implements Subsystem {
    Robot robot;
    DcMotorEx flywheel;
    Servo flicker;
    TimedAction action;
    public static double velo = 0;
    public static double start = 50;
    public static double end = 100;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(150, 0, 0, 15);

    public Shooter(Robot robot) {
        this.robot = robot;
        flywheel = robot.hwMap.get(DcMotorEx.class, "flywheel");
        flicker = robot.hwMap.get(Servo.class, "kicker");

        action = new TimedAction(
                () -> flicker.setPosition(0.12),
                () -> flicker.setPosition(0.22),
                start,
                end
        );

        for (LynxModule module : robot.hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);

        // Turn on RUN_USING_ENCODER
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / robot.hwMap.voltageSensor.iterator().next().getVoltage()
        ));
    }

    public void update() {
        double targetVelo = velo;
        flywheel.setVelocity(targetVelo);

        if (robot.gamepad2.x && !action.running()) action.reset();
        action.run();
    }
}
