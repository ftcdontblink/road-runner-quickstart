package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake implements Subsystem {
    Robot robot;
    DcMotor motor1;
    DcMotor motor2;

    Servo leftIntakeServo;
    Servo rightIntakeServo;

    public static double posLeft = 1;
    public static double posRight = 0;

    public Intake(Robot robot) {
        this.robot = robot;
        motor1 = robot.hwMap.get(DcMotor.class, "m1");
        motor2 = robot.hwMap.get(DcMotor.class, "m2");

//        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo = robot.hwMap.get(Servo.class, "lis");
        rightIntakeServo = robot.hwMap.get(Servo.class, "ris");

        leftIntakeServo.setPosition(posLeft);
        rightIntakeServo.setPosition(posRight);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        motor1.setPower(robot.gamepad2.right_trigger - robot.gamepad2.left_trigger);
        motor2.setPower(robot.gamepad2.right_trigger - robot.gamepad2.left_trigger);

        if(robot.gamepad1.dpad_up) {
            leftIntakeServo.setPosition(0.9);
            rightIntakeServo.setPosition(0.1);
        }

        if(robot.gamepad1.dpad_left) {
            leftIntakeServo.setPosition(0.95);
            rightIntakeServo.setPosition(0.05);
        }

        if(robot.gamepad1.dpad_down) {
            leftIntakeServo.setPosition(0.97);
            rightIntakeServo.setPosition(0.03);
        }
    }

    public void on() {
        motor1.setPower(1);
        motor2.setPower(1);
    }

    public void off() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void reverse() {
        motor1.setPower(-1);
        motor2.setPower(-1);
    }
}
