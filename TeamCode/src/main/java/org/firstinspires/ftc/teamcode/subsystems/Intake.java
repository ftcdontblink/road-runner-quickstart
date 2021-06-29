package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake implements Subsystem {
    Robot robot;
    DcMotor motor1;
    DcMotor motor2;

    public Intake(Robot robot) {
        this.robot = robot;
        motor1 = robot.hwMap.get(DcMotor.class, "m1");
        motor2 = robot.hwMap.get(DcMotor.class, "m2");

//        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        motor1.setPower(robot.gamepad2.right_trigger - robot.gamepad2.left_trigger);
        motor2.setPower(robot.gamepad2.right_trigger - robot.gamepad2.left_trigger);
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
