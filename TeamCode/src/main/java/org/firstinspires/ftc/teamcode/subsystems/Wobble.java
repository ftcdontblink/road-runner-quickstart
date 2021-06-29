package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.Collection;

public class Wobble implements Subsystem {
    Robot robot;
    Servo arm1;
    Servo arm2;
    Servo claw1;
    Servo claw2;

    public enum State {
        WOBBLE_DOWN,
        WOBBLE_INIT,
        WOBBLE_RELEASES
    }

    State state = State.WOBBLE_INIT;

    public Wobble(Robot robot) {
        this.robot = robot;
        arm1 = robot.hwMap.get(Servo.class, "a1");
        arm2 = robot.hwMap.get(Servo.class, "a2");
        claw1 = robot.hwMap.get(Servo.class, "c1");
        claw2 = robot.hwMap.get(Servo.class, "c2");
    }

    public void openClaw() {

    }

    public void closeClaw() {

    }

    public void raiseArm() {

    }

    public void dropArm() {

    }

    public void initArm() {

    }

    public void update() {

    }
}
