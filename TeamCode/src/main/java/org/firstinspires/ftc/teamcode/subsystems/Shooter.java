package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

public class Shooter implements Subsystem {
    Robot robot;
    DcMotorEx flywheel;

    public Shooter(Robot robot) {
        this.robot = robot;
        flywheel = (DcMotorEx) robot.hwMap.get(DcMotor.class, "flywheel");
    }

    @Override
    public void update() {

    }
}
