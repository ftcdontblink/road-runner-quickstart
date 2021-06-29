package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

public class Turret implements Subsystem {
    Robot robot;
    DcMotor turretMotor;

    public Turret(Robot robot) {
        this.robot = robot;
        turretMotor = robot.hwMap.get(DcMotor.class, "turret");
    }

    public void update() {

    }
}
