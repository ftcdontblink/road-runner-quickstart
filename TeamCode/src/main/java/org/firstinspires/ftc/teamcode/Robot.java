package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;

public class Robot {
    public HardwareMap hwMap;
    public SampleMecanumDrive drive;
    public Intake intake;
    public Turret turret;
    public Shooter shooter;
    public MecanumDrive mecanumDrive;
    public Wobble wobble;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drive = new SampleMecanumDrive(hwMap);
        mecanumDrive = new MecanumDrive(this);
        wobble = new Wobble(this);
        turret = new Turret(this);
        intake = new Intake(this);
        shooter = new Shooter(this);
    }

    public void Robot(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hwMap = hwMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update() {

    }
}
