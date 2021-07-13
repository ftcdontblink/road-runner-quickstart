package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Robot {
    List<Subsystem> subsystems = new ArrayList<>();

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
        initAll(hwMap);
    }

    public Robot(HardwareMap hwMap, Gamepad gamepad1, Gamepad gamepad2) {
        initAll(hwMap);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void initAll(HardwareMap hwMap) {
        this.hwMap = hwMap;
//        drive = new SampleMecanumDrive(hwMap);
//        mecanumDrive = new MecanumDrive(this);
//        wobble = new Wobble(this);
//        turret = new Turret(this);
        intake = new Intake(this);
        shooter = new Shooter(this);

        addAll();
    }

    public void addAll() {
        Collections.addAll(
                subsystems,
//                wobble,
//                mecanumDrive,
//                turret,
                intake,
                shooter
        );
    }

    public void update() {
        for(Subsystem s : subsystems) s.update();
    }
}
