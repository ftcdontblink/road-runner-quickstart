package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.AutoAim;

@Config
@TeleOp
public class NewTestForTurret extends LinearOpMode {
    Robot robot;
    public static double x = 0;
    public static double y = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);
        AutoAim aim = new AutoAim(robot.turret);
        waitForStart();

        while(opModeIsActive()) {
            aim.setTargetPosition(new Vector2d(x, y));
            aim.updateLocalization(robot.drive.getPoseEstimate(), robot.drive.getPoseVelocity());
            robot.update();
        }
    }
}
