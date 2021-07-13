package org.firstinspires.ftc.teamcode.subsystems.subclasses.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class AimingTest extends LinearOpMode {
    Aiming aiming;

    @Override
    public void runOpMode() throws InterruptedException {
        aiming = new Aiming(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Relative: ", aiming.getCenterGoal());
            telemetry.addData("Relative (Deg): ", Math.toDegrees(aiming.getCenterGoal()));
            telemetry.update();

            FtcDashboard.getInstance().getTelemetry().addData("Relative: ", aiming.getCenterGoal());
            FtcDashboard.getInstance().getTelemetry().addData("Relative (Deg): ", Math.toDegrees(aiming.getCenterGoal()));
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
