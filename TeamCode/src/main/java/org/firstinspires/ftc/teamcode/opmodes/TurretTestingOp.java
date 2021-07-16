package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class TurretTestingOp extends LinearOpMode {
    DcMotor turret;
    public final double TICKS = 537.6;
    public final double RATIO = 72.0/24.0;
    public final double TICKS_TO_RAD = (TICKS*RATIO)/(2*Math.PI);
    public final double TICKS_TO_DEG = (TICKS*RATIO)/(360);
    public static double OFFSET = 0;

    double zeroPos;
    double currAngle;
    FtcDashboard dashboard;
    Telemetry t;

    public static double kP = 0.02;
    public static double kS = 0.05;
    public static double angle = 15;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();
        t = dashboard.getTelemetry();

        waitForStart();
        zeroPos = ticksToDeg(turret.getCurrentPosition());
        currAngle = 0;

        while(opModeIsActive()) {
            double power = kP*(angle-currAngle) + Math.copySign(kS, angle-currAngle);

            if(gamepad1.a) {
                turret.setPower(power);
            } else {
                turret.setPower(0);
            }

            currAngle = ticksToDeg(turret.getCurrentPosition() - zeroPos);
            if(currAngle > 180) {
                currAngle -= 360;
            }

            if(currAngle < 180) {
                currAngle += 360;
            }

            addData("Current Angle: ", currAngle);
            addData("Zero Pos: ", zeroPos);
            addData("Power: ", power);
            updateT();
        }
    }

    private void updateT() {
        telemetry.update();
        t.update();
    }

    public double ticksToRad(double ticks) {
        return TICKS_TO_RAD*ticks;
    }

    public double ticksToDeg(double ticks) {
        return TICKS_TO_DEG*ticks;
    }

    public void addData(String caption, Object value) {
        t.addData(caption, value);
        telemetry.addData(caption, value);
    }
}
