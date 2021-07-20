package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGAdvancedHighGoalPipeline;
import com.arcrobotics.ftclib.vision.UGAngleHighGoalPipeline;
import com.arcrobotics.ftclib.vision.UGBasicHighGoalPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@TeleOp
public class SampleTele extends LinearOpMode {
    UGAngleHighGoalPipeline pipeline;
    private OpenCvWebcam camera;
    public FtcDashboard dash;
    public Telemetry tele;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);
        dash = FtcDashboard.getInstance();
        tele = dash.getTelemetry();

        pipeline = new UGAngleHighGoalPipeline(453.4608731755518);

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        dash.startCameraStream((CameraStreamSource) camera, 30);

        waitForStart();

//        robot.drive.setPoseEstimate(new Pose2d(0+(-17.75/2.0), 36, 0));

        while (opModeIsActive()) {
            double angle = pipeline.calculateYaw(UGAngleHighGoalPipeline.Target.BLUE);
            tele.addData("angle", angle);
            tele.update();
            robot.turret.setAngle(angle);
            robot.update();
        }
    }
}
