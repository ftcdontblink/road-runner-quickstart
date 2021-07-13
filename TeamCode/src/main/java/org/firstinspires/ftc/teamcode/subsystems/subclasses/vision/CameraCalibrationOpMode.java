package org.firstinspires.ftc.teamcode.subsystems.subclasses.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CameraCalibrationOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CalibrationPipeline calibrationPipeline = new CalibrationPipeline(telemetry);
        camera.setPipeline(calibrationPipeline);

        waitForStart();

        camera.openCameraDeviceAsync(() -> {
            FtcDashboard.getInstance().startCameraStream(camera, 30);
            camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        });

        while (!isStopRequested()) {
            idle();
        }
    }
}