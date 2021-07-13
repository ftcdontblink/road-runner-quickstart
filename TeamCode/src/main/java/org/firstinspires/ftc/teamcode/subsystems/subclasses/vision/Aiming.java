package org.firstinspires.ftc.teamcode.subsystems.subclasses.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.vision.KtAimingPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
public class Aiming {
    private final OpenCvWebcam webcam;
    public static double GAIN = 1.0;
    public static double EXPOSURE = 0.12;
    private KtAimingPipeline pipeline = new KtAimingPipeline();

    public Aiming(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(() -> {
            FtcDashboard.getInstance().startCameraStream(webcam, 30);
            webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

            GainControl gainControl = webcam.getGainControl();
            gainControl.setGain((int) Range.scale(GAIN, 0, 1, gainControl.getMinGain(), gainControl.getMaxGain()));
            ExposureControl exposureControl = webcam.getExposureControl();
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long) Range.scale(EXPOSURE, 0, 1,
                    exposureControl.getMinExposure(TimeUnit.NANOSECONDS),
                    exposureControl.getMaxExposure(TimeUnit.NANOSECONDS)), TimeUnit.NANOSECONDS);
        });
    }

    public double getCenterGoal() {
        return pipeline.getLastOutput().getHighGoalAngle();
    }

    public void setColor(KtAimingPipeline.Color color) {
        pipeline.setColor(color);
    }

    public KtAimingPipeline.Color getColor() {
        return pipeline.getColor();
    }
}