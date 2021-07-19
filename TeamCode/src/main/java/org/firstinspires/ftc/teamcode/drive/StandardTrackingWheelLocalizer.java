package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 35.0/2.0/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double X_MULTIPLIER = 1.0;
    public static double Y_MULTIPLIER = 1.0;

    public static double LATERAL_DISTANCE = 14.0125; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -2.85; // in; offset of the lateral wheel
    private ElapsedTime timer;
    private BNO055IMU imu;
    private double baseExtHeading;

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private double IMU_INTERVAL = 0.5;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lbm"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rfm"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rbm"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)


        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imuaa");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        timer = new ElapsedTime();

        baseExtHeading = getRawExternalHeading();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }

    private double getRawExternalHeading() {
        return Angle.norm(imu.getAngularOrientation().firstAngle);
    }

    private double getExternalHeading() {
        return Angle.norm(getRawExternalHeading() - baseExtHeading);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose) {
        baseExtHeading = Angle.norm(getRawExternalHeading() - pose.getHeading());

        super.setPoseEstimate(pose);
    }

    @Override
    public void update() {
        if (timer.seconds() > IMU_INTERVAL) {
            super.update();
            double extHeading = getExternalHeading();
            Pose2d pose = new Pose2d(getPoseEstimate().vec(), extHeading);
            super.setPoseEstimate(pose);
            timer.reset();
        }

        super.update();
    }
}
