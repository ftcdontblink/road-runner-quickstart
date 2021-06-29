package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.subclasses.PoseStorage;

public class MecanumDrive implements Subsystem {
    Robot robot;
    SampleMecanumDrive drive;

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    public enum Side {
        RED,
        BLUE
    }

    public Side side = Side.RED;

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public Vector2d targetPosition;

    public MecanumDrive(Robot robot) {
        this.robot = robot;
        this.drive = robot.drive;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
    }

    public void update() {
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        switch (currentMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                if (robot.gamepad1.a) {
                    currentMode = Mode.ALIGN_TO_POINT;
                }

                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driveDirection = new Pose2d(
                        -robot.gamepad1.left_stick_y,
                        -robot.gamepad1.left_stick_x,
                        -robot.gamepad1.right_stick_x
                );
                break;
            case ALIGN_TO_POINT:
                // Switch back into normal driver control mode if `b` is pressed
                if (robot.gamepad1.b) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        -robot.gamepad1.left_stick_y,
                        -robot.gamepad1.left_stick_x
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );


                break;
        }


        drive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        drive.getLocalizer().update();

    }
}

