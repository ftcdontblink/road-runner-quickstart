package org.firstinspires.ftc.teamcode.subsystems.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Constants.PI;

@Config
public class AutoAim {
    public static Vector2d TURRET_OFFSET = new Vector2d(5.0, 0.0);

    private Turret turret;
    private Vector2d targetPosition = new Vector2d();

    public AutoAim(Turret turret) {
        this.turret = turret;
    }

    public void setTargetPosition(Vector2d position) {
        targetPosition = position;
    }

    public void updateLocalization(Pose2d currentPose, Pose2d currentVelocity) {
        Vector2d fieldFrameTurretOffset = TURRET_OFFSET.rotated(currentPose.getHeading());
        Vector2d fieldFrameTurretPos = currentPose.vec().plus(fieldFrameTurretOffset);
        Vector2d fieldFrameRobotToGoal = targetPosition.minus(fieldFrameTurretPos);
        double targetTurretAngle = fieldFrameRobotToGoal.angle() - currentPose.getHeading();

        // R_{theta} * (v + w * R_{PI / 2} * r_{offset})
        Vector2d fieldFrameTurretLinearVelocity = currentVelocity.vec().plus(
                TURRET_OFFSET.rotated(PI / 2).times(currentVelocity.getHeading())).rotated(currentPose.getHeading());
        double targetTurretAngularVelocity = fieldFrameTurretLinearVelocity.rotated(PI / 2).dot(fieldFrameRobotToGoal)
                / (fieldFrameRobotToGoal.dot(fieldFrameRobotToGoal)) - currentVelocity.getHeading();

        turret.setTargetAngle(targetTurretAngle);
        turret.setTargetAngularVelocity(targetTurretAngularVelocity);

        turret.telemetry.addData("Target turret angle: ", targetTurretAngle);
        turret.telemetry.addData("Target turret angle velo: ", targetTurretAngularVelocity);
        turret.telemetry.update();
    }

    public void updateVision(double relTurretAngle, double relTurretDistance, Pose2d currentVelocity) {
        double targetTurretAngle = relTurretAngle + turret.getPosition();

        Vector2d robotFrameTurretLinearVelocity = currentVelocity.vec()
                .plus(TURRET_OFFSET.rotated(PI / 2).times(currentVelocity.getHeading()));
        Vector2d robotFrameTurretToGoal = Vector2d.polar(relTurretDistance, relTurretAngle).rotated(targetTurretAngle);
        double targetTurretAngularVelocity = robotFrameTurretLinearVelocity.rotated(PI / 2).dot(robotFrameTurretToGoal)
                / (robotFrameTurretToGoal.dot(robotFrameTurretToGoal)) - currentVelocity.getHeading();
    }
}