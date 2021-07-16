package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.subsystems.Turret.*
import java.lang.Math.toDegrees

@Config
@TeleOp
class TestOp: LinearOpMode() {
    enum class State {
        HIGH,
        POWERSHOTS,
        CUSTOM
    }

    override fun runOpMode() {
        val targetPose = Vector2d(72.0, 36.0)
        val turret = Turret(hardwareMap)
        var state = State.CUSTOM

        waitForStart()

        while(opModeIsActive()) {
            val currentPose = Vector2d(x, y)
            val angle = currentPose.minus(targetPose).angle()
            val angleInDegrees = toDegrees(angle)
            val angleNorm = Angle.normDelta(angle)
            var angleWrapped = toDegrees(angleNorm)

            telemetry.addData("angle", angle)
            telemetry.addData("angle deg", angleInDegrees)
            telemetry.addData("angle wrap", angleWrapped)
            telemetry.update()

//            when(state) {
//                State.CUSTOM ->
//                State.HIGH -> {
//
//                }
//                State.POWERSHOTS ->
//            }

            turret.update()

            state = when {
                gamepad1.a -> State.HIGH
                gamepad1.b -> State.POWERSHOTS
                gamepad1.x -> State.CUSTOM
                else -> State.CUSTOM
            }
        }
    }
}