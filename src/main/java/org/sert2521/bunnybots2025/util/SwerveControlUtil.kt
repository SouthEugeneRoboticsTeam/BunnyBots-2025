package org.sert2521.bunnybots2025.util

import dev.doglog.DogLog
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.sert2521.bunnybots2025.Input
import java.util.function.Supplier
import kotlin.math.*

object SwerveControlUtil {
    private var squareness = 3.0

    fun squarenessCommand(xInputSupplier: Supplier<Double>, yInputSupplier:Supplier<Double>): Command {
        val applySquareness = Trigger(DogLog.tunable("ApplySquareness", false))
        var x: Double
        var y: Double
        var corrInputs: Pair<Double, Double>

        applySquareness.onChange(Commands.runOnce({
            squareness = squarenessCharacterize(xInputSupplier.get(), yInputSupplier.get())
        }))

        return Commands.run({
            x = xInputSupplier.get()
            y = yInputSupplier.get()
            corrInputs = reverseSquareness(x, y)

            DogLog.log("Uncorrected Controller Input", ChassisSpeeds(x, y, 0.0))
            DogLog.log("Corrected Controller Input", ChassisSpeeds(corrInputs.first, corrInputs.second, 0.0))

            DogLog.log("Calculated Squareness", squarenessCharacterize(x, y))
            DogLog.log("Angle from 45", squarenessAngleToDiagonal(x, y))
            DogLog.log("Current Squareness", squareness)
        })
    }


    /**
     * Finds the actual magnitude of controller input based on the current input and squareness value of the input
     *
     * @return The unbiased controller input.
     */
    fun reverseSquareness(x:Double, y:Double, squarenessValue: Double): Pair<Double, Double> {
        val thetaUnconstrained = atan(y/x)

        // It's ok to constrain it like this because the squareness graph is symmetric in 8 ways
        // It will turn out to have the same magnitude bias
        // This is just because if not constrained to the first quadrant it creates insanely big numbers or divide by zero
        val theta = MathUtil.inputModulus(thetaUnconstrained, 0.0, PI/2)

        // This models the output curve of the xbox controller with the specified squareness
        val magnitudeBias = sqrt((tan(theta).pow(2) + 1) /
                    ((1 + tan(theta).pow(squarenessValue)).pow(2 / squarenessValue)))

        return Pair(x / magnitudeBias, y / magnitudeBias)
    }

    fun reverseSquareness(x:Double, y:Double): Pair<Double, Double>{
        return reverseSquareness(x, y, squareness)
    }

    /**
     * Finds the squareness value based on a maximum diagonal input.
     * The units here are NOT meters per second.
     * Here the numbers for vx and vy in [joystickInput] should be taken directly from the controller.
     *
     * @return A pair of the calculated squareness and the difference of the controller input from 45 degrees (closer to 0 is better).
     */
    fun squarenessCharacterize(x:Double, y:Double):Double{
        val leg = hypot(x, y)/sqrt(2.0)

        return log(0.5, leg)
    }

    fun squarenessAngleToDiagonal(x:Double, y:Double):Double{
        val thetaUnconstrained = atan(y/x)
        val theta = MathUtil.inputModulus(thetaUnconstrained, 0.0, PI/2)

        return abs(theta-PI/4)
    }

    fun speedLimit(targetFieldSpeeds: ChassisSpeeds, speedLimit: LinearVelocity): ChassisSpeeds {
        val targetSpeed = MetersPerSecond.of(
            hypot(targetFieldSpeeds.vxMetersPerSecond, targetFieldSpeeds.vyMetersPerSecond)
        )
        val magnitudeToApply = if (targetSpeed > speedLimit) {
            speedLimit.div(targetSpeed).magnitude()
        } else {
            1.0
        }



        return ChassisSpeeds(targetFieldSpeeds.vxMetersPerSecond * magnitudeToApply,
            targetFieldSpeeds.vyMetersPerSecond * magnitudeToApply,
            targetFieldSpeeds.omegaRadiansPerSecond)
    }

    fun accelLimit(
        targetFieldSpeeds: ChassisSpeeds,
        lastFieldSpeeds: ChassisSpeeds,
        accelLimit: LinearAcceleration
    ): ChassisSpeeds {

        // divide by 0.02 (loop time) to get real acceleration limits in /s^2
        val changeInSpeeds = targetFieldSpeeds.minus(lastFieldSpeeds)
        val totalAcceleration = changeInSpeeds.div(0.02)
        val totalLinearAcceleration = MetersPerSecondPerSecond.of(
            hypot(totalAcceleration.vxMetersPerSecond, totalAcceleration.vyMetersPerSecond)
        )

        // Yes I know that totalAcceleration should have units of meters per second squared.
        // There's no class for translational acceleration in components so whatever
        val magnitudeToApply = if (totalLinearAcceleration > accelLimit) {
            accelLimit.div(totalLinearAcceleration).magnitude()
        } else {
            1.0
        }

        val x = lastFieldSpeeds.plus(changeInSpeeds.times(magnitudeToApply))

        return ChassisSpeeds(x.vxMetersPerSecond, x.vyMetersPerSecond, targetFieldSpeeds.omegaRadiansPerSecond)
    }
}