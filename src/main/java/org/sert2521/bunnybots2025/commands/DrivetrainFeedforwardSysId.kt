package org.sert2521.bunnybots2025.commands

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.wheelRadius
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.*


object DrivetrainFeedforwardSysId {
    private val velocitySamples: MutableList<Double> = LinkedList()
    private val voltageSamples: MutableList<Double> = LinkedList()
    private val timer = Timer()

    fun get(): Command {
        return Commands.sequence( // Reset data
            Commands.runOnce(
                {
                    Drivetrain.stopDrivePID()
                    velocitySamples.clear()
                    voltageSamples.clear()
                }
            ),  // Allow modules to orient

            Commands.run(
                { Drivetrain.runFFCharacterization(0.0) },
                Drivetrain
            )
                .withTimeout(1.0),

            // Start timer
            Commands.runOnce(timer::restart),  // Accelerate and gather data

            Commands.run(
                {
                    val voltage: Double = timer.get() * SwerveConstants.SYS_ID_FF_RAMP_RATE
                    velocitySamples.add(Drivetrain.runFFCharacterization(voltage))
                    voltageSamples.add(voltage)
                },
                Drivetrain
            ) // When cancelled, calculate and print results

            .finallyDo {
                _ ->
                val n = velocitySamples.size
                var sumX = 0.0
                var sumY = 0.0
                var sumXY = 0.0
                var sumX2 = 0.0
                for (i in 0..<n) {
                    sumX += velocitySamples[i]
                    sumY += voltageSamples[i]
                    sumXY += velocitySamples[i] * voltageSamples[i]
                    sumX2 += velocitySamples[i] * velocitySamples[i]
                }
                val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
                val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

                val formatter: NumberFormat = DecimalFormat("#0.00000")
                println("********** Drive FF Characterization Results **********")
                println("\tkS: " + formatter.format(kS))
                println("\tkV: " + formatter.format(kV))
                println("\tSample Size" + formatter.format(velocitySamples.size))
                println("\tMax speed attained: "
                        + formatter.format(velocitySamples.last()*wheelRadius.`in`(Meters)) + "MPS at "
                        + formatter.format(voltageSamples.last()) + "V")
            }
        )
    }
}