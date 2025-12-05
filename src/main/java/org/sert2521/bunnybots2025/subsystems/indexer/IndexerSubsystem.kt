package org.sert2521.bunnybots2025.subsystems.indexer

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.IndexerConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object IndexerSubsystem  : SubsystemBase() {
    private val indexerMotor = SparkMax(ElectronicIDs.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val indexerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(
            IndexerConstants.indexerGearing
        )
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Indexer Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val indexerSMC = SparkWrapper(indexerMotor, DCMotor.getNEO(1), indexerMotorConfig)

    private val kickerMotor = SparkMax(ElectronicIDs.KICKER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val kickerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(
            IndexerConstants.kickerGearing
        )
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Kicker Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val kickerSMC = SparkWrapper(kickerMotor, DCMotor.getNEO(1), kickerMotorConfig)

    override fun periodic() {
        indexerSMC.updateTelemetry()
        kickerSMC.updateTelemetry()
    }

    override fun simulationPeriodic() {
        indexerSMC.simIterate()
        kickerSMC.simIterate()
    }

    private fun setIndexerMotor(dutyCycle: Double) {
            indexerSMC.dutyCycle = dutyCycle
    }

    private fun setKickerMotor(dutyCycle: Double) {
            kickerSMC.dutyCycle = dutyCycle
    }

    fun index(): Command {
        return runOnce {
            setIndexerMotor(0.3)
            setKickerMotor(-0.5)
        }
    }

    fun kick(): Command {
        return runOnce {
            setIndexerMotor(0.2)
            setKickerMotor(0.5)
        }
    }
}