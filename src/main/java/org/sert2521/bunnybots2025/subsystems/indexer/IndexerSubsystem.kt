package org.sert2521.bunnybots2025.subsystems.indexer

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.IndexerConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object IndexerSubsystem  : SubsystemBase() {
    private val indexerMotor = SparkMax(ElectronicIDs.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigIndexer = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            IndexerConstants.P,
            IndexerConstants.I,
            IndexerConstants.D
        )
        .withGearing(
            MechanismGearing(
                GearBox.fromReductionStages(
                    3.0,
                    4.0,
                    54.0 / 22.0
                )
            )
        )
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Indexer Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val fullMotorIndexer = SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfigIndexer)

    private val kickerMotor = SparkMax(ElectronicIDs.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigKicker = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            IndexerConstants.P,
            IndexerConstants.I,
            IndexerConstants.D
        )
        .withGearing(
            MechanismGearing(
                GearBox.fromReductionStages(
                    3.0,
                    4.0,
                    54.0 / 22.0
                )
            )
        )
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Kicker Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val fullMotorKicker = SparkWrapper(indexerMotor, DCMotor.getNEO(1), motorConfigKicker)

    override fun periodic() {
        fullMotorIndexer.updateTelemetry()
        fullMotorKicker.updateTelemetry()
    }

    override fun simulationPeriodic() {

    }

    fun setVelocities(angle: AngularVelocity): Command {
        return runOnce {
            fullMotorIndexer.setVelocity(angle)
            fullMotorKicker.setVelocity(angle)
        }
    }
}