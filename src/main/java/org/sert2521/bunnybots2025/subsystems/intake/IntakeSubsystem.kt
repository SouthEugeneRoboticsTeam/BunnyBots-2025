package org.sert2521.bunnybots2025.subsystems.intake

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.IntakeConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object IntakeSubsystem : SubsystemBase() {
    private val intakeMotor = SparkMax(ElectronicIDs.INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val intakeMotorConfig = SmartMotorControllerConfig(this)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withTelemetry("Intake Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
        .withGearing(IntakeConstants.gearing)

    private val intakeSMC = SparkWrapper(intakeMotor, DCMotor.getNEO(1), intakeMotorConfig)

    init {
        defaultCommand = stop()
    }

    override fun periodic() {
        intakeSMC.updateTelemetry()
    }

    private fun setMotor(dutyCycle: Double) {
        intakeSMC.dutyCycle = dutyCycle
    }

    fun runIntake(): Command {
        return runOnce {
            setMotor(IntakeConstants.INTAKE_SPEED)
        }.andThen(
            Commands.idle()
        )
    }

    fun runReverse(): Command {
        return runOnce {
            setMotor(IntakeConstants.REVERSE_SPEED)
        }.andThen(
            Commands.idle()
        )
    }

    fun stop(): Command {
        return runOnce {
            setMotor(0.0)
        }.andThen(
            Commands.idle()
        )
    }
}