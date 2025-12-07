package org.sert2521.bunnybots2025.subsystems.flywheels

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.FlywheelsConstants
import yams.math.ExponentialProfilePIDController
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object FlywheelsSubsystem : SubsystemBase() {
    private val motorTop = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorBottom =
        SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_BOTTOM_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigTop = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createConstraints(
                    Volts.of(12.0),
                    RotationsPerSecond.of(3000.0),
                    RotationsPerSecondPerSecond.of(1.0)
                )
//                ExponentialProfilePIDController.createFlywheelConstraints(
//                    Volts.of(12.0),
//                    DCMotor.getNEO(1),
//                    FlywheelsConstants.mass,
//                    FlywheelsConstants.drumRadius,
//                    FlywheelsConstants.gearing
//                )
            )
        )
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Top", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)

    private val motorConfigBottom = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createConstraints(
                    Volts.of(12.0),
                    RotationsPerSecond.of(3000.0),
                    RotationsPerSecondPerSecond.of(1.0)
                )
//                ExponentialProfilePIDController.createFlywheelConstraints(
//                    Volts.of(12.0),
//                    DCMotor.getNEO(1),
//                    FlywheelsConstants.mass,
//                    FlywheelsConstants.drumRadius,
//                    FlywheelsConstants.gearing
//                )
            )
        )
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Bottom", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)

    private val topSMC = SparkWrapper(motorTop, DCMotor.getNEO(1), motorConfigTop)
    private val bottomSMC = SparkWrapper(motorBottom, DCMotor.getNEO(1), motorConfigBottom)

    init {
        defaultCommand = stop()
    }

    override fun periodic() {
        topSMC.updateTelemetry()
        bottomSMC.updateTelemetry()
    }

    override fun simulationPeriodic() {
        topSMC.simIterate()
        bottomSMC.simIterate()
    }

    private fun setVelocitiesCommand(velocityTop: AngularVelocity, velocityBottom: AngularVelocity): Command {
        return runOnce {
            topSMC.setVelocity(velocityTop)
            bottomSMC.setVelocity(velocityBottom)
        }.andThen(
            Commands.waitUntil {
                MathUtil.isNear(velocityTop.`in`(RPM), topSMC.mechanismVelocity.`in`(RPM), 10.0)
                        && MathUtil.isNear(velocityBottom.`in`(RPM), bottomSMC.mechanismVelocity.`in`(RPM), 10.0)
            }
        )
    }

    fun rev(): Command {
        return setVelocitiesCommand(FlywheelsConstants.topShootTarget, FlywheelsConstants.bottomShootTarget)
    }

    fun stop(): Command {
        return runOnce {
            topSMC.dutyCycle = 0.0
            bottomSMC.dutyCycle = 0.0
        }
    }
}