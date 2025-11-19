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
import org.sert2521.bunnybots2025.WristConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.math.ExponentialProfilePIDController
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.mechanisms.config.FlyWheelConfig

object FlywheelsSubsystem : SubsystemBase() {
    private val flywheelMotorTop = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)
    private val flywheelMotorBottom =
        SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_BOTTOM_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigTop = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createFlywheelConstraints(
                    Volts.of(12.0),
                    DCMotor.getNEO(1),
                    FlywheelsConstants.mass,
                    FlywheelsConstants.drumRadius,
                    FlywheelsConstants.gearing
                )
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
                ExponentialProfilePIDController.createFlywheelConstraints(
                    Volts.of(12.0),
                    DCMotor.getNEO(1),
                    FlywheelsConstants.mass,
                    FlywheelsConstants.drumRadius,
                    FlywheelsConstants.gearing
                )
            )
        )
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Bottom", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)

    private val SMCTop = SparkWrapper(flywheelMotorTop, DCMotor.getNEO(1), motorConfigTop)
    private val SMCBottom = SparkWrapper(flywheelMotorBottom, DCMotor.getNEO(1), motorConfigBottom)

    override fun periodic() {
        SMCTop.updateTelemetry()
        SMCBottom.updateTelemetry()
    }

    override fun simulationPeriodic() {
        SMCTop.simIterate()
        SMCBottom.simIterate()
    }

    private fun setVelocities(velocityTop: AngularVelocity, velocityBottom: AngularVelocity): Command {
        return runOnce {
            SMCTop.setVelocity(velocityTop)
            SMCBottom.setVelocity(velocityBottom)
        }.andThen(
            Commands.waitUntil {
                MathUtil.isNear(velocityTop.`in`(RPM), SMCTop.mechanismVelocity.`in`(RPM), 10.0)
                        && MathUtil.isNear(velocityBottom.`in`(RPM), SMCBottom.mechanismVelocity.`in`(RPM), 10.0)
            }
        )
    }

    fun rev(): Command {
        return setVelocities(FlywheelsConstants.topShootTarget, FlywheelsConstants.bottomShootTarget)
    }

    fun stop(): Command {
        return runOnce {
            SMCTop.dutyCycle = 0.0
            SMCBottom.dutyCycle = 0.0
        }
    }
}