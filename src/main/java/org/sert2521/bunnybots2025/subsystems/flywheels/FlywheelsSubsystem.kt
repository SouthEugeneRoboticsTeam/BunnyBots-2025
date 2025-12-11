package org.sert2521.bunnybots2025.subsystems.flywheels

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.FlywheelsConstants
import org.sert2521.bunnybots2025.subsystems.indexer.IndexerSubsystem
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import java.util.function.Supplier

object FlywheelsSubsystem : SubsystemBase() {
    private val motorTop = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorBottom =
        SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_BOTTOM_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigTop = SmartMotorControllerConfig(this)
        .withClosedLoopController(FlywheelsConstants.P, 0.0, FlywheelsConstants.D)
        .withFeedforward(SimpleMotorFeedforward(FlywheelsConstants.S, FlywheelsConstants.V, FlywheelsConstants.A))
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Top", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)

    private val motorConfigBottom = SmartMotorControllerConfig(this)
        .withClosedLoopController(FlywheelsConstants.P, 0.0, FlywheelsConstants.D)
        .withFeedforward(SimpleMotorFeedforward(FlywheelsConstants.S, FlywheelsConstants.V, FlywheelsConstants.A))
        .withGearing(FlywheelsConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Flywheel Motor Bottom", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)

    private val topSMC = SparkWrapper(motorTop, DCMotor.getNEO(1), motorConfigTop)
    private val bottomSMC = SparkWrapper(motorBottom, DCMotor.getNEO(1), motorConfigBottom)

    private var topLastSetpoint = RPM.zero()
    private var bottomLastSetpoint = RPM.zero()

    init {
        defaultCommand = holdCommand(::topLastSetpoint, ::bottomLastSetpoint)

        topSMC.setupTelemetry(
            NetworkTableInstance.getDefault().getTable("Tuning")
                .getSubTable("Flywheels"),
            NetworkTableInstance.getDefault().getTable("Mechanisms")
                .getSubTable("Flywheels")
        )
        bottomSMC.setupTelemetry(
            NetworkTableInstance.getDefault().getTable("Tuning")
                .getSubTable("Flywheels"),
            NetworkTableInstance.getDefault().getTable("Mechanisms")
                .getSubTable("Flywheels")
        )
    }

    override fun periodic() {
        topSMC.updateTelemetry()
        bottomSMC.updateTelemetry()
        DogLog.log("Top Flywheel Speed", topSMC.mechanismVelocity.`in`(RPM))
        DogLog.log("Bottom Flywheel Speed", bottomSMC.mechanismVelocity.`in`(RPM))
    }

    override fun simulationPeriodic() {
        topSMC.simIterate()
        bottomSMC.simIterate()
    }

    private fun setVelocitiesCommand(velocityTop: AngularVelocity, velocityBottom: AngularVelocity): Command {
        return runOnce {
            topSMC.setVelocity(velocityTop)
            bottomSMC.setVelocity(velocityBottom)
            topLastSetpoint = velocityTop
            bottomLastSetpoint = velocityBottom
        }.until {
            MathUtil.isNear(velocityTop.`in`(RPM), topSMC.mechanismVelocity.`in`(RPM), 10.0)
                    && MathUtil.isNear(velocityBottom.`in`(RPM), bottomSMC.mechanismVelocity.`in`(RPM), 10.0)
        }
    }

    private fun holdCommand(velocityTop: Supplier<AngularVelocity>,
                            velocityBottom: Supplier<AngularVelocity>):Command{
        return runOnce{
            topSMC.setVelocity(velocityTop.get())
            bottomSMC.setVelocity(velocityBottom.get())
//            topLastSetpoint = velocityTop.get()
//            bottomLastSetpoint = velocityBottom.get()
        }.andThen(
            Commands.idle()
        )
    }

    fun rev(): Command {
        return setVelocitiesCommand(FlywheelsConstants.topShootTarget, FlywheelsConstants.bottomShootTarget)
    }

    fun stop(): Command {
        return setVelocitiesCommand(RPM.zero(), RPM.zero())
    }
}