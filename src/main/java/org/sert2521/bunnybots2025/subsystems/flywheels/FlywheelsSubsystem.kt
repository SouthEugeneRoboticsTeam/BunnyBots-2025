package org.sert2521.bunnybots2025.subsystems.flywheels

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.FlywheelsConstants
import org.sert2521.bunnybots2025.FlywheelsConstants.bottomMotorVelocity
import org.sert2521.bunnybots2025.FlywheelsConstants.topMotorVelocity
import org.sert2521.bunnybots2025.RobotConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.math.ExponentialProfilePIDController
import yams.mechanisms.config.ArmConfig
import yams.mechanisms.config.MechanismPositionConfig
import yams.mechanisms.config.ShooterConfig
import yams.mechanisms.positional.Arm
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object FlywheelsSubsystem : SubsystemBase() {
    private val flywheelMotorTop = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigTop = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createFlywheelConstraints(
                    Volts.of(12.0),
                    DCMotor.getNEO(1),
                    Kilograms.of(0.0),
                    Meters.of(0.0),
                    MechanismGearing(
                        GearBox.fromReductionStages(
                            1.0
                        )
                    )
                )
            )
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
        .withTelemetry("Flywheel Motor Top", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val fullMotorTop = SparkWrapper(flywheelMotorTop, DCMotor.getNEO(1), motorConfigTop)

    private val flywheelMotorBottom =
        SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_BOTTOM_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigBottom = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createFlywheelConstraints(
                    Volts.of(12.0),
                    DCMotor.getNEO(1),
                    Kilograms.of(0.0),
                    Meters.of(0.0),
                    MechanismGearing(
                        GearBox.fromReductionStages(
                            1.0
                        )
                    )
                )
            )
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
        .withTelemetry("Flywheel Motor Bottom", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(true)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val fullMotorBottom = SparkWrapper(flywheelMotorBottom, DCMotor.getNEO(1), motorConfigBottom)


    override fun periodic() {
        fullMotorBottom.updateTelemetry()
        fullMotorTop.updateTelemetry()
    }

    override fun simulationPeriodic() {

    }

    fun setVelocities(angle: AngularVelocity): Command {
        return runOnce {
            fullMotorBottom.setVelocity(angle)
            fullMotorTop.setVelocity(angle)
        }
    }
}