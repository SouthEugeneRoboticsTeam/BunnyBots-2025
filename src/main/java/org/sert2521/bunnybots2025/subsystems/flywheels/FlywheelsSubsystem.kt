package org.sert2521.bunnybots2025.subsystems.flywheels

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.ExponentialProfile
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.FlywheelsConstants
import org.sert2521.bunnybots2025.RobotConstants
import org.sert2521.bunnybots2025.WristConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.math.ExponentialProfilePIDController
import yams.mechanisms.config.ArmConfig
import yams.mechanisms.config.MechanismPositionConfig
import yams.mechanisms.positional.Arm
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object FlywheelsSubsystem {
    private val flywheelMotorTop = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_TOP_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createFlywheelConstraints()
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

    private val fullMotor = SparkWrapper(flywheelMotorTop, DCMotor.getNEO(1),
        org.sert2521.offseason2025.subsystems.FlywheelSubsystem.motorConfig
    )

    private val flywheelMotorBottom = SparkMax(ElectronicIDs.FLYWHEEL_MOTOR_BOTTOM_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            ExponentialProfilePIDController(
                FlywheelsConstants.P,
                FlywheelsConstants.I,
                FlywheelsConstants.D,
                ExponentialProfilePIDController.createFlywheelConstraints()
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
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val fullMotor = SparkWrapper(flywheelMotorBottom, DCMotor.getNEO(1),
        org.sert2521.offseason2025.subsystems.FlywheelSubsystem.motorConfig
    )

    private val positionConfig = MechanismPositionConfig()
        .withMaxRobotHeight(RobotConstants.maxHeight)
        .withMaxRobotLength(RobotConstants.maxLength)


    private val fullConfig = ArmConfig(fullMotor)
        .withMOI(FlywheelsConstants.moi.`in`(KilogramSquareMeters))
        .withHardLimit(FlywheelsConstants.hardMin, FlywheelsConstants.hardMax)
        .withTelemetry("Flywheels", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLength(FlywheelsConstants.length)
        .withStartingPosition(Rotations.of(-0.27))

    private val arm = Arm(fullConfig)



    override fun periodic() {
        arm.updateTelemetry()
    }

    override fun simulationPeriodic() {
        arm.simIterate()
    }

    fun getMechanism2dForElevator(): MechanismObject2d {
        return arm.mechanismLigament
    }

    private fun setAngleInstantCommand(angle: Angle): Command {
        return arm.setAngle(angle)
    }

    fun setAngleCommand(angle: Angle): Command {
        return setAngleInstantCommand(angle).andThen(
            Commands.waitUntil {
                MathUtil.isNear(angle.`in`(Rotations), arm.angle.`in`(Rotations), 0.05)
            }
        )
    }

    fun sysId(): Command {
        return arm.sysId(Volts.of(3.0), Volts.of(3.0).per(Second), Seconds.of(30.0))
    }
}

}