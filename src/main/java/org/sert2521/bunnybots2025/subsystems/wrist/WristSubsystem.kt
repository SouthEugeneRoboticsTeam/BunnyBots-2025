package org.sert2521.bunnybots2025.subsystems.wrist

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.bunnybots2025.ElectronicIDs
import org.sert2521.bunnybots2025.RobotConstants
import org.sert2521.bunnybots2025.WristConstants
import yams.mechanisms.config.ArmConfig
import yams.mechanisms.config.MechanismPositionConfig
import yams.mechanisms.positional.Arm
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object WristSubsystem : SubsystemBase() {
    private val wristMotor = SparkMax(ElectronicIDs.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            WristConstants.P,
            WristConstants.I,
            WristConstants.D
        )
        .withGearing(WristConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Wrist Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val smc = SparkWrapper(wristMotor, DCMotor.getNEO(1), motorConfig)

    private val positionConfig = MechanismPositionConfig()
        .withMaxRobotHeight(RobotConstants.maxHeight)
        .withMaxRobotLength(RobotConstants.maxLength)

    private val armConfig = ArmConfig(smc)
        .withMOI(WristConstants.moi.`in`(KilogramSquareMeters))
        .withHardLimit(WristConstants.hardMin, WristConstants.hardMax)
        .withTelemetry("Wrist", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLength(WristConstants.length)
        .withStartingPosition(Rotations.of(-0.27))

    private val arm = Arm(armConfig)

    private val stallDebouncer = Debouncer(0.2)

    override fun periodic() {
        arm.updateTelemetry()
    }

    override fun simulationPeriodic() {
        arm.simIterate()
    }

    private fun setAngleInstantCommand(angle: Angle): Command {
        return arm.setAngle(angle)
    }

    private fun setAngleCommand(angle: Angle): Command {
        return setAngleInstantCommand(angle).andThen(
            Commands.waitUntil {
                MathUtil.isNear(angle.`in`(Rotations), arm.angle.`in`(Rotations), 0.05)
            }
        )
    }

    fun toStow(): Command {
        return setAngleCommand(WristConstants.stowPosition)
    }

    fun toIntake(): Command {
        return setAngleCommand(WristConstants.intakePosition)
    }

    fun setPower(dutyCycle: Double) {
        arm.set(dutyCycle)
    }

    fun resetWristCommand(): Command {
        return runOnce {
            setPower(-0.3)
        }.andThen(Commands.waitUntil {
            stallDebouncer.calculate(arm.motor.statorCurrent > Amps.of(30.0))
        }
        ).andThen(
            runOnce {
                arm.motor.setEncoderPosition(Rotations.zero())
            }
        )
    }

    fun sysId(): Command {
        return arm.sysId(Volts.of(12.0), Volts.of(0.2).per(Second), Seconds.of(30.0))
    }
}