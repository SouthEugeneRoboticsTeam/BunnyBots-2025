package org.sert2521.bunnybots2025

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.bunnybots2025.commands.VisionAlign
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import org.sert2521.bunnybots2025.subsystems.flywheels.FlywheelsSubsystem
import org.sert2521.bunnybots2025.subsystems.indexer.IndexerSubsystem
import org.sert2521.bunnybots2025.subsystems.intake.IntakeSubsystem
import org.sert2521.bunnybots2025.subsystems.wrist.WristSubsystem
import kotlin.jvm.optionals.getOrElse
import kotlin.math.pow

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    private val driverRev = driverController.leftBumper()
    private val driverShoot = driverController.rightBumper()
    private val driverAlign = driverController.a()

    private val gunnerIntake = gunnerController.button(1)
    private val gunnerReverse = gunnerController.button(2)
    private val gunnerStow = gunnerController.button(4)
    private val gunnerRunIntake = gunnerController.button(10)
    private val gunnerLowerWrist = gunnerController.button(5)
    private val gunnerResetWrist = gunnerController.button(99) // TODO: Set

    private val resetRotOffset = driverController.y()

    var rotationOffset = Rotation2d.kZero

    init {
        driverRev.whileTrue(FlywheelsSubsystem.rev())
        driverRev.onFalse(FlywheelsSubsystem.stop())

        driverShoot.whileTrue(IndexerSubsystem.kick())

        driverAlign.onTrue(VisionAlign(RobotConstants.targetVisionPose))


        gunnerIntake.onTrue(
            WristSubsystem.toIntake()
                .andThen(
                    IntakeSubsystem.runIntake()
                        .alongWith(
                            IndexerSubsystem.index()

                        )
                )
        )
        gunnerIntake.onFalse(
            WristSubsystem.toStow()
                .andThen(IntakeSubsystem.stop())
        )

        gunnerReverse.whileTrue(IntakeSubsystem.runReverse()
            .alongWith(
                IndexerSubsystem.reverse()
            )
        )

        gunnerStow.onTrue(
            WristSubsystem.toStow()
                .andThen(IntakeSubsystem.stop())
        )

        gunnerRunIntake.whileTrue(IntakeSubsystem.runIntake())

        gunnerLowerWrist.onTrue(WristSubsystem.toIntake())

        gunnerResetWrist.onTrue(WristSubsystem.resetWristCommand())


        resetRotOffset.onTrue(Commands.runOnce({
            if (DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red) {
                Drivetrain.setRotation(Rotation2d.k180deg)
                rotationOffset = Rotation2d.k180deg
            } else {
                Drivetrain.setRotation(Rotation2d.kZero)
                rotationOffset = Rotation2d.kZero
            }
        }))
    }


    /**
     * Gets all inputs for driving from xbox controller.
     *
     * @return Left joystick X, Left joystick Y, Right joystick X
     */
    fun getJoystickInputs(): Triple<Double, Double, Double> {
        return Triple(getLeftX(), getLeftY(), getRightX())
    }

    fun getLeftX(): Double {
        return -driverController.leftX.pow(3)
    }

    fun getLeftY(): Double {
        return -driverController.leftY.pow(3)
    }

    fun getRightX(): Double {
        return -driverController.rightX.pow(3)
    }

    fun getRotOffset(): Rotation2d {
        return rotationOffset
    }

    fun setRumble(amount: Double) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
    }

    fun rumbleBlip(): Command {
        return Commands.runOnce({ setRumble(0.8) })
            .andThen(Commands.waitSeconds(0.2))
            .andThen(Commands.runOnce({ setRumble(0.0) }))
    }
}