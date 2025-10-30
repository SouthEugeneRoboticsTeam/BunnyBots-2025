package org.sert2521.bunnybots2025

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import yams.motorcontrollers.SmartMotorController
import yams.motorcontrollers.SmartMotorControllerConfig

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    private val exampleDriverButton = driverController.x()
    private val exampleGunnerButton = gunnerController.button(2)

    private val resetRotOffset = driverController.y()



    private var rotationOffset = Rotation2d(0.0)

    init {
        exampleDriverButton.whileTrue(Commands.none())
        exampleGunnerButton.onTrue(Commands.none())

        resetRotOffset.onTrue(Commands.runOnce({ rotationOffset = Rotation2d.kZero /*Drivetrain.getPose().rotation*/ }))

    }


    /**
     * Gets all inputs for driving from xbox controller.
     *
     * @return Left joystick X, Left joystick Y, Right joystick X
     */
    fun getJoystickInputs():Triple<Double, Double, Double>{
        return Triple(getLeftX(), getLeftY(), getRightX())
    }

    fun getLeftX():Double{
        return -driverController.leftX
    }

    fun getLeftY():Double{
        return -driverController.leftY
    }

    fun getRightX():Double{
        return -driverController.rightX
    }

    fun getRotOffset():Rotation2d{
        return rotationOffset
    }
}