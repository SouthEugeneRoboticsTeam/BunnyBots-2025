package org.sert2521.bunnybots2025.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.bunnybots2025.Input
import org.sert2521.bunnybots2025.subsystems.drivetrain.DriveConfig.DRIVE_SPEED
import org.sert2521.bunnybots2025.subsystems.drivetrain.DriveConfig.ROT_SPEED
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import org.sert2521.bunnybots2025.util.SwerveControlUtil

class JoystickDrive(private val fieldOriented: Boolean = true) : Command() {

    var correctedInput = Translation2d()
    var inputChassisSpeeds = ChassisSpeeds()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        Drivetrain.stopDrivePID()
    }

    override fun execute() {
        correctedInput = SwerveControlUtil.reverseSquareness(Input.getLeftX(), Input.getLeftY())
        inputChassisSpeeds = ChassisSpeeds(
            correctedInput.y * DRIVE_SPEED,
            correctedInput.x * DRIVE_SPEED,
            Input.getRightX() * ROT_SPEED
        )

        // Accel limiting should go here if needed


        if (fieldOriented) {
            Drivetrain.driveRobotRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    inputChassisSpeeds, Drivetrain.getPose().rotation.minus(Input.getRotOffset())
                )
            )
        } else {
            Drivetrain.driveRobotRelative(
                inputChassisSpeeds
            )
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
