package org.sert2521.bunnybots2025.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.bunnybots2025.Input
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants

class VisionAlign(private val target: Pose2d) : Command() {
    private var poseError = Pose2d.kZero
    private val translationPID =
        PIDController(SwerveConstants.TRANSLATION_VISION_P, 0.0, SwerveConstants.TRANSLATION_VISION_D)
    private val rotationPID = PIDController(SwerveConstants.HEADING_VISION_P, 0.0, SwerveConstants.HEADING_VISION_D)
    private val alignDebouncer = Debouncer(0.3, Debouncer.DebounceType.kRising)
    private val rumbleBlip = Input.rumbleBlip()
    private var rumbled = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        Drivetrain.startDrivePID()
    }

    override fun execute() {
        poseError = Drivetrain.getVisionPoseToTarget(target)

        val translationOutput = translationPID.calculate(poseError.translation.norm)
        val rotationOutput = rotationPID.calculate(poseError.rotation.radians)
        val xOverNorm = poseError.x / poseError.translation.norm
        val yOverNorm = poseError.y / poseError.translation.norm

        if (translationOutput < SwerveConstants.TRANSLATION_OUTPUT_MIN) {
            Drivetrain.driveRobotRelative(ChassisSpeeds(0.0, 0.0, rotationOutput))
            if (alignDebouncer.calculate(true)){
                if (!rumbled){
                    if (!rumbleBlip.isScheduled){
                        rumbleBlip.schedule()
                        rumbled = true
                    }
                }
            }
        } else {
            Drivetrain.driveRobotRelative(
                ChassisSpeeds(
                    xOverNorm * translationOutput,
                    yOverNorm * translationOutput,
                    rotationOutput
                )
            )
            rumbled = false
        }
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {}
}
