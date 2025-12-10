package org.sert2521.bunnybots2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.PathPlannerLogging
import dev.doglog.DogLog
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.bunnybots2025.subsystems.flywheels.FlywheelsSubsystem
import org.sert2521.bunnybots2025.subsystems.intake.IntakeSubsystem
import org.sert2521.bunnybots2025.subsystems.wrist.WristSubsystem
import kotlin.jvm.optionals.getOrElse

object Autos {

    private val autoChooser: SendableChooser<Command>

    private val namedCommands = mapOf(
        "Init Wrist" to WristSubsystem.resetWristCommand(),

        "Flywheel Rev" to FlywheelsSubsystem.rev(),

        "Wrist to Intake" to WristSubsystem.toIntake(),
        "Wrist to Stow" to WristSubsystem.toStow(),

        "Run Intake" to IntakeSubsystem.runIntake(),
        "Reverse Intake" to IntakeSubsystem.runReverse(),

        "Intake Routine" to WristSubsystem.toIntake()
            .andThen(IntakeSubsystem.runIntake()),

        "Stow Routine" to WristSubsystem.toStow()
            .andThen(IntakeSubsystem.stop()),

        "Run Indexer" to Commands.none(),
        "Reverse Indexer" to Commands.none(),

        "Shoot" to Commands.none()
    )

    init {
        NamedCommands.registerCommands(namedCommands)

        AutoBuilder.configure(
            Drivetrain::getPose,
            Drivetrain::setPose,
            Drivetrain::getChassisSpeeds,
            Drivetrain::driveRobotRelative,
            PPHolonomicDriveController(
                PIDConstants(
                    SwerveConstants.AUTO_TRANSLATION_P,
                    SwerveConstants.AUTO_TRANSLATION_I,
                    SwerveConstants.AUTO_TRANSLATION_D
                ),
                PIDConstants(
                    SwerveConstants.AUTO_HEADING_P,
                    SwerveConstants.AUTO_HEADING_I,
                    SwerveConstants.AUTO_HEADING_D
                )
            ),
            RobotConfig(
                RobotConstants.mass,
                RobotConstants.moi,
                ModuleConfig(
                    SwerveConstants.wheelRadius,
                    SwerveConstants.maxSpeed,
                    SwerveConstants.WHEEL_COF,
                    DCMotor.getNEO(1).withReduction(SwerveConstants.driveGearing.mechanismToRotorRatio),
                    SwerveConstants.driveCurrentLimit,
                    1
                ),
                *SwerveConstants.moduleTranslations
            ),
            { DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red },
            Drivetrain
        )

        PathPlannerLogging.setLogTargetPoseCallback {
            DogLog.log("Odometry/Trajectory Setpoint", it)
        }

        autoChooser = AutoBuilder.buildAutoChooser()
        autoChooser.setDefaultOption("None", Commands.none())
    }

    fun getAutonomousCommand(): Command {
        return autoChooser.selected
    }
}