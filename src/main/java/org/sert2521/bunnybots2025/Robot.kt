package org.sert2521.bunnybots2025

import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.UsbCamera
import edu.wpi.first.hal.DriverStationJNI
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.bunnybots2025.subsystems.drivetrain.Drivetrain
import org.sert2521.bunnybots2025.subsystems.flywheels.FlywheelsSubsystem
import org.sert2521.bunnybots2025.subsystems.wrist.WristSubsystem
import org.sert2521.bunnybots2025.util.SwerveControlUtil
import kotlin.jvm.optionals.getOrElse

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {
    private var autonomousCommand = Commands.none()

    override fun robotInit() {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
        // Access the RobotContainer object so that it is initialized. This will perform all our
        // button bindings, and put our autonomous chooser on the dashboard.
        DogLog.setOptions(
            DogLogOptions()
                .withCaptureDs(true)
                .withCaptureConsole(true)
                .withNtPublish { !DriverStation.isFMSAttached() }
        )
        Drivetrain
        Input
        WristSubsystem
        FlywheelsSubsystem

        CameraServer.startAutomaticCapture()
    }


    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

    }

    override fun autonomousInit() {
        Drivetrain.startDrivePID()

        // autonomousCommand = Autos.getAutonomousCommand()
        // autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        autonomousCommand.cancel()

        if (!isReal()) {
            SwerveControlUtil.squarenessCommand(Input::getLeftX, Input::getLeftY).schedule()
        }

        // This is so it starts at the right orientation, no redoing needed
        if (DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red
            && Input.rotationOffset === Rotation2d.kZero) {
            Input.rotationOffset = Rotation2d.k180deg
        }

        WristSubsystem.resetWristCommand().schedule()
    }


    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {

    }

    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {

    }

    override fun simulationInit() {
        SwerveControlUtil.squarenessCommand(Input::getLeftX, Input::getLeftY)
    }

    override fun simulationPeriodic() {

    }
}