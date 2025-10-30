package org.sert2521.bunnybots2025.subsystems.drivetrain

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.studica.frc.AHRS
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight
import limelight.networktables.target.AprilTagFiducial
import org.sert2521.bunnybots2025.commands.JoystickDrive
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.ANGLE_CURRENT_LIMIT
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.ANGLE_D
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.ANGLE_I
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.ANGLE_P
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.DRIVE_CURRENT_LIMIT
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.DRIVE_D
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.DRIVE_I
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.DRIVE_P
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.DRIVE_S
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.DRIVE_V
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.HEADING_D
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.HEADING_I
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.HEADING_P
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.TRANSLATION_D
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.TRANSLATION_I
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.TRANSLATION_P
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.angleGearing
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.angleIDs
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.driveGearing
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.driveIDs
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.encoderIDs
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.moduleNames
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.moduleTranslations
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.moduleZeroRotations
import yams.mechanisms.config.SwerveDriveConfig
import yams.mechanisms.config.SwerveModuleConfig
import yams.mechanisms.swerve.SwerveDrive
import yams.mechanisms.swerve.SwerveModule
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import java.util.*
import javax.swing.text.html.Option
import kotlin.math.PI

object Drivetrain : SubsystemBase() {
    private fun createModule(
        driveMotor: SparkMax, angleMotor: SparkMax, absoluteEncoder: CANcoder,
        moduleName: String, location: Translation2d, rotationZero: Angle
    ): SwerveModule {
        val driveConfig = SmartMotorControllerConfig(this)
            .withWheelDiameter(Inches.of(4.0))
            .withFeedforward(SimpleMotorFeedforward(DRIVE_S, DRIVE_V))
            .withClosedLoopController(DRIVE_P, DRIVE_I, DRIVE_D)
            .withGearing(driveGearing)
            .withStatorCurrentLimit(Amps.of(DRIVE_CURRENT_LIMIT))
            .withTelemetry("Drive Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val angleConfig = SmartMotorControllerConfig(this)
            .withClosedLoopController(ANGLE_P, ANGLE_I, ANGLE_D)
            .withContinuousWrapping(Radians.of(-PI), Radians.of(PI))
            .withGearing(angleGearing)
            .withStatorCurrentLimit(Amps.of(ANGLE_CURRENT_LIMIT))
            .withTelemetry("Angle Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val fullDriveMotorController = SparkWrapper(driveMotor, DCMotor.getNEO(1), driveConfig)
        val fullAngleMotorController = SparkWrapper(angleMotor, DCMotor.getNEO(1), angleConfig)

        val moduleConfig = SwerveModuleConfig(fullDriveMotorController, fullAngleMotorController)
            .withAbsoluteEncoderOffset(rotationZero)
            .withAbsoluteEncoder(absoluteEncoder.absolutePosition.asSupplier())
            .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withLocation(location)
            .withOptimization(true)

        return SwerveModule(moduleConfig)
    }

    private val modules = Array(4) { index ->
        createModule(
            SparkMax(driveIDs[index], SparkLowLevel.MotorType.kBrushless),
            SparkMax(angleIDs[index], SparkLowLevel.MotorType.kBrushless),
            CANcoder(encoderIDs[index]),
            moduleNames[index],
            moduleTranslations[index],
            moduleZeroRotations[index]
        )
    }

    private val gyro = AHRS(AHRS.NavXComType.kUSB1)

    private val swerveConfig = SwerveDriveConfig(this, *modules)
        .withGyro { Degrees.of(gyro.angle) }
        .withStartingPose(Pose2d.kZero)
        .withTranslationController(PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D))
        .withRotationController(PIDController(HEADING_P, HEADING_I, HEADING_D))
        .withMaximumModuleSpeed(SwerveConstants.maxSpeed)

    private val swerveDrive = SwerveDrive(swerveConfig)

    private val limelight = Limelight("limelight")

    private val field = Field2d()

    init {
        SmartDashboard.putData(field)

        defaultCommand = JoystickDrive(true)
    }

    override fun periodic() {
        swerveDrive.updateTelemetry()
        field.robotPose = swerveDrive.pose
    }

    override fun simulationPeriodic() {
        swerveDrive.simIterate()
    }

    fun getPose():Pose2d{
        return swerveDrive.pose
    }

    fun setPose(pose:Pose2d){
        swerveDrive.resetOdometry(pose)
    }

    fun getVisionPose(): Optional<Pose2d> {
        if (!limelight.latestResults.isPresent){
            return Optional.empty()
        }
        if (!limelight.latestResults.get().valid){
            return Optional.empty()
        }

        // Finding the biggest tag in view of the limelight, chooses that one for vision align
        var biggestTag = 0
        val latestResults = limelight.latestResults.get().targets_Fiducials
        for (i in latestResults.indices){
            if (latestResults[i].ta > latestResults[biggestTag].ta){
                biggestTag = i
            }
        }
        return Optional.of(latestResults[biggestTag].targetPose_RobotSpace2D)
    }

    fun driveRobotRelative(speeds:ChassisSpeeds){
        swerveDrive.setRobotRelativeChassisSpeeds(speeds)
    }
}