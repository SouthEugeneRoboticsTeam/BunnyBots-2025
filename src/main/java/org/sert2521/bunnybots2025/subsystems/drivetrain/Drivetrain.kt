package org.sert2521.bunnybots2025.subsystems.drivetrain

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.PoseEstimator
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight
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
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.angleGearing
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.angleIDs
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.driveGearing
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.driveIDs
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.encoderIDs
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.moduleNames
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.moduleTranslations
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.moduleZeroRotations
import org.sert2521.bunnybots2025.subsystems.drivetrain.SwerveConstants.wheelRadius
import yams.mechanisms.config.SwerveModuleConfig
import yams.mechanisms.swerve.SwerveDrive
import yams.mechanisms.swerve.SwerveModule
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import java.util.*
import kotlin.math.PI
import kotlin.math.hypot

object Drivetrain : SubsystemBase() {
    private fun createModule(
        driveMotor: SparkMax, angleMotor: SparkMax, absoluteEncoder: CANcoder,
        moduleName: String, location: Translation2d, rotationZero: Angle
    ): SwerveModule {
        val driveConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withWheelDiameter(wheelRadius * 2.0)
            .withFeedforward(SimpleMotorFeedforward(DRIVE_S, DRIVE_V))
            .withClosedLoopController(DRIVE_P, DRIVE_I, DRIVE_D)
            .withGearing(driveGearing)
            .withStatorCurrentLimit(Amps.of(DRIVE_CURRENT_LIMIT))
            //.withTelemetry("Drive Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val angleConfig = SmartMotorControllerConfig(this)
            .withClosedLoopController(ANGLE_P, ANGLE_I, ANGLE_D)
            .withContinuousWrapping(Radians.of(-PI), Radians.of(PI))
            .withGearing(angleGearing)
            .withStatorCurrentLimit(Amps.of(ANGLE_CURRENT_LIMIT))
            //.withTelemetry("Angle Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val fullDriveMotorController = SparkWrapper(driveMotor, DCMotor.getNEO(1), driveConfig)
        val fullAngleMotorController = SparkWrapper(angleMotor, DCMotor.getNEO(1), angleConfig)

        val moduleConfig = SwerveModuleConfig(fullDriveMotorController, fullAngleMotorController)
            .withAbsoluteEncoderOffset(rotationZero)
            .withAbsoluteEncoder(absoluteEncoder.absolutePosition.asSupplier())
            .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.LOW)
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

    private val gyro = Pigeon2(0)
    private val gyroYaw = gyro.yaw.asSupplier()

    private val kinematics = SwerveDriveKinematics(*moduleTranslations)

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.kZero,
        Array(4) { modules[it].position },
        Pose2d.kZero
    )

    private val visionPoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.kZero,
        Array(4) { modules[it].position },
        Pose2d.kZero
    )

    private var simGyroAngle = Rotation.zero()

    private val limelight = Limelight("limelight")

    private val field = Field2d()

    private val gyroConnected = Debouncer(0.1)

    private val simTimer = Timer()

    init {
        SmartDashboard.putData(field)

        defaultCommand = JoystickDrive(true)
    }

    override fun periodic() {
        field.robotPose = poseEstimator.estimatedPosition

        val currentGyroConnected = gyroConnected.calculate(gyro.isConnected)
        if (!currentGyroConnected) {
            DogLog.logFault("Gyro Disconnected", Alert.AlertType.kError)
        }

        poseEstimator.update(Rotation2d(getGyroAngle()), getModulePositions())

        val moduleStates = getModuleStates()
        DogLog.log("Drivetrain/SwerveModuleStates/Measured", moduleStates)

        val chassisSpeeds = kinematics.toChassisSpeeds(moduleStates)
        DogLog.log("Drivetrain/ChassisSpeeds/Measured", chassisSpeeds)
        DogLog.log("Drivetrain/ChassisSpeeds/Measured Drive Speed", hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond))

        if (DriverStation.isDisabled()) {
            DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", Array(4) { SwerveModuleState() })
            DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", Array(4) { SwerveModuleState() })
        }
    }

    override fun simulationPeriodic() {
        if (!simTimer.isRunning) {
            simTimer.start()
        }
        // modules.forEach { it.simIterate() }
        simGyroAngle = simGyroAngle.plus(
            Radians.of(
                kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * simTimer.get()
            )
        )
        simTimer.reset()
    }

    /* Private Functions */
    /**
     * @return The optimized states
     */
    private fun setModuleStates(vararg states: SwerveModuleState): Array<SwerveModuleState> {
        modules.forEachIndexed { index, module ->
            module.setSwerveModuleState(states[index])
        }
        return Array(4) { modules[it].config.getOptimizedState(states[it]) }
    }

    private fun getGyroAngle(): Angle {
        return if (RobotBase.isReal()) {
            gyroYaw.get()
        } else {
            simGyroAngle
        }
    }

    private fun getModuleStates(): Array<SwerveModuleState> {
        return Array(4) { modules[it].state }
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        return Array(4) { modules[it].position }
    }

    /* Public Functions */
    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        DogLog.log("Drivetrain/ChassisSpeeds/Setpoints", speeds)
        DogLog.log("Drivetrain/ChassisSpeeds/Setpoint Drive Speed", hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))

        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.maxSpeed)

        DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", setpointStates)

        // Modules are set here
        val optimizedStates = setModuleStates(*setpointStates)
        DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", optimizedStates)
    }

    fun stopDrivePID() {
        for (module in modules) {
            module.config.driveMotor.setKp(0.0)
            module.config.driveMotor.setKd(0.0)
        }
    }

    fun startDrivePID() {
        for (module in modules) {
            module.config.driveMotor.setKp(DRIVE_P)
            module.config.driveMotor.setKd(DRIVE_D)
        }
    }

    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    fun setPose(pose: Pose2d) {
        poseEstimator.resetPose(pose)
    }

    fun getVisionPose(): Optional<Pose2d> {
        if (!limelight.latestResults.isPresent) {
            return Optional.empty()
        }
        if (!limelight.latestResults.get().valid) {
            return Optional.empty()
        }

        // Finding the biggest tag in view of the limelight, chooses that one for vision align
        var biggestTag = 0
        val latestResults = limelight.latestResults.get().targets_Fiducials
        for (i in latestResults.indices) {
            if (latestResults[i].ta > latestResults[biggestTag].ta) {
                biggestTag = i
            }
        }
        return Optional.of(latestResults[biggestTag].targetPose_RobotSpace2D)
    }

    fun runFFCharacterization(output: Double): Double {
        modules.forEach {
            it.config.azimuthMotor.setPosition(Rotations.zero())
            it.config.driveMotor.voltage = Volts.of(output)
        }

        var avgVelocity = 0.0
        modules.forEach {
            avgVelocity += (it.state.speedMetersPerSecond / wheelRadius.`in`(Meters)) / 4.0
        }
        return avgVelocity
    }
}