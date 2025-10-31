package org.sert2521.bunnybots2025.subsystems.drivetrain

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.LinearVelocity
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object SwerveConstants {
    val driveGearing = MechanismGearing(GearBox.fromReductionStages(6.75))
    val angleGearing = MechanismGearing(GearBox.fromReductionStages(150.0/7.0))

    val moduleNames = arrayOf("Front Left", "Front Right", "Back Left", "Back Right")
    val moduleTranslations = arrayOf(
        Translation2d(Meters.of(0.288925), Meters.of(0.288925)),
        Translation2d(Meters.of(0.288925), Meters.of(0.288925)),
        Translation2d(Meters.of(0.288925), Meters.of(0.288925)),
        Translation2d(Meters.of(0.288925), Meters.of(0.288925))
    )
    val moduleZeroRotations = arrayOf(
        Radians.of(0.0),
        Radians.of(0.0),
        Radians.of(0.0),
        Radians.of(0.0)
    )
    val encoderIDs = arrayOf(1, 2, 3, 4)
    val driveIDs = arrayOf(5, 6, 7, 8)
    val angleIDs = arrayOf(9, 10, 11, 12)

    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0
    const val DRIVE_S = 0.0
    const val DRIVE_V = 0.0
    const val DRIVE_CURRENT_LIMIT = 0.0

    const val ANGLE_P = 0.0
    const val ANGLE_I = 0.0
    const val ANGLE_D = 0.0
    const val ANGLE_CURRENT_LIMIT = 0.0

    const val TRANSLATION_P = 0.0
    const val TRANSLATION_I = 0.0
    const val TRANSLATION_D = 0.0

    const val HEADING_P = 0.0
    const val HEADING_I = 0.0
    const val HEADING_D = 0.0

    val maxSpeed: LinearVelocity = MetersPerSecond.of(4.571)
}

object DriveConfig {
    const val DRIVE_SPEED = 0.0
    const val DRIVE_SPEED_SLOW = 0.0

    const val ROT_SPEED = 0.0
    const val ROT_SPEED_SLOW = 0.0

    const val DRIVE_ACCELERATION = 0.0
}