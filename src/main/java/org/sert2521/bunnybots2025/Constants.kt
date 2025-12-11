package org.sert2521.bunnybots2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.*
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

// If you make more subsystems/constants objects, put them ABOVE this one
// ElectronicIDs always goes on the bottom for quick access/readability
object ElectronicIDs {
    /* If you're looking for drivetrain IDs, they are in subsystems/drivetrain/SwerveConstants */

    const val WRIST_MOTOR_ID = 14
    const val INTAKE_MOTOR_ID = 15
    const val FLYWHEEL_MOTOR_TOP_ID = 17
    const val FLYWHEEL_MOTOR_BOTTOM_ID = 18
    const val INDEXER_MOTOR_ID = 19
    const val KICKER_MOTOR_ID = 20
}

object RobotConstants {
    val maxHeight = Inches.of(84.0)
    val maxLength = Inches.of(21.4).plus(Inches.of(14.0))

    val mass = Kilograms.of(129.0)
    val moi = KilogramSquareMeters.of(7.068515803)

    val targetVisionPose = Pose2d()
}

object WristConstants {
    // TODO: Tune
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val moi = KilogramSquareMeters.of(0.346)

    val length = Inches.of(15.1)

    // Found this in cad by picking a point on the end of the wrist that you decide to be parallel
    // and then using arctan with the measurements on both the top and bottom positions
    val hardMin = Degrees.of(-24.36)
    val hardMax = Degrees.of(96.49)

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            4.0,
            56.0 / 24.0,
            56.0 / 24.0
        )
    )

    // TODO: Test these values
    val stowPosition = Degrees.of(96.0)
    val intakePosition = Degrees.of(-22.87)
    val cabbagePositionFirst = Degrees.of(35.4)
    val cabbagePositionSecond = Degrees.of(52.94)

    const val RESET_DUTY_CYCLE = -0.3
}

object IntakeConstants {
    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            2.0
        )
    )

    const val INTAKE_SPEED = 0.5
    const val REVERSE_SPEED = -0.3
}

object FlywheelsConstants {
    const val P = 0.0
    const val D = 0.0

    const val S = 0.0
    const val V = 0.255
    const val A = 0.0

    val moi = KilogramSquareMeters.of(0.0006489431)

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            2.0
        )
    )

    val topShootTarget = RPM.of(2000.0) // TODO: Change
    val bottomShootTarget = RPM.of(2000.0) // TODO: Change
}

object IndexerConstants {
    val indexerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            6.75
        )
    )
    val kickerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            2.0
        )
    )

    const val MAIN_DEFAULT = 0.0
    const val KICKER_DEFAULT = 0.0

    const val MAIN_INDEXING = 0.3
    const val KICKER_INDEXING = -0.5

    const val MAIN_KICKING = 1.0
    const val KICKER_KICKING = 0.7
    const val KICK_TIME = 0.2

    const val MAIN_REVERSE = -0.2
    const val KICKER_REVERSE = -0.2
}

