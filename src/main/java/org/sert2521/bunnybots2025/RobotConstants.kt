package org.sert2521.bunnybots2025

import edu.wpi.first.units.Units.*
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

// If you make more subsystems/constants objects, put them ABOVE this one
// ElectronicIDs always goes on the bottom for quick access/readability
object ElectronicIDs{
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

    val mass = Kilograms.of(0.0)
}

object WristConstants {
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val moi = KilogramSquareMeters.of(0.0)

    val length = Inches.of(10.8) // TODO: Change

    val hardMin = Rotations.of(-0.27) // TODO: Change
    val hardMax = Rotations.of(0.254) // TODO: Change

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            4.0,
            54.0 / 22.0
        )
    )

    val stowPosition = Degrees.of(0.0)
    val intakePosition = Degrees.of(0.0)
    val cabbagePosition = Degrees.of(0.0)
}

object IntakeConstants {
    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0 // TODO: Change
        )
    )

    val intakeSpeed = 0.0 // TODO: Change
    val reverseSpeed = 0.0 // TODO: Change
}

object FlywheelsConstants {
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val mass = Kilograms.of(0.0) // TODO: Change
    val drumRadius = Inches.of(0.0) // TODO:Change

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0 // TODO: Change
        )
    )

    val topShootTarget = RPM.of(0.0) // TODO: Change
    val bottomShootTarget = RPM.of(0.0) // TODO: Change
}

object IndexerConstants {
    val indexerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0 // TODO: Change
        )
    )
    val kickerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0 // TODO: Change
        )
    )
}

