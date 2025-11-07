package org.sert2521.bunnybots2025

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj.RobotBase
import yams.gearing.GearBox
import yams.gearing.MechanismGearing


// If you make more subsystems/constants objects, put them ABOVE this one
// ElectronicIDs always goes on the bottom for quick access/readability
object ElectronicIDs{
    const val WRIST_MOTOR_ID = 14
    const val FLYWHEEL_MOTOR_TOP_ID = 17
    const val FLYWHEEL_MOTOR_BOTTOM_ID = 18
}

object RobotConstants {
    val maxHeight = Inches.of(84.0)
    val maxLength = Inches.of(21.4).plus(Inches.of(14.0))


}


object WristConstants {
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val moi = KilogramSquareMeters.of(0.0)

    val length = Inches.of(10.8)

    val hardMin = Rotations.of(-0.27)
    val hardMax = Rotations.of(0.254)

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            4.0,
            40.0 / 15.0
        )
    )
}
object FlywheelsConstants {
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val moi = KilogramSquareMeters.of(0.0)

    val length = Inches.of(0.0)

    val hardMin = Rotations.of(0.0)
    val hardMax = Rotations.of(0.0)

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            0.0,
            0.0,
            0.0 / 0.0
        )
    )
}

