package frc.lib.core

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL

fun coreInitialize() {

    // initialize HAL
    check(HAL.initialize(500, 0)) { "Failed to initialize. Terminating." }

    // report robot language as Kotlin
    // 6 Means kotlin in French
    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, 6)
}