package frc.lib

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json

@Serializable
class AutoSequence (val commands:Array<Array<String>>){
    fun toCommandGroup(commandsMap:Map<String, Command>):SequentialCommandGroup{
        val sq = SequentialCommandGroup()
        for (commandList in commands) {
            val pcg = ParallelCommandGroup()
            for (command in commandList) {
                pcg.addCommands (commandsMap[command])
            }
            sq.addCommands(pcg)
        }
        return sq
    }
}