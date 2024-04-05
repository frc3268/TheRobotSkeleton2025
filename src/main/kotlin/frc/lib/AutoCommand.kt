package frc.lib

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger

fun AutoCommand(command: Command, autoMap: MutableMap<String, Command>? = null, binding: Trigger? = null, name:String? = null, shuffleboardTab: ShuffleboardTab?  = null) {

    if(name != null){
        if(autoMap != null){
            autoMap[name] = command
        }
        shuffleboardTab?.add(name, command)

    }
    binding?.onTrue(command)
}