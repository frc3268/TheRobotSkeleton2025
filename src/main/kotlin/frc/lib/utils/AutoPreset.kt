package frc.lib.utils

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

/**
 * A group of commands to be run in auto.
 *
 * This class is a SequentialCommandGroup under the hood, but adds the ability to change the commands on runtime, and adds a name property for the SendableChooser.
 *
 * @param commands a mutable list of the commands in the preset. Can be created using mutableListOf()
 * @property name the name of this preset.
 * @constructor Creates a preset with the specified name and commands.
 */
class AutoPreset(private var commands: MutableList<Command>, val name: String) {
    fun generateCommandGroup(): SequentialCommandGroup {
        val s = SequentialCommandGroup()
        for (command: Command in commands) {
            s.addCommands(command)
        }
        return s
    }

    fun clear() {
        commands.clear()
    }

    fun addCommand(command: Command) {
        commands.add(command)
    }
}