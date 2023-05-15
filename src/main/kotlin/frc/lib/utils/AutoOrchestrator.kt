package frc.lib.utils

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class AutoOrchestrator(private val availibleCommands: Array<Command>, presets: Array<AutoPreset>, slots: Int) {
    private val presetChooser: SendableChooser<AutoPreset> = SendableChooser()
    private val customPreset: AutoPreset = AutoPreset(mutableListOf(), "Custom")

    private var commandChoosers: MutableList<SendableChooser<Command>> = mutableListOf()

    private val tab: ShuffleboardTab = Shuffleboard.getTab("Auto")

    init {
        for (preset: AutoPreset in presets) {
            presetChooser.addOption(preset.name, preset)
        }
        presetChooser.setDefaultOption(customPreset.name, customPreset)
        tab.add(presetChooser)
        //set up the tab
        reset(slots)
    }

    private fun reset(slots: Int) {
        commandChoosers.clear()
        val chooser: SendableChooser<Command> = SendableChooser()
        for (command in availibleCommands) {
            chooser.addOption(command.name, command)
        }
        for (i in 1..slots) {
            commandChoosers.add(chooser)
            tab.add(chooser)
        }
    }


    fun getAuto(): SequentialCommandGroup {
        customPreset.clear()
        if (presetChooser.selected == customPreset) {
            for (chooser: SendableChooser<Command> in commandChoosers) {
                customPreset.addCommand(chooser.selected)
            }
            return customPreset.generateCommandGroup()
        }
        return presetChooser.selected.generateCommandGroup()
    }
}