package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * FOR ANY AND ALL TIME TRAVELLERS AND PEOPLE OF THE FUTURE:
 *
 * THIS FILE IS A MUCH IMPROVED VERSION OF SELECTCOMMAND.java
 *
 * FIRST, IT DOESN'T BOG DOWN YOUR ROBORIO BY CONSTRUCTING YOUR COMMANDS OR COMMAND GROUPS REPEATEDLY
 * VIA UNNECCESSARY CALLS TO THE SUPPLIER'S GET()
 *
 * SECONDLY AND MORE IMPORTANTLY, THIS FILE ACTUALLY SCHEDULES THE COMMAND OR COMMANDGROUP SELECTED
 * BY THIS SELECT COMMAND.
 * THIS ALLOWS YOUR SELECTCOMMAND TO NOT REQUIRE THE THINGS THAT YOUR COMMANDGROUP MIGHT
 * SO THAT IF YOU PRESS A BUTTON ERRONEOUSLY (OUT OF STATE) YOU DO NOT INTERRUPT OTHER COMMANDGROUPS
 * THAT SHOULD ACTUALLY BE RUNNING, WHILE ALSO ALLOWING YOU TO INTERRUPT THE UNDERLYING COMMAND GROUP
 *
 * AT THE SAME TIME, IF THE SELECT COMMAND IS INTERRUPTED VIA WHILEHELD(), IT WILL INTERRUPT THE UNDERLYING
 * COMMAND GROUP
 *
 * ITS GREAT.
 */
public class SelectCommand3309 extends CommandBase {

    private final Supplier<Command> m_toRun;
    private Command m_selectedCommand;

    public SelectCommand3309(Supplier<Command> toRun) {
        m_toRun = requireNonNullParam(toRun, "toRun", "SelectCommand");
    }

    @Override
    public void initialize() {
        m_selectedCommand = m_toRun.get();
        m_selectedCommand.initialize();
        CommandScheduler.getInstance().schedule(m_selectedCommand);
    }

    @Override
    public void end(boolean interrupted) {
        m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_selectedCommand.isFinished();
    }
}
