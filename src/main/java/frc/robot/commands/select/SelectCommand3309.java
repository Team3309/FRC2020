package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

public class SelectCommand3309 extends InstantCommand {

    private final Supplier<Command> m_toRun;
    private Command m_selectedCommand;

    public SelectCommand3309(Supplier<Command> toRun) {
        m_toRun = requireNonNullParam(toRun, "toRun", "SelectCommand");
    }

    public void initialize() {
        m_selectedCommand = m_toRun.get();
    }


    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(m_selectedCommand);
    }
}
