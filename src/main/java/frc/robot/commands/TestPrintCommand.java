package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestPrintCommand extends CommandBase {
    public void execute() {
        System.out.println("Test print command printed this.");
    }

    public boolean isFinished() {
        return true;
    }
}
