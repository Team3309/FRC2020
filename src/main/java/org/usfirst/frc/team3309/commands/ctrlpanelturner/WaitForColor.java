package org.usfirst.frc.team3309.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;

public class WaitForColor extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.ctrlPanelTurner);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.ctrlPanelTurner)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
