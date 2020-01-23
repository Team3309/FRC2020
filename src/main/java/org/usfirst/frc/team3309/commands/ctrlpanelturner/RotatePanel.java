package org.usfirst.frc.team3309.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.CtrlPanelTurner;

public class RotatePanel extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.ctrlPanelTurner);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.ctrlPanelTurner)) return;
        Robot.ctrlPanelTurner.turn(4, true);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
