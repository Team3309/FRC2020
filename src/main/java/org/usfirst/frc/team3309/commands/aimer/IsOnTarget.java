package org.usfirst.frc.team3309.commands.aimer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Aimer;

public class IsOnTarget extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.aimer);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.aimer)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
