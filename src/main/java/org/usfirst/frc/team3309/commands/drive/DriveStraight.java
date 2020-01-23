package org.usfirst.frc.team3309.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Drive;

public class DriveStraight extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.drive);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.drive)) return;

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
