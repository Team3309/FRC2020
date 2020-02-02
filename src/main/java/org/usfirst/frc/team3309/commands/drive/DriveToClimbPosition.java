package org.usfirst.frc.team3309.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;

public class DriveToClimbPosition extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.drive, Robot.climber);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.drive) || !hasRequirement(Robot.climber)) return;

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
