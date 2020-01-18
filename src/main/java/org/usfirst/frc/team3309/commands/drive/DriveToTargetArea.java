package org.usfirst.frc.team3309.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.Drive;

public class DriveToTargetArea extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.drive, Robot.shooter);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.drive) || !hasRequirement(Robot.shooter)) return;


    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
