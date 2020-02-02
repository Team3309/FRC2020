package org.usfirst.frc.team3309.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;

public class LowerShooter extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.shooter)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
