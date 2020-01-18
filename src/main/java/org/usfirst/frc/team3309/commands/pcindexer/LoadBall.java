package org.usfirst.frc.team3309.commands.pcindexer;


import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team3309.Robot;
import org.usfirst.frc.team3309.subsystems.PCIndexer;

public class LoadBall extends CommandBase {

    @Override
    public void initialize() {
        addRequirements(Robot.indexer);
    }

    @Override
    public void execute() {
        if(!hasRequirement(Robot.indexer)) return;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
