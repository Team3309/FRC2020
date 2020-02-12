package frc.robot.commands.indexer;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class LoadShooter extends CommandBase {

    private IndexerSubsystem indexer;

    public LoadShooter(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        indexer.indexOut();
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
