package frc.robot.commands.pcindexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class StopIndexer extends CommandBase {

    private final IndexerSubsystem indexer;

    public StopIndexer(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void execute() {
        indexer.StopIndexer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
