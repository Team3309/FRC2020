package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ManageIndexer;

public class EnableIndexer extends CommandBase {
    private final IndexerSubsystem indexer;


    public EnableIndexer(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.updateIndexerState(IndexerSubsystem.IndexerState.INDEXING);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
