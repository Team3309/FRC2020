package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class UpdateIndexerState extends CommandBase {

    private final IndexerSubsystem indexer;
    private IndexerSubsystem.IndexerState state;

    public UpdateIndexerState(IndexerSubsystem indexer, IndexerSubsystem.IndexerState state) {
        this.indexer = indexer;
        this.state = state;
        addRequirements(indexer);
    }
    @Override
    public void execute() {
        indexer.updateIndexerState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
