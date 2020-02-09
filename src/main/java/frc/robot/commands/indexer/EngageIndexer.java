package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class EngageIndexer extends CommandBase {
    private final IndexerSubsystem indexer;

    public EngageIndexer(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.Load();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
