package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIndexerSpeed extends InstantCommand {

    private IndexerSubsystem indexer;
    private int indexerSpeed;

    public SetIndexerSpeed(IndexerSubsystem indexer, int indexerSpeed) {
        this.indexer = indexer;
        this.indexerSpeed = indexerSpeed;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.setIndexerSpeed(indexerSpeed);
    }

}
