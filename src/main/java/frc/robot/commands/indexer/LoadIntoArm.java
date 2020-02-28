package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class LoadIntoArm extends InstantCommand {

    private IndexerSubsystem indexer;

    public LoadIntoArm(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.indexIn();
    }

}
