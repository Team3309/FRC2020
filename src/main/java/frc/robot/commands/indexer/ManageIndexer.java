package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class ManageIndexer extends CommandBase {

    //default command of indexer.
    private IndexerSubsystem Indexer;
    public ManageIndexer(IndexerSubsystem indexer) {

        Indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        Indexer.manageSensorState();
    }

}
