package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class LoadIntoArm extends CommandBase {

    private IndexerSubsystem indexer;

    public LoadIntoArm(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        indexer.indexIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
