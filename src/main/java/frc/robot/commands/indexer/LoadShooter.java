package frc.robot.commands.indexer;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class LoadShooter extends CommandBase {

    private IndexerSubsystem Indexer;

    public LoadShooter(IndexerSubsystem indexer) {
        Indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        Indexer.indexerOut();
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
