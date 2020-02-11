package frc.robot.commands.indexer;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class LoadBall extends CommandBase {

    private IndexerSubsystem Indexer;

    public LoadBall(IndexerSubsystem indexer) {
        Indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        Indexer.load();
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
