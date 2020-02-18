package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShot extends CommandBase {

    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private boolean hasBeenShot;

    public SingleShot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
    }

    @Override
    public void initialize() {
        hasBeenShot = false;
    }

    @Override
    public void execute() {
        if (shooter.areFlywheelsToSpeed()) {
            indexer.indexOut();
            hasBeenShot = true;
        }
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return hasBeenShot;
    }
}
