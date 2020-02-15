package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShot extends CommandBase {

    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;

    public MultiShot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
    }

    @Override
    public void execute() {
        //only shoot if flywheel is up to speed
        if (shooter.areFlywheelsToSpeed()) {
            indexer.indexOut();
        }
    }



    @Override
    public boolean isFinished() {
        //could potentially stop early if index count turns out to be accurate
        return true;
    }
}
