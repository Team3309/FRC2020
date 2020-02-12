package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShot extends CommandBase {

    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private long lastPowerCellLoad;

    public MultiShot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
    }

    @Override
    public void execute() {
        if (shooter.isFlywheelToSpeed()) {
            indexer.indexerOut();
        } else {
            //wait
        }
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
