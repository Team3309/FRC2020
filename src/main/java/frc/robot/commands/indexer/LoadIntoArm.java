package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;

/** --------------------------------------------------------------------------------------------------------------------
 * Command to load power cells into the arm by indexing in manually.
 */
public class LoadIntoArm extends InstantCommand {

    private IndexerSubsystem indexer;
    private int powerCells;

    public LoadIntoArm(IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        powerCells = indexer.getCount();
    }

    @Override
    public void execute() {
        if (powerCells >= 0 && powerCells < 5) {
            indexer.setVelocity(-10000);
        }
    }

}
