package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;


public class SetNumPowerCells extends InstantCommand {

    private IndexerSubsystem indexer;
    private int numPowerCells;

    public SetNumPowerCells(IndexerSubsystem indexer, int numPowerCells) {
        this.indexer = indexer;
        this.numPowerCells = numPowerCells;
        // no requirements are needed because this command doesn't move the indexer
    }

    @Override
    public void execute() {
        indexer.setNumPowerCells(numPowerCells);
    }

}
