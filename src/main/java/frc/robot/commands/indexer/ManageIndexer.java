package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManageIndexer extends CommandBase {

    private final ShooterSubsystem shooter;
    //default command of indexer.
    private IndexerSubsystem Indexer;
    private boolean wasToSpeed;
    private double flywheelSpeedAverage;

    public ManageIndexer(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.shooter = shooter;
        Indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (shooter.areFlywheelsToSpeed()) {
            wasToSpeed = true;
            flywheelSpeedAverage = shooter.getTopMotorVelocity() / 2 + shooter.getBottomMotorVelocity() / 2;
        }
        //we aren't being given any sensor in the subsystem to determine
        //if there is a ball in the way so we need to figure
        //that out here using the shooter wheel now
        if (wasToSpeed && (shooter.getTopMotorVelocity() + shooter.getBottomMotorVelocity()) / 2 < flywheelSpeedAverage * Config.indexerPowerCellNeedsIndexingThreshold) {
            wasToSpeed = false; //reset this so we don't repeatedly index in.
            Indexer.indexIn();
        }
    }

}
