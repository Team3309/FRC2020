package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//default command of indexer
public class ManageIndexer extends CommandBase {

    private final ShooterSubsystem shooter;
    private IndexerSubsystem Indexer;
    private boolean wasToSpeed;
    private double flywheelSpeedAverage;

    public ManageIndexer(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.shooter = shooter;
        Indexer = indexer;
        addRequirements(indexer);
    }

    public void initialize() {
        wasToSpeed = false;
    }

    @Override
    public void execute() {
        if (shooter.areFlywheelsToSpeed()) {
            wasToSpeed = true;
            flywheelSpeedAverage = (shooter.getTopMotorVelocity() + shooter.getBottomMotorVelocity()) / 2;
        }
        //we aren't being given any sensor in the subsystem to determine
        //if there is a ball in the way so we need to figure
        //that out here using the shooter wheel now
        //basically if the flywheel average speed is below a threshold, index in.
        //use average to avoid errors as we use percent output mode on shooter intaking.
        if (Indexer.getCount() < Config.maxPowerCells)  {
            if (wasToSpeed && (shooter.getTopMotorVelocity() + shooter.getBottomMotorVelocity()) / 2 < flywheelSpeedAverage * Config.indexerPowerCellNeedsIndexingThreshold) {
                wasToSpeed = false; //reset this so we don't repeatedly index in.
                Indexer.indexIn();
            }
        }
    }
}
