package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//default command of indexer
public class AutoIndexIn extends CommandBase {

    private final ShooterSubsystem shooter;
    private IndexerSubsystem Indexer;
    private boolean isIndexing;
    private double maxFlywheelSpeed;

    public AutoIndexIn(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.Indexer = indexer;
        addRequirements(indexer);
    }

    public void initialize() {
        isIndexing = false;
        maxFlywheelSpeed = 0;
    }

    //we aren't being given any sensor in the subsystem to determine
    //if there is a ball in the way so we need to figure
    //that out here using the shooter wheel now
    //basically if the flywheel average speed is below a threshold, index in.
    //use average to avoid errors as we use percent output mode on shooter intaking.
    @Override
    public void execute() {

        // Not sure about the maxPowerCells check. Would be nice to drop maxPowerCells down to 3.
        // However, getting a power cell jammed at the front of the indexer because we're full isn't great.
        // Perhaps that is better than the alternative of jamming all power cells against the intake motors.
        // Probably better to switch to outtake when we're full instead.
        if (RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE  &&
                Indexer.getCount() < Config.maxPowerCells) {

            // flywheel speed is negative when intaking
            // use bottom flywheel only because top flywheel doesn't slow much when intaking
            double currentFlywheelSpeed = shooter.getBottomMotorVelocity();

            if (isIndexing) {
                if (currentFlywheelSpeed >= maxFlywheelSpeed) {
                    // flywheels are still slowing down
                    maxFlywheelSpeed = currentFlywheelSpeed;  // keep track of slowest speed
                } else if (currentFlywheelSpeed >= maxFlywheelSpeed - Config.autoIndexInFlywheelSpeedUpThreshold) {
                    // significant speed up detected after indexing
                    maxFlywheelSpeed = currentFlywheelSpeed;  // keep track of highest speed
                    isIndexing = false;  // enable detection logic for next power cell
                }
            } else if (currentFlywheelSpeed >=  maxFlywheelSpeed + Config.autoIndexInFlywheelSpeedDropThreshold) {
                // flywheel speed drop is over detection threshold
                Indexer.reset();  // force the indexer to be in-position so we get a full index in pulse
                Indexer.indexIn();
                maxFlywheelSpeed = currentFlywheelSpeed;
                isIndexing = true;  // don't index again if we get a second drop over threshold before speeding up again
            } else if (currentFlywheelSpeed <  maxFlywheelSpeed) {
                // flywheels are speeding up
                maxFlywheelSpeed = currentFlywheelSpeed; // keep track of highest speed
            }

            if (RobotContainer.getIndexerDebug()) {
                SmartDashboard.putNumber("Auto index maxFlywheelSpeed", maxFlywheelSpeed);
                SmartDashboard.putNumber("Auto index currentFlywheelSpeed", currentFlywheelSpeed);
                SmartDashboard.putBoolean("Auto index isIndexing", isIndexing);
            }
        }
        else {
            initialize(); // restart state machine when intake is complete
        }
    }
}
