package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//default command of indexer
public class AutoIndexIn extends CommandBase {

    private final ShooterSubsystem shooter;
    private IndexerSubsystem Indexer;
    private boolean wasToSpeed;
    private boolean isAccelerating;
    private double maxFlywheelSpeed;
    private double lastFlywheelSpeed;

    public AutoIndexIn(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.Indexer = indexer;
        addRequirements(indexer);
    }

    public void initialize() {
        wasToSpeed = false;
        isAccelerating = false;
        lastFlywheelSpeed = 0;
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
        if (RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE  && Indexer.getCount() < Config.maxPowerCells) {

            // Could get much fancier here with multiple sample filtering, but this core logic should be sound
            // so let's try the simple approach first.
            double currentFlywheelSpeed = (shooter.getTopMotorVelocity() + shooter.getBottomMotorVelocity()) / 2;
            if (currentFlywheelSpeed >= Config.autoIndexInMinFlywheelSpeed) {
                if (wasToSpeed) {
                    if (maxFlywheelSpeed - currentFlywheelSpeed >= Config.autoIndexInFlywheelSpeedDropDetectThreshold) {
                        // speed drop is over detection threshold
                        Indexer.indexIn();
                        wasToSpeed = false; //reset this so we don't repeatedly index in.
                        isAccelerating = false;
                    }
                }
                else if (isAccelerating) {
                    if (lastFlywheelSpeed - currentFlywheelSpeed < Config.autoIndexInMaxFlywheelSpeedTolerance) {
                        // we have plateaued at max speed after ramping up
                        wasToSpeed = true;
                        maxFlywheelSpeed = currentFlywheelSpeed;
                    }
                }
                else if (currentFlywheelSpeed - lastFlywheelSpeed >= Config.autoIndexInMaxFlywheelSpeedTolerance) {
                    // we are ramping up
                    isAccelerating = true;
                }
            }
            lastFlywheelSpeed = currentFlywheelSpeed;
        }
        else {
            initialize(); // restart state machine when resuming intake mode
        }
    }
}
