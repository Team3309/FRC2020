package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.indexer.SingleShot;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShotCommandGroup extends SequentialCommandGroup {
    public SingleShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.INIT_SINGLE_SHOT),
                new StartFlywheels(shooter),
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.SINGLE_SHOT),
                new SingleShot(indexer, shooter),
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.READY_TO_SHOOT)
        );

    }
}
