package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.shooter.SingleShot;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShotCommandGroup extends SequentialCommandGroup {
    public SingleShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_SINGLE_SHOT),
                new StartFlywheels(shooter),
                new UpdateHandlingState(RobotContainer.RobotState.SINGLE_SHOT),
                new SingleShot(indexer, shooter),
                new UpdateHandlingState(RobotContainer.RobotState.READY_TO_SHOOT)
        );

    }
}
