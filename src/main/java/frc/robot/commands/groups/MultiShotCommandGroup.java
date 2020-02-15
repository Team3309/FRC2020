package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.shooter.MultiShot;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShotCommandGroup extends SequentialCommandGroup {

    public MultiShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addCommands(
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.INIT_MULTI_SHOT),
                new StartFlywheels(shooter),
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.MULTI_SHOT),
                new MultiShot(indexer, shooter)
        );

    }



}
