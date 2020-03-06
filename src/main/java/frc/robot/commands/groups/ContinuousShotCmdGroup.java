package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.subsystems.IndexerSubsystem;

public class ContinuousShotCmdGroup extends SequentialCommandGroup {

    public ContinuousShotCmdGroup(IndexerSubsystem indexer) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.CONTINUOUS_SHOT),
                new InstantCommand(() -> {
                    indexer.setNumPowerCells(0);
                    indexer.velocityShooting();
                }, indexer)
        );

    }
}