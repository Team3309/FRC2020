package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.shooter.MultiShot;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ContinuousShotCmdGroup extends SequentialCommandGroup {

    public ContinuousShotCmdGroup(IndexerSubsystem indexer) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.CONTINUOUS_SHOT),
                new InstantCommand(() -> {
                    indexer.reset();
                    indexer.setNumPowerCells(0);
                    indexer.setVelocity(4000);
                }, indexer) //TODO: move this to config or pass in a firing solution if this survive week one
        );

    }
}