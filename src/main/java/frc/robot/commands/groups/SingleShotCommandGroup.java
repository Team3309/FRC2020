package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateState;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.indexer.LoadBall;
import frc.robot.commands.shooter.PrepareToFire;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShotCommandGroup extends SequentialCommandGroup {
    public SingleShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        addCommands(
                new UpdateState(RobotContainer.PowerCellHandlingState.SINGLE_SHOT),
                new Retract(intake),
                new PrepareToFire(shooter),
                new LoadBall(indexer),
                new UpdateState(RobotContainer.PowerCellHandlingState.INIT_READY_TO_SHOOT)
        );

    }
}
