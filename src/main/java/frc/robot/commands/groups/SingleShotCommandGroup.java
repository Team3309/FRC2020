package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.indexer.LoadShooter;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShotCommandGroup extends SequentialCommandGroup {
    public SingleShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.INIT_SINGLE_SHOT),
                new RetractIntake(intake, arm),
                new StartFlywheels(shooter),
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.SINGLE_SHOT),
                new LoadShooter(indexer),
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.READY_TO_SHOOT)
        );

    }
}
