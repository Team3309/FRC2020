package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateState;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.shooter.MultiShot;
import frc.robot.commands.shooter.StartFlywheel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShotCommandGroup extends SequentialCommandGroup {

    public MultiShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
                new UpdateState(RobotContainer.PowerCellHandlingState.INIT_MULTI_SHOT),
                new RetractIntake(intake, arm),
                new StartFlywheel(shooter),
                new UpdateState(RobotContainer.PowerCellHandlingState.MULTI_SHOT),
                new MultiShot(indexer, shooter),
        new UpdateState(RobotContainer.PowerCellHandlingState.READY_TO_SHOOT)
        );

    }



}
