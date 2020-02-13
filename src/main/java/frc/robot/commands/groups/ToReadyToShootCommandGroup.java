package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateState;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.SetFlywheelsSpeed;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToReadyToShootCommandGroup extends SequentialCommandGroup {
    public ToReadyToShootCommandGroup(ArmSubsystem.ArmPosition position, Double speedTop, Double speedBottom,
                                      IntakeSubsystem intake, IndexerSubsystem indexer,
                                      ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new UpdateState(RobotContainer.PowerCellHandlingState.INIT_READY_TO_SHOOT),
                new StopFlywheels(shooter),
                new StopIndexer(indexer),
                new StopIntake(intake),
                new MoveArmToPosition(position, arm),
                new RetractIntake(intake, arm),
                new SetFlywheelsSpeed(shooter, speedTop, speedBottom),
                new StartFlywheels(shooter),
                new UpdateState(RobotContainer.PowerCellHandlingState.READY_TO_SHOOT)
        );
    }
}
