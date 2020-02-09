package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateState;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.SetFlywheelSpeed;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToReadyToShootCommandGroup extends SequentialCommandGroup {
    public ToReadyToShootCommandGroup(ArmSubsystem.ArmPosition position, double speed, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new UpdateState(RobotContainer.PowerCellHandlingState.READY_TO_SHOOT),
                new StopFlywheel(shooter),
                new StopIndexer(indexer),
                new StopIntake(intake),
                new MoveArmToPosition(position, arm),
                new Retract(intake),
                new SetFlywheelSpeed(shooter, speed)
        );
    }
}