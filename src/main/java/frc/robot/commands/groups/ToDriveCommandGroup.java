package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateState;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToDriveCommandGroup extends SequentialCommandGroup {
    public ToDriveCommandGroup(ArmSubsystem.ArmPosition position, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(
                position == ArmSubsystem.ArmPosition.trench ?
                        new UpdateState(RobotContainer.PowerCellHandlingState.INIT_TRENCH_DRIVE) :
                        new UpdateState(RobotContainer.PowerCellHandlingState.INIT_ARM_UP_DRIVE),
                new StopIndexer(indexer),
                new StopIntake(intake),
                new StopFlywheel(shooter),
                new MoveArmToPosition(position, arm),
                new RetractIntake(intake, arm),
                position == ArmSubsystem.ArmPosition.trench ?
                        new UpdateState(RobotContainer.PowerCellHandlingState.TRENCH_DRIVE) :
                        new UpdateState(RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE));
    }
}
