package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateState;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.indexer.UpdateIndexerState;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.StartIntakeMotor;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//aka Command Group 1
//executes the following commands
//1) stop the the IIF (Intake, Indexer, Flywheel)
//2) Extend the Intake
//3) Move Arm down
//4) Engage the IIF
public class ToIntakeCommandGroup extends SequentialCommandGroup {

    public ToIntakeCommandGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new UpdateState(RobotContainer.PowerCellHandlingState.INIT_INTAKE),
                new UpdateIndexerState(indexer, IndexerSubsystem.IndexerState.INDEXING_IN),
                new StopIntake(intake),
                new StopFlywheels(shooter),
                new ExtendIntake(intake),
                new MoveArmToPosition(ArmSubsystem.ArmPosition.min, arm),
                new StartIntakeMotor(intake, shooter),
                new UpdateIndexerState(indexer, IndexerSubsystem.IndexerState.INDEXING_IN),
                new UpdateState(RobotContainer.PowerCellHandlingState.INTAKE)
        );
    }

}
