package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.indexer.UpdateIndexerState;
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
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.INIT_READY_TO_SHOOT),
                /*we only want to stop the flywheels if we are moving the arm.*/
                position == null ? new DoNothing() : new StopFlywheels(shooter),
                new UpdateIndexerState(indexer, IndexerSubsystem.IndexerState.OFF),
                new StopIntake(intake),
                new MoveArmAndRetractIntake(position, intake, arm),
                new SetFlywheelsSpeed(shooter, speedTop, speedBottom),
                new StartFlywheels(shooter),
                new UpdateHandlingState(RobotContainer.PowerCellHandlingState.READY_TO_SHOOT)
        );
    }
}
