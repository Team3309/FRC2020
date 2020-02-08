package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

public class SelectIntakeToggle extends SelectCommand {

    public SelectIntakeToggle(Supplier<Command> toRun, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.state == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.state == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT ||
                    RobotContainer.state == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE
            ) {
                return new ToIntakeCommandGroup(intake, indexer, shooter, arm); //aka command group 1 (See Slack for details)
            } else if (RobotContainer.state == RobotContainer.PowerCellHandlingState.INTAKE) {
                return new ToDriveCommandGroup(ArmSubsystem.ArmPosition.intermediate, intake, indexer, shooter, arm); //TODO execute command group 2 instead (see slack for command group descriptions).
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
