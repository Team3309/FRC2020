package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectIntakeToggle extends SelectCommand {

    public SelectIntakeToggle(IntakeSubsystem intake, IndexerSubsystem indexer,
                              ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE
            ) {
                return new ToIntakeCommandGroup(intake, indexer, shooter, arm); //aka command group 1 (See Slack for details)
            } else if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INTAKE) {
                return new ToDriveCommandGroup(ArmSubsystem.ArmPosition.intermediate, intake, indexer, shooter, arm); //aka command group 2 (See Slack for details)
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
