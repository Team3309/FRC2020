package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToDrive extends SelectCommand {
    public SelectToDrive(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.PowerCellHandlingState.INIT_INTAKE == RobotContainer.getPowerCellHandlingState() ||
                RobotContainer.PowerCellHandlingState.INTAKE == RobotContainer.getPowerCellHandlingState()) {
                return new ToDriveCommandGroup(ArmSubsystem.ArmPosition.intakeStowedLimit, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });

    }
}
