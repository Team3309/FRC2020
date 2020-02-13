package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToTrench extends SelectCommand3309 {
    public SelectToTrench(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_INTAKE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INTAKE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_READY_TO_SHOOT
            ) {
                return new ToDriveCommandGroup(ArmSubsystem.ArmPosition.trench, intake, indexer, shooter, arm);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
