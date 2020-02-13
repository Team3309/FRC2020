package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToScanCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToScan extends SelectCommand3309 {

    public SelectToScan(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INTAKE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE
            ) {
                return new ToScanCommandGroup();
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }

}
