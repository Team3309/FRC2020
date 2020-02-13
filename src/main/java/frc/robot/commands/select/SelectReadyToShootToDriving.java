package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectReadyToShootToDriving extends SelectCommand3309 {

    public SelectReadyToShootToDriving(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT
            ) {
                return new ToDriveCommandGroup(null, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }

}
