package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectSingleShotToReadyToShoot extends SelectCommand3309{
    public SelectSingleShotToReadyToShoot(IntakeSubsystem intake, IndexerSubsystem indexer,
                                          ShooterSubsystem shooter, ArmSubsystem arm) {
        super (() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_SINGLE_SHOT ||
                RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.SINGLE_SHOT
            ) {
                return new ToReadyToShootCommandGroup(null, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }
}
