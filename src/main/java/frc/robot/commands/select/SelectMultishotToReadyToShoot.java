package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectMultishotToReadyToShoot extends SelectCommand3309 {
    public SelectMultishotToReadyToShoot(IntakeSubsystem intake, IndexerSubsystem indexer,
                                         ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_MULTI_SHOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.MULTI_SHOT
            ) {
                return new ToReadyToShootCommandGroup(null, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }

}
