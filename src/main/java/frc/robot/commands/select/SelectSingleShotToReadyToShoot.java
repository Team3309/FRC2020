package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectSingleShotToReadyToShoot extends SelectCommand3309{
    public SelectSingleShotToReadyToShoot(IntakeSubsystem intake, IndexerSubsystem indexer,
                                          ShooterSubsystem shooter, ArmSubsystem arm) {
        super (() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_SINGLE_SHOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.SINGLE_SHOT
            ) {
                return new ToReadyToShootCmdGroup(null, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }
}
