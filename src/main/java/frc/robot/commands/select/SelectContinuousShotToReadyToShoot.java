package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectContinuousShotToReadyToShoot extends SelectCommand3309 {
    public SelectContinuousShotToReadyToShoot(IntakeSubsystem intake, IndexerSubsystem indexer,
                                         ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.CONTINUOUS_SHOT) {
                return new ToReadyToShootCmdGroup(null, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }

}
