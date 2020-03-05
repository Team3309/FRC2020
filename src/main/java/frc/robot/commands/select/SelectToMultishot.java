package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.MultiShotCmdGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToMultishot extends SelectCommand3309 {


    public SelectToMultishot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        super(() -> {
            if (    (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE &&
                    shooter.hasPresetSpeeds()) ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT) {
                return new MultiShotCmdGroup(shooter, indexer);
            } else {
                return new DoNothing();
            }

        });
    }

}
