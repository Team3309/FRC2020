package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ContinuousShotCmdGroup;
import frc.robot.subsystems.IndexerSubsystem;

public class SelectToContinuousShot extends SelectCommand3309 {

    public SelectToContinuousShot(IndexerSubsystem indexer) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT) {
                return new ContinuousShotCmdGroup(indexer);
            } else {
                return new DoNothing();
            }

        });
    }
}
