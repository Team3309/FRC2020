package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ContinuousShotCommandGroup;
import frc.robot.commands.groups.MultiShotCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToContinuousShot extends SelectCommand3309 {


    public SelectToContinuousShot(IndexerSubsystem indexer) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT) {
                return new ContinuousShotCommandGroup(indexer);
            } else {
                return new DoNothing();
            }

        });
    }

}
