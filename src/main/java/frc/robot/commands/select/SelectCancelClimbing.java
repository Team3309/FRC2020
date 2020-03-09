package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.CancelClimbingCmdGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class SelectCancelClimbing extends SelectCommand3309 {

    public SelectCancelClimbing(ClimberSubsystem climber) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_CLIMB ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.CLIMBING ) {
                return new CancelClimbingCmdGroup(climber);
            } else {
                return new DoNothing();
            }
        });
    }
}
