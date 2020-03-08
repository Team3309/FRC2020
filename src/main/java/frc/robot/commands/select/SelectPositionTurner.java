package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.PositionTurnerCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectPositionTurner extends SelectCommand3309 {

    public SelectPositionTurner(ArmSubsystem arm, IntakeSubsystem intake) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE) {
                return new PositionTurnerCmdGroup(arm, intake);
            } else {
                return new DoNothing();
            }
        });
    }

}
