package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.PositionTurnerCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectDeployTurner extends SelectCommand3309 {

    public SelectDeployTurner(ArmSubsystem arm, IntakeSubsystem intake) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.TRENCH_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT) {
                return new PositionTurnerCommandGroup(arm, intake);
            } else {
                return new DoNothing();
            }
        });
    }
}
