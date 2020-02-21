package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.DeployTurnerCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectDeployTurner extends SelectCommand3309 {

    public SelectDeployTurner(ArmSubsystem arm, IntakeSubsystem intake, CtrlPanelSubsystem manipulator) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT) {
                return new DeployTurnerCommandGroup(arm, intake, manipulator);
            } else {
                return new DoNothing();
            }
        });
    }
}
