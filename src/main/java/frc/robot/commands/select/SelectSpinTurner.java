package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.PositionTurnerCommandGroup;
import frc.robot.commands.groups.SpinTurnerCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectSpinTurner extends SelectCommand3309 {

    public SelectSpinTurner(DriveSubsystem drive, CtrlPanelSubsystem ctrlPanel) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION) {
                return new SpinTurnerCommandGroup(drive, ctrlPanel);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }

}
