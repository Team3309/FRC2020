package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.StopCtrlPanelSpinningCmdGroup;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SelectStopCtrlPanelSpinning extends SelectCommand3309 {

    public SelectStopCtrlPanelSpinning(CtrlPanelSubsystem manipulator, DriveSubsystem drive) {
        super(() -> {
            if ((RobotContainer.getRobotState() == RobotContainer.RobotState.SPIN_TURNER)) {
                return new StopCtrlPanelSpinningCmdGroup(manipulator, drive);
            }  else {
                return new DoNothing();
            }
        });
    }
}
