package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.StopCtrlPanelSpinningCommandGroup;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.commands.groups.TurnerInPositionToDriveCommandGroup;
import frc.robot.subsystems.*;

public class SelectStopCtrlPanelSpinning extends SelectCommand3309 {

    public SelectStopCtrlPanelSpinning(CtrlPanelSubsystem manipulator) {
        super(() -> {
            if ((RobotContainer.getRobotState() == RobotContainer.RobotState.SPIN_TURNER)) {
                return new StopCtrlPanelSpinningCommandGroup(manipulator);
            }  else {
                return new DoNothing();
            }
        });
    }
}
