package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.commands.groups.TurnerInPositionToDriveCommandGroup;
import frc.robot.subsystems.*;

public class SelectCtrlPanelToDrive extends SelectCommand3309 {

    public SelectCtrlPanelToDrive(ArmSubsystem arm, DriveSubsystem drive, IntakeSubsystem intake) {
        super(() -> {
            if ((RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.SPIN_TURNER)) {
                return new TurnerInPositionToDriveCommandGroup(arm, drive, intake);
            }  else {
                return new DoNothing();
            }
        });
    }
}
