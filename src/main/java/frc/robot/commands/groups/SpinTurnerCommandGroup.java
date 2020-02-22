package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.ctrlpanelturner.Rotate;
import frc.robot.subsystems.*;

public class SpinTurnerCommandGroup extends ParallelCommandGroup {

    public SpinTurnerCommandGroup(DriveSubsystem drive, CtrlPanelSubsystem ctrlPanel) {
        addCommands(
                //TODO: Need to engage drive against control panel, and stop when rotate is complete
                new Rotate(ctrlPanel), //TODO: Make it complete when spin is done
                new UpdateHandlingState(RobotContainer.RobotState.SPIN_TURNER)
        );
    }
}
