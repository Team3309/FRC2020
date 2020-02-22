package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.ctrlpanelturner.Rotate;
import frc.robot.commands.drive.DriveApplyPower;
import frc.robot.subsystems.*;

public class SpinTurnerCommandGroup extends SequentialCommandGroup {

    public SpinTurnerCommandGroup(DriveSubsystem drive, CtrlPanelSubsystem ctrlPanel) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.SPIN_TURNER),
                new DriveApplyPower(Config.turnerDriveHoldPower, drive),
                new Rotate(ctrlPanel),
                new UpdateHandlingState(RobotContainer.RobotState.TURNER_IN_POSITION)
        );
    }
}
