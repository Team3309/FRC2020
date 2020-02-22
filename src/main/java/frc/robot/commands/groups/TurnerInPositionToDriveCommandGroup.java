package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.drive.DriveApplyPower;
import frc.robot.subsystems.*;

public class TurnerInPositionToDriveCommandGroup extends SequentialCommandGroup {

    public TurnerInPositionToDriveCommandGroup(ArmSubsystem arm, DriveSubsystem drive, IntakeSubsystem intake) {
        addCommands(
                new DriveApplyPower(0, drive),
                new UpdateHandlingState(RobotContainer.RobotState.ARM_UP_DRIVE)
        );
    }

}
