package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class StopCtrlPanelSpinningCmdGroup extends SequentialCommandGroup {

    public StopCtrlPanelSpinningCmdGroup(CtrlPanelSubsystem manipulator, DriveSubsystem drive) {
        addCommands(
                new InstantCommand(manipulator::stop, manipulator),
                new UpdateHandlingState(RobotContainer.RobotState.TURNER_IN_POSITION)
        );
        addRequirements(drive); //To stop drive from pushing against control panel and to allow DriveManual to resume
        addRequirements(manipulator);
    }
}