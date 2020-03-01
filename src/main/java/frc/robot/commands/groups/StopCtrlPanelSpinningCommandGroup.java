package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class StopCtrlPanelSpinningCommandGroup extends SequentialCommandGroup {

    public StopCtrlPanelSpinningCommandGroup(CtrlPanelSubsystem manipulator, DriveSubsystem drive) {
        addCommands(
                //Why does this have to be an instant command?
                new InstantCommand(manipulator::stop),
                new UpdateHandlingState(RobotContainer.RobotState.TURNER_IN_POSITION)
        );
        addRequirements(drive); //To stop drive from pushing against control panel and to allow DriveManual to resume
        addRequirements(manipulator);
    }
}