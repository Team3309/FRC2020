package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.subsystems.*;

public class StopCtrlPanelSpinningCommandGroup extends SequentialCommandGroup {

    public StopCtrlPanelSpinningCommandGroup(CtrlPanelSubsystem manipulator) {
        addCommands(
                new InstantCommand(manipulator::stop),
                new UpdateHandlingState(RobotContainer.RobotState.TURNER_IN_POSITION)
        );
    }

}
