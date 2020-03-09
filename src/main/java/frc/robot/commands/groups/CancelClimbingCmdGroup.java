package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.subsystems.ClimberSubsystem;

public class CancelClimbingCmdGroup extends SequentialCommandGroup {
    public CancelClimbingCmdGroup(ClimberSubsystem climber) {
        addCommands(
                new ExtendRetractClimber(false, climber),
                new UpdateHandlingState(RobotContainer.RobotState.ARM_UP_DRIVE)
        );
    }
}

