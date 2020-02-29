package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.subsystems.ClimberSubsystem;

public class ToReadyToClimbCommandGroup extends SequentialCommandGroup {

    public ToReadyToClimbCommandGroup(ClimberSubsystem climber) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_READY_TO_CLIMB),
                new ExtendRetractClimber(true, climber),
                new UpdateHandlingState(RobotContainer.RobotState.READY_TO_CLIMB)
        );
    }
}
