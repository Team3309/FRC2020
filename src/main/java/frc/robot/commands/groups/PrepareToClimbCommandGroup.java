package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.subsystems.*;

public class PrepareToClimbCommandGroup extends SequentialCommandGroup {

    public PrepareToClimbCommandGroup(ClimberSubsystem climber) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_READY_TO_CLIMB),
                new ExtendClimber(climber),
                new UpdateHandlingState(RobotContainer.RobotState.READY_TO_CLIMB)
        );
    }
}
