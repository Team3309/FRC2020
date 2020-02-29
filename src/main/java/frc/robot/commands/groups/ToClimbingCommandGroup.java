package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.commands.climber.LiftRobot;
import frc.robot.subsystems.ClimberSubsystem;

public class ToClimbingCommandGroup extends SequentialCommandGroup {
    public ToClimbingCommandGroup(ClimberSubsystem climber, XboxController controller) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_CLIMBING),
                new ExtendRetractClimber(false, climber),
                new UpdateHandlingState(RobotContainer.RobotState.CLIMBING),
                new LiftRobot(climber, controller)
        );
    }
}
