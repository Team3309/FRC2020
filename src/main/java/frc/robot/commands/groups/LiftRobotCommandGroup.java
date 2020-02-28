package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.climber.LiftRobot;
import frc.robot.subsystems.ClimberSubsystem;

public class LiftRobotCommandGroup extends SequentialCommandGroup {

    public LiftRobotCommandGroup(ClimberSubsystem climber, double power) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.CLIMBING),
                new LiftRobot(climber, power)
        );
    }
}
