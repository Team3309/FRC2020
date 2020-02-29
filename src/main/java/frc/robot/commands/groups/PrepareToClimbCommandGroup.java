package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.climber.LiftRobot;
import frc.robot.commands.select.SelectLiftRobot;
import frc.robot.subsystems.ClimberSubsystem;

public class PrepareToClimbCommandGroup extends SequentialCommandGroup {

    public PrepareToClimbCommandGroup(ClimberSubsystem climber, XboxController controller) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_READY_TO_CLIMB),
                new ExtendClimber(climber),
                new UpdateHandlingState(RobotContainer.RobotState.CLIMBING),
                new LiftRobot(climber, controller)
        );
    }
}
