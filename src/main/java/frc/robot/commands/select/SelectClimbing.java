package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToClimbingCommandGroup;
import frc.robot.commands.groups.ToReadyToClimbCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class SelectClimbing extends SelectCommand3309 {

    public SelectClimbing(ClimberSubsystem climber, XboxController controller) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE) {
                return new ToReadyToClimbCommandGroup(climber);
            } else if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_CLIMB) {
                return new ToClimbingCommandGroup(climber, controller);
            } else {
                return new DoNothing();
            }
        });
    }
}