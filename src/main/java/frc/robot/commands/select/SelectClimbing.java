package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToClimbingCmdGroup;
import frc.robot.commands.groups.ToReadyToClimbCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectClimbing extends SelectCommand3309 {

    public SelectClimbing(ClimberSubsystem climber, IntakeSubsystem intake, ArmSubsystem arm, XboxController controller) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE) {
                return new ToReadyToClimbCmdGroup(climber, intake, arm);
            } else if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_CLIMB) {
                return new ToClimbingCmdGroup(climber, controller);
            } else {
                return new DoNothing();
            }
        });
    }
}
