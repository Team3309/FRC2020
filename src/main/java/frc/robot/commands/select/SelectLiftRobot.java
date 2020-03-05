package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.LiftRobotCmdGroup;
import frc.robot.subsystems.ClimberSubsystem;

public class SelectLiftRobot extends SelectCommand3309 {

    public SelectLiftRobot(ClimberSubsystem climber, XboxController controller) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_CLIMB) {
                return new LiftRobotCmdGroup(climber, controller);
            } else {
                return new DoNothing();
            }
        });
    }
}
