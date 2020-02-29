package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.PrepareToClimbCommandGroup;
import frc.robot.commands.groups.SpinTurnerCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SelectPrepareToClimb extends SelectCommand3309 {

    public SelectPrepareToClimb(ClimberSubsystem climber, XboxController controller) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE) {
                return new PrepareToClimbCommandGroup(climber, controller);
            } else {
                return new DoNothing();
            }
        });
    }
}
