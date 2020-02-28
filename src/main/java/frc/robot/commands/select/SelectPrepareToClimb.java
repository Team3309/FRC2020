package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.PrepareToClimbCommandGroup;
import frc.robot.commands.groups.SpinTurnerCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SelectPrepareToClimb extends SelectCommand3309 {

    public SelectPrepareToClimb(ClimberSubsystem climber) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE) {
                return new PrepareToClimbCommandGroup(climber);
            } else {
                return new DoNothing();
            }
        });
    }

}
