package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.LiftRobotCommandGroup;
import frc.robot.commands.groups.PrepareToClimbCommandGroup;
import frc.robot.commands.groups.SpinTurnerCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SelectLiftRobot extends SelectCommand3309 {

    public SelectLiftRobot(ClimberSubsystem climber, double power) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_CLIMB) {
                return new LiftRobotCommandGroup(climber, power);
            } else {
                return new DoNothing();
            }
        });
    }
}
