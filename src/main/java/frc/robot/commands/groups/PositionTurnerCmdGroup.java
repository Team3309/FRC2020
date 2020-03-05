package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PositionTurnerCmdGroup extends SequentialCommandGroup {

    public PositionTurnerCmdGroup(ArmSubsystem arm, IntakeSubsystem intake) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_POSITION_TURNER),
                new MoveArmAndRetractIntake(Config.armControlPanelPosition, intake, arm),
                new UpdateHandlingState(RobotContainer.RobotState.TURNER_IN_POSITION)
        );
    }
}
