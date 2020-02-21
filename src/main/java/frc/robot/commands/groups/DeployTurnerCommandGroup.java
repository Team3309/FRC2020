package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.select.SelectDeployTurner;
import frc.robot.subsystems.*;

public class DeployTurnerCommandGroup extends SequentialCommandGroup {

    public DeployTurnerCommandGroup(ArmSubsystem arm, IntakeSubsystem intake, CtrlPanelSubsystem manipulator) {
        addCommands(
                new MoveArmAndRetractIntake(Config.armControlPanelPosition, intake, arm),
                new SelectDeployTurner(arm, intake, manipulator)
        );

    }
}
