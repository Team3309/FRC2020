package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ctrlpanelturner.DeployTurner;
import frc.robot.commands.ctrlpanelturner.Rotate;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class DeployControlPanelTurnerCommandGroup extends SequentialCommandGroup {
    public DeployControlPanelTurnerCommandGroup(CtrlPanelSubsystem subsystem) {
        addCommands(new DeployTurner(subsystem),
                    new Rotate(subsystem));

    }
}
