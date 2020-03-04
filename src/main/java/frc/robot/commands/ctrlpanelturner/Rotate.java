package frc.robot.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class Rotate extends CommandBase {

    CtrlPanelSubsystem manipulator;

    public Rotate (CtrlPanelSubsystem manipulator) {
        this.manipulator = manipulator;
    }

    //Why is the command's action in the isFinished block?
    @Override
    public boolean isFinished() {
        return manipulator.spin();
    }
}
