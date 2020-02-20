package frc.robot.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class DeployTurner extends CommandBase {

    CtrlPanelSubsystem manipulator;

    public DeployTurner (CtrlPanelSubsystem manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        //Reset the variables associated with rotationControl
        manipulator.resetRotationControlCounter();
        manipulator.deploy();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
