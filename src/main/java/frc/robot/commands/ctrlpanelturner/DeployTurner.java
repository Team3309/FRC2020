package frc.robot.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class DeployTurner extends CommandBase {

    //TODO: Add a timer
    CtrlPanelSubsystem manipulator;

    public DeployTurner (CtrlPanelSubsystem manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        //Reset the variables associated with rotationControl
        manipulator.resetRotationControlCounter(); //TODO: Remove
    }

    @Override
    public void execute() {
        manipulator.deploy(); //TODO: Move to initialize
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
