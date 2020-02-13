package frc.robot.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class RetractTurner extends CommandBase{
    //TODO: Same as DeployTurner

    CtrlPanelSubsystem manipulator;

    public RetractTurner (CtrlPanelSubsystem manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        manipulator.retract();
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
