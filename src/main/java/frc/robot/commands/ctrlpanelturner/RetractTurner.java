package frc.robot.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class RetractTurner extends CommandBase{

    CtrlPanelSubsystem manipulator;

    public RetractTurner (CtrlPanelSubsystem manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        manipulator.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
