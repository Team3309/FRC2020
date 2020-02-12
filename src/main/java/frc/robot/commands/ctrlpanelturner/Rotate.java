package frc.robot.commands.ctrlpanelturner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class Rotate extends CommandBase {

    CtrlPanelSubsystem manipulator;

    public Rotate (CtrlPanelSubsystem manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        manipulator.spin();
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
