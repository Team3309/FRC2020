package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RaiseArm extends CommandBase {

    private ArmSubsystem Arm;
    private XboxController Controller;

    public RaiseArm(ArmSubsystem arm, XboxController controller) {
        Arm = arm;
        Controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Arm.adjustArm(Controller.getY(GenericHID.Hand.kRight));

    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

