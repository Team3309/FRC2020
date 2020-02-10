package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import frc.robot.subsystems.ArmSubsystem;


public class ManualArmAdjustment extends CommandBase {

    private final ArmSubsystem arm;
    private final XboxController controller;

    public ManualArmAdjustment(ArmSubsystem arm, XboxController controller) {
        this.controller = controller;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
