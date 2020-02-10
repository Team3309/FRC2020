package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OperatorInterface;
import frc.robot.subsystems.ArmSubsystem;


public class ManualArmAdjustment extends CommandBase {

    private final ArmSubsystem arm;
    private final double adjustmentAmount;

    public ManualArmAdjustment(ArmSubsystem arm, double adjustmentAmount) {
        this.adjustmentAmount = adjustmentAmount;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }
}
