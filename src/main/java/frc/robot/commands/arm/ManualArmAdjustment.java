package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ManualArmAdjustment extends CommandBase {


    private static final double DEAD_ZONE = 0.03;
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
        double yRaw = controller.getY(GenericHID.Hand.kRight);

        if (yRaw > DEAD_ZONE) {
            arm.adjustArm(yRaw * yRaw * yRaw);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
