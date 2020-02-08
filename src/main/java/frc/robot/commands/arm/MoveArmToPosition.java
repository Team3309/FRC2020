package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {

    private final ArmSubsystem arm;
    private final ArmSubsystem.ArmPosition position;

    public MoveArmToPosition(ArmSubsystem.ArmPosition position, ArmSubsystem arm) {
        this.position = position;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.MoveToPosition(ArmSubsystem.ArmPosition.min);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
