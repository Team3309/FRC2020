package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {

    private final ArmSubsystem Arm;
    private final ArmSubsystem.ArmPosition Position;

    public MoveArmToPosition(ArmSubsystem.ArmPosition position, ArmSubsystem arm) {
        Position = position;
        Arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Arm.MoveToPosition(ArmSubsystem.ArmPosition.min);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
