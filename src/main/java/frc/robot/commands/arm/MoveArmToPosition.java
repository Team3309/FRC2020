package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {

    private final ArmSubsystem Arm;
    private final ArmSubsystem.ArmPosition position;

    public MoveArmToPosition(ArmSubsystem.ArmPosition position, ArmSubsystem arm) {
        this.position = position;
        Arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    /**
     *
     * */
    @Override
    public void execute() {
        if(position != null) {
            Arm.MoveToPosition(position);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
