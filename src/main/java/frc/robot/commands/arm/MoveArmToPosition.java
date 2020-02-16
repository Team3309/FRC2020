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
        if (position != null) {
            Arm.moveToPosition(position);
        } //this code is in initialize because we want the command to not be restarted once it has been started.
    }

    /**
     *
     * */
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return Arm.isInPosition();
    }
}
