package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveArmAndExtendIntake extends CommandBase {

    private final ArmSubsystem.ArmPosition position;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;

    public MoveArmAndExtendIntake(ArmSubsystem.ArmPosition position, IntakeSubsystem intake, ArmSubsystem arm) {
        this.position = position;
        this.intake = intake;
        this.arm = arm;
    }
    //unlike movearmandretractintake, the order is reversed here.
    public void initialize() {
        intake.extend();
    }

    public void execute() {
        if (intake.isPistonTravelComplete()) {
            if (position != null) {
                arm.moveToPosition(position);
            } //we don't check for being above the intake stowed limit like movearmandretract because the piston extends first.
        }
    }

    public boolean isFinished() {
        return arm.isInPosition() && intake.isPistonTravelComplete();
    }
}
