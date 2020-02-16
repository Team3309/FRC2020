package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//An optimization that allows you to retract the intake and move the arm at the same time without them colliding,
//saving time in the competition
public class MoveArmAndRetractIntake extends CommandBase {


    private final ArmSubsystem.ArmPosition position;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;

    public MoveArmAndRetractIntake(ArmSubsystem.ArmPosition position, IntakeSubsystem intake, ArmSubsystem arm) {
        this.position = position;
        this.intake = intake;
        this.arm = arm;
    }

    public void initialize() {
        if (position != null) {
            arm.moveToPosition(position);
        } else {
            if (!arm.isArmAboveIntakeMinimum()) {
                arm.moveToPosition(ArmSubsystem.ArmPosition.intakeStowedLimit);
            } //else do nothing.
        }
    }

    public void execute() {

    }

    public boolean isFinished() {
        boolean isInPosition = arm.isInPosition(); //stops rare race condition
        if (isInPosition) intake.retract();
        return isInPosition;
    }
}
