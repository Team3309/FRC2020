package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends CommandBase {

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    public RetractIntake(IntakeSubsystem intake, ArmSubsystem arm) {
        this.intake = intake;
        this.arm = arm;
        addRequirements(intake, arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (arm.isArmAboveIntakeMinimum()) {
            intake.retract();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
