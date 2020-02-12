package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendIntake extends CommandBase {

    private IntakeSubsystem intake;

    public ExtendIntake(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(this.intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.extend();
    }

    @Override
    public boolean isFinished() {
        return !intake.isSolenoidSwappingStates();
    }
}
