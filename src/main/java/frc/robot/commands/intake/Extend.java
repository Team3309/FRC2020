package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Extend extends CommandBase {

    private IntakeSubsystem Intake;

    public Extend (IntakeSubsystem intake) {
        Intake = intake;

        addRequirements(Intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Intake.Extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
