package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class EngageIntake extends CommandBase {
    private final IntakeSubsystem intake;

    public EngageIntake(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.intake();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
