package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakeMotor extends CommandBase {
    private final IntakeSubsystem intake;

    public StartIntakeMotor(IntakeSubsystem intake) {

        this.intake = intake;
        addRequirements(intake);
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
