package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakeMotor extends InstantCommand {
    private final IntakeSubsystem intake;

    public StartIntakeMotor(IntakeSubsystem intake) {

        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
    }

}
