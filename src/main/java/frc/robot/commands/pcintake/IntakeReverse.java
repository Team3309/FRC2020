package frc.robot.commands.pcintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Config.IntakeInwardPower;

public class IntakeReverse extends CommandBase {

    private IntakeSubsystem Intake;
    private boolean isDone;

    public IntakeReverse(IntakeSubsystem intake) {
        Intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Intake.Outtake();
        //if indexer power cell count has been decremented to desired power cell count, isDone = true;
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
