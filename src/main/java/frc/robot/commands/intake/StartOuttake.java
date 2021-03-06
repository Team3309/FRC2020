package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartOuttake extends InstantCommand {

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public StartOuttake(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        shooter.stopFlywheels();
        intake.outtake();
    }
}
