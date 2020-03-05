package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Config;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartIntake extends InstantCommand {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public StartIntake(IntakeSubsystem intake, ShooterSubsystem shooter) {

        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        intake.intake();
        shooter.setPowerRaw(-Config.shooterIntakePowerTopMotor, -Config.shooterIntakePowerBottomMotor);
    }

}
