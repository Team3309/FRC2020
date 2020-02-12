package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Config;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooterIntake extends InstantCommand {

    private final ShooterSubsystem shooter;

    public StartShooterIntake(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPowerRaw(Config.shooterIntakePowerTopMotor, Config.shooterIntakePowerBottomMotor);
    }
}
