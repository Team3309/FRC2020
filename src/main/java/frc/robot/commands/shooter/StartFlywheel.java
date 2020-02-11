package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StartFlywheel extends CommandBase {


    private final ShooterSubsystem shooter;

    public StartFlywheel(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.spinUpFlywheels();
    }
}
