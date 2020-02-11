package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopFlywheel extends CommandBase {
    private ShooterSubsystem shooter;

    public StopFlywheel(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        shooter.stopFlywheels();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
