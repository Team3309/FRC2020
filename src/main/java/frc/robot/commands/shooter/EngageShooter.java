package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class EngageShooter extends CommandBase {
    private final ShooterSubsystem shooter;

    public EngageShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.spinUpFlywheels();
        addRequirements(shooter);
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
