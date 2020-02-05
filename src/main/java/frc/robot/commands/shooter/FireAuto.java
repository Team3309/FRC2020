package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FireAuto extends CommandBase {

    private ShooterSubsystem Shooter;

    public FireAuto (ShooterSubsystem shooter) {
        Shooter = shooter;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Shooter.SpinUpFlywheels(0);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
