package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StartFlywheels extends CommandBase {

    private final ShooterSubsystem shooter;

    public StartFlywheels(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.runFlywheelsAtPresetSpeeds();
    }

    @Override
    public boolean isFinished() {
        return shooter.areFlywheelsToSpeed();
    }
}
