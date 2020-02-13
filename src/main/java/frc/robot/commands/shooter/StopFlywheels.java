package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class StopFlywheels extends InstantCommand {
    private ShooterSubsystem shooter;

    public StopFlywheels(ShooterSubsystem shooter) {
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

}
