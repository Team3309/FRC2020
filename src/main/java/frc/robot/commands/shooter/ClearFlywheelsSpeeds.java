package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ClearFlywheelsSpeeds extends InstantCommand {

    private final ShooterSubsystem shooter;

    public ClearFlywheelsSpeeds(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setDesiredSpeed(null, null);
    }
}
