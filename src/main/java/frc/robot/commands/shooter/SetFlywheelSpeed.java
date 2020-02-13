package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetFlywheelSpeed extends InstantCommand {

    private final ShooterSubsystem shooter;
    private Double speedTop;
    private Double speedBottom;

    public SetFlywheelSpeed(ShooterSubsystem shooter, Double speedTop, Double speedBottom) {
        this.shooter = shooter;
        this.speedTop = speedTop;
        this.speedBottom = speedBottom;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (speedTop != null && speedBottom != null) {
            shooter.setDesiredSpeed(speedTop, speedBottom);
        }
    }

}
