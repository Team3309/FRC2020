package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.RaiseShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ProtectedRetract extends SequentialCommandGroup {
    public ProtectedRetract (IntakeSubsystem intake, ShooterSubsystem shooter) {
        addCommands(
                new RaiseShooter(),
                new Retract(intake)
        );
    }
}
