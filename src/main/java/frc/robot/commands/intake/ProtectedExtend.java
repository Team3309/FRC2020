package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.LowerShooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ProtectedExtend extends SequentialCommandGroup {
    public ProtectedExtend (IntakeSubsystem intake, ShooterSubsystem shooter) {
        addCommands(
                new Extend(intake),
                new LowerShooter()
        );
    }
}
