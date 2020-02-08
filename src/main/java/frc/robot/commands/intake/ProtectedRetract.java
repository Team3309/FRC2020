package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ProtectedRetract extends SequentialCommandGroup {
    public ProtectedRetract (IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new MoveArmToPosition(ArmSubsystem.ArmPosition.min, arm),
                new Retract(intake)
        );
    }
}
