package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ProtectedExtend extends SequentialCommandGroup {
    public ProtectedExtend (IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new Extend(intake),
                new MoveArmToPosition(ArmSubsystem.ArmPosition.min, arm)
        );
    }
}
