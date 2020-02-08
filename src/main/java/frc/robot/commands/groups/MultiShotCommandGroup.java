package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.pcindexer.LoadBall;
import frc.robot.commands.shooter.PrepareToFire;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShotCommandGroup extends SequentialCommandGroup {

    public MultiShotCommandGroup(ArmSubsystem arm, ShooterSubsystem shooter,
                                 IndexerSubsystem indexer, ArmSubsystem.ArmPosition position,
                                 Timer ctrlTimer, IntakeSubsystem intake) {

        addCommands(
                new Retract(intake),
                new MoveArmToPosition(position, arm),
                new PrepareToFire(shooter, ctrlTimer),
                new LoadBall(indexer)
        );

    }

    public MultiShotCommandGroup() {

    }


}
