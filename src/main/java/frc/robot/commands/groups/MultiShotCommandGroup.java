package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.shooter.MultiShot;
import frc.robot.commands.shooter.StartFlywheel;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShotCommandGroup extends SequentialCommandGroup {

    public MultiShotCommandGroup(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        addCommands(
                new Retract(intake),
                new StartFlywheel(shooter),
                new MultiShot(indexer, shooter)
        );

    }



}
