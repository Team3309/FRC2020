package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.commands.drive.DriveStraight;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.commands.shooter.SingleShot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ThreeBallAutoDriveForward extends SequentialCommandGroup {
    public ThreeBallAutoDriveForward(IndexerSubsystem indexer, ShooterSubsystem shooter, DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
                new ToReadyToShootCommandGroup(Config.shooterMidRangeSolution, intake, indexer, shooter, arm),
                new SingleShot(indexer, shooter),
                new SingleShot(indexer, shooter),
                new SingleShot(indexer, shooter),
                new SingleShot(indexer, shooter),
                new DriveStraight(12, drive)
        );
    }
}
