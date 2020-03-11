package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Config;
import frc.robot.commands.drive.DriveApplyPower;
import frc.robot.commands.groups.ToReadyToShootCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ThreeBallAutoDriveForward extends SequentialCommandGroup {
    public ThreeBallAutoDriveForward(IndexerSubsystem indexer, ShooterSubsystem shooter, DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
                new ToReadyToShootCmdGroup(Config.shooterMidRangeSolution, intake, indexer, shooter, arm),
                new ParallelCommandGroup(
                        new WaitCommand(3.0),
                        new InstantCommand(indexer::velocityShooting, indexer)
                ),
                new InstantCommand(indexer::reset, indexer),
                new InstantCommand(intake::extend, intake),  // drop the ball guard before the start of teleop

                // Super cheesy way to get across the line because DriveStraight isn't working right yet.
                new DriveApplyPower(0.2, drive),  // will be cancelled by DriveManual resuming at end of command group
                new WaitCommand(2.0)
        );
    }
}
