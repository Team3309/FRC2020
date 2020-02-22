package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.FiringSolution;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.indexer.ManageIndexer;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.commands.vision.CreateFiringSolution;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ToScanCommandGroup extends SequentialCommandGroup {

    public ToScanCommandGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, VisionSubsystem vision) {
        addCommands(
                //give drive angle to change by to aim at target
                new StopIntake(intake),
                new StopFlywheels(shooter),
                new MoveArmAndRetractIntake(Config.armPositionVision, intake, arm),
                new CreateFiringSolution(vision, intake, indexer, shooter, arm));

    }

}
