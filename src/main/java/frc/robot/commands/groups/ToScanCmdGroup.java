package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.commands.vision.CreateFiringSolution;
import frc.robot.commands.vision.IlluminationOn;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ToScanCmdGroup extends SequentialCommandGroup {

    public ToScanCmdGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, VisionSubsystem vision, DriveSubsystem drive) {
        addCommands(
                // turn to target and prepare to shoot
                new UpdateHandlingState(RobotContainer.RobotState.INIT_SCAN),
                new IlluminationOn(vision),
                new StopIntake(intake),
                new StopFlywheels(shooter),
                new MoveArmAndRetractIntake(Config.armPositionVision, intake, arm),
                new UpdateHandlingState(RobotContainer.RobotState.SCAN),
                new CreateFiringSolution(vision, intake, indexer, shooter, arm, drive));
    }

}
