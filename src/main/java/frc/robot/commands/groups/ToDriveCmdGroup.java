package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.ClearFlywheelsSpeeds;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToDriveCmdGroup extends SequentialCommandGroup {

    public ToDriveCmdGroup(Integer position, IntakeSubsystem intake, IndexerSubsystem indexer,
                           ShooterSubsystem shooter, ArmSubsystem arm) {
        super(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_ARM_UP_DRIVE),
                new StopIntake(intake),
                new InstantCommand(indexer::reset, indexer),
                new StopFlywheels(shooter),
                position == null ? new DoNothing() : new MoveArmAndRetractIntake(position, intake, arm),
                new UpdateHandlingState(RobotContainer.RobotState.ARM_UP_DRIVE)
        );
    }
}
