package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndExtendIntake;
import frc.robot.commands.intake.StartOuttakeMotor;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToEmergencyOuttakeCommandGroup extends SequentialCommandGroup {
    public ToEmergencyOuttakeCommandGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem manipulator) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_EMERGENCY_OUTTAKE),
                new StopIntake(intake),
                new StopFlywheels(shooter),
                new MoveArmAndExtendIntake(intake, arm, true),
                new StartOuttakeMotor(intake, shooter),
                new UpdateHandlingState(RobotContainer.RobotState.EMERGENCY_OUTTAKE));
    }
}
