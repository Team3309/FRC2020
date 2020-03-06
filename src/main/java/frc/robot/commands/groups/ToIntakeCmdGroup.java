package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndExtendIntake;
import frc.robot.commands.indexer.SetIndexerSpeed;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//aka Command Group 1
//executes the following commands
//1) stop the the IIF (Intake, Indexer, Flywheel)
//2) Extend the Intake
//3) Move Arm down
//4) Engage the IIF


public class ToIntakeCmdGroup extends SequentialCommandGroup {

    public ToIntakeCmdGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_INTAKE),
                new StopIntake(intake),
                new StopFlywheels(shooter),
                new SetIndexerSpeed(indexer, Config.indexInSpeed),
                new MoveArmAndExtendIntake(intake, arm),
                new StartIntake(intake, shooter),
                new UpdateHandlingState(RobotContainer.RobotState.INTAKE)
        );
    }

}
