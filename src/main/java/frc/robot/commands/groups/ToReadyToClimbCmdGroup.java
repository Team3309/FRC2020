package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndExtendIntake;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ToReadyToClimbCmdGroup extends SequentialCommandGroup {

    public ToReadyToClimbCmdGroup(ClimberSubsystem climber, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_READY_TO_CLIMB),
                new MoveArmAndExtendIntake(intake, arm),
                new ExtendRetractClimber(true, climber),
                new UpdateHandlingState(RobotContainer.RobotState.READY_TO_CLIMB)
        );
    }
}
