package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndExtendIntake;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ToReadyToClimbCommandGroup extends SequentialCommandGroup {

    public ToReadyToClimbCommandGroup(ClimberSubsystem climber, IntakeSubsystem intake, ArmSubsystem arm) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_READY_TO_CLIMB),
                new MoveArmAndExtendIntake(intake, arm, false),
                new ExtendRetractClimber(true, climber),
                new UpdateHandlingState(RobotContainer.RobotState.READY_TO_CLIMB)
        );
    }
}
