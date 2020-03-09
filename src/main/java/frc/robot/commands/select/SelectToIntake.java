package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToIntakeCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToIntake extends SelectCommand3309 {

    public SelectToIntake(IntakeSubsystem intake, IndexerSubsystem indexer,
                          ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_OUTTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.OUTTAKE
            ) {
                return new ToIntakeCmdGroup(intake, indexer, shooter, arm); //aka command group 1 (See Slack for details)
            }  else {
                return new DoNothing();
            }
        });
    }
}
