package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToIntake extends SelectCommand3309 {

    public SelectToIntake(IntakeSubsystem intake, IndexerSubsystem indexer,
                          ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if ((RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TRENCH_DRIVE) ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION
            ) {
                return new ToIntakeCommandGroup(intake, indexer, shooter, arm); //aka command group 1 (See Slack for details)
            }  else {
                //do nothing
                return new DoNothing();
            }
        });
    }

}
