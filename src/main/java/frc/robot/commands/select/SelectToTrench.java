package frc.robot.commands.select;

import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.commands.groups.ToOuttakeCommandGroup;
import frc.robot.subsystems.*;

public class SelectToTrench extends SelectCommand3309 {
    public SelectToTrench(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem manipulator) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION
            ) {
                return new ToOuttakeCommandGroup(intake, indexer, shooter, arm, drive, manipulator);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
