package frc.robot.commands.select;

import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.*;

public class SelectCancelOuttake extends SelectCommand3309 {

    public SelectCancelOuttake(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem manipulator) {
        super (() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_OUTTAKE ||
                RobotContainer.getRobotState() == RobotContainer.RobotState.OUTTAKE
            ) {
                return new ToDriveCommandGroup(Config.armPositionIntakeStowedTarget, intake, indexer, shooter, arm, drive, manipulator);
            } else {
                return new DoNothing();
            }
        });
    }
}
