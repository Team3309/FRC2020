package frc.robot.commands.select;

import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.*;

public class SelectCancelIntake extends SelectCommand3309 {
    public SelectCancelIntake(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem manipulator) {
        super(() -> {
            if (RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INIT_OUTTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.OUTTAKE == RobotContainer.getRobotState()) {
                return new ToDriveCommandGroup(Config.armPositionIntakeStowedTarget, intake, indexer, shooter, arm, drive, manipulator);
            } else {
                return new DoNothing();
            }
        });

    }

}
