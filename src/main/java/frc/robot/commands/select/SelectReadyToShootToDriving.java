package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectReadyToShootToDriving extends SelectCommand3309 {

    public SelectReadyToShootToDriving(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive,CtrlPanelSubsystem manipulator) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT
            ) {
                return new ToDriveCmdGroup(null, intake, indexer, shooter, arm, drive,manipulator);
            } else {
                return new DoNothing();
            }
        });
    }

}
