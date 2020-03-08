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

public class SelectCancelScan extends SelectCommand3309 {


    public SelectCancelScan(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem ctrl) {
        super(() -> {
            if(RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_SCAN ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.SCAN) {
                return new ToDriveCmdGroup(null, intake, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }
}
