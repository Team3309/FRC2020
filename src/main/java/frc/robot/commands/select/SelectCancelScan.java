package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

public class SelectCancelScan extends SelectCommand3309 {


    public SelectCancelScan(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem ctrl) {
        super(() -> {
            if(RobotContainer.getRobotState() == RobotContainer.RobotState.SCAN) {
                return new ToDriveCommandGroup(null, intake, indexer, shooter, arm, drive, ctrl);
            } else {
                return new DoNothing();
            }
        });
    }
}
