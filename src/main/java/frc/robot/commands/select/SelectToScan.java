package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToScanCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SelectToScan extends SelectCommand3309 {

    public SelectToScan(IntakeSubsystem intake, IndexerSubsystem indexer,
                        ShooterSubsystem shooter, ArmSubsystem arm, VisionSubsystem vision, DriveSubsystem drive) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TRENCH_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION
            ) {
                return new ToScanCommandGroup(intake, indexer, shooter, arm, vision, drive);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }

}
