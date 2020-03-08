package frc.robot.commands.select;

import frc.robot.FiringSolution;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToReadyToShoot extends SelectCommand3309 {

    public SelectToReadyToShoot(FiringSolution firingSolution, IntakeSubsystem intake,
                                IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_SCAN ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.SCAN ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION
            ) {
                return new ToReadyToShootCmdGroup(firingSolution, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }

}
