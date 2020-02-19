package frc.robot.commands.select;

import frc.robot.FiringSolution;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToReadyToShoot extends SelectCommand3309 {

    public SelectToReadyToShoot(FiringSolution firingSolution, IntakeSubsystem intake,
                                IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_TRENCH_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_SCAN ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.SCAN ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_INTAKE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INTAKE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT
            ) {
                return new ToReadyToShootCommandGroup(firingSolution, intake, indexer, shooter, arm);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }

}
