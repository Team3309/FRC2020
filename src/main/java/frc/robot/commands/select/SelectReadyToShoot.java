package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectReadyToShoot extends SelectCommand {

    public SelectReadyToShoot(ArmSubsystem.ArmPosition position, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INTAKE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.SCAN
            ) {
                return new DoNothing(); //TODO execute command group 4 instead (see slack for command group descriptions).
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
