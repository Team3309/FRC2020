package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.UpdateState;
import frc.robot.commands.groups.MultiShotCommandGroup;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectReadyToShoot extends SelectCommand {

    public SelectReadyToShoot(ArmSubsystem.ArmPosition position, Double speed, IntakeSubsystem intake,
                              IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.TRENCH_DRIVE ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.SCAN
            ) {
                return new ToReadyToShootCommandGroup(position, speed, intake, indexer, shooter, arm);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
