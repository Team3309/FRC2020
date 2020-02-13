package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToReadyToShootCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectMultishotToReadyToShoot extends SelectCommand {
    public SelectMultishotToReadyToShoot(IntakeSubsystem intake, IndexerSubsystem indexer,
                                         ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.INIT_MULTI_SHOT ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.MULTI_SHOT
            ) {
                return new ToReadyToShootCommandGroup(null, null, null, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }
}
