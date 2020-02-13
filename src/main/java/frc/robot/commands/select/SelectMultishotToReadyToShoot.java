package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
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
                return new ToReadyToShootCommandGroup(ArmSubsystem.ArmPosition.max, Double.valueOf(0), Double.valueOf(0), intake, indexer, shooter, arm);
            } else {
                //do nothing
                return new DoNothing();
            }
        });
    }
}
