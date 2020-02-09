package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectSingleShot extends SelectCommand {
    public SelectSingleShot(IntakeSubsystem intake, IndexerSubsystem indexer,
                            ShooterSubsystem shooter) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT) {
                return new DoNothing(); //Change to Command Group 5
            } else {
                return new DoNothing(); //
            }
        });
    }
}
