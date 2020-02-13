package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.SingleShotCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToSingleShot extends SelectCommand {
    public SelectToSingleShot(IntakeSubsystem intake, IndexerSubsystem indexer,
                              ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT) {
                return new SingleShotCommandGroup(shooter, indexer, intake, arm); //Change to Command Group 5
            } else {
                return new DoNothing(); //
            }
        });
    }

    public boolean isFinished() {
        return true;
    }
}
