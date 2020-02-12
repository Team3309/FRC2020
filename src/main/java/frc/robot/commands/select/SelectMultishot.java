package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.MultiShotCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

public class SelectMultishot extends SelectCommand {


    public SelectMultishot(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT) {
                //return new MultiShotCommandGroup();
                return new MultiShotCommandGroup(shooter, indexer, intake, arm); //change to the multi shot command group when it is finished.
            } else {
                return new DoNothing();
            }

        });
    }
}
