package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.MultiShotCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectToMultishot extends SelectCommand3309 {


    public SelectToMultishot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        super(() -> {
            if (    (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.ARM_UP_DRIVE &&
                    shooter.hasPresetSpeeds()) ||
                    RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT) {
                return new MultiShotCommandGroup(shooter, indexer);
            } else {
                return new DoNothing();
            }

        });
    }

}
