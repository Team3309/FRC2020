package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ToScanCommandGroup extends SequentialCommandGroup {

    public ToScanCommandGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ArmSubsystem arm, VisionSubsystem vision) {
        addCommands(
                //give drive angle to change by to aim at target
        );
    }

}
