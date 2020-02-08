package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

public class SelectMultishot extends SelectCommand {


    public SelectMultishot(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {
        super(() -> {
            return new DoNothing();
        });
    }
}
