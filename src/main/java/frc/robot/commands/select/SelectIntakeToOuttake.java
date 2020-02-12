package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.intake.SwitchToOuttake;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectIntakeToOuttake extends SelectCommand {

    public SelectIntakeToOuttake(IntakeSubsystem intake) {
        super(() -> {
            if (RobotContainer.PowerCellHandlingState.INTAKE == RobotContainer.getPowerCellHandlingState()) {
                return new SwitchToOuttake(intake);
            } else {
                return new DoNothing();
            }
        });
    }
}
