package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.intake.StartIntakeMotor;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class SelectOuttakeToIntake extends SelectCommand {

    public SelectOuttakeToIntake(IntakeSubsystem intake) {
        super(() -> {
            if (RobotContainer.PowerCellHandlingState.INTAKE == RobotContainer.getPowerCellHandlingState()) {
                return new StartIntakeMotor(intake);
            } else {
                return new DoNothing();
            }
        });
    }
}
