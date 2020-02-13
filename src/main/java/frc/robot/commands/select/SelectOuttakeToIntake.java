package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.intake.StartIntakeMotor;
import frc.robot.subsystems.IntakeSubsystem;

public class SelectOuttakeToIntake extends SelectCommand3309 {

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
