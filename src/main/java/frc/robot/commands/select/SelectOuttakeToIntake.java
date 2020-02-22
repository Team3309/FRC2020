package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.intake.StartIntakeMotor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectOuttakeToIntake extends SelectCommand3309 {

    public SelectOuttakeToIntake(IntakeSubsystem intake, ShooterSubsystem shooter) {
        super(() -> {
            if (RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState()) {
                return new StartIntakeMotor(intake, shooter);
            } else {
                return new DoNothing();
            }
        });
    }

}
