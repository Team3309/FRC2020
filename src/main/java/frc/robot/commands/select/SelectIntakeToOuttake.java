package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.intake.SwitchToOuttake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectIntakeToOuttake extends SelectCommand3309 {

    public SelectIntakeToOuttake(IntakeSubsystem intake, ShooterSubsystem shooter) {
        super(() -> {
            if (RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState()) {
                return new SwitchToOuttake(intake, shooter);
            } else {
                return new DoNothing();
            }
        });
    }


}
