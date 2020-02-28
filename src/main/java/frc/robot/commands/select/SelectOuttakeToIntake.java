package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectOuttakeToIntake extends SelectCommand3309 {

    public SelectOuttakeToIntake(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.OUTTAKE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.INIT_TRENCH_DRIVE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.TRENCH_DRIVE == RobotContainer.getRobotState()) {
                return new ToIntakeCommandGroup(intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }

}
