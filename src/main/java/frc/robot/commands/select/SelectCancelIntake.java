package frc.robot.commands.select;

import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectCancelIntake extends SelectCommand3309 {
    public SelectCancelIntake(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState()) {
                return new ToDriveCommandGroup(Config.armPositionIntakeStowedTarget, intake, indexer, shooter, arm);
            } else {
                return new DoNothing();
            }
        });

    }

}
