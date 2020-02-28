package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.commands.groups.ToOuttakeCommandGroup;
import frc.robot.subsystems.*;

public class SelectCancelIntake extends SelectCommand3309 {
    public SelectCancelIntake(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter,
                              ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem manipulator,
                              XboxController xboxController) {
        super(() -> {
            if (RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState()
            ) {
                if (xboxController.getRawAxis(XboxController.Axis.kRightTrigger.value) > Config.xBoxTriggerButtonThreshold) {
                    return new ToOuttakeCommandGroup(intake, indexer, shooter, arm, drive, manipulator);
                } else {
                    return new ToDriveCommandGroup(Config.armPositionIntakeStowedTarget, intake, indexer, shooter, arm, drive, manipulator);
                }
            } else {
                return new DoNothing();
            }
        });

    }

}
