package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCmdGroup;
import frc.robot.commands.groups.ToOuttakeCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectCancelIntake extends SelectCommand3309 {
    public SelectCancelIntake(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter,
                              ArmSubsystem arm, DriveSubsystem drive, CtrlPanelSubsystem manipulator,
                              XboxController xboxController) {
        super(() -> {
            if (RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState()
            ) {
                if (xboxController.getRawAxis(XboxController.Axis.kRightTrigger.value) > Config.xBoxTriggerButtonThreshold) {
                    return new ToOuttakeCmdGroup(intake, shooter, arm);
                } else {
                    return new ToDriveCmdGroup(Config.armPositionIntakeStowedTarget, intake, shooter, arm);
                }
            } else {
                return new DoNothing();
            }
        });

    }

}
