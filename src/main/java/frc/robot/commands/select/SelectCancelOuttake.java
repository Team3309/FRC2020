package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCmdGroup;
import frc.robot.commands.groups.ToIntakeCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectCancelOuttake extends SelectCommand3309 {

    public SelectCancelOuttake(IntakeSubsystem intake, IndexerSubsystem indexer,
                               ShooterSubsystem shooter, ArmSubsystem arm,
                               XboxController xboxController) {
        super (() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_OUTTAKE ||
                RobotContainer.getRobotState() == RobotContainer.RobotState.OUTTAKE
            ) {
                if (xboxController.getRawAxis(XboxController.Axis.kLeftTrigger.value) > Config.xBoxTriggerButtonThreshold) {
                    return new ToIntakeCmdGroup(intake, indexer, shooter, arm);

                } else {
                    return new ToDriveCmdGroup(Config.armPositionIntakeStowedTarget, intake, shooter, arm);

                }
            } else {
                return new DoNothing();
            }
        });
    }
}
