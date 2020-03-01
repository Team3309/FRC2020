package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCommandGroup;
import frc.robot.commands.groups.ToIntakeCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectCancelOuttake extends SelectCommand3309 {

    public SelectCancelOuttake(IntakeSubsystem intake, IndexerSubsystem indexer,
                               ShooterSubsystem shooter, ArmSubsystem arm,
                               DriveSubsystem drive, CtrlPanelSubsystem manipulator,
                               XboxController xboxController) {
        super (() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_OUTTAKE ||
                RobotContainer.getRobotState() == RobotContainer.RobotState.OUTTAKE
            ) {
                if (xboxController.getRawAxis(XboxController.Axis.kLeftTrigger.value) > Config.xBoxTriggerButtonThreshold) {
                    return new ToIntakeCommandGroup(intake, indexer, shooter, arm);

                } else {
                    return new ToDriveCommandGroup(Config.armPositionIntakeStowedTarget, intake, indexer, shooter, arm, drive, manipulator);

                }
            }else if (RobotContainer.getRobotState() == RobotContainer.RobotState.EMERGENCY_OUTTAKE
                    || RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_EMERGENCY_OUTTAKE
            ) {
                return new ToDriveCommandGroup(Config.armPositionIntakeStowedTarget, intake, indexer, shooter, arm, drive, manipulator);
            } else {
                return new DoNothing();
            }
        });
    }
}
