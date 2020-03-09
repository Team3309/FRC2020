package frc.robot.commands.select;

import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToDriveCmdGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectReadyToShootToDriving extends SelectCommand3309 {

    public SelectReadyToShootToDriving(IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
        super(() -> {
            if (RobotContainer.RobotState.INIT_READY_TO_SHOOT == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.READY_TO_SHOOT == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INIT_ARM_UP_DRIVE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.ARM_UP_DRIVE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INIT_OUTTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.OUTTAKE == RobotContainer.getRobotState()) {
                return new ToDriveCmdGroup(Config.armPositionIntakeStowedTarget, intake, shooter, arm);
            } else {
                return new DoNothing();
            }
        });
    }

}
