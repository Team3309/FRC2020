package frc.robot.commands.select;


import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.ToOuttakeCommandGroup;
import frc.robot.subsystems.*;

public class SelectToOuttake extends SelectCommand3309 {

    public SelectToOuttake(IntakeSubsystem intake, IndexerSubsystem indexer,
                           ShooterSubsystem shooter, ArmSubsystem arm,
                           DriveSubsystem drive, CtrlPanelSubsystem manipulator) {
        super(() -> {
            if ((RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TRENCH_DRIVE) ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE
            ) {
                return new ToOuttakeCommandGroup(0, intake, indexer, shooter, arm, drive, manipulator);
            } else {
                return new DoNothing();
            }});
    }
}
