package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FiringSolution;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.UpdateHandlingState;
import frc.robot.commands.arm.MoveArmAndRetractIntake;
import frc.robot.commands.indexer.SetIndexerSpeed;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.SetFlywheelsSpeed;
import frc.robot.commands.shooter.StartFlywheels;
import frc.robot.commands.shooter.StopFlywheels;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToReadyToShootCmdGroup extends SequentialCommandGroup {
    public ToReadyToShootCmdGroup(FiringSolution firingSolution,
                                  IntakeSubsystem intake, IndexerSubsystem indexer,
                                  ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(
                new UpdateHandlingState(RobotContainer.RobotState.INIT_READY_TO_SHOOT),
                /*we only want to stop the flywheels if we are moving the arm. because of the shooter shaking the system*/
                firingSolution == null ? new DoNothing() : new StopFlywheels(shooter),
                new StopIntake(intake),
                firingSolution == null ? new DoNothing() :
                        new MoveArmAndRetractIntake(firingSolution.getArmPosition(), intake, arm),
                firingSolution == null ? new DoNothing() :
                        new SetFlywheelsSpeed(shooter, firingSolution.getTopFlywheelSpeed(), firingSolution.getBottomFlywheelSpeed()),
                firingSolution == null ? new DoNothing() :
                        new SetIndexerSpeed(indexer, firingSolution.getIndexerSpeed()),
                new StartFlywheels(shooter),
                new UpdateHandlingState(RobotContainer.RobotState.READY_TO_SHOOT)
        );
    }
}
