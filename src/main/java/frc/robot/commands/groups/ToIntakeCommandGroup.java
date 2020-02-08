package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.intake.EngageIntakeFlywheelAndIndexer;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pcindexer.StopIndexer;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//aka Command Group 1
//executes the following commands
//1) stop the the IIF (Intake, Indexer, Flywheel)
//2) Extend the Intake
//3) Move Arm down
//4) Engage the IIF
public class ToIntakeCommandGroup extends SequentialCommandGroup {

    public ToIntakeCommandGroup(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ArmSubsystem arm) {
        addCommands(new StopIndexer(indexer), new StopIntake(intake), new StopFlywheel(shooter), new Extend(intake), new MoveArmToPosition(ArmSubsystem.ArmPosition.min, arm), new EngageIntakeFlywheelAndIndexer(intake, indexer, shooter));
    }

}
