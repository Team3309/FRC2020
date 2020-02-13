package frc.robot.commands.select;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.arm.ManualArmAdjustment;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SelectManualArmAdjustment extends SelectCommand {
    public SelectManualArmAdjustment(IntakeSubsystem intake, IndexerSubsystem indexer,
                                     ShooterSubsystem shooter, ArmSubsystem arm, XboxController controller) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.READY_TO_SHOOT) {
                return new ManualArmAdjustment(arm, controller); //Change to Command Group 6
            } else {
                return new DoNothing(); //
            }
        });
    }

}
