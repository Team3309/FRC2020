package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;

import java.util.function.Supplier;

public class SelectScanToReadyToShoot extends SelectCommand3309 {

    public SelectScanToReadyToShoot(Supplier<Command> toRun) {
        super(() -> {
            if (RobotContainer.getPowerCellHandlingState() == RobotContainer.PowerCellHandlingState.SCAN) {
                return new DoNothing(); //Change to Command Group 4
            } else {
                return new DoNothing(); //
            }
        });
    }
}
