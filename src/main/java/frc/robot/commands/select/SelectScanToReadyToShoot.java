package frc.robot.commands.select;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;

import java.util.function.Supplier;

public class SelectScanToReadyToShoot extends SelectCommand3309 {

    public SelectScanToReadyToShoot(Supplier<Command> toRun) {
        super(() -> {
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.SCAN) {
                return new DoNothing();
            } else {
                return new DoNothing();
            }
        });
    }
}
