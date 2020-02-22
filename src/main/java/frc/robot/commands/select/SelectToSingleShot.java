package frc.robot.commands.select;

import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.groups.SingleShotCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**---------------------------------------------------------------------------------------------------------------------
 * Command for selecting whether to perform a Single Shot (one power cell).
 * Precondition: The robot should either be already ready to shoot or should be in arm-up driving, one of the states
 * from which the robot can perform a SingleShot.
 *
 */
public class SelectToSingleShot extends SelectCommand3309 {

    /**-----------------------------------------------------------------------------------------------------------------
     * Constructs a new SelectToSingleShot selector command. Checks that the robot's power cell handling state is either
     * "arm-up driving" or "ready to shoot" and executes the command group for Single Shot; otherwise it does nothing.
     *
     * @param indexer The indexer which is used to move power cells into the shooter for shooting.
     * @param shooter The shooter which will shoot the power cells.
     *
     * @return A Single Shot command group if the robot is in the correct states.
     * @return A Do Nothing command otherwise.
     *
     */
    public SelectToSingleShot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        super(() -> {
            if (((RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE
                    && shooter.hasPresetSpeeds())
                    || RobotContainer.getRobotState()
                        == RobotContainer.RobotState.READY_TO_SHOOT)) {
                return new SingleShotCommandGroup(shooter, indexer); //Change to Command Group 5
            } else {
                return new DoNothing(); //
            }
        });
    }
}
