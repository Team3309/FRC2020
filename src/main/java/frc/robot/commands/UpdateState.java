package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class UpdateState extends CommandBase {

    private final RobotContainer.PowerCellHandlingState savedState;

    public UpdateState(RobotContainer.PowerCellHandlingState stateToUpdateTo) {
        savedState = stateToUpdateTo;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.setPowerCellHandlingState(savedState);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}