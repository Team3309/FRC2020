package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class UpdateHandlingState extends InstantCommand {

    private final RobotContainer.RobotState savedState;

    public UpdateHandlingState(RobotContainer.RobotState stateToUpdateTo) {
        savedState = stateToUpdateTo;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.setRobotState(savedState);
    }



}
