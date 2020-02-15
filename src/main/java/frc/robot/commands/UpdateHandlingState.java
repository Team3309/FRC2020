package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class UpdateHandlingState extends InstantCommand {

    private final RobotContainer.PowerCellHandlingState savedState;

    public UpdateHandlingState(RobotContainer.PowerCellHandlingState stateToUpdateTo) {
        savedState = stateToUpdateTo;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.println(savedState.name());
        RobotContainer.setPowerCellHandlingState(savedState);
    }



}
