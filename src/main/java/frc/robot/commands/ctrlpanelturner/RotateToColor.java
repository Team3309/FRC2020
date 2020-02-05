package frc.robot.commands.ctrlpanelturner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class RotateToColor extends CommandBase {

    private enum rotationDirection {
        clockwise,
        counterClockwise
    }

    private CtrlPanelSubsystem Manipulator;
    private rotationDirection Direction;
    private char TargetColor;

    private boolean Done = false;

    public RotateToColor (CtrlPanelSubsystem manipulator, rotationDirection direction, char color) {
        Manipulator = manipulator;
        Direction = direction;
        TargetColor = color;
    }

    @Override
    public void initialize() {
        addRequirements(Manipulator);
    }

    @Override
    public void execute() {
        char sensorColor = Manipulator.getColor();

        if (sensorColor == TargetColor) {
            Done = true;
        } else {
            if (Direction == rotationDirection.clockwise) {
                Manipulator.Rotate(ControlMode.PercentOutput, Constants.kTurnerRotationSpeed);
            } else {
                Manipulator.Rotate(ControlMode.PercentOutput, -Constants.kTurnerRotationSpeed);
            }
        }
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return Done;
    }
}
