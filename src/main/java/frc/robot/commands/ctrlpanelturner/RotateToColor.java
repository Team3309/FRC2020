package frc.robot.commands.ctrlpanelturner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.CtrlPanelSubsystem;

public class RotateToColor extends CommandBase {

    private enum rotationDirection {
        clockwise,
        counterClockwise
    }

    private CtrlPanelSubsystem Manipulator;
    private rotationDirection Direction;
    private CtrlPanelSubsystem.panelColor TargetColor;

    private boolean Done = false;

    public RotateToColor (CtrlPanelSubsystem manipulator, rotationDirection direction, CtrlPanelSubsystem.panelColor color) {
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
        CtrlPanelSubsystem.panelColor sensorColor = Manipulator.getColor();

        if (sensorColor == TargetColor) {
            Done = true;
        } else {
            if (Direction == rotationDirection.clockwise) {
                Manipulator.Rotate(ControlMode.PercentOutput, Config.TurnerRotationSpeed);
            } else {
                Manipulator.Rotate(ControlMode.PercentOutput, -Config.TurnerRotationSpeed);
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
