package frc.robot.commands.ctrlpanelturner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.CtrlPanelSubsystem;
import frc.robot.util.PanelColor;

public class RotateToColor extends CommandBase {

    private enum rotationDirection {
        clockwise,
        counterClockwise
    }

    private CtrlPanelSubsystem Manipulator;
    private rotationDirection Direction;
    private PanelColor TargetColor;

    private boolean Done = false;

    public RotateToColor (CtrlPanelSubsystem manipulator, rotationDirection direction, PanelColor color) {
        Manipulator = manipulator;
        Direction = direction;
        TargetColor = color;
        addRequirements(manipulator);

        switch (color) {
            case red :
                TargetColor = PanelColor.cyan;
                break;
            case yellow:
                TargetColor = PanelColor.green;
                break;
            case cyan :
                TargetColor = PanelColor.red;
                break;
            case green:
                TargetColor = PanelColor.yellow;
                break;
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        PanelColor sensorColor = Manipulator.getColor();

        if (sensorColor == TargetColor) {
            Done = true;
        } else {
            if (Direction == rotationDirection.clockwise) {
                Manipulator.Rotate(ControlMode.PercentOutput, Config.TurnerRotationPower);
            } else {
                Manipulator.Rotate(ControlMode.PercentOutput, -Config.TurnerRotationPower);
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
