package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.util.PanelColor;

/**
 * @author Joshua Badzey
 *
 * The class for the control panel manipulator mechanism, which will turn the control panel a desired
 * number of times, turn it to a specific color, and receive a color from the FMS. Will work with
 * FMS to determine what color is needed for position control.
 *
 */

public class CtrlPanelSubsystem extends SubsystemBase {

    public enum turnerState {
        nothing,
        turningToRotations,
        turningToColor
    }

    private Solenoid retractorPiston;
    private Solenoid heightAdjustmentPiston;

    private WPI_TalonFX ctrlPanelMotor;

    private ColorSensorV3 colorSensor;

    public CtrlPanelSubsystem() {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        ctrlPanelMotor = new WPI_TalonFX(Config.TurnerMotorID);
        ctrlPanelMotor.configFactoryDefault();
        retractorPiston = new Solenoid(Config.TurnerTractorPistonPdpChannel);
        heightAdjustmentPiston = new Solenoid(Config.TurnerHeightAdjustmentPistonID);
    }

    public void Rotate (ControlMode mode, double value) {
        ctrlPanelMotor.set(mode, value);
    }

    public PanelColor getColor () {
        Color color = colorSensor.getColor();
        if (color.equals(Color.kRed)) {
            return PanelColor.red;
        }
        else if (color.equals(Color.kYellow)) {
            return PanelColor.yellow;
        }
        else if (color.equals(Color.kGreen)) {
            return PanelColor.green;
        }
        else if (color.equals(Color.kCyan)) {
            return PanelColor.cyan;
        }
        else {
            return PanelColor.unknown;
        }
    }

    public void Engage() {
    }

    public void Disengage() {
    }

    public void RaiseTurner() {
    }

    public PanelColor GetFMSColor() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    return PanelColor.cyan;
                case 'G' :
                    return PanelColor.green;
                case 'R' :
                    return PanelColor.red;
                case 'Y' :
                    return PanelColor.yellow;
                default :
                    DriverStation.reportError("Corrupt FMS Value!", true);
                    return PanelColor.unknown;
            }
        } else {
            return PanelColor.noValue;
        }
    }

    public boolean IsFMSColorAvalible() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        return gameData.length() > 0;
    }

    public void DeployTurner() {
        retractorPiston.set(true);
    }

    public void RetractTurner() {
        retractorPiston.set(false);
    }
}
