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
import frc.robot.RobotContainer;
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

    private RobotContainer robotContainer;

    private Solenoid retractorPiston;
    private Solenoid heightAdjustmentPiston;
    private WPI_TalonFX ctrlPanelMotor;
    private ColorSensorV3 colorSensor;

    public CtrlPanelSubsystem() {
        if (Config.isCtrlPanelInstalled) {
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            ctrlPanelMotor = new WPI_TalonFX(Config.TurnerMotorID);
            ctrlPanelMotor.configFactoryDefault();
            retractorPiston = new Solenoid(Config.TurnerTractorPistonID);
            heightAdjustmentPiston = new Solenoid(Config.TurnerHeightAdjustmentPistonID);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Activates the control panel motor for turning the control panel. Numerical sign for each direction will be
     * decided later.
     *
     * @param mode - the ControlMode in which the motor is to operate.
     * @param value - how much the motor is to turn.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public void Rotate (ControlMode mode, double value) {
        ctrlPanelMotor.set(mode, value);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Returns the color that the color sensor is currently on. This, along with established control panel color
     * sequence, will enable the robot to turn the control panel to the correct color.
     *
     * @return PanelColor.[color] - the color which the color sensor is currently on.
     *
     -----------------------------------------------------------------------------------------------------------------*/
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

    /**
     * Let's put this method up for discussion. @JoshB doesn't know what it's doing.
     * */
    public void Engage() {
    }

    /**
     * Let's put this method up for discussion. @JoshB doesn't know what it's doing.
     * */
    public void Disengage() {
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Raises the control panel turner for correct positioning above the control panel.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public void RaiseTurner() {
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Obtains via the FMS the color to which the robot must turn.
     *
     * @return PanelColor.[color] - the color to which the robot must turn the control panel.
     -----------------------------------------------------------------------------------------------------------------*/
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

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks whether the color for position control is available via the FMS.
     *
     * @return Whether the FMS color is currently available.
     -----------------------------------------------------------------------------------------------------------------*/
    public boolean IsFMSColorAvailable() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        return gameData.length() > 0;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Extends the control panel turner forward.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public void DeployTurner() {
        retractorPiston.set(true);
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Retracts the control panel turner back.
     *
     -----------------------------------------------------------------------------------------------------------------*/
    public void RetractTurner() {
        retractorPiston.set(false);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }

}
