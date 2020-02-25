package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.util.PanelColor;

/**
 * @author Joshua Badzey & Mark Ghebrial
 *
 * The class for the control panel manipulator mechanism, which will turn the control panel a desired
 * number of times, turn it to a specific color, and receive a color from the FMS. Works with
 * FMS to determine what color is needed for position control.
 */

public class CtrlPanelSubsystem extends SubsystemBase {

    private WPI_VictorSPX ctrlPanelMotor;
    private ColorSensorV3 colorSensor;

    //Used for rotation control
    private int slicesTurned = 0;
    private PanelColor lastColor = PanelColor.noValue;

    public CtrlPanelSubsystem() {
        if (Config.isCtrlPanelInstalled) {
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            ctrlPanelMotor = new WPI_VictorSPX(Config.turnerMotorID);
            ctrlPanelMotor.configFactoryDefault();
            ctrlPanelMotor.setNeutralMode(NeutralMode.Brake);
        }
    }

    private boolean hasSensorColor () {
        return getColor() != PanelColor.unknown;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * // TODO: Add proper documentation
     * // DMK: don't reference what calls this in comments because it will change as the code evolves.
     * // DMK: don't say resets variables - anyone can see that from the code.
     * // DMK: documentation should explain the purpose of the method and the physical conditions under which it is called,
     * // DMK: not the line of code that calls it.
     * // DMK: same comments apply to other methods as well.
     -----------------------------------------------------------------------------------------------------------------*/
    public void resetRotationControlCounter () {
        slicesTurned = 0;
        lastColor = PanelColor.noValue;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     -----------------------------------------------------------------------------------------------------------------*/
    public boolean spin() {
        if (Config.isCtrlPanelInstalled) {
            if (hasSensorColor()) {
                if (isFMSColorAvailable()) {
                    /*
                    * Position control
                    *
                    * Tell the motor to spin until the color sensor sees the correct color.
                    */
                    PanelColor sensorColor = getColor();

                    // Get the desired color from the FMS and change it to the corresponding color that the sensor
                    // needs to be seeing
                    PanelColor TargetColor = getFMSColor();
                        switch (TargetColor) {
                            case red :
                                //Red corresponds to cyan...
                                TargetColor = PanelColor.cyan;
                                break;
                            case yellow:
                                //...Yellow to green...
                                TargetColor = PanelColor.green;
                                break;
                            case cyan :
                                //...Cyan to red...
                                TargetColor = PanelColor.red;
                                break;
                            case green:
                                //...And green to yellow
                                TargetColor = PanelColor.yellow;
                                break;
                            case unknown:
                                return false;
                            case noValue:
                                return true;
                        }

                    if (sensorColor == TargetColor) {
                        ctrlPanelMotor.stopMotor(); //We are done, so apply brakes
                        return true;
                    } else {
                        ctrlPanelMotor.set(ControlMode.PercentOutput, Config.turnerRotationPower);
                        return false;
                    }
                } //End of position control
                else {
                    /*
                     * Rotation Control
                     *
                     * Spin the motor until the correct amount of rotations has been reached. If the color from the
                     * sensor is different from the last color detected, then increment slicesTurned
                     *
                     * slicesTurned and lastColor are class member variables. They are reset whenever the
                     * manipulator is deployed.
                     */

                    PanelColor sensorColor = getColor();

                    if (sensorColor != lastColor && lastColor != PanelColor.unknown && lastColor != PanelColor.noValue) {
                        slicesTurned++;
                    }
                    lastColor = sensorColor;

                    if (slicesTurned <= Config.rotationControlSlices) {
                        ctrlPanelMotor.set(ControlMode.PercentOutput, Config.turnerRotationPower);
                        return false;
                    } else {
                        ctrlPanelMotor.stopMotor(); //We are done, so apply brakes
                        return true;
                    }
                } //End of rotation control
            } else {
                return false;
            }
        }
        return true;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Returns the color that the color sensor is currently sensing. This, along with established control panel color
     * sequence, will enable the robot to turn the control panel to the correct color.
     *
     * @return PanelColor.[color] - the color which the color sensor is currently on.
     -----------------------------------------------------------------------------------------------------------------*/
    double confidence = 0;
    private PanelColor getColor() {
        if (Config.isCtrlPanelInstalled) {
            ColorMatch colorMatcher = new ColorMatch();
            colorMatcher.addColorMatch(Color.kRed);
            colorMatcher.addColorMatch(Color.kGreen);
            colorMatcher.addColorMatch(Color.kYellow);
            colorMatcher.addColorMatch(Color.kCyan);
            colorMatcher.setConfidenceThreshold(Config.colorConfidenceThreshold);

            ColorMatchResult match = colorMatcher.matchClosestColor(colorSensor.getColor());
            confidence = match.confidence;
            if (match.color == Color.kRed) {
                return PanelColor.red;
            } else if (match.color == Color.kGreen) {
                return PanelColor.green;
            } else if (match.color == Color.kYellow) {
                return PanelColor.yellow;
            } else if (match.color == Color.kCyan) {
                return PanelColor.cyan;
            } else {
                return PanelColor.unknown;
            }
        } else {
            return PanelColor.noValue;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Obtains via the FMS the color to which the robot must turn the control panel.
     *
     * @return PanelColor.[color] - the color to which the robot must turn the control panel.
     -----------------------------------------------------------------------------------------------------------------*/
    public PanelColor getFMSColor() {
        if (Config.isCtrlPanelInstalled) {
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
                        DriverStation.reportError("Unknown color received from FMS", true);
                        return PanelColor.unknown;
                }
            } else {
                return PanelColor.noValue;
            }
        } else {
            return PanelColor.unknown;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks whether the color for position control is available from the FMS.
     *
     * @return Whether the FMS color is currently available.
     -----------------------------------------------------------------------------------------------------------------*/
    public boolean isFMSColorAvailable() {
        return getFMSColor() != PanelColor.noValue;
    }

    public void stop() {
        ctrlPanelMotor.stopMotor();
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        SmartDashboard.putNumber("Turner motor current", Robot.pdp.getCurrent(Config.turnerMotorPdpChannel));
        SmartDashboard.putString("Sensor color ", getColor().name());
        SmartDashboard.putNumber("Confidence ", confidence);
        SmartDashboard.putString("FMS color ", getFMSColor().name());
        SmartDashboard.putString("Last color", lastColor.name());
        SmartDashboard.putNumber("Slices turned ", slicesTurned);

        SmartDashboard.putNumber("R ", colorSensor.getColor().red);
        SmartDashboard.putNumber("G ", colorSensor.getColor().green);
        SmartDashboard.putNumber("B ", colorSensor.getColor().blue);
    }
}