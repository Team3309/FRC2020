package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.util.PanelColor;

/**
 * @author Joshua Badzey & Mark Ghebrial
 *
 * The class for the control panel manipulator mechanism, which will turn the control panel a desired
 * number of times, turn it to a specific color, and receive a color from the FMS. Works with
 * FMS to determine what color is needed for position control.
 */

public class CtrlPanelSubsystem extends SubsystemBase {

    public enum turnerState {
        deployedSpinning,
        deployedStopped,
        stowed
    }

    private Solenoid retractorPiston;
    private WPI_TalonSRX ctrlPanelMotor;
    private ColorSensorV3 colorSensor;

    //Used for rotation control
    private int slicesTurned = 0;
    private PanelColor lastColor = PanelColor.noValue;

    public CtrlPanelSubsystem() {
        if (Config.isCtrlPanelInstalled) {
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            ctrlPanelMotor = new WPI_TalonSRX(Config.TurnerMotorID);
            ctrlPanelMotor.configFactoryDefault();
            if (Config.isPcmInstalled) {
                retractorPiston = new Solenoid(Config.TurnerTractorPistonID);
            }
        }
    }

    private boolean hasColor () {
        return getColor() != PanelColor.unknown;
    }

    private boolean deployed () {
        //TODO: Add timer
        if(Config.isPcmInstalled) {
            return retractorPiston.get();
        } else {
            return false;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Resets the class member variables used by rotation control. This method is used in DeployTurner in the initialize
     * method.
     -----------------------------------------------------------------------------------------------------------------*/
    public void resetRotationControlCounter () {
        slicesTurned = 0;
        lastColor = PanelColor.noValue;
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Decides either to do rotation control, position control, or nothing. Contains the logic for each. This method is
     * called continuously in the Rotate command, which is default for this subsystem.
     -----------------------------------------------------------------------------------------------------------------*/
    public void spin() {
        if (Config.isCtrlPanelInstalled && Config.isPcmInstalled) {
            if (deployed()) {
                if (hasColor()) {
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

                        if (sensorColor == TargetColor) {
                            ctrlPanelMotor.stopMotor(); //We are done, so apply brakes
                        } else {
                            ctrlPanelMotor.set(ControlMode.PercentOutput, Config.TurnerRotationPower);
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

                        if (slicesTurned <= Config.RotationControlSlices) {
                            ctrlPanelMotor.set(ControlMode.PercentOutput, Config.TurnerRotationPower);
                        } else {
                            ctrlPanelMotor.stopMotor(); //We are done, so apply brakes
                        }

                        if (sensorColor != lastColor) {
                            slicesTurned++;
                        }
                        lastColor = sensorColor;
                    } //End of rotation control
                }
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Returns the color that the color sensor is currently sensing. This, along with established control panel color
     * sequence, will enable the robot to turn the control panel to the correct color.
     *
     * @return PanelColor.[color] - the color which the color sensor is currently on.
     -----------------------------------------------------------------------------------------------------------------*/
    public PanelColor getColor () {
        //TODO: check if pistion is enabled
        if (Config.isCtrlPanelInstalled) {
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
        } else {
            return PanelColor.unknown;
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Flips the manipulator up out of the trench run configuration
     -----------------------------------------------------------------------------------------------------------------*/
    public void deploy() {

        if(Config.isCtrlPanelInstalled) {
            retractorPiston.set(true);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Pulls the manipulator into the trench run configuration
     -----------------------------------------------------------------------------------------------------------------*/
    public void retract() {
        if(Config.isCtrlPanelInstalled) {
            retractorPiston.set(false);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Obtains via the FMS the color to which the robot must turn the control panel.
     *
     * @return PanelColor.[color] - the color to which the robot must turn the control panel.
     -----------------------------------------------------------------------------------------------------------------*/
    public PanelColor getFMSColor() {
        if (Config.isCtrlPanelInstalled) {
            return PanelColor.unknown;
        } else {
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
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Checks whether the color for position control is available from the FMS.
     *
     * @return Whether the FMS color is currently available.
     -----------------------------------------------------------------------------------------------------------------*/
    public boolean isFMSColorAvailable() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        return gameData.length() > 0;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }

}
