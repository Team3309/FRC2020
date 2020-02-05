package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

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

    public enum panelColor {

        red(1),
        yellow(2),
        green(3),
        cyan(4),
        noValue(5),
        unknown(6);

        private int index;

        panelColor(int index) {this.index = index;}
    }

    private Solenoid retractorPiston;
    private Solenoid heightAdjustmentPiston;

    private WPI_TalonFX ctrlPanelMotor;

    private ColorSensorV3 colorSensor;

    public CtrlPanelSubsystem() {
        ctrlPanelMotor = new WPI_TalonFX(Config.TurnerMotorID);
        ctrlPanelMotor.configFactoryDefault();
        retractorPiston = new Solenoid(Config.TurnerTractorPistonPdpChannel);
        heightAdjustmentPiston = new Solenoid(Config.TurnerHeightAdjustmentPistonID);
    }

    //TODO: account for wacky control panel start positions.
    //turns the control panel by amount; if inRevs == true, will turn in revolutions, else, will turn in degrees.
    public void Turn(double segments) {

        ctrlPanelMotor.set(ControlMode.Position, segments*Config.EncoderCountsPerDegree*
                    45*(20/Config.TurnerWheelInchesPerRevolution));

    }

    public void Rotate (ControlMode mode, double value) {
        ctrlPanelMotor.set(mode, value);
    }

    public panelColor getColor () {
        Color color = colorSensor.getColor();
        if (color.equals(Color.kRed)) {
            return panelColor.red;
        }
        else if (color.equals(Color.kYellow)) {
            return panelColor.yellow;
        }
        else if (color.equals(Color.kGreen)) {
            return panelColor.green;
        }
        else if (color.equals(Color.kCyan)) {
            return panelColor.cyan;
        }
        else {
            return panelColor.unknown;
        }
    }

    public void Engage() {
        /*
        * Pseudocode:
        *
        */
    }

    public void Disengage() {
        /*
        * Pseudocode:
        *
        */
    }

    public void RaiseTurner() {
        /*
        * Pseudocode:
        *
        */
    }

    public panelColor GetFMSColor() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    return panelColor.cyan;
                case 'G' :
                    return panelColor.green;
                case 'R' :
                    return panelColor.red;
                case 'Y' :
                    return panelColor.yellow;
                default :
                    DriverStation.reportError("Corrupt FMS Value!", true);
                    return panelColor.unknown;
            }
        } else {
            return panelColor.noValue;
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
