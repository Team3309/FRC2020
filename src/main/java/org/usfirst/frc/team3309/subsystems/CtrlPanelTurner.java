package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import org.usfirst.frc.team3309.Constants;
import edu.wpi.first.wpilibj.I2C.Port;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

/**
 * @author Joshua Badzey
 *
 * The class for the control panel manipulator mechanism, which will turn the control panel a desired
 * number of times, turn it to a specific color, and receive a color from the FMS. Will work with
 * FMS to determine what color is needed for position control.
 *
 */

public class CtrlPanelTurner extends SubsystemBase {

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
        unknown(5);

        private int index;

        panelColor(int index) {this.index = index;}
    }

    private Solenoid retractorPiston;
    private Solenoid heightAdjustmentPiston;

    private WPI_TalonFX ctrlPanelMotor;
    private ColorSensorV3 colorSensor;

    public CtrlPanelTurner() {
        ctrlPanelMotor = new WPI_TalonFX(Constants.TURNER_MOTOR_ID);
        ctrlPanelMotor.configFactoryDefault();
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        retractorPiston = new Solenoid(Constants.kTurnerRetractorPistonPdpChannel);
        heightAdjustmentPiston = new Solenoid(Constants.kTurnerHeightAdjustmentPistonPdpChannel);
    }

    //TODO: account for wacky control panel start positions.
    //turns the control panel by amount; if inRevs == true, will turn in revolutions, else, will turn in degrees.
    public void turn(double segments) {
        ctrlPanelMotor.set(ControlMode.Position, segments*Constants.ENCODER_COUNTS_PER_DEGREE*
                    45*(20/Constants.TURNER_INCHES_PER_REV));

    }

    public void engage() {
        /*
        * Pseudocode:
        *
        */
    }

    public void disengage() {
        /*
        * Pseudocode:
        *
        */
    }

    public void raiseTurner() {
        /*
        * Pseudocode:
        *
        */
    }

    public void getFMSColor() {}

    public Color getColor() {
        return colorSensor.getColor();
    }

}
