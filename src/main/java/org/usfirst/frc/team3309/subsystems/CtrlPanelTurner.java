package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import org.usfirst.frc.team3309.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the control panel manipulator mechanism, which will turn the control panel a desired
 * number of times, turn it to a specific color, and receive a color from the FMS. Will work with
 * FMS to determine what color is needed for position control.
 *
 */

public class CtrlPanelTurner extends SubsystemBase {

    private WPI_TalonFX ctrlPanelMotor;
    public CtrlPanelTurner() {
        ctrlPanelMotor = new WPI_TalonFX(Constants.TURNER_MOTOR_ID);
        ctrlPanelMotor.configFactoryDefault();
    }

    //turns the control panel by amount; if inRevs == true, will turn in revolutions, else, will turn in degrees.
    public void turn(double amount,  boolean inRevs) {
        if(inRevs) {
            ctrlPanelMotor.set(ControlMode.Position, amount*Constants.ENCODER_COUNTS_PER_DEGREE*
                    360*(20/Constants.TURNER_INCHES_PER_REV));
        } else {
            ctrlPanelMotor.set(ControlMode.Position, amount*Constants.ENCODER_COUNTS_PER_DEGREE);
        }
    }

    public void getFMSColor() {}

    public void getColor() {}

}
