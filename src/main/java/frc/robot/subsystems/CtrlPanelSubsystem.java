package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * @author Joshua Badzey
 *
 * The class for the control panel manipulator mechanism, which will turn the control panel a desired
 * number of times, turn it to a specific color, and receive a color from the FMS. Will work with
 * FMS to determine what color is needed for position control.
 *
 */

public class CtrlPanelSubsystem extends SubsystemBase {

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

    public CtrlPanelSubsystem() {
        ctrlPanelMotor = new WPI_TalonFX(Constants.TURNER_MOTOR_ID);
        ctrlPanelMotor.configFactoryDefault();
        retractorPiston = new Solenoid(Constants.TURNER_RETRACTOR_PISTON_ID);
        heightAdjustmentPiston = new Solenoid(Constants.TURNER_HEIGHT_ADJUST_PISTON_ID);
    }

    //TODO: account for wacky control panel start positions.
    //turns the control panel by amount; if inRevs == true, will turn in revolutions, else, will turn in degrees.
    public void Turn(double segments) {

        ctrlPanelMotor.set(ControlMode.Position, segments*Constants.ENCODER_COUNTS_PER_DEGREE*
                    45*(20/Constants.TURNER_INCHES_PER_REV));

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

    public void GetFMSColor() {
    }

    public void DeployTurner() {
        retractorPiston.set(true);
    }
    public void RetractTurner() {
        retractorPiston.set(false);
    }

}
