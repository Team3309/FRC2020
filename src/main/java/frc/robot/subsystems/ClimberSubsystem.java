package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

/**
 * @author Mark Ghebrial
 *
 * The climber class for the climber subsystem, which will grab the shield generator switch and pull the robot up.
 * Will work with drive to determine whether it should begin climbing.
 *
 */

public class ClimberSubsystem extends SubsystemBase {

    private WPI_TalonFX winchMotor;
    //private WPI_TalonFX winchMotorSlave; //Uncomment this for a second motor
    private Solenoid deployPiston;

    public ClimberSubsystem() {
        if (Config.isClimberInstalled) {
            winchMotor = new WPI_TalonFX(Config.climbMotorOneId);
            winchMotor.setNeutralMode(NeutralMode.Brake);
            winchMotor.setInverted(true); // TODO: Move this to config

            //winchMotorSlave = new WPI_TalonFX(Config.climbMotorTwoId); //Uncomment these to initialize the second motor
            //winchMotorSlave.follow(climberMotor);
            if (Config.isPcmInstalled) {
                deployPiston = new Solenoid(Config.climberDeploySolenoidId);
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Activates the piston for the climber.
     */
    public void deployClimber() {
        if (Config.isClimberInstalled && Config.isPcmInstalled) {
            deployPiston.set(true);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Activates the winch
     */
    public void moveWinch (double power) {
        if (Config.isClimberInstalled) {
            winchMotor.set(ControlMode.PercentOutput, power);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
