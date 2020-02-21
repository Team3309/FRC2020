package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;


/**
 * @author Joshua Badzey
 *
 * The climber class for the climber subsystem, which will grab the shield generator switch and pull the robot up.
 * Will work with drive to determine whether it should begin climbing.
 *
 */

public class ClimberSubsystem extends SubsystemBase {

    private WPI_TalonFX climberMotor;
    //private WPI_TalonFX climberMotorSlave; //Uncomment this for a second motor
    private Solenoid deployPiston;

    public ClimberSubsystem() {
        if (Config.isClimberInstalled) {
            climberMotor = new WPI_TalonFX(Config.climbMotorOneId);
            //climberMotorSlave = new WPI_TalonFX(Config.climbMotorTwoId); //Uncomment these to initialize the second motor
            //climberMotorSlave.follow(climberMotor);
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

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }
}
