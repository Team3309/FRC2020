package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
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

    private boolean defaultCoastMode = false;
    private boolean coastMode = defaultCoastMode;

    public ClimberSubsystem() {
        if (Config.isClimberInstalled) {
            winchMotor = new WPI_TalonFX(Config.climbMotorOneId);
            setCoastMode(defaultCoastMode);
            winchMotor.setInverted(true); // TODO: Move this to config

            //winchMotorSlave = new WPI_TalonFX(Config.climbMotorTwoId); //Uncomment these to initialize the second motor
            //winchMotorSlave.follow(climberMotor);
            if (Config.isPcmInstalled) {
                deployPiston = new Solenoid(Config.climberDeploySolenoidId);
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty(
                "test coast mode",
                () -> coastMode,
                (boolean bool) -> setCoastMode(bool));
    }

    /**-----------------------------------------------------------------------------------------------------------------
     *
     */
    public void setCoastMode(boolean coast) {
        coastMode = coast;
        winchMotor.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
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
     * Activates the piston for the climber.
     */
    public void retractClimber() {
        if (Config.isClimberInstalled && Config.isPcmInstalled) {
            deployPiston.set(false);
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
