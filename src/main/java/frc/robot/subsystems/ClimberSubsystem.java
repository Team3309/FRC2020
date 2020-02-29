package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;

/**
 * @author Mark Ghebrial
 *
 * The climber class for the climber subsystem, which will grab the shield generator switch and pull the robot up.
 * Will work with drive to determine whether it should begin climbing.
 *
 */

public class ClimberSubsystem extends SubsystemBase {

    private WPI_TalonFX winchMotor;
    private Solenoid deployPiston;

    public ClimberSubsystem() {
        if (Config.isClimberInstalled) {
            winchMotor = new WPI_TalonFX(Config.climbMotorId);
            winchMotor.setNeutralMode(NeutralMode.Brake);
            winchMotor.setInverted(true);

            if (Config.isPcmInstalled) {
                deployPiston = new Solenoid(Config.climberDeploySolenoidId);
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     *
     */
    public void setCoastMode(boolean coast) {
        if (Config.isClimberInstalled) {
            winchMotor.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
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
        SmartDashboard.putNumber("Climber current", Robot.pdp.getCurrent(Config.climbPdpChannel));
    }
}
