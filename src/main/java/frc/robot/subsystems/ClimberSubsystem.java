package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
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

    private boolean defaultCoastMode = false;
    private boolean coastMode = defaultCoastMode;

    public ClimberSubsystem() {
        if (Config.isClimberInstalled) {
            winchMotor = new WPI_TalonFX(Config.climbMotorId);
            setCoastMode(defaultCoastMode);
            winchMotor.setInverted(true);

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
