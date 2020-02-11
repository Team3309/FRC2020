package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    private WPI_TalonSRX primaryClimberMotor, secondaryClimberMotor;
    private Solenoid climberDeploy, hookDeploy, buddyClimbDeploy;

    public ClimberSubsystem() {
        if (Config.isClimberInstalled) {
            primaryClimberMotor = new WPI_TalonSRX(Config.ClimbMotorOneId);
            secondaryClimberMotor = new WPI_TalonSRX(Config.ClimbMotorTwoId);
            if (Config.isPcmInstalled) {
                climberDeploy = new Solenoid(Config.ClimberDeploySolenoidId);
                hookDeploy = new Solenoid(Config.HookDeploySolenoidId);
                buddyClimbDeploy = new Solenoid(Config.BuddyClimbDeploySolenoidId);
            }
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Activates the piston for the climber.
     */
    public void deployClimber() {
        if (Config.isClimberInstalled && Config.isPcmInstalled) {
            climberDeploy.set(true);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Activates the piston for the climber hook.
     */
    public void deployHook() {
        if (Config.isClimberInstalled && Config.isPcmInstalled) {
            hookDeploy.set(true);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Activates the piston for the buddy climb mechanism.
     */
    public void deployBuddyClimb() {
        if (Config.isClimberInstalled && Config.isPcmInstalled) {
            buddyClimbDeploy.set(true);
        }
    }

    /**-----------------------------------------------------------------------------------------------------------------
     * Programs the balancer motor to move by @param amount.
     */
    public void balanceRobot(double amount) {
        if(Config.isClimberInstalled) {
            primaryClimberMotor.set(ControlMode.MotionMagic, amount);
            secondaryClimberMotor.follow(primaryClimberMotor);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Sends motor data to SmartDashboard
     */
    public void outputToDashboard() {
        //SmartDashboard.putNumber("Key", value);
    }

}
