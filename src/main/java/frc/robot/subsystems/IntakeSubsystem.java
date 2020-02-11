package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

/** --------------------------------------------------------------------------------------------------------------------
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 */
public class IntakeSubsystem extends SubsystemBase {

    private WPI_VictorSPX intakeMotor;
    private Solenoid solenoid;


    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public IntakeSubsystem() {
        if (Config.isIntakeInstalled) {
            intakeMotor = new WPI_VictorSPX(Config.IntakeMotorID);
            intakeMotor.configFactoryDefault();
            solenoid = new Solenoid(Config.IntakeSolenoidChannel);
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Set power to the motors
     * @param power -1 to 1
     */
    private void setPowerRaw(double power) {
        if (Config.isIntakeInstalled) {
            intakeMotor.set(ControlMode.PercentOutput, power);
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for intaking a power cell.
     */
    public void intake() {
        setPowerRaw(Config.intakeInwardPower);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for outtaking a power cell.
     */
    public void outtake() {
        setPowerRaw(-Config.intakeOutwardPower);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Stops the intake wheels from spinning
     */
    public void stop() {
        if (Config.isIntakeInstalled) {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Activates intake piston to extend the intake forward.
     */
    public void extend() {
        if (Config.isIntakeInstalled && Config.isPcmInstalled) {
            solenoid.set(true);
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Deactivates the intake piston to retract the intake back.
     */
    public void retract() {
        if (Config.isIntakeInstalled && Config.isPcmInstalled) {
            solenoid.set(false);
        }
    }

     /** ----------------------------------------------------------------------------------------------------------------
      * Sends motor data to SmartDashboard
      */
     public void outputToDashboard() {
         //SmartDashboard.putNumber("Key", value);
     }
}
