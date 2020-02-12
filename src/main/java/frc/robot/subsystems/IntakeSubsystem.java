package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

/** --------------------------------------------------------------------------------------------------------------------
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 */
public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonSRX intakeMotor;
    private Solenoid solenoid;
    private long solenoidStateSwapTimer;


    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public IntakeSubsystem() {
        if (Config.isIntakeInstalled) {
            intakeMotor = new WPI_TalonSRX(Config.IntakeMotorID);
            intakeMotor.configFactoryDefault();
            intakeMotor.config_kP(0, Config.IntakeMotorVelocityP, 0);
            intakeMotor.config_kI(0, Config.IntakeMotorVelocityI, 0);
            intakeMotor.config_kD(0, Config.IntakeMotorVelocityD, 0);
            intakeMotor.setNeutralMode(NeutralMode.Coast);
            if (Config.isPcmInstalled) {
                solenoid = new Solenoid(Config.IntakeSolenoidChannel);
            }
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

        if (Config.isIntakeInstalled && !isSolenoidSwappingStates()) {
            setPowerRaw(Config.intakeInwardPower);
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for outtaking a power cell.
     */
    public void outtake() {
        if (Config.isIntakeInstalled && !isSolenoidSwappingStates()) {
            setPowerRaw(-Config.intakeOutwardPower);
        }

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
        if (Config.isIntakeInstalled && Config.isPcmInstalled && !isSolenoidSwappingStates()) {
            solenoid.set(true);
            solenoidStateSwapTimer = System.currentTimeMillis();
        }
    }

    public boolean isSolenoidSwappingStates() {
        return System.currentTimeMillis() - solenoidStateSwapTimer > Config.IntakePistonDelayTimer;
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Deactivates the intake piston to retract the intake back.
     */
    public void retract() {
        if (Config.isIntakeInstalled && Config.isPcmInstalled && !isSolenoidSwappingStates()) {
            solenoid.set(false);
            solenoidStateSwapTimer = System.currentTimeMillis();
        }
    }

     /** ----------------------------------------------------------------------------------------------------------------
      * Sends motor data to SmartDashboard
      */
     public void outputToDashboard() {
         //SmartDashboard.putNumber("Key", value);
     }
}
