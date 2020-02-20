package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;

/** --------------------------------------------------------------------------------------------------------------------
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 *
 * We have many states we consider in this class such as:
 *
 * Sudden disable (fixed by only swapping states via operator input)
 * Enabled in wrong state (fixed by only swapping states via operator input)
 *
 * Essentially, the intake should not automatically change states as a safety precaution
 */
public class IntakeSubsystem extends SubsystemBase {

    private Timer timer;
    private WPI_TalonSRX intakeMotor;
    private DoubleSolenoid solenoid;
    private double solenoidStateExtendSwapTime;

    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public IntakeSubsystem() {

        if (Config.isIntakeInstalled) {
            timer = new Timer();
            timer.start();
            intakeMotor = new WPI_TalonSRX(Config.intakeMotorID);
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
            if (Config.isPcmInstalled) {
                solenoid = new DoubleSolenoid(Config.intakeSolenoidChannel1, Config.intakeSolenoidChannel2);
            }
            if (Config.armNoPositionSensors || Config.armPIDTuningMode) {
                extend();
            }
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for intaking a power cell.
     */
    public void intake() {
        if (Config.isIntakeInstalled) {
            intakeMotor.set(ControlMode.PercentOutput, Config.intakeInwardPower);
        }
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for outtaking a power cell.
     */
    public void outtake() {
        if (Config.isIntakeInstalled) {
            intakeMotor.set(ControlMode.PercentOutput, -Config.intakeOutwardPower);
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
        if (Config.isIntakeInstalled && Config.isPcmInstalled) {
            DoubleSolenoid.Value solenoidState = solenoid.get();
            if (solenoidState == DoubleSolenoid.Value.kReverse ||
                    solenoidState == DoubleSolenoid.Value.kOff) {
                solenoid.set(DoubleSolenoid.Value.kForward);
                solenoidStateExtendSwapTime = timer.get();
            }
        }
    }

    public boolean isPistonTravelComplete() {
        if (!Config.isIntakeInstalled || !Config.isPcmInstalled) return true;
        double timestamp = timer.get();
        //if we are off we don't know what state was last so we just check both for safety
        if (solenoid.get() == DoubleSolenoid.Value.kOff) {
            return timestamp - solenoidStateExtendSwapTime > Config.intakePistonExtendDelaySeconds &&
                    timestamp - solenoidStateExtendSwapTime > Config.intakePistonRetractDelaySeconds ;
        }
        return timestamp - solenoidStateExtendSwapTime >
                (solenoid.get() == DoubleSolenoid.Value.kForward ?
                        Config.intakePistonExtendDelaySeconds :
                        Config.intakePistonRetractDelaySeconds);
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Deactivates the intake piston to retract the intake back.
     */
    public void retract() {
        if (Config.isIntakeInstalled && Config.isPcmInstalled) {
            DoubleSolenoid.Value solenoidState = solenoid.get();
            if (solenoidState == DoubleSolenoid.Value.kForward ||
                solenoidState == DoubleSolenoid.Value.kOff) {
                solenoid.set(DoubleSolenoid.Value.kReverse);
                solenoidStateExtendSwapTime = timer.get();
            }
        }
    }

     /** ----------------------------------------------------------------------------------------------------------------
      * Sends motor data to SmartDashboard
      */
     public void outputToDashboard() {
         SmartDashboard.putNumber("Intake motor current", Robot.pdp.getCurrent(Config.intakeMotorPdpChannel));
         SmartDashboard.putBoolean("Intake extended", isExtended());
         SmartDashboard.putBoolean("Intake travel complete", isPistonTravelComplete());
     }

    public boolean isExtended() {
        if (!Config.isIntakeInstalled || !Config.isPcmInstalled) return true;
        return solenoid.get() == DoubleSolenoid.Value.kForward;
    }
}
