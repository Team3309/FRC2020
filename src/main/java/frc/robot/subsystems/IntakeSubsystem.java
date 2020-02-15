package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

/** --------------------------------------------------------------------------------------------------------------------
 * The class for the power cell intake, which will intake or expel power cells. Will work with shooter and
 * indexer to move power cells around within the robot.
 */
public class IntakeSubsystem extends SubsystemBase {

    private Timer timer;
    private WPI_TalonSRX intakeMotor;
    private Solenoid solenoid;
    private double solenoidStateExtendSwapTime;
    private boolean isSolenoidExtended;


    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public IntakeSubsystem() {

        if (Config.isIntakeInstalled) {

            timer = new Timer();
            timer.start();
            intakeMotor = new WPI_TalonSRX(Config.IntakeMotorID);
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
            if (Config.isPcmInstalled) {
                solenoid = new Solenoid(Config.IntakeSolenoidChannel);
            }
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for intaking a power cell.
     */
    public void intake() {
        if (Config.isIntakeInstalled && isPistonTravelComplete()) {
            intakeMotor.set(ControlMode.PercentOutput, Config.intakeInwardPower);
        }
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for outtaking a power cell.
     */
    public void outtake() {
        if (Config.isIntakeInstalled && isPistonTravelComplete()) {
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
            solenoid.set(Config.intakeDefaultIsRetracted);
            solenoidStateExtendSwapTime = timer.get();
            isSolenoidExtended = true;
        }
    }

    public boolean isPistonTravelComplete() {
        if (!Config.isIntakeInstalled || !Config.isPcmInstalled) return true;
        return timer.get() - solenoidStateExtendSwapTime >
                (isSolenoidExtended ? Config.IntakePistonExtendDelaySeconds : Config.IntakePistonRetractDelaySeconds);
    }


    /** ----------------------------------------------------------------------------------------------------------------
     * Deactivates the intake piston to retract the intake back.
     */
    public void retract() {
        if (Config.isIntakeInstalled && Config.isPcmInstalled) {
            solenoid.set(!Config.intakeDefaultIsRetracted);
            solenoidStateExtendSwapTime = timer.get();
            isSolenoidExtended = false;
        }
    }

     /** ----------------------------------------------------------------------------------------------------------------
      * Sends motor data to SmartDashboard
      */
     public void outputToDashboard() {
         SmartDashboard.putBoolean("Intake Extended", isSolenoidExtended);
     }
}
