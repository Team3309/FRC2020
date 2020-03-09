package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.groups.ToDriveCmdGroup;
import frc.robot.commands.groups.ToOuttakeCmdGroup;

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

    private Timer pistonTimer = new Timer();;
    private Timer pressIntakeTimer = new Timer();;
    private Timer pressOuttakeTimer = new Timer();;
    private WPI_TalonSRX intakeMotor;
    private DoubleSolenoid solenoid;
    private double solenoidStateExtendSwapTime;
    private boolean emergencyOuttakeActive;

    /** ----------------------------------------------------------------------------------------------------------------
     * Constructor
     */
    public IntakeSubsystem() {

        if (Config.isIntakeInstalled) {
            pistonTimer.start();
            pressIntakeTimer.start();
            pressOuttakeTimer.start();
            intakeMotor = new WPI_TalonSRX(Config.intakeMotorID);
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
            intakeMotor.configOpenloopRamp(Config.intakeOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);
            if (Config.isPcmInstalled) {
                solenoid = new DoubleSolenoid(Config.intakeSolenoidChannel1, Config.intakeSolenoidChannel2);
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
        emergencyOuttakeActive = false;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Spins the intake wheels for outtaking a power cell.
     */
    public void outtake() {
        if (Config.isIntakeInstalled) {
            intakeMotor.set(ControlMode.PercentOutput, -Config.intakeOutwardPower);
        }
        emergencyOuttakeActive = false;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Stops the intake wheels from spinning
     */
    public void stop() {
        if (Config.isIntakeInstalled) {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
        emergencyOuttakeActive = false;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Start emergency outtake to release trapped power cell
     */
    public void startEmergencyOuttake() {
        if (Config.isIntakeInstalled && !isExtended() && isPistonTravelComplete()) {
            intakeMotor.set(ControlMode.PercentOutput, -Config.intakeEmergencyOutwardPower);
        }
        emergencyOuttakeActive = true;
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Emergency outtake complete
     */
    public void stopEmergencyOuttake() {
        if (emergencyOuttakeActive) {
            stop();
        }
    }

    public void pressIntake(Command startIntake, Command cancelIntake) {
        if (pressIntakeTimer.hasElapsed(Config.doubleClickSec)) {
            // single-click, activate intake if allowed
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_OUTTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.OUTTAKE) {
                CommandScheduler.getInstance().schedule(startIntake);
            }
        } else {
            // double-click, cancel intake if allowed
            if (RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INIT_OUTTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.OUTTAKE == RobotContainer.getRobotState()) {
                CommandScheduler.getInstance().schedule(cancelIntake);
            }
        }
        pressIntakeTimer.reset();
    }

    public void pressOuttake(Command startOuttake, Command cancelOuttake) {
        if (pressOuttakeTimer.hasElapsed(Config.doubleClickSec)) {
            // single-click, activate outtake if allowed
            if (RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.ARM_UP_DRIVE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_POSITION_TURNER ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.TURNER_IN_POSITION ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INIT_INTAKE ||
                    RobotContainer.getRobotState() == RobotContainer.RobotState.INTAKE) {
                CommandScheduler.getInstance().schedule(startOuttake);
            }
        } else {
            // double-click, cancel outtake if allowed
            if (RobotContainer.RobotState.INIT_INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.INIT_OUTTAKE == RobotContainer.getRobotState() ||
                    RobotContainer.RobotState.OUTTAKE == RobotContainer.getRobotState()) {
                CommandScheduler.getInstance().schedule(cancelOuttake);
            }
        }
        pressOuttakeTimer.reset();
    }

    /** ----------------------------------------------------------------------------------------------------------------
     * Activates intake piston to extend the intake forward.
     */
    public void extend() {
        if (Config.isIntakeInstalled && Config.isPcmInstalled) {
            stopEmergencyOuttake();
            DoubleSolenoid.Value solenoidState = solenoid.get();
            if (solenoidState == DoubleSolenoid.Value.kReverse ||
                    solenoidState == DoubleSolenoid.Value.kOff) {
                solenoid.set(DoubleSolenoid.Value.kForward);
                solenoidStateExtendSwapTime = pistonTimer.get();
            }
        }
    }

    public boolean isPistonTravelComplete() {
        if (!Config.isIntakeInstalled || !Config.isPcmInstalled) return true;
        double timestamp = pistonTimer.get();
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
                solenoidStateExtendSwapTime = pistonTimer.get();
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
         SmartDashboard.putBoolean("Intake emergencyOuttakeActive", emergencyOuttakeActive);
     }

    public boolean isExtended() {
        if (!Config.isIntakeInstalled || !Config.isPcmInstalled) return true;
        return solenoid.get() == DoubleSolenoid.Value.kForward;
    }
}
