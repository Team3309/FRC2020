package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** -------------------------------------------------------------------------------------------------------------------
 * The class for the shooter subsystem, which will launch the power cells to desired targets.
 * Will work in tandem with indexer to determine whether to shoot or not, and will work with
 * aimer and drive to determine what level of power to use to achieve an accurate shot.
 */
public class ShooterSubsystem extends SubsystemBase {

    private Integer flywheelSpeedTop;
    private Integer flywheelSpeedBottom;

    private WPI_TalonFX topMotor;
    private WPI_TalonFX bottomMotor;

    public ShooterSubsystem() {
        if (Config.shooterAvailable) {
            topMotor = new WPI_TalonFX(Config.topShooterMotorID);
            bottomMotor = new WPI_TalonFX(Config.bottomShooterMotorID);
            configTalon(topMotor);
            configTalon(bottomMotor);
        }
    }

     /**---------------------------------------------------------------------------------------------------------------\
     * Configures shooter motors to shooter constants, including PID values, ramp rates and motor settings.
     *
     * @param talon - the talon to be configured.
     */
    public void configTalon(WPI_TalonFX talon) {

        talon.configFactoryDefault();
        talon.setInverted(true);
        talon.setSensorPhase(true);

        talon.configClosedloopRamp(Config.shooterClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.configOpenloopRamp(Config.shooterOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);

        talon.config_kP(0, Config.shooterVelocityP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.shooterVelocityI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.shooterVelocityIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.shooterVelocityD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(0, Config.shooterVelocityF, Config.motorControllerConfigTimeoutMs);

        talon.setNeutralMode(NeutralMode.Coast);
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Spins up the flywheels in preparation for firing.
     */
    public void runFlywheelsAtPresetSpeeds() {
        runFlywheels(flywheelSpeedTop, flywheelSpeedBottom);
    }

    public boolean areFlywheelsToSpeed() {
        if (!Config.shooterAvailable || flywheelSpeedBottom == null || flywheelSpeedTop == null) return true;
        return (
            Math.abs(getTopMotorVelocity() - flywheelSpeedTop) < Config.shooterSpeedTolerance &&
            Math.abs(getBottomMotorVelocity() - flywheelSpeedBottom) < Config.shooterSpeedTolerance);
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Stops the flywheels by having them slow down to a complete stop.
     */
    public void stopFlywheels() {
        if (Config.shooterAvailable) {
            topMotor.set(ControlMode.PercentOutput, 0);
            bottomMotor.set(ControlMode.PercentOutput, 0);
        }
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Spins up the flywheels at different velocities so that backspin or forward spin can be achieved in addition to a
     * straight shot.
     *
     * @param topSpeed - the desired speed of the top motor.
     * @param bottomSpeed - the desired speed of the bottom motor.
     */
    public void runFlywheels(double topSpeed, double bottomSpeed) {
        if (Config.shooterAvailable && flywheelSpeedTop != null && flywheelSpeedBottom != null) {
            topMotor.set(ControlMode.Velocity, topSpeed);
            bottomMotor.set(ControlMode.Velocity, bottomSpeed);
        }
    }

     /** ---------------------------------------------------------------------------------------------------------------
      * Stores motor speed for both motors equally
      *
      * @param speedTop - stores speed for for later engagement
      * @param speedBottom - stores speed for bottom motor for later engagement
      */
     public void setDesiredSpeed(Integer speedTop, Integer speedBottom) {
         flywheelSpeedTop = speedTop;
         flywheelSpeedBottom = speedBottom;
     }

     /** ---------------------------------------------------------------------------------------------------------------
     * Returns the top motor's current speed.
     *
     * @return The top motor's current speed.
     */
    public double getTopMotorVelocity() {
        if (Config.shooterAvailable) {
            return topMotor.getSelectedSensorVelocity(0);
        }
        return 0;
    }

     /** ---------------------------------------------------------------------------------------------------------------
     * Returns the bottom motor's current speed.
     *
     * @return The bottom motor's current speed.
     */
    public double getBottomMotorVelocity() {
        if (Config.shooterAvailable) {
            return bottomMotor.getSelectedSensorVelocity(0);
        }
        return 0;
    }

     /** ---------------------------------------------------------------------------------------------------------------
      * Run the flywheels in voltage control mode for intake testing.
      * Positive power is outtaking, negative power is intaking.
      *
      * @param topPower - power of the top motor.
      * @param bottomPower - power of the bottom motor.
      */
     public void setPowerRaw(double topPower, double bottomPower) {
         if (Config.shooterAvailable) {
             topMotor.set(ControlMode.PercentOutput, topPower);
             bottomMotor.set(ControlMode.PercentOutput, bottomPower);
         }
     }

     /** ---------------------------------------------------------------------------------------------------------------
      * @return true if flywheel speeds have been set
      */
     public boolean hasPresetSpeeds() {
         return flywheelSpeedTop != null && flywheelSpeedBottom != null;
     }

    /** ----------------------------------------------------------------------------------------------------------------
     * Called continuously while the multi-shot button/trigger is held so we can
     * begin shooting as soon as the flywheels are up to speed. We do this here rather than in a
     * SelectCommand because we want to be able to immediately jump to the INIT_MULTI_SHOT state without waiting
     * for the command scheduler. This prevents multiple instances of multiShotCmdGroup from being
     * scheduled and thereby canceling each other.
     *
     * @param multiShotCmdGroup: command to be run
     */
     public void holdMultiShot(Command multiShotCmdGroup) {
         if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT) {
             RobotContainer.setRobotState(RobotContainer.RobotState.INIT_MULTI_SHOT);
             CommandScheduler.getInstance().schedule(multiShotCmdGroup);
         }
     }

    /** ----------------------------------------------------------------------------------------------------------------
     * Called continuously while the single-shot button/trigger is held so we can
     * begin shooting as soon as the flywheels are up to speed. We do this here rather than in a
     * SelectCommand because we want to be able to immediately jump to the INIT_SINGLE_SHOT state without waiting
     * for the command scheduler. This prevents multiple instances of singleShotCmdGroup from being
     * scheduled and thereby canceling each other.
     *
     * @param singleShotCmdGroup: command to be run
     */
    public void holdSingleShot(Command singleShotCmdGroup) {
        if (RobotContainer.getRobotState() == RobotContainer.RobotState.READY_TO_SHOOT) {
            RobotContainer.setRobotState(RobotContainer.RobotState.INIT_SINGLE_SHOT);
            CommandScheduler.getInstance().schedule(singleShotCmdGroup);
        }
    }

     /** ---------------------------------------------------------------------------------------------------------------
      * Sends motor data to SmartDashboard
      */
     public void outputToDashboard() {
         SmartDashboard.putNumber("Top flywheel speed", getTopMotorVelocity());
         SmartDashboard.putNumber("Top flywheel desired speed", flywheelSpeedTop == null ? 0 : flywheelSpeedTop);
         SmartDashboard.putNumber("Top flywheel speed error", flywheelSpeedTop == null ? 0 : getTopMotorVelocity() - flywheelSpeedTop);
         SmartDashboard.putNumber("Top flywheel power", topMotor.getMotorOutputPercent());
         SmartDashboard.putNumber("Top flywheel current", Robot.pdp.getCurrent(Config.topShooterPdpChannel));
         SmartDashboard.putNumber("Bottom flywheel speed", getBottomMotorVelocity());
         SmartDashboard.putNumber("Bottom flywheel desired speed", flywheelSpeedBottom == null ? 0 : flywheelSpeedBottom);
         SmartDashboard.putNumber("Bottom flywheel speed error", flywheelSpeedBottom == null ? 0 : getBottomMotorVelocity() - flywheelSpeedBottom);
         SmartDashboard.putNumber("Bottom flywheel power", topMotor.getMotorOutputPercent());
         SmartDashboard.putNumber("Bottom flywheel current", Robot.pdp.getCurrent(Config.topShooterPdpChannel));
     }
 }
